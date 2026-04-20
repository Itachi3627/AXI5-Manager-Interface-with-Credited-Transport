// ============================================================
// AXI5 Credited Transport – Simple Implementation
// Channels: AW, W, B, AR, R
// Each channel has: VALID + PENDING + CRDT signals
// ============================================================

`timescale 1ns/1ps

// ============================================================
// MANAGER
// - Sends AWVALID/ARVALID only when it has credits
// - Gets credits back via AWCRDT/ARCRDT from Subordinate
// - Returns credits to Subordinate via BCRDT/RCRDT
// - Asserts PENDING one cycle before VALID
// ============================================================
module axi_manager #(
    parameter AW = 32,
    parameter DW = 32,
    parameter INIT_CRDT = 4
)(
    input  wire        clk, rstn,

    // AW channel
    output reg  [AW-1:0] AWADDR,
    output reg           AWVALID,
    input  wire          AWREADY,
    output reg           AWPENDING,   // hint: VALID coming next cycle
    input  wire          AWCRDT,      // Subordinate returns AW credit

    // W channel
    output reg  [DW-1:0] WDATA,
    output reg           WVALID,
    input  wire          WREADY,
    output reg           WPENDING,
    input  wire          WCRDT,

    // B channel (response from Subordinate)
    input  wire          BVALID,
    output reg           BREADY,
    output reg           BCRDT,       // Manager returns B credit to Subordinate

    // AR channel
    output reg  [AW-1:0] ARADDR,
    output reg           ARVALID,
    input  wire          ARREADY,
    output reg           ARPENDING,
    input  wire          ARCRDT,

    // R channel (data from Subordinate)
    input  wire [DW-1:0] RDATA,
    input  wire          RVALID,
    output reg           RREADY,
    output reg           RCRDT        // Manager returns R credit to Subordinate
);
    // Credit counters
    reg [3:0] aw_crdt, w_crdt, ar_crdt;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            aw_crdt  <= INIT_CRDT;
            w_crdt   <= INIT_CRDT;
            ar_crdt  <= INIT_CRDT;
            AWVALID  <= 0; AWADDR   <= 0; AWPENDING <= 0;
            WVALID   <= 0; WDATA    <= 0; WPENDING  <= 0;
            ARVALID  <= 0; ARADDR   <= 0; ARPENDING <= 0;
            BREADY   <= 1; BCRDT    <= 0;
            RREADY   <= 1; RCRDT    <= 0;
        end else begin

            // --- Replenish credits from Subordinate ---
            if (AWCRDT) aw_crdt <= aw_crdt + 1;
            if (WCRDT)  w_crdt  <= w_crdt  + 1;
            if (ARCRDT) ar_crdt <= ar_crdt  + 1;

            // --- AW: send if we have credit ---
            AWPENDING <= (aw_crdt > 1);          // hint one cycle early
            if (!AWVALID && aw_crdt > 0) begin
                AWVALID <= 1;
                AWADDR  <= AWADDR + 4;
                aw_crdt <= aw_crdt - 1;
            end else if (AWVALID && AWREADY)
                AWVALID <= 0;

            // --- W: send if we have credit ---
            WPENDING <= (w_crdt > 1);
            if (!WVALID && w_crdt > 0) begin
                WVALID <= 1;
                WDATA  <= WDATA + 1;
                w_crdt <= w_crdt - 1;
            end else if (WVALID && WREADY)
                WVALID <= 0;

            // --- B: accept response, return credit to Subordinate ---
            BCRDT <= BVALID && BREADY;           // pulse credit on acceptance

            // --- AR: send if we have credit ---
            ARPENDING <= (ar_crdt > 1);
            if (!ARVALID && ar_crdt > 0) begin
                ARVALID <= 1;
                ARADDR  <= ARADDR + 4;
                ar_crdt <= ar_crdt - 1;
            end else if (ARVALID && ARREADY)
                ARVALID <= 0;

            // --- R: accept data, return credit to Subordinate ---
            RCRDT <= RVALID && RREADY;
        end
    end
endmodule


// ============================================================
// SUBORDINATE
// - Accepts requests, issues responses
// - Returns CRDT to Manager after accepting AW/W/AR
// - Uses BCRDT/RCRDT from Manager to know it can send B/R
// ============================================================
module axi_subordinate #(
    parameter AW = 32,
    parameter DW = 32,
    parameter INIT_CRDT = 4
)(
    input  wire        clk, rstn,

    // AW channel
    input  wire [AW-1:0] AWADDR,
    input  wire          AWVALID,
    output reg           AWREADY,
    input  wire          AWPENDING,   // lookahead hint from Manager
    output reg           AWCRDT,      // return AW credit to Manager

    // W channel
    input  wire [DW-1:0] WDATA,
    input  wire          WVALID,
    output reg           WREADY,
    input  wire          WPENDING,
    output reg           WCRDT,

    // B channel
    output reg           BVALID,
    input  wire          BREADY,
    output reg           BPENDING,
    input  wire          BCRDT,       // credit from Manager: may send B

    // AR channel
    input  wire [AW-1:0] ARADDR,
    input  wire          ARVALID,
    output reg           ARREADY,
    input  wire          ARPENDING,
    output reg           ARCRDT,

    // R channel
    output reg  [DW-1:0] RDATA,
    output reg           RVALID,
    input  wire          RREADY,
    output reg           RPENDING,
    input  wire          RCRDT        // credit from Manager: may send R
);
    reg [3:0] b_crdt, r_crdt;        // how many B/R responses we may send
    reg       aw_q, w_q, ar_q;       // simple 1-entry queues

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            b_crdt   <= INIT_CRDT;
            r_crdt   <= INIT_CRDT;
            AWREADY  <= 1; AWCRDT <= 0; aw_q <= 0;
            WREADY   <= 1; WCRDT  <= 0; w_q  <= 0;
            ARREADY  <= 1; ARCRDT <= 0; ar_q <= 0;
            BVALID   <= 0; BPENDING <= 0;
            RVALID   <= 0; RPENDING <= 0; RDATA <= 0;
        end else begin

            // --- Replenish B/R credits from Manager ---
            if (BCRDT) b_crdt <= b_crdt + 1;
            if (RCRDT) r_crdt <= r_crdt + 1;

            // --- Accept AW, return credit ---
            AWCRDT <= 0;
            if (AWVALID && AWREADY) begin
                aw_q   <= 1;
                AWCRDT <= 1;          // credit back immediately
                AWREADY<= 0;
            end

            // --- Accept W, return credit ---
            WCRDT <= 0;
            if (WVALID && WREADY) begin
                w_q   <= 1;
                WCRDT <= 1;
                WREADY<= 0;
            end

            // --- Send B once AW+W received and we have B credit ---
            BPENDING <= aw_q && w_q && (b_crdt > 0);
            if (aw_q && w_q && b_crdt > 0 && !BVALID) begin
                BVALID <= 1;
                b_crdt <= b_crdt - 1;
            end
            if (BVALID && BREADY) begin
                BVALID  <= 0;
                aw_q    <= 0;
                w_q     <= 0;
                AWREADY <= 1;
                WREADY  <= 1;
            end

            // --- Accept AR, return credit ---
            ARCRDT <= 0;
            if (ARVALID && ARREADY) begin
                ar_q   <= 1;
                ARCRDT <= 1;
                ARREADY<= 0;
            end

            // --- Send R once AR received and we have R credit ---
            RPENDING <= ar_q && (r_crdt > 0);
            if (ar_q && r_crdt > 0 && !RVALID) begin
                RVALID <= 1;
                RDATA  <= RDATA + 1;  // dummy read data
                r_crdt <= r_crdt - 1;
            end
            if (RVALID && RREADY) begin
                RVALID  <= 0;
                ar_q    <= 0;
                ARREADY <= 1;
            end

        end
    end
endmodule

