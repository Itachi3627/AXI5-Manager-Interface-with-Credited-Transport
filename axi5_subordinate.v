// =============================================================================
// AXI5 Subordinate Interface with Credited Transport
// =============================================================================
// Implements AXI5 Subordinate (formerly Slave) with:
//   - Standard VALID/READY handshake on all 5 channels
//   - Credit management: Subordinate issues initial credits to Manager on
//     reset, then returns credits as it processes each transaction.
//   - CRDT outputs: pulsed HIGH for one cycle to return one credit per RP
//     to the Manager after processing a request.
//   - CRDTSH: shared-credit return path.
//   - PENDING inputs: used for lookahead (pipeline pre-fetch hint).
//
// Parameters match the Manager module.
// =============================================================================

module axi5_subordinate #(
    parameter NUM_RP_AWW  = 4,
    parameter NUM_RP_AR   = 4,
    parameter INIT_CRDT   = 8,
    parameter DATA_WIDTH  = 32,
    parameter ADDR_WIDTH  = 32,
    parameter ID_WIDTH    = 4
)(
    input  wire                          ACLK,
    input  wire                          ARESETn,

    // ------------------------------------------------------------------ //
    //  AW Channel
    // ------------------------------------------------------------------ //
    input  wire [ID_WIDTH-1:0]           AWID,
    input  wire [ADDR_WIDTH-1:0]         AWADDR,
    input  wire [7:0]                    AWLEN,
    input  wire [2:0]                    AWSIZE,
    input  wire [1:0]                    AWBURST,
    input  wire                          AWVALID,
    output reg                           AWREADY,
    // Credited transport – AW
    input  wire                          AWPENDING,        // Manager hint
    output reg  [NUM_RP_AWW-1:0]         AWCRDT,           // credit return per RP
    output reg                           AWCRDTSH,         // shared credit return
    input  wire [$clog2(NUM_RP_AWW)-1:0] AWRP,
    input  wire                          AWSHAREDCRD,

    // ------------------------------------------------------------------ //
    //  W Channel
    // ------------------------------------------------------------------ //
    input  wire [DATA_WIDTH-1:0]         WDATA,
    input  wire [DATA_WIDTH/8-1:0]       WSTRB,
    input  wire                          WLAST,
    input  wire                          WVALID,
    output reg                           WREADY,
    // Credited transport – W
    input  wire                          WPENDING,
    output reg  [NUM_RP_AWW-1:0]         WCRDT,
    output reg                           WCRDTSH,
    input  wire [$clog2(NUM_RP_AWW)-1:0] WRP,
    input  wire                          WSHAREDCRD,

    // ------------------------------------------------------------------ //
    //  B Channel – Write Response
    // ------------------------------------------------------------------ //
    output reg  [ID_WIDTH-1:0]           BID,
    output reg  [1:0]                    BRESP,
    output reg                           BVALID,
    input  wire                          BREADY,
    // Credited transport – B (Subordinate receives credits back from Manager)
    output reg                           BPENDING,
    input  wire                          BCRDT,            // credit from Manager

    // ------------------------------------------------------------------ //
    //  AR Channel
    // ------------------------------------------------------------------ //
    input  wire [ID_WIDTH-1:0]           ARID,
    input  wire [ADDR_WIDTH-1:0]         ARADDR,
    input  wire [7:0]                    ARLEN,
    input  wire [2:0]                    ARSIZE,
    input  wire [1:0]                    ARBURST,
    input  wire                          ARVALID,
    output reg                           ARREADY,
    // Credited transport – AR
    input  wire                          ARPENDING,
    output reg  [NUM_RP_AR-1:0]          ARCRDT,
    output reg                           ARCRDTSH,
    input  wire [$clog2(NUM_RP_AR)-1:0]  ARRP,
    input  wire                          ARSHAREDCRD,

    // ------------------------------------------------------------------ //
    //  R Channel – Read Data
    // ------------------------------------------------------------------ //
    output reg  [ID_WIDTH-1:0]           RID,
    output reg  [DATA_WIDTH-1:0]         RDATA,
    output reg  [1:0]                    RRESP,
    output reg                           RLAST,
    output reg                           RVALID,
    input  wire                          RREADY,
    // Credited transport – R (Subordinate receives credits from Manager)
    output reg                           RPENDING,
    input  wire                          RCRDT             // credit from Manager
);

    // ================================================================== //
    //  Internal credit tracking
    //  Subordinate tracks how many B/R responses it may issue.
    //  It starts with INIT_CRDT and increments on BCRDT/RCRDT from Manager.
    // ================================================================== //
    localparam CRDT_W = $clog2(INIT_CRDT * 4 + 1);

    reg [CRDT_W-1:0] b_crdt_cnt;   // credits available to send B responses
    reg [CRDT_W-1:0] r_crdt_cnt;   // credits available to send R beats

    // ================================================================== //
    //  Internal FIFOs / queues (simplified single-entry for clarity)
    // ================================================================== //
    // AW queue
    reg                   aw_valid_q;
    reg [ID_WIDTH-1:0]    aw_id_q;
    reg [ADDR_WIDTH-1:0]  aw_addr_q;

    // W queue
    reg                   w_valid_q;
    reg [DATA_WIDTH-1:0]  w_data_q;
    reg [DATA_WIDTH/8-1:0]w_strb_q;
    reg                   w_last_q;

    // AR queue
    reg                   ar_valid_q;
    reg [ID_WIDTH-1:0]    ar_id_q;
    reg [ADDR_WIDTH-1:0]  ar_addr_q;

    // ================================================================== //
    //  Credit initialisation
    //  On reset Subordinate effectively "grants" INIT_CRDT credits to the
    //  Manager by pulsing CRDT signals for INIT_CRDT cycles.
    //  Here we model that via the initial credit counters and the READY
    //  assertion: if the Subordinate has buffer space, it is ready to accept
    //  and will return a credit after each acceptance.
    // ================================================================== //
    always @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            b_crdt_cnt <= INIT_CRDT;
            r_crdt_cnt <= INIT_CRDT;
        end else begin
            // B credit: replenished by Manager via BCRDT pulse
            case ({BCRDT, (BVALID && BREADY)})
                2'b10: b_crdt_cnt <= b_crdt_cnt + 1;
                2'b01: b_crdt_cnt <= b_crdt_cnt - 1;
                default: ;
            endcase
            // R credit: replenished by Manager via RCRDT pulse
            case ({RCRDT, (RVALID && RREADY && RLAST)})
                2'b10: r_crdt_cnt <= r_crdt_cnt + 1;
                2'b01: r_crdt_cnt <= r_crdt_cnt - 1;
                default: ;
            endcase
        end
    end

    // ================================================================== //
    //  AW acceptance
    //  AWREADY = 1 when internal queue has space.
    //  Upon acceptance we store the request and return a credit for the RP.
    // ================================================================== //
    always @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            AWREADY   <= 1'b1;
            aw_valid_q<= 1'b0;
            AWCRDT    <= {NUM_RP_AWW{1'b0}};
            AWCRDTSH  <= 1'b0;
        end else begin
            // Default: no credit pulse
            AWCRDT   <= {NUM_RP_AWW{1'b0}};
            AWCRDTSH <= 1'b0;

            if (AWVALID && AWREADY) begin
                // Accept transaction into internal queue
                aw_valid_q <= 1'b1;
                aw_id_q    <= AWID;
                aw_addr_q  <= AWADDR;
                // Return credit to Manager for the RP just used
                if (AWSHAREDCRD)
                    AWCRDTSH <= 1'b1;
                else
                    AWCRDT[AWRP] <= 1'b1;
                AWREADY <= 1'b0;  // queue full
            end else if (aw_valid_q && w_valid_q) begin
                // Both AW and W received → can process → free queue
                aw_valid_q <= 1'b0;
                AWREADY    <= 1'b1;
            end

            // PENDING hint: pre-allocate internal pipeline (optional)
            // (not consumed here, shown for completeness)
        end
    end

    // ================================================================== //
    //  W acceptance
    // ================================================================== //
    always @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            WREADY    <= 1'b1;
            w_valid_q <= 1'b0;
            WCRDT     <= {NUM_RP_AWW{1'b0}};
            WCRDTSH   <= 1'b0;
        end else begin
            WCRDT   <= {NUM_RP_AWW{1'b0}};
            WCRDTSH <= 1'b0;

            if (WVALID && WREADY) begin
                w_valid_q <= 1'b1;
                w_data_q  <= WDATA;
                w_strb_q  <= WSTRB;
                w_last_q  <= WLAST;
                if (WSHAREDCRD)
                    WCRDTSH <= 1'b1;
                else
                    WCRDT[WRP] <= 1'b1;
                WREADY <= 1'b0;
            end else if (aw_valid_q && w_valid_q) begin
                w_valid_q <= 1'b0;
                WREADY    <= 1'b1;
            end
        end
    end

    // ================================================================== //
    //  B response generation
    //  Issued when both AW and W have been captured AND Manager has B credit.
    // ================================================================== //
    always @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            BVALID   <= 1'b0;
            BID      <= 0;
            BRESP    <= 2'b00;
            BPENDING <= 1'b0;
        end else begin
            BPENDING <= aw_valid_q && w_valid_q && (b_crdt_cnt > 0);

            if (!BVALID && aw_valid_q && w_valid_q && (b_crdt_cnt > 0)) begin
                BVALID <= 1'b1;
                BID    <= aw_id_q;
                BRESP  <= 2'b00;  // OKAY
            end else if (BVALID && BREADY) begin
                BVALID <= 1'b0;
            end
        end
    end

    // ================================================================== //
    //  AR acceptance
    // ================================================================== //
    always @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            ARREADY    <= 1'b1;
            ar_valid_q <= 1'b0;
            ARCRDT     <= {NUM_RP_AR{1'b0}};
            ARCRDTSH   <= 1'b0;
        end else begin
            ARCRDT   <= {NUM_RP_AR{1'b0}};
            ARCRDTSH <= 1'b0;

            if (ARVALID && ARREADY) begin
                ar_valid_q <= 1'b1;
                ar_id_q    <= ARID;
                ar_addr_q  <= ARADDR;
                if (ARSHAREDCRD)
                    ARCRDTSH <= 1'b1;
                else
                    ARCRDT[ARRP] <= 1'b1;
                ARREADY <= 1'b0;
            end else if (ar_valid_q && RVALID && RREADY && RLAST) begin
                ar_valid_q <= 1'b0;
                ARREADY    <= 1'b1;
            end
        end
    end

    // ================================================================== //
    //  R response generation
    //  Single-beat read for simplicity (RLAST always 1).
    //  Gated by r_crdt_cnt (credits granted by Manager).
    // ================================================================== //
    always @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            RVALID   <= 1'b0;
            RID      <= 0;
            RDATA    <= 0;
            RRESP    <= 2'b00;
            RLAST    <= 1'b1;
            RPENDING <= 1'b0;
        end else begin
            RPENDING <= ar_valid_q && (r_crdt_cnt > 0);

            if (!RVALID && ar_valid_q && (r_crdt_cnt > 0)) begin
                RVALID <= 1'b1;
                RID    <= ar_id_q;
                RDATA  <= {DATA_WIDTH{1'b1}};  // dummy read data
                RRESP  <= 2'b00;
                RLAST  <= 1'b1;
            end else if (RVALID && RREADY) begin
                RVALID <= 1'b0;
            end
        end
    end

endmodule
