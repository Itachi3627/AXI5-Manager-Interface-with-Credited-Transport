// =============================================================================
// AXI5 Manager Interface with Credited Transport
// =============================================================================
// Implements AXI5 Manager (formerly Master) with:
//   - Standard VALID/READY handshake on all 5 channels
//   - Credited transport: Manager tracks credits per Resource Plane (RP)
//     and may only issue a transfer when credits > 0.
//   - PENDING signals: asserted one cycle before VALID to allow
//     Subordinate pipeline warm-up.
//   - CRDT  signals: received from Subordinate to replenish Manager credits.
//   - CRDTSH / SHAREDCRD / RP signals handled per channel.
//
// Parameters
//   NUM_RP_AWW  : Number of Resource Planes for AW/W channels (default 4)
//   NUM_RP_AR   : Number of Resource Planes for AR/R channels (default 4)
//   INIT_CRDT   : Initial credit count per RP per channel    (default 8)
//   DATA_WIDTH  : AXI data bus width                         (default 32)
//   ADDR_WIDTH  : AXI address bus width                      (default 32)
//   ID_WIDTH    : AXI ID width                               (default 4)
// =============================================================================

module axi5_manager #(
    parameter NUM_RP_AWW  = 4,
    parameter NUM_RP_AR   = 4,
    parameter INIT_CRDT   = 8,
    parameter DATA_WIDTH  = 32,
    parameter ADDR_WIDTH  = 32,
    parameter ID_WIDTH    = 4
)(
    input  wire                         ACLK,
    input  wire                         ARESETn,

    // ------------------------------------------------------------------ //
    //  AW Channel – Write Address
    // ------------------------------------------------------------------ //
    output reg  [ID_WIDTH-1:0]          AWID,
    output reg  [ADDR_WIDTH-1:0]        AWADDR,
    output reg  [7:0]                   AWLEN,
    output reg  [2:0]                   AWSIZE,
    output reg  [1:0]                   AWBURST,
    output reg                          AWVALID,
    input  wire                         AWREADY,
    // Credited transport – AW
    output reg                          AWPENDING,
    input  wire [NUM_RP_AWW-1:0]        AWCRDT,      // one credit bit per RP
    input  wire                         AWCRDTSH,    // shared credit return
    output reg  [$clog2(NUM_RP_AWW)-1:0] AWRP,       // RP selector for this xfer
    output reg                          AWSHAREDCRD, // this xfer uses shared credit

    // ------------------------------------------------------------------ //
    //  W Channel – Write Data
    // ------------------------------------------------------------------ //
    output reg  [DATA_WIDTH-1:0]        WDATA,
    output reg  [DATA_WIDTH/8-1:0]      WSTRB,
    output reg                          WLAST,
    output reg                          WVALID,
    input  wire                         WREADY,
    // Credited transport – W
    output reg                          WPENDING,
    input  wire [NUM_RP_AWW-1:0]        WCRDT,
    input  wire                         WCRDTSH,
    output reg  [$clog2(NUM_RP_AWW)-1:0] WRP,
    output reg                          WSHAREDCRD,

    // ------------------------------------------------------------------ //
    //  B Channel – Write Response (credits flow M→S for B)
    // ------------------------------------------------------------------ //
    input  wire [ID_WIDTH-1:0]          BID,
    input  wire [1:0]                   BRESP,
    input  wire                         BVALID,
    output reg                          BREADY,
    // Credited transport – B (Manager sends credits to Subordinate)
    output reg                          BPENDING,
    output reg                          BCRDT,       // Manager returns B credit

    // ------------------------------------------------------------------ //
    //  AR Channel – Read Address
    // ------------------------------------------------------------------ //
    output reg  [ID_WIDTH-1:0]          ARID,
    output reg  [ADDR_WIDTH-1:0]        ARADDR,
    output reg  [7:0]                   ARLEN,
    output reg  [2:0]                   ARSIZE,
    output reg  [1:0]                   ARBURST,
    output reg                          ARVALID,
    input  wire                         ARREADY,
    // Credited transport – AR
    output reg                          ARPENDING,
    input  wire [NUM_RP_AR-1:0]         ARCRDT,
    input  wire                         ARCRDTSH,
    output reg  [$clog2(NUM_RP_AR)-1:0] ARRP,
    output reg                          ARSHAREDCRD,

    // ------------------------------------------------------------------ //
    //  R Channel – Read Data (credits flow M→S for R)
    // ------------------------------------------------------------------ //
    input  wire [ID_WIDTH-1:0]          RID,
    input  wire [DATA_WIDTH-1:0]        RDATA,
    input  wire [1:0]                   RRESP,
    input  wire                         RLAST,
    input  wire                         RVALID,
    output reg                          RREADY,
    // Credited transport – R
    output reg                          RPENDING,
    output reg                          RCRDT        // Manager returns R credit
);

    // ================================================================== //
    //  Credit counters
    //  One counter per Resource Plane per channel.
    //  Width: enough to hold INIT_CRDT * 2 safely.
    // ================================================================== //
    localparam CRDT_W = $clog2(INIT_CRDT * 4 + 1);

    reg [CRDT_W-1:0] aw_crdt [0:NUM_RP_AWW-1];
    reg [CRDT_W-1:0] w_crdt  [0:NUM_RP_AWW-1];
    reg [CRDT_W-1:0] ar_crdt [0:NUM_RP_AR -1];
    reg [CRDT_W-1:0] aw_shared_crdt;
    reg [CRDT_W-1:0] w_shared_crdt;
    reg [CRDT_W-1:0] ar_shared_crdt;

    integer i;

    // ================================================================== //
    //  Credit initialisation & replenishment
    // ================================================================== //
    always @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            for (i = 0; i < NUM_RP_AWW; i = i + 1) begin
                aw_crdt[i] <= INIT_CRDT;
                w_crdt [i] <= INIT_CRDT;
            end
            for (i = 0; i < NUM_RP_AR; i = i + 1)
                ar_crdt[i] <= INIT_CRDT;
            aw_shared_crdt <= INIT_CRDT;
            w_shared_crdt  <= INIT_CRDT;
            ar_shared_crdt <= INIT_CRDT;
        end else begin
            // --- AW dedicated credits ---
            for (i = 0; i < NUM_RP_AWW; i = i + 1) begin
                // Subordinate returns credit on AWCRDT[i]
                // Manager consumes credit when it issues AWVALID on RP=i
                case ({AWCRDT[i], (AWVALID && AWREADY && (AWRP == i) && !AWSHAREDCRD)})
                    2'b10: aw_crdt[i] <= aw_crdt[i] + 1;   // replenish only
                    2'b01: aw_crdt[i] <= aw_crdt[i] - 1;   // consume only
                    default: ;                               // no change / both cancel
                endcase
            end
            // AW shared credit
            case ({AWCRDTSH, (AWVALID && AWREADY && AWSHAREDCRD)})
                2'b10: aw_shared_crdt <= aw_shared_crdt + 1;
                2'b01: aw_shared_crdt <= aw_shared_crdt - 1;
                default: ;
            endcase

            // --- W dedicated credits ---
            for (i = 0; i < NUM_RP_AWW; i = i + 1) begin
                case ({WCRDT[i], (WVALID && WREADY && (WRP == i) && !WSHAREDCRD)})
                    2'b10: w_crdt[i] <= w_crdt[i] + 1;
                    2'b01: w_crdt[i] <= w_crdt[i] - 1;
                    default: ;
                endcase
            end
            case ({WCRDTSH, (WVALID && WREADY && WSHAREDCRD)})
                2'b10: w_shared_crdt <= w_shared_crdt + 1;
                2'b01: w_shared_crdt <= w_shared_crdt - 1;
                default: ;
            endcase

            // --- AR dedicated credits ---
            for (i = 0; i < NUM_RP_AR; i = i + 1) begin
                case ({ARCRDT[i], (ARVALID && ARREADY && (ARRP == i) && !ARSHAREDCRD)})
                    2'b10: ar_crdt[i] <= ar_crdt[i] + 1;
                    2'b01: ar_crdt[i] <= ar_crdt[i] - 1;
                    default: ;
                endcase
            end
            case ({ARCRDTSH, (ARVALID && ARREADY && ARSHAREDCRD)})
                2'b10: ar_shared_crdt <= ar_shared_crdt + 1;
                2'b01: ar_shared_crdt <= ar_shared_crdt - 1;
                default: ;
            endcase
        end
    end

    // ================================================================== //
    //  Credit availability check (combinational helpers)
    // ================================================================== //
    wire aw_has_crdt = (AWSHAREDCRD) ? (aw_shared_crdt > 0)
                                     : (aw_crdt[AWRP] > 0);
    wire w_has_crdt  = (WSHAREDCRD)  ? (w_shared_crdt  > 0)
                                     : (w_crdt [WRP]  > 0);
    wire ar_has_crdt = (ARSHAREDCRD) ? (ar_shared_crdt > 0)
                                     : (ar_crdt[ARRP] > 0);

    // ================================================================== //
    //  PENDING: asserted one cycle before VALID (pipeline hint)
    //  Simple model: pending = intent to send next cycle if credit ok.
    //  In a real design this would be driven by the transaction queue.
    // ================================================================== //
    // Here we register a "want_to_send" flag per channel (user logic drives
    // this; for demonstration we tie it to a permanent request).
    // The pending output is: want_to_send AND credit available.

    reg aw_req_q, w_req_q, ar_req_q, b_req_q, r_req_q;

    always @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            aw_req_q <= 0; w_req_q <= 0; ar_req_q <= 0;
            b_req_q  <= 0; r_req_q  <= 0;
        end else begin
            // Latch when a new transaction enters the queue (simplified)
            if (!AWVALID || AWREADY) aw_req_q <= 1'b1; // always pending for demo
            if (!WVALID  || WREADY)  w_req_q  <= 1'b1;
            if (!ARVALID || ARREADY) ar_req_q <= 1'b1;
            if (BVALID)              b_req_q  <= 1'b1;
            if (RVALID)              r_req_q  <= 1'b1;
        end
    end

    always @(*) begin
        AWPENDING = aw_req_q && aw_has_crdt;
        WPENDING  = w_req_q  && w_has_crdt;
        ARPENDING = ar_req_q && ar_has_crdt;
        BPENDING  = b_req_q;   // B credits flow M→S; pending = ready to accept
        RPENDING  = r_req_q;
    end

    // ================================================================== //
    //  VALID generation: gated by credit availability
    // ================================================================== //
    always @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            AWVALID <= 0; WVALID <= 0; ARVALID <= 0;
        end else begin
            // AW: assert VALID only when credit is available
            if (!AWVALID && aw_has_crdt && aw_req_q)
                AWVALID <= 1'b1;
            else if (AWVALID && AWREADY)
                AWVALID <= 1'b0;

            // W: assert VALID only when credit is available
            if (!WVALID && w_has_crdt && w_req_q)
                WVALID <= 1'b1;
            else if (WVALID && WREADY)
                WVALID <= 1'b0;

            // AR: assert VALID only when credit is available
            if (!ARVALID && ar_has_crdt && ar_req_q)
                ARVALID <= 1'b1;
            else if (ARVALID && ARREADY)
                ARVALID <= 1'b0;
        end
    end

    // ================================================================== //
    //  B / R channel: Manager returns credits to Subordinate
    //  BCRDT / RCRDT pulsed for one cycle when Manager accepts a response
    // ================================================================== //
    always @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            BREADY <= 1'b1;
            RREADY <= 1'b1;
            BCRDT  <= 1'b0;
            RCRDT  <= 1'b0;
        end else begin
            // Accept B response and immediately return a credit to Subordinate
            BCRDT <= BVALID && BREADY;   // pulse credit same cycle as acceptance
            RCRDT <= RVALID && RREADY && RLAST;
            BREADY <= 1'b1;  // always ready (simplification)
            RREADY <= 1'b1;
        end
    end

    // ================================================================== //
    //  Stub address / data (replace with actual request queue in real use)
    // ================================================================== //
    always @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            AWID   <= 0; AWADDR <= 0; AWLEN <= 0; AWSIZE <= 3'b010;
            AWBURST<= 2'b01; AWRP <= 0; AWSHAREDCRD <= 0;
            WDATA  <= 0; WSTRB <= {(DATA_WIDTH/8){1'b1}}; WLAST <= 1;
            WRP    <= 0; WSHAREDCRD <= 0;
            ARID   <= 0; ARADDR <= 0; ARLEN <= 0; ARSIZE <= 3'b010;
            ARBURST<= 2'b01; ARRP <= 0; ARSHAREDCRD <= 0;
        end else begin
            // Increment address each transaction (simple incrementing pattern)
            if (AWVALID && AWREADY) AWADDR <= AWADDR + (1 << AWSIZE);
            if (ARVALID && ARREADY) ARADDR <= ARADDR + (1 << ARSIZE);
        end
    end

endmodule
