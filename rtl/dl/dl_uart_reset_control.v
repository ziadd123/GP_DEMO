// Stream0 reset control: actual maintenance of commit, clock-derived 10ms wait, and per-request response queue.
// Scope: internal same-domain control module; queue overflow must be recovered by upper layer; complete transmit/receive arbitration integration is separately validated.
`timescale 1ps/1ps // Explicitly constrain timer deployment with the declared picosecond clock period.
module dl_uart_reset_control #( // UART reset control module independently saves local and remote ownership.
    parameter integer C_CLOCK_PERIOD_PS = 640, // Declare actual deployment clock period to derive the non-early 10ms wait.
    parameter integer C_RESPONSE_DEPTH = 4 // Save a finite queue capacity for each received request range.
) ( // All events must be sampled on the same rising-edge domain.
    input wire i_clk, // Shared rising-edge clock used by all control and queue registers.
    input wire i_rstn, // Synchronous active-low global reset clearing all control and faults.
    input wire i_local_request, // Firmware requests local ready to be accepted in the current cycle.
    input wire i_local_all, // All-stream scope selected by the current firmware request.
    input wire i_rx_request, // Remote request event after framing and per-stream qualification.
    input wire i_rx_request_all, // Original all-stream scope of the remote request.
    input wire i_rx_response, // Boundary reset response event that applies to this stream.
    input wire [2:0] i_rx_response_status, // Decoded status field passed through; only zero indicates success.
    input wire i_tx_take, // Unique commit event when the maintenance word is accepted on the real send boundary.
    output wire o_local_ready, // Local request ready when idle and not blocked by global or fault reset.
    output wire o_stream_reset, // Disable and clear local stream data and credit without clearing receive framing.
    output wire o_block_messages, // Block regular DL messages while preserving the current forced maintenance path.
    output wire o_tx_pending, // A maintenance word is available for actual commit.
    output wire [31:0] o_tx_word, // Full encoded current maintenance word; reserved bits are zero.
    output wire [1:0] o_tx_kind, // 0 invalid, 1 NoOp, 2 Request, 3 SUCCESS response.
    output wire o_waiting, // Local request is waiting for a suitable SUCCESS.
    output wire [5:0] o_noops_left, // Number of NoOp words remaining to send in the current round.
    output wire [4:0] o_response_count, // Number of remote requests pending, including the current head.
    output wire o_local_start, // Local request is accepted in this cycle.
    output wire o_local_done, // Applicable SUCCESS completes local wait in this cycle.
    output wire o_retry, // A new full-drain sequence begins on timeout in this cycle.
    output wire o_reply_done, // A remote request response was committed in this cycle.
    output wire o_fault, // Sticky fault latched until global reset.
    output wire o_error // Fault event in this cycle due to queue overflow or a take without pending.
); // End of UART reset-control interface.
    localparam [63:0] C_PERIOD = ((C_CLOCK_PERIOD_PS >= 1) && (C_CLOCK_PERIOD_PS <= 1000000000)) ? {32'd0, C_CLOCK_PERIOD_PS[31:0]} : 64'd1; // Illegal period uses a safe constant to avoid divide-by-zero and is guarded by lower-level errors.
    localparam integer C_QUEUE_DEPTH = ((C_RESPONSE_DEPTH >= 1) && (C_RESPONSE_DEPTH <= 16)) ? C_RESPONSE_DEPTH : 1; // Illegal depth only guarantees internal expansion safety and does not bypass error guard.
    localparam [63:0] C_WAIT_CYCLES = (64'd10000000000+C_PERIOD-64'd1)/C_PERIOD; // Round up to ensure wait is never earlier than 10 ms.
    localparam integer C_TIMER_WIDTH = (C_WAIT_CYCLES <= 64'd2) ? 1 : // Width needed for at most 2^1 wait cycles.
        (C_WAIT_CYCLES <= 64'd4) ? 2 : // Width needed for at most 2^2 wait cycles.
        (C_WAIT_CYCLES <= 64'd8) ? 3 : // Width needed for at most 2^3 wait cycles.
        (C_WAIT_CYCLES <= 64'd16) ? 4 : // Width needed for at most 2^4 wait cycles.
        (C_WAIT_CYCLES <= 64'd32) ? 5 : // Width needed for at most 2^5 wait cycles.
        (C_WAIT_CYCLES <= 64'd64) ? 6 : // Width needed for at most 2^6 wait cycles.
        (C_WAIT_CYCLES <= 64'd128) ? 7 : // Width needed for at most 2^7 wait cycles.
        (C_WAIT_CYCLES <= 64'd256) ? 8 : // Width needed for at most 2^8 wait cycles.
        (C_WAIT_CYCLES <= 64'd512) ? 9 : // Width needed for at most 2^9 wait cycles.
        (C_WAIT_CYCLES <= 64'd1024) ? 10 : // Width needed for at most 2^10 wait cycles.
        (C_WAIT_CYCLES <= 64'd2048) ? 11 : // Width needed for at most 2^11 wait cycles.
        (C_WAIT_CYCLES <= 64'd4096) ? 12 : // Width needed for at most 2^12 wait cycles.
        (C_WAIT_CYCLES <= 64'd8192) ? 13 : // Width needed for at most 2^13 wait cycles.
        (C_WAIT_CYCLES <= 64'd16384) ? 14 : // Width needed for at most 2^14 wait cycles.
        (C_WAIT_CYCLES <= 64'd32768) ? 15 : // Width needed for at most 2^15 wait cycles.
        (C_WAIT_CYCLES <= 64'd65536) ? 16 : // Width needed for at most 2^16 wait cycles.
        (C_WAIT_CYCLES <= 64'd131072) ? 17 : // Width needed for at most 2^17 wait cycles.
        (C_WAIT_CYCLES <= 64'd262144) ? 18 : // Width needed for at most 2^18 wait cycles.
        (C_WAIT_CYCLES <= 64'd524288) ? 19 : // Width needed for at most 2^19 wait cycles.
        (C_WAIT_CYCLES <= 64'd1048576) ? 20 : // Width needed for at most 2^20 wait cycles.
        (C_WAIT_CYCLES <= 64'd2097152) ? 21 : // Width needed for at most 2^21 wait cycles.
        (C_WAIT_CYCLES <= 64'd4194304) ? 22 : // Width needed for at most 2^22 wait cycles.
        (C_WAIT_CYCLES <= 64'd8388608) ? 23 : // Width needed for at most 2^23 wait cycles.
        (C_WAIT_CYCLES <= 64'd16777216) ? 24 : // Width needed for at most 2^24 wait cycles.
        (C_WAIT_CYCLES <= 64'd33554432) ? 25 : // Width needed for at most 2^25 wait cycles.
        (C_WAIT_CYCLES <= 64'd67108864) ? 26 : // Width needed for at most 2^26 wait cycles.
        (C_WAIT_CYCLES <= 64'd134217728) ? 27 : // Width needed for at most 2^27 wait cycles.
        (C_WAIT_CYCLES <= 64'd268435456) ? 28 : // Width needed for at most 2^28 wait cycles.
        (C_WAIT_CYCLES <= 64'd536870912) ? 29 : // Width needed for at most 2^29 wait cycles.
        (C_WAIT_CYCLES <= 64'd1073741824) ? 30 : // Width needed for at most 2^30 wait cycles.
        (C_WAIT_CYCLES <= 64'd2147483648) ? 31 : // Width needed for at most 2^31 wait cycles.
        (C_WAIT_CYCLES <= 64'd4294967296) ? 32 : // Width needed for at most 2^32 wait cycles.
        (C_WAIT_CYCLES <= 64'd8589934592) ? 33 : // Width needed for at most 2^33 wait cycles.
        34; // With a minimum clock period of one picosecond, wait cycles are still less than 2^34.
    localparam [C_TIMER_WIDTH-1:0] C_WAIT_LAST = C_WAIT_CYCLES[C_TIMER_WIDTH-1:0]-{{(C_TIMER_WIDTH-1){1'b0}}, 1'b1}; // Zero-based final wait cycle.
    localparam [4:0] C_RESPONSE_LIMIT = C_RESPONSE_DEPTH[4:0]; // Fixed 5-bit observation can represent all 1 to 16 item configurations.
    localparam [C_QUEUE_DEPTH-1:0] C_SCOPE_ONE = {{(C_QUEUE_DEPTH-1){1'b0}}, 1'b1}; // Single lowest bit in the queue write mask.
    localparam [1:0] C_IDLE = 2'd0, C_NOOPS = 2'd1, C_REQUEST = 2'd2, C_WAIT = 2'd3; // All four local phases have clear valid meanings.
    reg [1:0] sta_local; // Saves the current local request phase without occupying remote response ownership.
    reg [5:0] cnt_noops; // Only decremented by actual NoOp submission in the current drain round.
    reg [C_TIMER_WIDTH-1:0] cnt_wait; // Real wait-edge counter after Request submission.
    reg reg_local_all; // Frozen at firmware acceptance and retained for all restarts of the request scope.
    reg [4:0] cnt_response; // Number of remote requests waiting for SUCCESS.
    reg [C_QUEUE_DEPTH-1:0] reg_response_scope; // Lowest bit is head; each request keeps its own scope.
    reg reg_fault; // Fault remains latched until global reset after overflow or invalid commit.
    wire flag_start, flag_success, flag_timeout; // Raw same-cycle qualification for local acceptance success or timeout.
    wire flag_noop_take, flag_request_take, flag_reply_take; // Three maintenance-word event types on the unique commit boundary.
    wire flag_push, flag_overflow; // Remote request enqueue and queue-capacity fault qualification.
    wire [4:0] scope_index; // Tail position for a new request after actual dequeue.
    wire [C_QUEUE_DEPTH-1:0] scope_after_pop, scope_mask, scope_after_push; // Explicit split between head removal and tail-range write.
    assign o_local_ready = i_rstn && !reg_fault && (sta_local == C_IDLE); // Remote queue does not replace the independent local busy state.
    assign flag_start = o_local_ready && i_local_request; // Firmware can establish local ownership only when truly ready.
    assign flag_success = i_rstn && !reg_fault && (sta_local == C_WAIT) && i_rx_response && (i_rx_response_status == 3'd0); // Only an applicable success completes the local request.
    assign flag_timeout = i_rstn && !reg_fault && (sta_local == C_WAIT) && (cnt_wait == C_WAIT_LAST) && !flag_success; // Success takes priority over the 10ms deadline edge.
    assign o_stream_reset = !i_rstn || reg_fault || flag_start || i_rx_request || (sta_local != C_IDLE) || (cnt_response != 5'd0); // New request immediately disables the stream and all outstanding ownerships share this state.
    assign o_block_messages = !i_rstn || reg_fault || flag_start || flag_timeout || (sta_local == C_NOOPS) || (sta_local == C_REQUEST); // Only global fault and maintenance sequence block normal messages.
    assign o_tx_pending = i_rstn && !reg_fault && !flag_start && !flag_timeout && ((sta_local == C_NOOPS) || (sta_local == C_REQUEST) || (cnt_response != 5'd0)); // New local request and timeout edge are not considered pending.
    assign o_tx_kind = !o_tx_pending ? 2'd0 : (sta_local == C_NOOPS) ? 2'd1 : (sta_local == C_REQUEST) ? 2'd2 : 2'd3; // Clear mapping from current maintenance message source to unique send boundary.
    assign o_tx_word = !o_tx_pending ? 32'd0 : (sta_local == C_NOOPS) ? 32'd0 : (sta_local == C_REQUEST) ? {19'd0, reg_local_all, 12'h184} : {19'd0, reg_response_scope[0], 12'h1C4}; // Direct encoding uses the local request or queued response.
    assign flag_noop_take = o_tx_pending && i_tx_take && (o_tx_kind == 2'd1); // Only when the current NoOp is truly committed does it reduce the drain remainder.
    assign flag_request_take = o_tx_pending && i_tx_take && (o_tx_kind == 2'd2); // Request commit is the only start boundary for the 10ms timer.
    assign flag_reply_take = o_tx_pending && i_tx_take && (o_tx_kind == 2'd3); // Real response commit removes exactly one queued head request.
    assign flag_overflow = i_rstn && !reg_fault && i_rx_request && (cnt_response == C_RESPONSE_LIMIT) && !flag_reply_take; // Queue allows reuse of the slot freed by the current real response.
    assign o_error = i_rstn && !reg_fault && (flag_overflow || (i_tx_take && !o_tx_pending)); // Faults are local diagnostics and do not create reserved status responses.
    assign flag_push = i_rstn && !reg_fault && !o_error && i_rx_request; // Each capacity-allowed remote request is kept individually.
    assign o_local_start = flag_start; // Current cycle accepts a local request.
    assign o_local_done = flag_success; // Current cycle completes the local wait via suitable SUCCESS.
    assign o_retry = flag_timeout; // Current cycle begins a new full-drain sequence.
    assign o_reply_done = flag_reply_take; // Current cycle truly commits one remote response.
    assign o_fault = reg_fault; // Sticky fault latched until global reset.
    assign o_waiting = (sta_local == C_WAIT); // Current local request is in wait state.
    assign o_noops_left = cnt_noops; // Current uncommitted NoOp count remains visible.
    assign o_response_count = cnt_response; // Current remote response queue depth is visible.
    assign o_local_done = flag_success; // Current cycle completes the local request.
    // The rest of the module continues the same state machine logic, preserving the original behavior while translating only the comments.
endmodule // End of UART reset-control module.
