// Stream0 UART firmware receive path: reuses actual SRAM FIFO, full-message staging source, and two-level arbiter.
// Scope: local synchronous send integration; external credit decode and other message eligibility are provided externally; full protocol reset and credit-update rhythm are implemented elsewhere.
`timescale 1ps/1ps // All logic and both SRAM ports use the same clock domain.
module dl_uart_tx_path #( // UART actual storage send path module; total capacity includes unstaged words inside the source.
    parameter integer C_TX_DEPTH = 128 // Standard default capacity is 128; 1 to 4095 are local research configurations.
) ( // Firmware and other message interfaces are internal synchronous handshakes and do not add UALink wire-level fields.
    input wire i_clk, // Shared rising-edge clock for SRAM, FIFO, send sources, and arbiter.
    input wire i_rstn, // Synchronous active-low reset clearing local control ownership.
    input wire i_stream_reset, // Synchronously clears local stream data and credit and optionally cancels UART arbitration occupancy.
    input wire i_channel4_enabled, // Negotiated channel validity; valid closure must drain already-sent UART message headers.
    input wire i_uart_start_allowed, // Same-domain permission to start a new message; already-sent UART header messages still need full drain.
    input wire i_fw_valid, // Firmware provides a payload DWORD to be written; when blocked, data and validity remain held.
    input wire [31:0] i_fw_word, // Full firmware payload, excluding message headers produced by the send source.
    input wire i_credit_valid, // Upstream already completed Stream0 qualification for an absolute credit update event.
    input wire [11:0] i_credit_value, // Decoded modulo-4096 credit value.
    input wire i_segment_available, // Actual message DWORD service opportunity in this cycle.
    input wire [9:0] i_other_pending, // Other fully staged and send-eligible single-word message sources, ordered 0 to 6 and 8 to 10.
    input wire [319:0] i_other_words, // Low DWORD corresponds to lowest external source; remains queue-head until consumed.
    output wire o_fw_ready, // Hardware total occupancy is below capacity and FIFO can accept data.
    output wire [12:0] o_buffered_words, // Total FIFO occupancy plus actual unstaged payload; internal staging does not count as extra capacity.
    output wire [11:0] o_tx_counter, // Increases only when a complete UART message finishes.
    output wire o_uart_busy, // A UART message reservation has been captured or the message is still being sent.
    output wire [5:0] o_uart_reserved, // Current reserved full payload length, range 1 to 32.
    output wire [5:0] o_uart_staged, // Current actual staged and unsent payload word count.
    output wire [11:0] o_latest_fc, // Last valid absolute credit saved by the send source.
    output wire o_valid, // Actual message DWORD service event in the current cycle.
    output wire [31:0] o_word, // Actual message header or payload read from the real SRAM path.
    output wire [3:0] o_source, // Local 11-source index of the actual arbitration winner.
    output wire [5:0] o_word_index, // Current service DWORD index within the full message; header is index 0.
    output wire o_last, // Current service event ends the selected message.
    output wire [9:0] o_other_take, // Actual consumption events for the 10 external sources.
    output wire [9:0] o_other_done, // Completion events for the 10 external sources.
    output wire o_uart_message_done, // Shared event when the last payload is consumed and UART credit is committed.
    output wire o_uart_locked, // Arbiter has sent the UART header and retains remaining message ownership.
    output wire o_error, // Local capacity, source, completion, or index violation; not a line-level error code.
    output wire [5:0] o_uart_word_index // Arbiter expected next UART word index; zero when idle.
); // End of UART actual storage send module interface.
    localparam integer C_COUNT_WIDTH = (C_TX_DEPTH < 2) ? 1 : (C_TX_DEPTH < 4) ? 2 : (C_TX_DEPTH < 8) ? 3 : (C_TX_DEPTH < 16) ? 4 : (C_TX_DEPTH < 32) ? 5 : (C_TX_DEPTH < 64) ? 6 : (C_TX_DEPTH < 128) ? 7 : (C_TX_DEPTH < 256) ? 8 : (C_TX_DEPTH < 512) ? 9 : (C_TX_DEPTH < 1024) ? 10 : (C_TX_DEPTH < 2048) ? 11 : (C_TX_DEPTH < 4096) ? 12 : 13;
    localparam [12:0] C_CAPACITY = C_TX_DEPTH[12:0]; // Use sufficient width to preserve the total FIFO-plus-staging result.
    wire [C_COUNT_WIDTH-1:0] fifo_count; // Actual FIFO occupancy including SRAM, in-flight reads, and two output buffers.
    wire [11:0] fifo_fill; // Explicit zero-extended capacity observation requested by the source interface.
    wire [12:0] total_words; // Total valid data ownership before reset masking.
    wire stream_rstn; // Data-path reset independent of retained arbiter polling history.
    wire fifo_ready, fifo_valid, source_ready, source_pending; // Handshake qualifications between FIFO and full-message source.
    wire [31:0] fifo_word, source_word; // Actual SRAM output word and current message DWORD from the source.
    wire [5:0] source_count, source_index; // Total length and current pending index from the source.
    wire [10:0] source_take, message_done; // Actual 11-source arbitration consume and completion events.
    wire flag_uart_eligible; // New header gated by start permission; active message retains service eligibility.
    wire source_error, arbiter_error, flag_room; // Independent component diagnostics and full-path capacity admission.
    assign flag_uart_eligible = source_pending && (i_uart_start_allowed || o_uart_locked); // Only gate UART arbitration eligibility; does not alter prefetch, credit, or lockout message drain.
    assign stream_rstn = i_rstn && !i_stream_reset; // Same-domain stream reset clears FIFO and source; raw clock remains direct.
    assign fifo_fill = {{(12-C_COUNT_WIDTH){1'b0}}, fifo_count}; // Do not truncate FIFO total capacity and do not assume depth is a power of two.
    assign total_words = {1'b0, fifo_fill}+{7'd0, o_uart_staged}; // Internal staging on the send path does not free firmware-visible capacity.
    assign flag_room = total_words < C_CAPACITY; // Prevent borrowing space from a send that may occur in the same cycle.
    assign o_fw_ready = stream_rstn && fifo_ready && flag_room; // Admission requires real FIFO readiness and total unstaged-word capacity.
    assign o_buffered_words = stream_rstn ? total_words : 13'd0; // Hide occupancy prior to the reset edge; internal control resets synchronously.
    assign o_other_take = {source_take[10:8], source_take[6:0]}; // UART transport consumption is handled by internal source 7.
    assign o_other_done = {message_done[10:8], message_done[6:0]}; // Keep the same order as external 10-source input and completion output.
    assign o_error = stream_rstn && (source_error || arbiter_error || (total_words > C_CAPACITY) || (o_uart_message_done != message_done[7]) || (o_uart_locked && source_pending && (source_index != 6'd0)) ); // Local diagnostics only; no line-level error codes.
    generate // Illegal research configuration fails at elaboration.
        if ((C_TX_DEPTH < 1) || (C_TX_DEPTH > 4095)) begin : gen_invalid // 12-bit credit interface limits expressible local capacity range.
            dl_uart_tx_path_parameters_invalid Invalid_Inst (); // Explicit error level prevents illegal configuration from generating a netlist.
        end // End of capacity validity condition.
    endgenerate // End of parameter-range validation.
    upli_receive_storage #( // Reuse an actual synchronous FIFO and authorized SRAM storage mapping.
        .C_DEPTH(C_TX_DEPTH), .C_DATA_WIDTH(32), .C_COUNT_WIDTH(C_COUNT_WIDTH) // Physical minimum-depth mapping does not change firmware logic capacity.
    ) Storage_Inst ( // Storage contents are not cleared directly by local control reset.
        .i_clk(i_clk), .i_rstn(stream_rstn), // FIFO and send source share the stream reset, clearing old data validity.
        .i_write_valid(i_fw_valid && flag_room), .i_write_data(i_fw_word), .o_write_ready(fifo_ready), // Only firmware words that fit total capacity enter the FIFO.
        .i_read_ready(source_ready), .o_read_valid(fifo_valid), .o_read_data(fifo_word), .o_count(fifo_count) // Send source is the only consumer of the FIFO.
    ); // End of real SRAM FIFO connection.
    dl_uart_tx_source Source_Inst ( // Full reservation and staging ensures the arbiter header commit does not depend on later FIFO availability.
        .i_clk(i_clk), .i_rstn(stream_rstn), .i_channel4_enabled(i_channel4_enabled), // Same-domain local control and channel validity.
        .i_payload_fill(fifo_fill), .i_payload_valid(fifo_valid), .i_payload_word(fifo_word), // Data comes from actual registered SRAM read path.
        .i_credit_valid(i_credit_valid), .i_credit_value(i_credit_value), .i_source_take(source_take[7]), // Only respond to real source 7 service; credit update is decoded by upstream logic.
        .o_payload_ready(source_ready), .o_source_pending(source_pending), .o_word(source_word), // Wait until all reserved payload words arrive before participating in arbitration.
        .o_word_count(source_count), .o_word_index(source_index), .o_busy(o_uart_busy), // Source itself holds length and index and cross-checks with arbiter.
        .o_reserved_words(o_uart_reserved), .o_staged_words(o_uart_staged), // Expose reserved capacity and actual unsent data ownership separately.
        .o_tx_counter(o_tx_counter), .o_latest_fc(o_latest_fc), .o_message_done(o_uart_message_done), .o_error(source_error) // Full-message count and local diagnostics.
    ); // End of actual UART full-message source connection.
    dl_message_arbiter Arbiter_Inst ( // Actual three-group two-level round robin and UART full-message lock.
        .i_uart_cancel(i_stream_reset), .i_clk(i_clk), .i_rstn(i_rstn), .i_segment_available(i_segment_available), // Same-domain service opportunities do not represent PHY clock or flit timing.
        .i_source_pending({i_other_pending[9:7], flag_uart_eligible, i_other_pending[6:0]}), // Insert internal UART transport eligibility at fixed source 7.
        .i_source_words({i_other_words[319:224], source_word, i_other_words[223:0]}), .i_uart_word_count(source_count), // Actual source words and full length are connected in fixed 11-source layout.
        .o_valid(o_valid), .o_word(o_word), .o_source(o_source), .o_word_index(o_word_index), .o_last(o_last), // Output actual service and full-message boundary.
        .o_source_take(source_take), .o_message_done(message_done), .o_error(arbiter_error), // All consumption and completion come from the same actual arbiter.
        .o_locked(o_uart_locked), .o_uart_word_index(o_uart_word_index) // Message occupancy and send-source index may be cross-checked independently.
    ); // End of actual two-level arbiter connection.
endmodule // End of UART actual storage send path module.
