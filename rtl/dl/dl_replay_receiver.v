// LLR receive event state for UALink 200G DL/PL 2.0.
// LLR receive event state; based on sections 2.6.4 and 2.6.6.1 through 2.6.6.4.
// Single-clock synchronous reset; no payload storage, CDC, CRC computation, or transmit watchdog.
module dl_replay_receiver ( // Declares the independent receive-state module.
    input wire i_clk, // Only receive-state clock.
    input wire i_rstn, // Active-low synchronous reset; release must satisfy local timing.
    input wire i_link_reset, // Synchronous link clear, takes priority over all receive events.
    input wire i_event_valid, // An event classified as received exists in the current cycle.
    input wire i_event_discard, // The event has already been classified by the upstream block as discard, so header and CRC are not considered.
    input wire i_crc_ok, // External CRC decision for normal events.
    input wire [23:0] i_header, // Full 24-bit logical LLR header, not yet serial bit order on the wire.
    input wire [7:0] i_replay_limit, // Same-domain 8-bit replay threshold; default 50 provided by external CSR.
    output wire o_ingress_event, // Indicates a real event to other receive consumers in the current cycle.
    output wire o_accept, // Accepts matching payload or NOP.
    output wire o_payload_accept, // Allows payload to be submitted to the receive path in the current cycle.
    output wire o_sequence_valid, // Sequence diagnostic valid; does not imply order match.
    output wire [8:0] o_sequence, // Recovered 9-bit sequence; zero when invalid.
    output wire o_replay_request, // Requests the send side to set the remaining Replay Request count to three.
    output wire o_command_valid, // CRC- and encoding-valid ACK or Replay command, not dependent on payload acceptance.
    output wire o_command_request, // Valid command is Replay Request; otherwise ACK.
    output wire [8:0] o_command_target, // Command target, zero when invalid.
    output wire o_crc_error, // CRC error or reserved-op count event.
    output wire o_zero_sequence, // Log event for CRC-valid but explicit zero sequence.
    output wire o_zero_command, // Log event for CRC-valid but zero command target.
    output wire o_backpressure_drop, // Log event for already-classified storage discard.
    output wire o_unexpected, // Trusted recovered sequence does not match expectation.
    output wire o_ambiguous_drop, // Only because state is ambiguous and compressed sequence cannot be trusted.
    output wire o_replay_drop, // Reject compressed sequence while in replay mode.
    output wire [8:0] o_last_sequence, // Current last accepted sequence.
    output wire [2:0] o_bad_crc_count, // Current 3-bit saturated error count.
    output wire [7:0] o_unexpected_count, // Current 8-bit saturated replay anomaly count.
    output wire o_ambiguous, // Current compressed-sequence ambiguity state.
    output wire o_replay // Current receive replay state.
); // End of interface declaration.
    reg [8:0] reg_last_sequence; // Last fully accepted sequence; zero cannot be produced by a normal reachable state.
    reg [2:0] cnt_bad_crc; // 3-bit saturated error count.
    reg [7:0] cnt_unexpected; // 8-bit saturated replay anomaly count.
    reg reg_ambiguous; // Whether the compressed sequence is untrustworthy.
    reg reg_replay; // Whether the receive side is in replay state.
    wire flag_event; // Real receive event after reset qualification.
    wire flag_explicit; // Explicit format selection; does not alone imply Op is valid.
    wire flag_command; // Command-format selection.
    wire flag_op_valid; // Op and payload combination is valid.
    wire flag_storage; // Already-classified storage discard.
    wire flag_invalid; // Normal event CRC or op error.
    wire flag_zero_sequence; // Normal CRC-valid explicit zero sequence.
    wire flag_zero_command; // Normal CRC-valid command zero target.
    wire flag_valid; // Normal CRC and fields are valid.
    wire flag_sequence_wrap; // Low 3 bits cross boundary or payload bit indicates high 6-bit carry.
    wire [5:0] dec_sequence_high_next; // High bits precomputed to increment and wrap at 6-bit boundary.
    wire [8:0] dec_sequence; // 9-bit addition truncation implements modulo-512 recovery.
    wire [8:0] dec_expected; // Next payload or current NOP expected sequence in valid ring.
    wire flag_trusted; // Current full or trusted compressed sequence.
    wire flag_accept; // Current-cycle acceptance qualification.
    wire flag_unexpected; // Current-cycle trusted but not expected sequence.
    wire flag_untrusted; // Current-cycle valid but compressed sequence untrusted.
    wire flag_start_replay; // Begins replay request from non-replay state.
    wire flag_count_unexpected; // Real anomaly event while in replay state.
    wire [2:0] cnt_bad_next; // Saturating increment for error count.
    wire [7:0] cnt_unexpected_next; // Saturating increment for replay anomaly count.
    wire flag_retry; // Triggered when the increment reaches the configured threshold and sets three copies again.
    assign flag_event = i_rstn && !i_link_reset && i_event_valid; // All commit pulses are zero when reset or no event.
    assign flag_explicit = (i_header[23:21] == 3'd0) || (i_header[23:21] == 3'd1); // Explicit format.
    assign flag_command = (i_header[23:21] == 3'd2) || (i_header[23:21] == 3'd3); // ACK or Replay Request.
    assign flag_op_valid = (i_header[23:21] == 3'd0) || ((i_header[23:21] == 3'd1) && i_header[20]) || flag_command; // Replay op cannot be used with NOP.
    assign flag_storage = flag_event && i_event_discard; // Upstream already completed classification; not decided by this module's physical accept priority.
    assign flag_invalid = flag_event && !i_event_discard && (!i_crc_ok || !flag_op_valid); // Only normal events count CRC-like errors.
    assign flag_zero_sequence = flag_event && !i_event_discard && i_crc_ok && flag_op_valid && flag_explicit && (i_header[16:8] == 9'd0); // CRC-valid zero-sequence log only.
    assign flag_zero_command = flag_event && !i_event_discard && i_crc_ok && flag_command && (i_header[19:11] == 9'd0); // CRC-valid zero-command-target log only.
    assign flag_valid = flag_event && !i_event_discard && i_crc_ok && flag_op_valid && (flag_explicit ? (i_header[16:8] != 9'd0) : (i_header[19:11] != 9'd0)); // Reserved bits are ignored per spec.
    assign flag_sequence_wrap = (i_header[10:8] < reg_last_sequence[2:0]) || (i_header[20] && (i_header[10:8] == reg_last_sequence[2:0])); // Encoded recovery carry does not depend on full 9-bit addition.
    assign dec_sequence_high_next = reg_last_sequence[8:3] + 6'd1; // High bits increment via current register state.
    assign dec_sequence = flag_explicit ? i_header[16:8] : {(flag_sequence_wrap ? dec_sequence_high_next : reg_last_sequence[8:3]), i_header[10:8]}; // Low bits come directly from received field; high bits are reconstructed.
    assign dec_expected = i_header[20] ? ((reg_last_sequence == 9'd511) ? 9'd1 : reg_last_sequence + 9'd1) : reg_last_sequence; // Payload skips zero; NOP does not consume sequence.
    assign flag_trusted = flag_explicit || (!reg_ambiguous && !reg_replay); // The compressed sequence is trusted only when both states are cleared.
    assign flag_accept = flag_valid && flag_trusted && (flag_explicit ? (i_header[16:8] == dec_expected) : (i_header[10:8] == dec_expected[2:0])); // Full explicit comparison; compressed comparison uses expected low bits.
    assign flag_unexpected = flag_valid && flag_trusted && !flag_accept; // Does not change the last accepted sequence.
    assign flag_untrusted = flag_valid && !flag_trusted; // Still allows a separate command consumer to process the command.
    assign flag_start_replay = flag_unexpected && !reg_replay; // First anomaly starts replay and requests three copies.
    assign flag_count_unexpected = reg_replay && (flag_storage || flag_invalid || (flag_valid && !flag_accept)); // Zero fields and no-event logs do not increment anomaly count.
    assign cnt_bad_next = (cnt_bad_crc == 3'd7) ? 3'd7 : cnt_bad_crc + 3'd1; // 3-bit count saturates without wraparound.
    assign cnt_unexpected_next = (cnt_unexpected == 8'd255) ? 8'd255 : cnt_unexpected + 8'd1; // 8-bit count saturates without wraparound.
    assign flag_retry = flag_count_unexpected && (cnt_unexpected_next >= i_replay_limit); // Threshold zero still requires a real anomaly event to trigger.
    always @(posedge i_clk) begin // Last accepted sequence register.
        if (!i_rstn) reg_last_sequence <= 9'd511; // Synchronous initialization to valid ring end.
        else if (i_link_reset) reg_last_sequence <= 9'd511; // Synchronous link clear uses same initial value.
        else if (flag_accept) reg_last_sequence <= dec_sequence; // Only accepted messages update; anomalies remain.
    end // End of sequence register.
    always @(posedge i_clk) begin // Bad-CRC counter register.
        if (!i_rstn) cnt_bad_crc <= 3'd0; // Synchronous reset.
        else if (i_link_reset) cnt_bad_crc <= 3'd0; // Link clear uses same initial value.
        else if (flag_accept) cnt_bad_crc <= 3'd0; // Matched payload or NOP resets the count.
        else if (flag_storage || flag_invalid) cnt_bad_crc <= cnt_bad_next; // Storage discard or invalid event increments count.
    end // End of error counter.
    always @(posedge i_clk) begin // Replay anomaly counter register.
        if (!i_rstn) cnt_unexpected <= 8'd0; // Synchronous reset.
        else if (i_link_reset) cnt_unexpected <= 8'd0; // Link clear uses same initial value.
        else if (flag_accept || flag_start_replay || flag_retry) cnt_unexpected <= 8'd0; // Clear on recovery or on request-trigger.
        else if (flag_count_unexpected) cnt_unexpected <= cnt_unexpected_next; // Increment only for real anomaly events.
    end // End of replay anomaly counter.
    always @(posedge i_clk) begin // Ambiguity-state register.
        if (!i_rstn) reg_ambiguous <= 1'b0; // Synchronous reset.
        else if (i_link_reset) reg_ambiguous <= 1'b0; // Link clear uses same initial value.
        else if (flag_accept) reg_ambiguous <= 1'b0; // Matching trusted sequence clears ambiguity.
        else if ((flag_storage || flag_invalid) && (cnt_bad_next == 3'd7)) reg_ambiguous <= 1'b1; // Set after the seventh error.
    end // End of ambiguity state.
    always @(posedge i_clk) begin // Receive replay-state register.
        if (!i_rstn) reg_replay <= 1'b0; // Synchronous reset.
        else if (i_link_reset) reg_replay <= 1'b0; // Link clear uses same initial value.
        else if (flag_accept) reg_replay <= 1'b0; // Matching payload or NOP exits replay.
        else if (flag_start_replay) reg_replay <= 1'b1; // First trusted anomaly enters replay.
    end // End of replay state.
    assign o_ingress_event = flag_event; // Provide all ingress events to the independent ACK handler.
    assign o_accept = flag_accept; // Matching NOP or payload.
    assign o_payload_accept = flag_accept && i_header[20]; // Payload write qualification.
    assign o_sequence_valid = flag_valid; // Sequence diagnostic is not the same as acceptance.
    assign o_sequence = flag_valid ? dec_sequence : 9'd0; // Clear diagnostic bus for invalid events.
    assign o_replay_request = flag_start_replay || flag_retry; // Request the send side to set the count to three.
    assign o_command_valid = flag_valid && flag_command; // Commands do not require payload acceptance.
    assign o_command_request = flag_valid && (i_header[23:21] == 3'd3); // Valid Replay Request command category.
    assign o_command_target = (flag_valid && flag_command) ? i_header[19:11] : 9'd0; // Valid command target.
    assign o_crc_error = flag_invalid; // CRC and reserved-op log events.
    assign o_zero_sequence = flag_zero_sequence; // Zero-sequence log event.
    assign o_zero_command = flag_zero_command; // Zero-command-target log event.
    assign o_backpressure_drop = flag_storage; // Upstream classified storage-discard event.
    assign o_unexpected = flag_unexpected; // Trusted recovered sequence unexpected.
    assign o_ambiguous_drop = flag_untrusted && !reg_replay; // Ambiguous only; cannot initiate replay by itself.
    assign o_replay_drop = flag_untrusted && reg_replay; // Do not trust compressed sequence while replaying.
    assign o_last_sequence = reg_last_sequence; // Direct observation of current accepted sequence.
    assign o_bad_crc_count = cnt_bad_crc; // Direct observation of current saturated error count.
    assign o_unexpected_count = cnt_unexpected; // Direct observation of current replay anomaly count.
    assign o_ambiguous = reg_ambiguous; // Direct observation of current ambiguity state.
    assign o_replay = reg_replay; // Direct observation of current replay state.
endmodule // End of LLR receive-event finite-state module.
