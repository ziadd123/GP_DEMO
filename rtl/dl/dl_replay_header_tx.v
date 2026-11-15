module dl_replay_header_tx #( // dl_replay_header_tx module: LLR send-header scheduling with optional early metadata protection.
    parameter integer C_EARLY_TX_METADATA = 0 // When nonzero, early send-source metadata qualification is explicitly proven by the integration layer.
) ( // dl_replay_header_tx module: LLR send-header scheduling, one-cycle pre-output and edge-based commit.
    input wire i_clk, // Local unique clock.
    input wire i_rstn, // Active-low synchronous reset.
    input wire i_link_reset, // Synchronous link state clear.
    input wire i_flit_send, // Real commit of the selected Flit in the current cycle.
    input wire i_payload, // Selected source contains payload.
    input wire i_replay, // Selected source comes from replay queue and contains the last item.
    input wire i_first_replay, // Selected source is the first replay in this cycle.
    input wire [8:0] i_tx_sequence, // Full non-zero sequence number of the selected source.
    input wire i_tx_metadata_ok, // Only used in early mode; default mode still enforces full original metadata protection.
    input wire [8:0] i_rx_sequence, // Receive non-zero sequence number including this-cycle admission result.
    input wire i_rx_request, // Current receive event reloads three request copies.
    input wire i_new_group, // Current cycle enters a new actual FEC group.
    output wire [23:0] o_header, // Full header output when valid, reserved bits set to zero.
    output wire o_header_valid, // Current cycle generated and committed valid header.
    output wire o_metadata_error, // Metadata invalid in the send cycle.
    output wire [2:0] o_explicit_count, // Current explicit send interval state.
    output wire [1:0] o_request_count, // Current remaining request-copy state.
    output wire [8:0] o_request_sequence, // Target captured when the first send actually occurs.
    output wire o_group_used // Current group has already sent a request.
); // End of port declaration.
reg [2:0] reg_explicit_count; // Explicit interval count.
reg [1:0] reg_request_count; // Request copy count.
reg [8:0] reg_request_sequence; // Request target snapshot.
reg reg_group_used; // Current group request occupancy.
wire flag_active; // Working qualification outside reset and clear.
wire flag_bad_metadata; // Internal metadata protection.
wire flag_send; // Legal actual send.
wire flag_explicit; // Current header uses full sequence number.
wire flag_request; // Current header uses request encoding.
wire [1:0] dec_pending; // Copy count after current-cycle receive event.
wire [8:0] dec_successor; // Ring successor of the valid receive sequence.
wire [8:0] dec_target; // Request target for this cycle.
wire [23:0] dec_explicit_header; // Full sequence header.
wire [23:0] dec_command_header; // Compressed sequence with command header.
assign flag_active = i_rstn && !i_link_reset; // All commit pulses are silent during reset.
assign flag_bad_metadata = (i_rx_sequence == 9'd0) || ((C_EARLY_TX_METADATA != 0) ? !i_tx_metadata_ok : ((i_tx_sequence == 9'd0) || (i_replay && !i_payload) || (i_first_replay && !i_replay))); // Metadata is checked together with payload and replay state.
assign flag_send = flag_active && i_flit_send && !flag_bad_metadata; // Only legal metadata is committed.
assign dec_pending = i_rx_request ? 2'd3 : reg_request_count; // Process current receive request before other actions.
assign flag_explicit = i_first_replay || (reg_explicit_count <= 3'd1); // Decrement first, then test zero; first replay has priority.
assign flag_request = !flag_explicit && (dec_pending != 2'd0) && (!reg_group_used || i_new_group); // No more than one request per actual group.
assign dec_successor = (i_rx_sequence == 9'd511) ? 9'd1 : i_rx_sequence + 9'd1; // Valid sequence wraps around zero.
assign dec_target = (dec_pending == 2'd3) ? dec_successor : reg_request_sequence; // Capture target on first actual send; subsequent sends keep the same target.
assign dec_explicit_header = {2'b00, i_replay, i_payload, 3'b000, i_tx_sequence, 8'b00000000}; // op is Original or Replay.
assign dec_command_header = {2'b01, flag_request, i_payload, (flag_request ? dec_target : i_rx_sequence), i_tx_sequence[2:0], 8'b00000000}; // Ack or Replay Request fields.
assign o_header = flag_send ? (flag_explicit ? dec_explicit_header : dec_command_header) : 24'd0; // Empty output when no commit.
assign o_header_valid = flag_send; // Valid means the header was generated in this cycle.
assign o_metadata_error = flag_active && i_flit_send && flag_bad_metadata; // No diagnosis when idle metadata is not present.
assign o_explicit_count = reg_explicit_count; // Current state observation.
assign o_request_count = reg_request_count; // Current state observation.
assign o_request_sequence = reg_request_sequence; // Current state observation.
assign o_group_used = reg_group_used; // Current state observation.
always @(posedge i_clk) begin // Explicit interval advances only on valid sends.
    if (!i_rstn) begin // Global synchronous reset.
        reg_explicit_count <= 3'd7; // Initial interval is seven flits.
    end else if (i_link_reset) begin // Link reinitialization.
        reg_explicit_count <= 3'd7; // Reinitialize interval.
    end else if (flag_send) begin // Valid commit.
        if (flag_explicit) begin // Explicit send reload.
            reg_explicit_count <= 3'd7; // Keep seven-flit interval.
        end else begin // Non-explicit send advances.
            reg_explicit_count <= reg_explicit_count - 3'd1; // Zero/one are excluded, so it will not underflow.
        end // End of explicit branch.
    end // Other cycles hold state.
end // End of explicit count register.
always @(posedge i_clk) begin // Receive requests and current send copies are handled in sequence.
    if (!i_rstn) begin // Global synchronous reset.
        reg_request_count <= 2'd0; // Initially no requests.
    end else if (i_link_reset) begin // Link reset.
        reg_request_count <= 2'd0; // Clear queued requests.
    end else if (flag_send && flag_request) begin // A request is sent in this cycle.
        reg_request_count <= dec_pending - 2'd1; // Consume the copy count after a receive reload.
    end else if (i_rx_request) begin // Receive request not yet sent.
        reg_request_count <= 2'd3; // Assign three copies instead of accumulating.
    end // Other cycles hold state.
end // End of request copy register.
always @(posedge i_clk) begin // Freeze the request target when the first send occurs.
    if (!i_rstn) begin // Global synchronous reset.
        reg_request_sequence <= 9'd0; // No captured request.
    end else if (i_link_reset) begin // Link reset.
        reg_request_sequence <= 9'd0; // Clear old target.
    end else if (flag_send && flag_request && (dec_pending == 2'd3)) begin // First send in this cycle.
        reg_request_sequence <= dec_successor; // Capture the valid receive sequence successor.
    end // Other cycles hold target.
end // End of request target register.
always @(posedge i_clk) begin // Actual request occupancy of the FEC group.
    if (!i_rstn) begin // Global synchronous reset.
        reg_group_used <= 1'b0; // Initially the group may send a request.
    end else if (i_link_reset) begin // Link reset.
        reg_group_used <= 1'b0; // Clear group usage.
    end else if (flag_send && flag_request) begin // A request must also be recorded when sent in a new group.
        reg_group_used <= 1'b1; // This group already sent a request.
    end else if (i_new_group) begin // Entering a new group without a request being sent.
        reg_group_used <= 1'b0; // Release old group occupancy even when idle.
    end // Other cycles hold state.
end // End of group occupancy register.
endmodule // End of dl_replay_header_tx module.
