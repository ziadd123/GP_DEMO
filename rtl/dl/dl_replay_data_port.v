module dl_replay_data_port #( // dl_replay_data_port module: reserves a flit slot and returns a one-cycle header response.
  parameter integer C_DEPTH = 255, // Exact logic replay capacity.
  parameter integer C_DATA_WIDTH = 32, // Opaque payload width, must be a positive integer number of bytes.
  parameter integer C_ADDR_WIDTH = (C_DEPTH <= 2) ? 1 : (C_DEPTH <= 4) ? 2 : (C_DEPTH <= 8) ? 3 : (C_DEPTH <= 16) ? 4 : (C_DEPTH <= 32) ? 5 : (C_DEPTH <= 64) ? 6 : (C_DEPTH <= 128) ? 7 : (C_DEPTH <= 256) ? 8 : (C_DEPTH <= 512) ? 9 : (C_DEPTH <= 1024) ? 10 : (C_DEPTH <= 2048) ? 11 : (C_DEPTH <= 4096) ? 12 : 13,
) ( // Reserve a slot that has already received a confirmed downstream latency response.
  input wire i_clk, // Same-domain transaction, complete event, or state observation.
  input wire i_rstn, // Same-domain transaction, complete event, or state observation.
  input wire i_link_reset, // Same-domain transaction, complete event, or state observation.
  input wire i_rx_event_valid, // Same-domain transaction, complete event, or state observation.
  input wire i_rx_event_discard, // Same-domain transaction, complete event, or state observation.
  input wire i_rx_crc_ok, // Same-domain transaction, complete event, or state observation.
  input wire [23:0] i_rx_header, // Same-domain transaction, complete event, or state observation.
  input wire [7:0] i_rx_replay_limit, // Same-domain transaction, complete event, or state observation.
  input wire [C_DATA_WIDTH-1:0] i_rx_data, // Unspecified payload of the received event in the current cycle.
  input wire i_flit_request, // Reserve a Flit slot in this cycle; the full response returns on the next clock edge.
  input wire i_new_group, // Only valid for the start of an FEC group at a valid reservation.
  input wire i_payload, // Same-domain transaction, complete event, or state observation.
  input wire [C_DATA_WIDTH-1:0] i_data, // Source keeps the full word until payload_accept.
  output wire o_issue_ready, // Same-domain transaction, complete event, or state observation.
  output wire o_issue_accept, // Same-domain transaction, complete event, or state observation.
  output wire o_payload_accept, // Same-domain transaction, complete event, or state observation.
  output wire o_issue_payload, // Same-domain transaction, complete event, or state observation.
  output wire o_issue_replay, // Same-domain transaction, complete event, or state observation.
  output wire o_issue_first, // Same-domain transaction, complete event, or state observation.
  output wire [8:0] o_issue_sequence, // Same-domain transaction, complete event, or state observation.
  output wire o_out_valid, // Same-domain transaction, complete event, or state observation.
  output wire o_out_payload, // Same-domain transaction, complete event, or state observation.
  output wire o_out_replay, // Same-domain transaction, complete event, or state observation.
  output wire o_out_first, // Same-domain transaction, complete event, or state observation.
  output wire [8:0] o_out_sequence, // Same-domain transaction, complete event, or state observation.
  output wire [C_DATA_WIDTH-1:0] o_out_data, // Same-domain transaction, complete event, or state observation.
  output wire o_tag_error, // Same-domain transaction, complete event, or state observation.
  output wire [8:0] o_out_stored_sequence, // Same-domain transaction, complete event, or state observation.
  output wire o_ack_accept, // Same-domain transaction, complete event, or state observation.
  output wire [7:0] o_ack_count, // Same-domain transaction, complete event, or state observation.
  output wire o_request_accept, // Same-domain transaction, complete event, or state observation.
  output wire o_command_reject, // Same-domain transaction, complete event, or state observation.
  output wire [7:0] o_resident_count, // Same-domain transaction, complete event, or state observation.
  output wire [8:0] o_ctl_last_sequence, // Same-domain transaction, complete event, or state observation.
  output wire [8:0] o_ctl_last_ack, // Same-domain transaction, complete event, or state observation.
  output wire [3:0] o_ctl_ignore_count, // Same-domain transaction, complete event, or state observation.
  output wire [7:0] o_ctl_unacked_count, // Same-domain transaction, complete event, or state observation.
  output wire [C_ADDR_WIDTH-1:0] o_ctl_head_pointer, // Same-domain transaction, complete event, or state observation.
  output wire [C_ADDR_WIDTH-1:0] o_ctl_write_pointer, // Same-domain transaction, complete event, or state observation.
  output wire [8:0] o_ctl_scheduled_sequence, // Same-domain transaction, complete event, or state observation.
  output wire [7:0] o_ctl_scheduled_count, // Same-domain transaction, complete event, or state observation.
  output wire [C_ADDR_WIDTH-1:0] o_ctl_scheduled_pointer, // Same-domain transaction, complete event, or state observation.
  output wire o_ctl_first_pending, // Same-domain transaction, complete event, or state observation.
  output wire o_rx_ingress_event, // Same-domain transaction, complete event, or state observation.
  output wire o_rx_accept, // Same-domain transaction, complete event, or state observation.
  output wire o_rx_payload_accept, // Same-domain transaction, complete event, or state observation.
  output wire o_rx_sequence_valid, // Same-domain transaction, complete event, or state observation.
  output wire [8:0] o_rx_sequence, // Same-domain transaction, complete event, or state observation.
  output wire o_rx_replay_request, // Same-domain transaction, complete event, or state observation.
  output wire o_rx_command_valid, // Same-domain transaction, complete event, or state observation.
  output wire o_rx_command_request, // Same-domain transaction, complete event, or state observation.
  output wire [8:0] o_rx_command_target, // Same-domain transaction, complete event, or state observation.
  output wire o_rx_crc_error, // Same-domain transaction, complete event, or state observation.
  output wire o_rx_zero_sequence, // Same-domain transaction, complete event, or state observation.
  output wire o_rx_zero_command, // Same-domain transaction, complete event, or state observation.
  output wire o_rx_backpressure_drop, // Same-domain transaction, complete event, or state observation.
  output wire o_rx_unexpected, // Same-domain transaction, complete event, or state observation.
  output wire o_rx_ambiguous_drop, // Same-domain transaction, complete event, or state observation.
  output wire o_rx_replay_drop, // Same-domain transaction, complete event, or state observation.
  output wire [8:0] o_rx_last_sequence, // Same-domain transaction, complete event, or state observation.
  output wire [2:0] o_rx_bad_crc_count, // Same-domain transaction, complete event, or state observation.
  output wire [7:0] o_rx_unexpected_count, // Same-domain transaction, complete event, or state observation.
  output wire o_rx_ambiguous, // Same-domain transaction, complete event, or state observation.
  output wire o_rx_replay, // Same-domain transaction, complete event, or state observation.
  output wire [23:0] o_issue_header, // Same-domain transaction, complete event, or state observation.
  output wire o_issue_header_valid, // Same-domain transaction, complete event, or state observation.
  output wire o_issue_metadata_error, // Same-domain transaction, complete event, or state observation.
  output wire [2:0] o_tx_explicit_count, // Same-domain transaction, complete event, or state observation.
  output wire [1:0] o_tx_request_count, // Same-domain transaction, complete event, or state observation.
  output wire [8:0] o_tx_request_sequence, // Same-domain transaction, complete event, or state observation.
  output wire o_tx_group_used, // Same-domain transaction, complete event, or state observation.
  output wire [8:0] o_rx_effective_sequence, // Same-domain transaction, complete event, or state observation.
  output wire [23:0] o_out_header, // Full header registered on the same edge as the actual SRAM return data.
  output wire [C_DATA_WIDTH-1:0] o_rx_data // Only forwards the received payload in the cycle when accepted.
); // End of public interface
wire flag_tx_metadata_ok; // Uses proven real source type and register to determine send metadata early.
assign flag_tx_metadata_ok = o_payload_accept || (o_issue_replay ? (o_request_accept || (o_ctl_scheduled_sequence != 9'd0)) : (o_ctl_last_sequence != 9'd0)); // Early metadata is checked against the complete original RTL proof.
reg [23:0] reg_header; // Full header register synchronized with the storage output slot.
assign o_out_header = o_out_valid ? reg_header : 24'd0; // Do not expose stale header when slot is empty or cleared.
assign o_rx_data = o_rx_payload_accept ? i_rx_data : {C_DATA_WIDTH{1'b0}}; // Forward only the actually accepted receive payload.
always @(posedge i_clk) begin // The only additional register updates only on real clock edges.
  if (!i_rstn) reg_header <= 24'd0; // Synchronous reset of header state.
  else if (i_link_reset) reg_header <= 24'd0; // Link clear cancels stale response.
  else if (o_issue_accept) reg_header <= o_issue_header; // Save the metadata-generated header on the same edge as the actual selection.
end // No new reservation keeps the internal header; output is gated by the real slot validity.
endmodule // End of replay data port.
