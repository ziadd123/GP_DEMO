module dl_replay_tx_storage #( // dl_replay_tx_storage module: actual synchronous SRAM replay and output-hold slot.
  parameter integer C_DEPTH = 255, // Exact logic storage capacity.
  parameter integer C_DATA_WIDTH = 32, // Opaque payload data width, must be a positive integer number of bytes.
  parameter integer C_ADDR_WIDTH = (C_DEPTH <= 2) ? 1 : (C_DEPTH <= 4) ? 2 : (C_DEPTH <= 8) ? 3 : (C_DEPTH <= 16) ? 4 : (C_DEPTH <= 32) ? 5 : (C_DEPTH <= 64) ? 6 : (C_DEPTH <= 128) ? 7 : (C_DEPTH <= 256) ? 8 : (C_DEPTH <= 512) ? 9 : (C_DEPTH <= 1024) ? 10 : (C_DEPTH <= 2048) ? 11 : (C_DEPTH <= 4096) ? 12 : 13,
) ( // Declare same-domain transaction and output handshake.
  input wire i_clk, // Same-domain transaction, control state, or full output observation.
  input wire i_rstn, // Same-domain transaction, control state, or full output observation.
  input wire i_link_reset, // Same-domain transaction, control state, or full output observation.
  input wire i_ingress_event, // Same-domain transaction, control state, or full output observation.
  input wire i_command_valid, // Same-domain transaction, control state, or full output observation.
  input wire i_command_request, // Same-domain transaction, control state, or full output observation.
  input wire [9-1:0] i_command_target, // Same-domain transaction, control state, or full output observation.
  input wire i_issue, // Same-domain transaction, control state, or full output observation.
  input wire i_payload, // Same-domain transaction, control state, or full output observation.
  input wire [C_DATA_WIDTH-1:0] i_data, // Same-domain transaction, control state, or full output observation.
  input wire i_out_ready, // Same-domain transaction, control state, or full output observation.
  output wire o_issue_ready, // Same-domain transaction, control state, or full output observation.
  output wire o_issue_accept, // Same-domain transaction, control state, or full output observation.
  output wire o_payload_accept, // Same-domain transaction, control state, or full output observation.
  output wire o_issue_payload, // Same-domain transaction, control state, or full output observation.
  output wire o_issue_replay, // Same-domain transaction, control state, or full output observation.
  output wire o_issue_first, // Same-domain transaction, control state, or full output observation.
  output wire [9-1:0] o_issue_sequence, // Same-domain transaction, control state, or full output observation.
  output wire o_out_valid, // Same-domain transaction, control state, or full output observation.
  output wire o_out_payload, // Same-domain transaction, control state, or full output observation.
  output wire o_out_replay, // Same-domain transaction, control state, or full output observation.
  output wire o_out_first, // Same-domain transaction, control state, or full output observation.
  output wire [9-1:0] o_out_sequence, // Same-domain transaction, control state, or full output observation.
  output wire [C_DATA_WIDTH-1:0] o_out_data, // Same-domain transaction, control state, or full output observation.
  output wire o_tag_error, // Same-domain transaction, control state, or full output observation.
  output wire [9-1:0] o_out_stored_sequence, // Same-domain transaction, control state, or full output observation.
  output wire o_ack_accept, // Same-domain transaction, control state, or full output observation.
  output wire [8-1:0] o_ack_count, // Same-domain transaction, control state, or full output observation.
  output wire o_request_accept, // Same-domain transaction, control state, or full output observation.
  output wire o_command_reject, // Same-domain transaction, control state, or full output observation.
  output wire [8-1:0] o_resident_count, // Same-domain transaction, control state, or full output observation.
  output wire [9-1:0] o_ctl_last_sequence, // Same-domain transaction, control state, or full output observation.
  output wire [9-1:0] o_ctl_last_ack, // Same-domain transaction, control state, or full output observation.
  output wire [4-1:0] o_ctl_ignore_count, // Same-domain transaction, control state, or full output observation.
  output wire [8-1:0] o_ctl_unacked_count, // Same-domain transaction, control state, or full output observation.
  output wire [C_ADDR_WIDTH-1:0] o_ctl_head_pointer, // Same-domain transaction, control state, or full output observation.
  output wire [C_ADDR_WIDTH-1:0] o_ctl_write_pointer, // Same-domain transaction, control state, or full output observation.
  output wire [9-1:0] o_ctl_scheduled_sequence, // Same-domain transaction, control state, or full output observation.
  output wire [8-1:0] o_ctl_scheduled_count, // Same-domain transaction, control state, or full output observation.
  output wire [C_ADDR_WIDTH-1:0] o_ctl_scheduled_pointer, // Same-domain transaction, control state, or full output observation.
  output wire o_ctl_first_pending // Same-domain transaction, control state, or full output observation.
); // End of interface declaration
localparam integer C_MAPPED_DEPTH = (C_DEPTH < 2) ? 2 : C_DEPTH; // Minimum physical mapping does not enlarge logical capacity.
localparam integer C_WORD_WIDTH = C_DATA_WIDTH + 16; // Nine-bit sequence plus seven-bit padding are stored together as a word.
generate // Compile-time check on data width.
  if ((C_DATA_WIDTH < 8) || ((C_DATA_WIDTH % 8) != 0)) begin : gen_invalid_width // Reject non-byte-aligned data interfaces.
   UALINK_REPLAY_STORAGE_WIDTH_INVALID invalid_width (); // Illegal configuration cannot generate usable hardware.
  end // End of invalid-width branch.
endgenerate // End of parameter check.
reg reg_valid; // Independent output-slot state register.
reg reg_payload; // Independent output-slot state register.
reg reg_replay; // Independent output-slot state register.
reg reg_first; // Independent output-slot state register.
reg [9-1:0] reg_sequence; // Independent output-slot state register.
reg [C_DATA_WIDTH-1:0] reg_normal_data; // Independent output-slot state register.
wire flag_active; // Working qualification outside reset and link clear.
wire normal_push; // Normal source write request that obtains the output slot.
wire [C_WORD_WIDTH-1:0] memory_write_data; // Complete word including sequence and payload written to actual SRAM.
wire [C_WORD_WIDTH-1:0] memory_read_data; // Actual synchronous SRAM output registered after read.
wire ctl_ack_accept; // Proven native observation of the controller.
wire [8-1:0] ctl_ack_count; // Proven native observation of the controller.
wire ctl_request_accept; // Proven native observation of the controller.
wire ctl_command_reject; // Proven native observation of the controller.
wire ctl_push_ready; // Proven native observation of the controller.
wire ctl_push_accept; // Proven native observation of the controller.
wire [9-1:0] ctl_push_sequence; // Proven native observation of the controller.
wire [C_ADDR_WIDTH-1:0] ctl_write_addr; // Proven native observation of the controller.
wire ctl_replay_valid; // Proven native observation of the controller.
wire ctl_replay_take; // Proven native observation of the controller.
wire [9-1:0] ctl_replay_sequence; // Proven native observation of the controller.
wire [C_ADDR_WIDTH-1:0] ctl_replay_addr; // Proven native observation of the controller.
wire ctl_first_replay; // Proven native observation of the controller.
wire [8-1:0] ctl_resident_count; // Proven native observation of the controller.
wire [9-1:0] ctl_last_sequence; // Proven native observation of the controller.
wire [9-1:0] ctl_last_ack; // Proven native observation of the controller.
wire [4-1:0] ctl_ignore_count; // Proven native observation of the controller.
wire [8-1:0] ctl_unacked_count; // Proven native observation of the controller.
wire [C_ADDR_WIDTH-1:0] ctl_head_pointer; // Proven native observation of the controller.
wire [C_ADDR_WIDTH-1:0] ctl_write_pointer; // Proven native observation of the controller.
wire [9-1:0] ctl_scheduled_sequence; // Proven native observation of the controller.
wire [8-1:0] ctl_scheduled_count; // Proven native observation of the controller.
wire [C_ADDR_WIDTH-1:0] ctl_scheduled_pointer; // Proven native observation of the controller.
wire ctl_first_pending; // Proven native observation of the controller.
assign flag_active = i_rstn && !i_link_reset; // Native handshake, state, or actual data selection.
assign o_issue_ready = flag_active && (!reg_valid || i_out_ready); // Native handshake, state, or actual data selection.
assign o_issue_accept = i_issue && o_issue_ready; // Native handshake, state, or actual data selection.
assign normal_push = o_issue_accept && i_payload; // Native handshake, state, or actual data selection.
assign o_payload_accept = ctl_push_accept; // Native handshake, state, or actual data selection.
assign o_issue_payload = o_issue_accept && (ctl_replay_take || ctl_push_accept); // Native handshake, state, or actual data selection.
assign o_issue_replay = ctl_replay_take; // Native handshake, state, or actual data selection.
assign o_issue_first = ctl_replay_take && ctl_first_replay; // Native handshake, state, or actual data selection.
assign o_issue_sequence = o_issue_accept ? (ctl_replay_take ? ctl_replay_sequence : (ctl_push_accept ? ctl_push_sequence : ctl_last_sequence)) : 9'd0; // Native handshake, state, or actual data selection.
assign memory_write_data = {7'd0, ctl_push_sequence, i_data}; // Native handshake, state, or actual data selection.
assign o_out_valid = flag_active && reg_valid; // Native handshake, state, or actual data selection.
assign o_out_payload = o_out_valid && reg_payload; // Native handshake, state, or actual data selection.
assign o_out_replay = o_out_valid && reg_replay; // Native handshake, state, or actual data selection.
assign o_out_first = o_out_valid && reg_first; // Native handshake, state, or actual data selection.
assign o_out_sequence = o_out_valid ? reg_sequence : 9'd0; // Native handshake, state, or actual data selection.
assign o_out_data = o_out_payload ? (reg_replay ? memory_read_data[C_DATA_WIDTH-1:0] : reg_normal_data) : {C_DATA_WIDTH{1'b0}}; // Native handshake, state, or actual data selection.
assign o_out_stored_sequence = o_out_replay ? memory_read_data[C_DATA_WIDTH +: 9] : 9'd0; // Native handshake, state, or actual data selection.
assign o_tag_error = o_out_replay && (o_out_stored_sequence != reg_sequence); // Native handshake, state, or actual data selection.
assign o_ack_accept = ctl_ack_accept; // Native handshake, state, or actual data selection.
assign o_ack_count = ctl_ack_count; // Native handshake, state, or actual data selection.
assign o_request_accept = ctl_request_accept; // Native handshake, state, or actual data selection.
assign o_command_reject = ctl_command_reject; // Native handshake, state, or actual data selection.
assign o_resident_count = ctl_resident_count; // Native handshake, state, or actual data selection.
assign o_ctl_last_sequence = ctl_last_sequence; // Native handshake, state, or actual data selection.
assign o_ctl_last_ack = ctl_last_ack; // Native handshake, state, or actual data selection.
assign o_ctl_ignore_count = ctl_ignore_count; // Native handshake, state, or actual data selection.
assign o_ctl_unacked_count = ctl_unacked_count; // Native handshake, state, or actual data selection.
assign o_ctl_head_pointer = ctl_head_pointer; // Native handshake, state, or actual data selection.
assign o_ctl_write_pointer = ctl_write_pointer; // Native handshake, state, or actual data selection.
assign o_ctl_scheduled_sequence = ctl_scheduled_sequence; // Native handshake, state, or actual data selection.
assign o_ctl_scheduled_count = ctl_scheduled_count; // Native handshake, state, or actual data selection.
assign o_ctl_scheduled_pointer = ctl_scheduled_pointer; // Native handshake, state, or actual data selection.
assign o_ctl_first_pending = ctl_first_pending; // Native handshake, state, or actual data selection.

endmodule // End of dl_replay_tx_storage module.
