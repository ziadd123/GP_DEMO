module dl_replay_event_port #( // dl_replay_event_port module: same-domain connection of LLR receive events and send header.
    parameter integer C_EARLY_TX_METADATA = 0 // When nonzero, use early send-source metadata qualification explicitly proven by the integration layer.
) ( // dl_replay_event_port module: actual LLR receive events and send-header connection.
    input wire i_clk, // Same-domain receive or send input.
    input wire i_rstn, // Same-domain receive or send input.
    input wire i_link_reset, // Same-domain receive or send input.
    input wire i_rx_event_valid, // Same-domain receive or send input.
    input wire i_rx_event_discard, // Same-domain receive or send input.
    input wire i_rx_crc_ok, // Same-domain receive or send input.
    input wire [23:0] i_rx_header, // Same-domain receive or send input.
    input wire [7:0] i_rx_replay_limit, // Same-domain receive or send input.
    input wire i_tx_flit_send, // Same-domain receive or send input.
    input wire i_tx_payload, // Same-domain receive or send input.
    input wire i_tx_replay, // Same-domain receive or send input.
    input wire i_tx_first_replay, // Same-domain receive or send input.
    input wire [8:0] i_tx_sequence, // Same-domain receive or send input.
    input wire i_tx_metadata_ok, // Only used in early mode; default mode still enforces full original metadata protection.
    input wire i_tx_new_group, // Same-domain receive or send input.
    output wire o_rx_ingress_event, // Full state or current-cycle native event observation.
    output wire o_rx_accept, // Full state or current-cycle native event observation.
    output wire o_rx_payload_accept, // Full state or current-cycle native event observation.
    output wire o_rx_sequence_valid, // Full state or current-cycle native event observation.
    output wire [8:0] o_rx_sequence, // Full state or current-cycle native event observation.
    output wire o_rx_replay_request, // Full state or current-cycle native event observation.
    output wire o_rx_command_valid, // Full state or current-cycle native event observation.
    output wire o_rx_command_request, // Full state or current-cycle native event observation.
    output wire [8:0] o_rx_command_target, // Full state or current-cycle native event observation.
    output wire o_rx_crc_error, // Full state or current-cycle native event observation.
    output wire o_rx_zero_sequence, // Full state or current-cycle native event observation.
    output wire o_rx_zero_command, // Full state or current-cycle native event observation.
    output wire o_rx_backpressure_drop, // Full state or current-cycle native event observation.
    output wire o_rx_unexpected, // Full state or current-cycle native event observation.
    output wire o_rx_ambiguous_drop, // Full state or current-cycle native event observation.
    output wire o_rx_replay_drop, // Full state or current-cycle native event observation.
    output wire [8:0] o_rx_last_sequence, // Full state or current-cycle native event observation.
    output wire [2:0] o_rx_bad_crc_count, // Full state or current-cycle native event observation.
    output wire [7:0] o_rx_unexpected_count, // Full state or current-cycle native event observation.
    output wire o_rx_ambiguous, // Full state or current-cycle native event observation.
    output wire o_rx_replay, // Full state or current-cycle native event observation.
    output wire [23:0] o_tx_header, // Full state or current-cycle native event observation.
    output wire o_tx_header_valid, // Full state or current-cycle native event observation.
    output wire o_tx_metadata_error, // Full state or current-cycle native event observation.
    output wire [2:0] o_tx_explicit_count, // Full state or current-cycle native event observation.
    output wire [1:0] o_tx_request_count, // Full state or current-cycle native event observation.
    output wire [8:0] o_tx_request_sequence, // Full state or current-cycle native event observation.
    output wire o_tx_group_used, // Full state or current-cycle native event observation.
    output wire [8:0] o_rx_effective_sequence // Full state or current-cycle native event observation.
); // End of port declaration
assign o_rx_effective_sequence = o_rx_accept ? o_rx_sequence : o_rx_last_sequence; // Only bypass the new sequence when accepted in the current cycle; otherwise keep the current state.
dl_replay_receiver u_receiver ( // Instantiate u_receiver with a single shared clock and reset.
    .i_clk(i_clk), // Directly connect native interface.
    .i_rstn(i_rstn), // Directly connect native interface.
    .i_link_reset(i_link_reset), // Directly connect native interface.
    .i_event_valid(i_rx_event_valid), // Directly connect native interface.
    .i_event_discard(i_rx_event_discard), // Directly connect native interface.
    .i_crc_ok(i_rx_crc_ok), // Directly connect native interface.
    .i_header(i_rx_header), // Directly connect native interface.
    .i_replay_limit(i_rx_replay_limit), // Directly connect native interface.
    .o_ingress_event(o_rx_ingress_event), // Directly connect native interface.
    .o_accept(o_rx_accept), // Directly connect native interface.
    .o_payload_accept(o_rx_payload_accept), // Directly connect native interface.
    .o_sequence_valid(o_rx_sequence_valid), // Directly connect native interface.
    .o_sequence(o_rx_sequence), // Directly connect native interface.
    .o_replay_request(o_rx_replay_request), // Directly connect native interface.
    .o_command_valid(o_rx_command_valid), // Directly connect native interface.
    .o_command_request(o_rx_command_request), // Directly connect native interface.
    .o_command_target(o_rx_command_target), // Directly connect native interface.
    .o_crc_error(o_rx_crc_error), // Directly connect native interface.
    .o_zero_sequence(o_rx_zero_sequence), // Directly connect native interface.
    .o_zero_command(o_rx_zero_command), // Directly connect native interface.
    .o_backpressure_drop(o_rx_backpressure_drop), // Directly connect native interface.
    .o_unexpected(o_rx_unexpected), // Directly connect native interface.
    .o_ambiguous_drop(o_rx_ambiguous_drop), // Directly connect native interface.
    .o_replay_drop(o_rx_replay_drop), // Directly connect native interface.
    .o_last_sequence(o_rx_last_sequence), // Directly connect native interface.
    .o_bad_crc_count(o_rx_bad_crc_count), // Directly connect native interface.
    .o_unexpected_count(o_rx_unexpected_count), // Directly connect native interface.
    .o_ambiguous(o_rx_ambiguous), // Directly connect native interface.
    .o_replay(o_rx_replay) // Directly connect native interface.
); // End of u_receiver instantiation

dl_replay_header_tx #(.C_EARLY_TX_METADATA(C_EARLY_TX_METADATA)) u_header_tx ( // Instantiate u_header_tx with a shared clock and reset.
    .i_clk(i_clk), // Directly connect native interface.
    .i_rstn(i_rstn), // Directly connect native interface.
    .i_link_reset(i_link_reset), // Directly connect native interface.
    .i_flit_send(i_tx_flit_send), // Directly connect native interface.
    .i_payload(i_tx_payload), // Directly connect native interface.
    .i_replay(i_tx_replay), // Directly connect native interface.
    .i_first_replay(i_tx_first_replay), // Directly connect native interface.
    .i_tx_sequence(i_tx_sequence), // Directly connect native interface.
    .i_tx_metadata_ok(i_tx_metadata_ok), // Explicitly pass optional early qualification; receive sequence still checked by the original receiver.
    .i_rx_sequence(o_rx_effective_sequence), // Directly connect native interface.
    .i_rx_request(o_rx_replay_request), // Directly connect native interface.
    .i_new_group(i_tx_new_group), // Directly connect native interface.
    .o_header(o_tx_header), // Directly connect native interface.
    .o_header_valid(o_tx_header_valid), // Directly connect native interface.
    .o_metadata_error(o_tx_metadata_error), // Directly connect native interface.
    .o_explicit_count(o_tx_explicit_count), // Directly connect native interface.
    .o_request_count(o_tx_request_count), // Directly connect native interface.
    .o_request_sequence(o_tx_request_sequence), // Directly connect native interface.
    .o_group_used(o_tx_group_used) // Directly connect native interface.
); // End of u_header_tx instantiation
endmodule // End of dl_replay_event_port module.
