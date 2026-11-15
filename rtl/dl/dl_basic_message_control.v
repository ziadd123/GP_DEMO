`timescale 1ps/1ps // Explicitly declare the control clock period in picoseconds.
module dl_basic_message_control #( // Module implements single local Basic request, independent remote reply, and real commit deadline.
    parameter C_CLOCK_PERIOD_PS = 640 // This clock must be stable and cannot silently inherit a variable UPLI period.
)( // Native same-domain inputs and all observable outputs.
    input wire i_clk, // The only original control clock.
    input wire i_rstn, // Synchronous active-low global reset.
    input wire i_local_valid, // Local request remains valid until it is actually accepted.
    input wire [2:0] i_local_kind, // Local mtype supports only 1, 4, 5, and 6.
    input wire [15:0] i_local_rate, // Rate encoding normalized per protocol.
    input wire i_device_valid, // Whether the current local device identity is configured.
    input wire [9:0] i_device_id, // Current 10-bit local device number.
    input wire i_device_type, // 0 means Switch, 1 means Accelerator.
    input wire i_port_valid, // Whether the current local port number is configured.
    input wire [11:0] i_port, // Current 12-bit local port number.
    input wire i_folding, // Whether the local side supports Folding.
    input wire i_tx_ready_advertised, // Whether it declares the ability to send TxReady notifications actively.
    input wire i_symbols_valid, // External qualification that the PL transmitter to be activated has provided valid symbols.
    input wire i_tx_limit_valid, // Whether the externally effective pacing limit is available.
    input wire [15:0] i_tx_limit, // Actual external pacing limit, not the value to be configured.
    input wire i_rx_valid, // Real message receive event after reliable and correct framing.
    input wire [31:0] i_rx_word, // Full received DWORD; reserved fields are ignored.
    input wire [3:0] i_source_take, // Real external arbitration consumption, from low to high corresponds to mtypes 1, 4, 5, 6.
    output wire o_local_ready, // Whether the current local request can be accepted.
    output wire o_local_pending, // Local request has been accepted but has not yet received a matching ACK.
    output wire o_local_waiting, // Local request has actually been issued and is waiting for ACK.
    output wire o_remote_pending, // Independent remote request has not yet been truly answered.
    output wire [3:0] o_source_pending, // Ready sources arranged by mtype 1, 4, 5, 6.
    output wire [127:0] o_source_words, // Four complete, stable queue-head DWORDs.
    output wire o_local_start, // Real local request acceptance occurs on this edge.
    output wire o_local_commit, // Real local request transmission occurs on this edge.
    output wire o_local_done, // Matching ACK completes the previous local wait.
    output wire o_reply_done, // Previous remote request is truly replied to on this edge.
    output wire o_rx_request, // Remote request is accepted on this edge and reply obligation is saved.
    output wire o_rx_noop, // NoOp received and no ACK generated.
    output wire o_rx_unhandled, // Non-basic or reserved message passed to upper layer.
    output wire o_rx_unsupported, // TxReady message when Folding is disabled.
    output wire o_rx_unmatched_ack, // ACK does not match a previously sent local request.
    output wire o_rx_overlap, // Second illegal overlapping request while previous remote request is unanswered.
    output wire o_peer_rate_valid, // Whether a valid Rate request has been received.
    output wire [15:0] o_peer_rate, // Most recently accepted peer Rate encoding.
    output wire o_peer_device_valid, // Whether the most recent peer device identity is valid.
    output wire [1:0] o_peer_device_type, // Original peer type encoding; reserved encodings are not promoted to local type.
    output wire [9:0] o_peer_device_id, // Most recent valid peer device number.
    output wire o_peer_port_valid, // Whether the most recent peer port number is valid.
    output wire [11:0] o_peer_port, // Most recent valid peer port number.
    output wire o_peer_rate_update, // New Rate accepted on this edge and pacing update required.
    output wire o_deadline_miss, // First actual sampling edge exceeding the one-microsecond reply budget.
    output wire o_deadline_fault, // Sticky late diagnosis preserved before global reset.
    output wire o_protocol_fault, // Sticky protocol input diagnosis preserved before global reset.
    output wire o_error // Illegal commit, request type, or receive ownership error on this edge.
); // End of native control boundary; does not include PLL or PHY implementation.
    localparam C_ACK_CYCLES = 1000000/C_CLOCK_PERIOD_PS; // The one-microsecond limit must round down instead of easing to the next cycle.
    localparam [19:0] C_ACK_LAST = C_ACK_CYCLES[19:0]; // With a minimum of one picosecond, at most one million complete cycles are allowed.
    reg reg_local_busy; // Saves a single uncompleted local request.
    reg reg_local_sent; // Local request has been sent on a real commit edge.
    reg [2:0] reg_local_kind; // Local request type is frozen at acceptance time.
    reg [15:0] reg_local_upper; // Freezes the local message upper field; fixed header rebuilt from type.
    reg reg_remote_busy; // Saves a single independent remote reply obligation.
    reg [2:0] reg_remote_kind; // Remote request type is frozen at acceptance time.
    reg [15:0] reg_remote_upper; // Freezes reply upper field and local identity; fixed header does not need to be stored.
    reg [15:0] reg_remote_rate; // Rate limit corresponding to this reply.
    reg reg_local_first; // Whether the local request entered queue first when two slots of same type coexist.
    reg [19:0] cnt_remote_age; // Records elapsed clock edges and saturates at deadline end.
    reg reg_remote_missed; // A remote request triggers the timeout event only once.
    reg reg_deadline_fault; // Sticky independent deadline diagnosis does not clear the reply.
    reg reg_protocol_fault; // Sticky protocol diagnosis does not clear saved ownership.
    reg reg_peer_rate_valid; // Saves whether the peer Rate has been announced.
    reg [15:0] reg_peer_rate; // Saves the original encoding of the latest accepted Rate.
    reg reg_peer_device_valid; // Saves the most recently learned Device Valid.
    reg [1:0] reg_peer_device_type; // Saves the most recently learned original device type.
    reg [9:0] reg_peer_device_id; // Saves the most recently learned valid device number.
    reg reg_peer_port_valid; // Saves the most recently learned Port Valid.
    reg [11:0] reg_peer_port; // Saves the most recently learned valid port number.
    wire [2:0] rx_kind; // mtype field already present on the line.
    wire [15:0] rx_rate; // Explicit 16-bit Rate and remote field snapshot.
    wire rx_identity_valid; // Explicit single-bit device or port Valid.
    wire [1:0] rx_device_type; // Two-bit type after invalid identity is cleared to zero.
    wire [9:0] rx_device_id; // 10-bit number after invalid identity is cleared to zero.
    wire [11:0] rx_port; // 12-bit port after invalid identity is cleared to zero.
    wire flag_local_supported, flag_local_capable; // Local type and condition send capability.
    wire flag_rx_basic, flag_rx_known, flag_rx_supported, flag_rx_ack, flag_rx_request; // Boundary message qualifications.
    wire flag_same_kind, flag_local_head, flag_remote_head, flag_remote_ready; // Two-slot FIFO head and actual pacing qualification.
    wire [3:0] sel_local, sel_remote; // Four classes of sources directly select the current complete queue head.
    wire flag_take_onehot; // Reject multiple sources or no offer on commit.
    wire [3:0] sel_local_unqualified; // Local source qualification without reset masking, used only for internal error checking.
    wire flag_local_head_unqualified, flag_remote_head_unqualified; // Raw head relationship separated from reset masking.
    wire flag_rx_error_supported, flag_rx_error_request, flag_rx_error_unmatched_ack; // Internal receive errors computed independently before public diagnosis is filtered by reset.
    wire [3:0] sel_remote_unqualified; // Remote type and head selection independent of late pacing comparison.
    wire flag_remote_take_unqualified; // Single-resource consumption and remote head qualification completed early.
    wire flag_error_ready, flag_error_blocked; // Precompute input diagnosis for allowed reply and blocked reply cases separately.
    wire [3:0] flag_rate_lt, flag_rate_eq; // Four independent 4-bit comparisons completed in parallel.
    wire flag_rate_limit_ok; // High-priority combination of complete 16-bit less-than-or-equal comparison.
    wire flag_learn_device, flag_learn_port; // Only update identity on accepted request or matched ACK.
    wire [15:0] word_device, word_port, word_local, word_reply; // Only form message upper 16-bit identity and Rate snapshot.
    wire unused_rx_reserved; // Explicitly consume ignored reserved bits for static checking.
    assign unused_rx_reserved = ^{i_rx_word[15:13], i_rx_word[11:9], i_rx_word[1:0]}; // Reserved bits do not participate in any qualification, state, or response encoding.
    assign rx_kind = i_rx_word[8:6]; // Bit positions come from the verified message table.
    assign rx_rate = i_rx_word[31:16]; // All 16 bits are used as fixed-width register input.
    assign rx_identity_valid = i_rx_word[31]; // The device and port share the same Valid bit position.
    assign rx_device_type = rx_identity_valid ? i_rx_word[30:29] : 2'd0; // Reserved type only observed as raw data.

endmodule // End of Basic message control module.
