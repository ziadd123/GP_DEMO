// DL two-level message arbiter: selects a complete and send-eligible external message source.
// Version and scope: UALink 200G DL/PL 2.0 §2.4.1.2; 2026-09-08 initial standalone module.
// History: implements source-group round robin and UART whole-message occupancy; encoding, credits, timers, and queue handling are performed by the integration layer.
`timescale 1ps/1ps // Use the same time unit as the local simulation environment; no delays are used inside the RTL.
module dl_message_arbiter ( // DL message arbiter module; each valid segment can consume at most one DWORD.
    input wire i_clk, // Unique rising-edge clock for all scheduling state.
    input wire i_rstn, // Synchronous active-low local reset, not the UART protocol reset message.
    input wire i_uart_cancel, // Synchronously cancels the local UART transport while keeping other sources and all polling history.
    input wire i_segment_available, // A DL segment service opportunity is actually provided in the current cycle.
    input wire [10:0] i_source_pending, // Queue head of each source is fully staged and eligible to send.
    input wire [351:0] i_source_words, // Source 0 is the lowest DWORD; UART provides the current word by published index.
    input wire [5:0] i_uart_word_count, // UART first-word sampling total length, including header, legal range 2 to 33.
    output wire o_valid, // Real message DWORD is sent in the current cycle.
    output wire [31:0] o_word, // Sent DWORD; output zero when invalid.
    output wire [3:0] o_source, // Current local source index 0 to 10; not a real-line message field.
    output wire [5:0] o_word_index, // Current valid word index within the full message, counted from zero.
    output wire o_last, // Current DWORD completes a whole message; for non-UART sources, every word completes the message.
    output wire [10:0] o_source_take, // Each source can consume one DWORD only when its corresponding bit is 1.
    output wire [10:0] o_message_done, // Each source can remove its complete queue-head message only when its corresponding bit is 1.
    output wire o_error, // Local interface length or staged-eligibility violation; not a protocol error code on the line.
    output wire o_locked, // UART header has been consumed and remaining service opportunity remains reserved.
    output wire [5:0] o_uart_word_index // Index of the current word that the external UART staging buffer should present; zero when idle.
); // End of fixed 11-source arbiter interface.
    reg [1:0] reg_group_next; // Which source group starts the next message.
    reg [2:0] reg_basic_next; // Next initial candidate position in the Basic group.
    reg reg_control_next; // Next initial candidate position in the Control group (two sources).
    reg [1:0] reg_uart_next; // Next initial candidate position in the UART group (four sources).
    reg reg_uart_locked; // Saves UART transport ownership only after the first word is served.
    reg [5:0] cnt_uart_index; // Next pending DWORD index of the locked message.
    reg [5:0] reg_uart_last; // End index captured on the first-word edge; later input length changes do not affect this message.
    wire flag_uart_locked; // Cancel edge immediately masks old ownership, allowing other messages to use the current service opportunity.
    wire flag_uart_length_valid; // Local length check at first-word admission.
    wire [10:0] flag_eligible; // Eligible idle candidates after filtering invalid UART lengths and cancellation edges.
    wire flag_basic, flag_control, flag_uart; // Whether each source group has a serviceable queue head.
    reg [3:0] sel_basic, sel_control, sel_uart; // Locally chosen source index inside each group.
    reg [1:0] sel_group; // Final selected group; forced to UART when locked.
    reg [3:0] sel_source; // Final selected source; forced to UART transport when locked.
    wire flag_found, flag_emit, flag_last, flag_complete, flag_start; // Qualification, actual service, and message state update conditions.
    genvar gen_source; // Fixed expansion of 11 one-hot events without introducing a variable protocol source count.
    assign flag_uart_locked = reg_uart_locked && !i_uart_cancel; // Selection logic uses valid occupancy; the real register is cleared on the sampling edge.
    assign flag_uart_length_valid = (i_uart_word_count >= 6'd2) && (i_uart_word_count <= 6'd33); // Valid transport includes one header and 1 to 32 payload words.
    assign flag_eligible = i_source_pending & {3'b111, (flag_uart_length_valid && !i_uart_cancel), 7'b1111111}; // Idle selection also masks invalid lengths and new UART transport after cancel edge.
    assign flag_basic = |flag_eligible[4:0]; // Five Basic messages participate in final-level polling as a group.
    assign flag_control = |flag_eligible[6:5]; // Two Control messages participate in final-level polling as a group.
    assign flag_uart = |flag_eligible[10:7]; // Four UART messages still have their own in-group priority order.
    always @(*) begin // Check messages starting from the saved Basic polling start point.
        sel_basic = 4'd0; // Deterministic index when no request exists; actual service is still gated by group qualification.
        case (reg_basic_next) // Explicitly list each cycle order to avoid a single fixed priority.
            3'd0: begin // Start from the No-Op source.
                if (flag_eligible[0]) sel_basic = 4'd0; else if (flag_eligible[1]) sel_basic = 4'd1; else if (flag_eligible[2]) sel_basic = 4'd2; else if (flag_eligible[3]) sel_basic = 4'd3; else if (flag_eligible[4]) sel_basic = 4'd4;
            end // End of Basic starting point 0.
            3'd1: begin // Start from the Tx Ready source and allow wraparound.
                if (flag_eligible[1]) sel_basic = 4'd1; else if (flag_eligible[2]) sel_basic = 4'd2; else if (flag_eligible[3]) sel_basic = 4'd3; else if (flag_eligible[4]) sel_basic = 4'd4; else if (flag_eligible[0]) sel_basic = 4'd0;
            end // End of Basic starting point 1.
            3'd2: begin // Start from the TL Rate source and allow wraparound.
                if (flag_eligible[2]) sel_basic = 4'd2; else if (flag_eligible[3]) sel_basic = 4'd3; else if (flag_eligible[4]) sel_basic = 4'd4; else if (flag_eligible[0]) sel_basic = 4'd0; else if (flag_eligible[1]) sel_basic = 4'd1;
            end // End of Basic starting point 2.
            3'd3: begin // Start from the Device ID source and allow wraparound.
                if (flag_eligible[3]) sel_basic = 4'd3; else if (flag_eligible[4]) sel_basic = 4'd4; else if (flag_eligible[0]) sel_basic = 4'd0; else if (flag_eligible[1]) sel_basic = 4'd1; else if (flag_eligible[2]) sel_basic = 4'd2;
            end // End of Basic starting point 3.
            3'd4: begin // Start from the Port Number source and allow wraparound.
                if (flag_eligible[4]) sel_basic = 4'd4; else if (flag_eligible[0]) sel_basic = 4'd0; else if (flag_eligible[1]) sel_basic = 4'd1; else if (flag_eligible[2]) sel_basic = 4'd2; else if (flag_eligible[3]) sel_basic = 4'd3;
            end // End of Basic starting point 4.
            default: sel_basic = 4'd0; // Deterministic default for unreachable simulation values; no fault-recovery protocol is promised.
        endcase // End of Basic group polling selection.
    end // End of Basic combined arbitration.
    always @(*) begin // Control keeps the fair order of its two sources.
        sel_control = 4'd5; // Invalid index when no request exists, but still deterministic.
        if (!reg_control_next) begin // First check Link Width, then Channel On/Offline.
            if (flag_eligible[5]) sel_control = 4'd5; else if (flag_eligible[6]) sel_control = 4'd6; // Allows skipping the current empty source.
        end else begin // After finishing the previous source, swap the starting point.
            if (flag_eligible[6]) sel_control = 4'd6; else if (flag_eligible[5]) sel_control = 4'd5; // Poll order is 6 then 5.
        end // End of Control current start selection.
    end // End of Control combined arbitration.
    always @(*) begin // UART transport, credit, and the two reset messages are polled separately.
        sel_uart = 4'd7; // Keep a deterministic index when no request exists; group qualification prevents service.
        case (reg_uart_next) // Select one complete message according to the captured group start point.
            2'd0: begin // Start from the transport source.
                if (flag_eligible[7]) sel_uart = 4'd7; else if (flag_eligible[8]) sel_uart = 4'd8; else if (flag_eligible[9]) sel_uart = 4'd9; else if (flag_eligible[10]) sel_uart = 4'd10;
            end // End of UART start point 0.
            2'd1: begin // Start from the credit update source.
                if (flag_eligible[8]) sel_uart = 4'd8; else if (flag_eligible[9]) sel_uart = 4'd9; else if (flag_eligible[10]) sel_uart = 4'd10; else if (flag_eligible[7]) sel_uart = 4'd7;
            end // End of UART start point 1.
            2'd2: begin // Start from the reset request source.
                if (flag_eligible[9]) sel_uart = 4'd9; else if (flag_eligible[10]) sel_uart = 4'd10; else if (flag_eligible[7]) sel_uart = 4'd7; else if (flag_eligible[8]) sel_uart = 4'd8;
            end // End of UART start point 2.
            2'd3: begin // Start from the reset response source.
                if (flag_eligible[10]) sel_uart = 4'd10; else if (flag_eligible[7]) sel_uart = 4'd7; else if (flag_eligible[8]) sel_uart = 4'd8; else if (flag_eligible[9]) sel_uart = 4'd9;
            end // End of UART start point 3.
            default: sel_uart = 4'd7; // Provide a deterministic default branch for unknown simulation values.
        endcase // End of UART group polling selection.
    end // End of UART combined arbitration.
    always @(*) begin // Final-level polling among the three source groups, without collapsing 11 sources into a single ring.
        sel_group = 2'd0; // Idle default points to Basic; actual service still requires group qualification.
        case (reg_group_next) // Only message completion changes this polling start point.
            2'd0: begin // Initial order is Basic, Control, UART.
                if (flag_basic) sel_group = 2'd0; else if (flag_control) sel_group = 2'd1; else if (flag_uart) sel_group = 2'd2;
            end // End of final-level start point Basic.
            2'd1: begin // Start from Control and wrap to Basic.
                if (flag_control) sel_group = 2'd1; else if (flag_uart) sel_group = 2'd2; else if (flag_basic) sel_group = 2'd0;
            end // Keep inter-group fairness.
            2'd2: begin // Start from UART and wrap to Basic.
                if (flag_uart) sel_group = 2'd2; else if (flag_basic) sel_group = 2'd0; else if (flag_control) sel_group = 2'd1;
            end // Keep inter-group fairness.
            default: sel_group = 2'd0; // Deterministic default for unreachable states.
        endcase // End of final group selection.
    end // End of final pair of group arbitration.
endmodule // End of DL two-level arbiter module.
