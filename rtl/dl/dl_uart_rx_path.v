// UALink stream0 receive framing, actual SRAM storage and firmware read credits.
// Date: 2026-09-08; local receive path; full reset handshakes and credit update scheduling are handled above.
`timescale 1ps/1ps // Receive processing and SRAM both use the same synchronous clock domain.
module dl_uart_rx_path #( // UART receive module parses ordered message words and provides opaque payload to firmware.
    parameter integer C_RX_DEPTH = 128 // Default recommended capacity; other values 1 to 4095 are research settings.
) ( // Local interface does not add extra ready or transaction fields on the wire.
    input wire i_clk, // Shared sampling clock for protocol decode and actual SRAM storage.
    input wire i_rstn, // Synchronous active-low global reset clearing message and data ownership.
    input wire i_stream_reset, // Stream reset clears storage credit, but keeps line-message drain active.
    input wire i_channel4_enabled, // Negotiated Channel4 receive running qualification.
    input wire i_word_valid, // Upstream has completed reliable processing of a full message word.
    input wire [31:0] i_word, // Ordered 32-bit message word without duplicates or corrupted transfers.
    input wire i_fw_ready, // Firmware accepts the current cached data queue head.
    output wire o_fw_valid, // Actual SRAM data has entered the consumable output cache.
    output wire [31:0] o_fw_word, // Firmware opaque data word; zero when invalid.
    output wire [11:0] o_rx_fill, // Includes SRAM in-flight read and cache occupancy.
    output wire o_initialized, // Stream reset has been released and capacity initialization is complete.
    output wire [11:0] o_rx_counter, // Only actual firmware reads increase credit; modulo 4096.
    output wire o_credit_available, // Credit qualification visible to the upper scheduler.
    output wire [5:0] o_remaining, // Remaining payload words in the current transport to receive or discard.
    output wire o_dropping, // Current message must be discarded and remain until its true end.
    output wire o_header, // Recognizes a new transport header without storing the header in the buffer.
    output wire o_payload_write, // Current real payload write to storage occurs on the prior edge.
    output wire o_payload_discard, // Current payload is discarded because the whole message is invalid.
    output wire o_done, // Final valid word of the original transport completes drain.
    output wire o_credit_valid, // Boundary recognizes a credit to deliver to send source.
    output wire [11:0] o_credit_value, // Absolute credit value extracted from DataFCSeq field.
    output wire o_reset_request, // Boundary recognizes a reset request applicable to this stream.
    output wire o_request_all, // Original field indicating the reset request applies to all streams.
    output wire o_reset_response, // Boundary recognizes a reset response applicable to this stream.
    output wire o_response_all, // Original field indicating the reset response applies to all streams.
    output wire [2:0] o_response_status, // Unmodified status field delivered to upper reset state machine.
    output wire o_other_valid, // Current boundary word must be passed to other DL message handlers.
    output wire [31:0] o_other_word, // Original other DL message content passed through.
    output wire o_error // Local capacity or storage contract diagnosis; no new wire-level error code.
); // End of single-clock UART receive path port.
    localparam integer C_COUNT_WIDTH = (C_RX_DEPTH < 2) ? 1 : (C_RX_DEPTH < 4) ? 2 : (C_RX_DEPTH < 8) ? 3 : (C_RX_DEPTH < 16) ? 4 : (C_RX_DEPTH < 32) ? 5 : (C_RX_DEPTH < 64) ? 6 : (C_RX_DEPTH < 128) ? 7 : (C_RX_DEPTH < 256) ? 8 : (C_RX_DEPTH < 512) ? 9 : (C_RX_DEPTH < 1024) ? 10 : (C_RX_DEPTH < 2048) ? 11 : (C_RX_DEPTH < 4096) ? 12 : 13;
    localparam [11:0] C_INITIAL_CREDIT = C_RX_DEPTH[11:0]; // Full valid depth enters the 12-bit initial credit value.
    localparam [12:0] C_CAPACITY = C_RX_DEPTH[12:0]; // 13-bit admission comparison avoids wraparound when message length is added.
    reg [5:0] cnt_remaining; // Message drain state tracking the remaining payload words for each valid line transport.
    reg reg_drop; // Once the message loses receive qualification, it remains until the true end.
    reg reg_initialized; // First release edge completes capacity initialization only.
    reg [11:0] cnt_rx_credit; // Receive credit only tracks actual firmware reads.
    wire storage_rstn, storage_ready, storage_valid; // Same-domain storage reset and real read/write handshake qualifications.
    wire [31:0] storage_word; // Full data word from the actual SRAM cache.
    wire [C_COUNT_WIDTH-1:0] storage_count; // Accurate FIFO occupancy including background flow control.
    wire [11:0] rx_count; // Unified zero-extended 12-bit local capacity occupancy.
    wire [5:0] header_count; // Recovery of 1 to 32 payload words from 5-bit line length.
    wire [12:0] required_capacity; // Full admission calculation of current occupancy plus new message length.
    wire flag_boundary, flag_uart, flag_stream0, flag_all; // Classification and stream applicability only at message boundaries.
    wire flag_credit_kind, flag_request_kind, flag_response_kind; // Control-message classification independent of payload receive enable.
    wire flag_admit_error, flag_payload, flag_drop_now, flag_write_error; // Full-message admission and current payload actual acceptance conditions.
    wire flag_consume; // Real firmware read event in the current cycle.
    assign storage_rstn = i_rstn && !i_stream_reset && reg_initialized; // Release the first edge to continue draining storage and forbid borrowing uninitialized qualification.
    assign rx_count = {{(12-C_COUNT_WIDTH){1'b0}}, storage_count}; // Do not leak physical padding or output-cache occupancy.
    assign header_count = {1'b0, i_word[31:27]} + 6'd1; // Length zero is legal and means a single payload word.
    assign required_capacity = {1'b0, rx_count} + {7'd0, header_count}; // Admission uses pre-cycle space and does not borrow same-cycle firmware reads.
    assign flag_boundary = i_rstn && i_word_valid && (cnt_remaining == 6'd0); // Words inside a message may not be reinterpreted as headers even if they look like one.
    assign flag_uart = i_word[5:2] == 4'd1; // Only the standard defined UART message class field is used.
    assign flag_stream0 = i_word[11:9] == 3'd0; // This product only defines stream zero.
    assign flag_all = flag_stream0 || i_word[12]; // All-stream reset also applies to implemented stream zero.
    assign o_header = flag_boundary && flag_uart && (i_word[8:6] == 3'd0); // Reserved bits do not participate in header matching; all streams continue length drain.
    assign flag_credit_kind = flag_boundary && flag_uart && (i_word[8:6] == 3'd1) && flag_stream0; // Remote credit only applies to implemented stream zero.
    assign flag_request_kind = flag_boundary && flag_uart && (i_word[8:6] == 3'd6) && flag_all; // Reset-request events are still passed upward during stream reset.
    assign flag_response_kind = flag_boundary && flag_uart && (i_word[8:6] == 3'd7) && flag_all; // Responses are not lost when the local stream remains in reset.
    assign flag_admit_error = o_header && flag_stream0 && storage_rstn && i_channel4_enabled && (required_capacity > C_CAPACITY); // Messages exceeding the published capacity are rejected at the header.
    assign flag_payload = i_rstn && i_word_valid && (cnt_remaining != 6'd0); // Only subsequent words after tracking begin are payload.
    assign flag_drop_now = reg_drop || !storage_rstn || !i_channel4_enabled; // Reset or channel offline immediately blocks writes and holds the whole message as discarded.
    assign flag_write_error = flag_payload && !flag_drop_now && !storage_ready; // Correctly reserved messages should never see unwritable storage.
    assign o_payload_write = flag_payload && !flag_drop_now && storage_ready; // Real write event is the only entry into SRAM.
    assign o_payload_discard = flag_payload && !o_payload_write; // Explicitly observe each line payload that did not enter the buffer.
    assign o_done = flag_payload && (cnt_remaining == 6'd1); // Accept and discard cases must both accurately drain the original length.
    assign o_fw_valid = storage_rstn && storage_valid; // Firmware may read already-cached data while Channel4 is offline.
    assign o_fw_word = o_fw_valid ? storage_word : 32'd0; // Reset and empty queue do not expose stale SRAM words.
    assign flag_consume = o_fw_valid && i_fw_ready; // Empty reads and prefetches do not return remote credit.
    assign o_rx_fill = storage_rstn ? rx_count : 12'd0; // Current reset input immediately hides old ownership prior to clear.
    assign o_initialized = storage_rstn; // Normal receive qualification is available only after the release and initialization phase.
    assign o_rx_counter = storage_rstn ? cnt_rx_credit : 12'd0; // Credit remains zero during stream reset.
    assign o_credit_available = storage_rstn && i_channel4_enabled; // Only provide a snapshot qualification here; no update throttling is defined.
    assign o_remaining = i_rstn ? cnt_remaining : 6'd0; // Global reset clears drain state, while stream reset preserves in-progress drain.
    assign o_dropping = i_rstn && (cnt_remaining != 6'd0) && flag_drop_now; // Explicitly show current activity message discard qualification.
    assign o_credit_valid = flag_credit_kind && storage_rstn; // Absolute credit can be delivered to send side when initialized even if the channel is offline.
    assign o_credit_value = o_credit_valid ? i_word[31:20] : 12'd0; // No invalid field is emitted when credit is not recognized.
    assign o_reset_request = flag_request_kind; // Reset request is decoded here but not executed in this block.
    assign o_request_all = flag_request_kind && i_word[12]; // Preserve request scope for upper-layer matching.
    assign o_reset_response = flag_response_kind; // All status responses are handled by upper reset state machine.
    assign o_response_all = flag_response_kind && i_word[12]; // Preserve response scope for upper-layer matching.
    assign o_response_status = flag_response_kind ? i_word[15:13] : 3'd0; // Preserve non-success encodings for upper-layer interpretation without misreporting success.
    assign o_other_valid = flag_boundary && !(o_header || flag_credit_kind || flag_request_kind || flag_response_kind); // Pass through unhandled single-word messages to upper DL dispatcher.
    assign o_other_word = o_other_valid ? i_word : 32'd0; // Do not leak active transport payload to other message interpreters.
    assign o_error = flag_admit_error || flag_write_error; // Offline and stream-reset discards do not generate capacity diagnostics.
    generate // Illegal logic capacity cannot continue with bit-width truncation.
        if ((C_RX_DEPTH < 1) || (C_RX_DEPTH > 4095)) begin : gen_invalid // Explicitly reject undeclared receive depth.
            dl_uart_rx_parameters_invalid Invalid_Inst (); // Force elaboration failure for illegal configuration.
        end // End of receive logic capacity parameter check.
    endgenerate // End of UART receive configuration legality structure.
    always @(posedge i_clk) begin // Independently track remaining payload words in the line transport.
        if (!i_rstn) cnt_remaining <= 6'd0; // Only global reset clears message boundary state.
        else if (o_header) cnt_remaining <= header_count; // Header only establishes length and does not enter payload storage.
        else if (flag_payload) cnt_remaining <= cnt_remaining - 6'd1; // Each valid subsequent word accurately reduces remaining length.
    end // End of message drain counter.
    always @(posedge i_clk) begin // Independently latch the discard decision for one transport.
        if (!i_rstn) reg_drop <= 1'b0; // Global reset clears old discard state.
        else if (o_header) reg_drop <= !storage_rstn || !i_channel4_enabled || !flag_stream0 || flag_admit_error; // The message admission decision is latched at the header edge.
        else if (o_done) reg_drop <= 1'b0; // Old message must drain to the true end before new message qualification returns.
        else if ((cnt_remaining != 6'd0) && (flag_drop_now || flag_write_error)) reg_drop <= 1'b1; // Even without input words, offline or reset conditions latch until the end.
    end // End of whole-message discard state.
    always @(posedge i_clk) begin // Independently record initialization completion after stream reset release.
        if (!i_rstn || i_stream_reset) reg_initialized <= 1'b0; // Local or global reset revokes receive credit qualification.
        else reg_initialized <= 1'b1; // The first release edge establishes initial capacity; next cycle permits receive.
    end // End of receive-credit initialization register.
    always @(posedge i_clk) begin // Independently implement the 12-bit modulo receive credit counter.
        if (!i_rstn || i_stream_reset) cnt_rx_credit <= 12'd0; // Reset clears old data without returning normal read credit.
        else if (!reg_initialized) cnt_rx_credit <= C_INITIAL_CREDIT; // Initial published capacity equals exact receive buffer depth.
        else if (flag_consume) cnt_rx_credit <= cnt_rx_credit + 12'd1; // Only real firmware read handshake increases credit by one word, naturally wrapping modulo.
    end // End of receive credit counter.
    upli_receive_storage #( // Reuse actual synchronous FIFO and authorized SRAM storage mapping.
        .C_DEPTH(C_RX_DEPTH), .C_DATA_WIDTH(32), .C_COUNT_WIDTH(C_COUNT_WIDTH) // Exact logical depth independent of physical macro padding.
    ) Storage_Inst ( // Receive side stores only opaque payload, not the transport header.
        .i_clk(i_clk), .i_rstn(storage_rstn), // SRAM control still uses the same raw clock and synchronous reset.
        .i_write_valid(o_payload_write), .i_write_data(i_word), .o_write_ready(storage_ready), // Only actual payload values enter the back end.
        .i_read_ready(flag_consume), .o_read_valid(storage_valid), .o_read_data(storage_word), // Firmware reads from actual storage cache instead of bypass expectations.
        .o_count(storage_count) // Publish full capacity ownership for header admission checks.
    ); // End of actual SRAM receive data path.
endmodule // End of UART stream0 receive and firmware credit-return implementation.
