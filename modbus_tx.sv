import types::*;

module modbus_tx #(
	parameter [7:0] SLAVE_ADDR = 8'h02 )
(
	input  logic        clk,
	input  logic        rst,

	output logic        txv,
	output logic [7:0]  txd,

	input  logic        cts,
	output logic        empty,

	input modbus_packet_t pkt,
	input  logic        resp_send,
	input  logic        ex_send,
	input  logic        ex_code,

	output logic        crc_req,
	output logic [7:0]  crc_dat,

	input  logic        crc_done,
	input  logic [15:0] crc_value,
	output logic        crc_rst,
	output logic        fsm_rst,

	output logic [15:0] hr_a,
	input  logic [15:0] hr_q
);

logic resp_sent;
logic tx_start;
logic lsbyte;
logic [15:0] crc_value_tx;

// Buffer that holds response message until UART module sends it
fifo_if #(8,8) tx_buf (.*);
fifo_dc #(8,8) TX_MSG_BUFFER ( .ifc ( tx_buf ) );

assign tx_buf.r_v = (cts && !tx_buf.empty);

assign crc_req = tx_buf.w_v;
assign crc_dat = tx_buf.w_d;

assign tx_buf.r_clk = clk;
assign tx_buf.r_rst = fsm_rst;
assign tx_buf.w_clk = clk;
assign tx_buf.w_rst = fsm_rst;

assign txd = tx_buf.r_q;

always @ (posedge clk) begin
	if (fsm_rst) begin
		txv <= 1'b0;
	end
	else begin
		txv <= tx_buf.r_v;
	end
end

always @ (posedge clk) begin
	if (rst) tx_start <= 0;
	else tx_start <= resp_send;
end

modbus_fsm_tx_t TX_FSM;

always @ (posedge clk) begin
	if (fsm_rst) begin
		TX_FSM     <= TX_IDLE_S;
		tx_buf.w_d <= 0;
		lsbyte     <= 0;
		resp_sent  <= 0;
		crc_rst    <= 0;
		tx_buf.w_v <= 0;
	end
	else begin
		case (TX_FSM)
			TX_IDLE_S : begin
				if (tx_start) begin
					TX_FSM     <= (ex_send) ? TX_EX_CODE_S : TX_FUNC_CODE_S;
					tx_buf.w_d <= SLAVE_ADDR;
					tx_buf.w_v <= 1;
					hr_a       <= pkt.start_addr_r;
				end 
				else tx_buf.w_v <= 0;
			end
			TX_EX_CODE_S : begin
				if (crc_done) begin
					tx_buf.w_d <= pkt.ex_code;
					tx_buf.w_v <= 1;
					TX_FSM     <= TX_CRC_HI_S;
				end
				else tx_buf.w_v <= 0;
			end
			TX_FUNC_CODE_S : begin
				if (crc_done) begin
					tx_buf.w_d <= pkt.func_code;
					tx_buf.w_v <= 1;
					case ( pkt.func_code ) 
						READ_HOLDING_REGISTERS        : TX_FSM <= TX_BYTE_COUNT_S;
						READ_WRITE_MULTIPLE_REGISTERS : TX_FSM <= TX_BYTE_COUNT_S;
						WRITE_SINGLE_REGISTER         : TX_FSM <= TX_START_ADDR_HI_S;
						WRITE_MULTIPLE_REGISTERS      : TX_FSM <= TX_START_ADDR_HI_S;
						READ_EXCEPTION_STATUS         : TX_FSM <= TX_DATA_S;
						default                       : TX_FSM <= TX_IDLE_S;
					endcase
				end
				else tx_buf.w_v <= 0;
			end
			TX_BYTE_COUNT_S : begin
				if (crc_done) begin
					tx_buf.w_v <= 1;
					tx_buf.w_d <= ( pkt.func_code == READ_WRITE_MULTIPLE_REGISTERS ) ? ( pkt.quantity_r ) << 1 : ( pkt.byte_count );
					TX_FSM     <= TX_DATA_S;
				end
				else tx_buf.w_v <= 0;
			end
			TX_START_ADDR_HI_S : begin
				if (crc_done) begin
					tx_buf.w_v <= 1;
					tx_buf.w_d <= pkt.start_addr_w[15-:8];
					TX_FSM     <= TX_START_ADDR_LO_S;
				end
				else tx_buf.w_v <= 0;
			end
			TX_START_ADDR_LO_S : begin
				if (crc_done) begin
					tx_buf.w_v <= 1;
					tx_buf.w_d <= pkt.start_addr_w[7-:8];
					case (pkt.func_code)
						WRITE_MULTIPLE_REGISTERS : TX_FSM <= TX_QUANTITY_HI_S;
						WRITE_SINGLE_REGISTER    : TX_FSM <= TX_DATA_S;
						default                  : TX_FSM <= TX_IDLE_S;
					endcase
				end
				else tx_buf.w_v <= 0;
			end
			TX_QUANTITY_HI_S : begin
				if (crc_done) begin
					tx_buf.w_v <= 1;
					tx_buf.w_d <= pkt.quantity_w[15-:8];
					TX_FSM     <= TX_QUANTITY_LO_S;
				end
				else tx_buf.w_v <= 0;
			end
			TX_QUANTITY_LO_S : begin
				if (crc_done) begin
					tx_buf.w_v   <= 1;
					tx_buf.w_d   <= pkt.quantity_w[7-:8];
					TX_FSM       <= TX_CRC_HI_S;
					crc_value_tx <= crc_value;
				end
				else tx_buf.w_v <= 0;
			end
			TX_DATA_S : begin
				if (crc_done) begin
					if (lsbyte) hr_a <= hr_a + 1;
					tx_buf.w_v <= 1;
					tx_buf.w_d <= (lsbyte) ? hr_q[7:0] : hr_q[15-:8];
					lsbyte     <= ~lsbyte;
					if (pkt.func_code == WRITE_SINGLE_REGISTER && lsbyte) TX_FSM <= TX_CRC_HI_S;
					else if ((hr_a == ( pkt.start_addr_r + pkt.quantity_r - 1)) && lsbyte) TX_FSM <= TX_CRC_HI_S;
				end
				else tx_buf.w_v <= 0;
			end
			TX_CRC_HI_S : begin
				if (crc_done) begin
					tx_buf.w_v <= 1;
					TX_FSM     <= TX_CRC_LO_S;
					tx_buf.w_d <= crc_value[7:0];
					crc_value_tx <= crc_value;
				end
				else tx_buf.w_v <= 0;
			end
			TX_CRC_LO_S : begin
				if (crc_done) begin
					tx_buf.w_v <= 1;
					tx_buf.w_d <= crc_value_tx[15-:8];
					crc_rst    <= 1;
					resp_sent  <= 1;
				end
				else tx_buf.w_v <= 0;
			end
		endcase
	end
end

always @ (posedge clk) begin
	if (rst) fsm_rst <= 1;
	else begin
		if (resp_sent && tx_buf.empty) fsm_rst <= 1;
		else fsm_rst <= 0;
	end
end

endmodule