
import types::*;

module modbus_rx #(
	parameter [7:0]   SLAVE_ADDR = 8'h02,
	parameter integer PRESCALER  = 100
)
 (
	input  logic         clk,
	input  logic         rst,

	input  logic         rxv,
	input  logic [7:0]   rxd,

	output modbus_packet_t pkt,
	output logic        resp_send,
	output logic        ex_send,

	output logic [15:0] hr_addr,
	output logic [15:0] hr_data,
	output logic        hr_valid,

	output logic        crc_rst,
	output logic        crc_req,
	output logic [7:0]  crc_dat,
	input  logic [15:0] crc_value,

	input  logic        fsm_rst
);

parameter integer TIMEOUT = 40; // maximum uart bit intervals between received bytes in one packet (~4 bytes)
parameter integer TIMEOUT_CLOCK_TICKS = TIMEOUT*PRESCALER; // clock ticks between 
parameter integer RESET_NP_TICKS      = PRESCALER*30; // ~3 bytes

logic [$clog2(TIMEOUT_CLOCK_TICKS)-1:0] to_ctr;
logic [$clog2(RESET_NP_TICKS)-1:0]      np_ctr;
logic        np;

logic [7:0]  byte_ctr;
logic [7:0]  crc_hi;
logic [15:0] crc_value_rx;
logic [7:0]  crc_lo_rx;
logic        buf_req;
logic        crc_ok;

// Buffer that holds received data until crc check is passed
fifo_if #(8,16) rx_buf(.*);
fifo_dc #(8,16) RX_BUFFER ( .ifc ( rx_buf ) );

assign rx_buf.w_rst = fsm_rst;
assign rx_buf.w_clk = clk;
assign rx_buf.r_rst = fsm_rst;
assign rx_buf.r_clk = clk;

assign crc_req = rxv; // CRC-16 module takes 16 clock cycles to update CRC-16 for each new byte
assign crc_dat = rxd;

// 
always @ (posedge clk) begin
	if (rst_np) np <= 0;
	else if (rxv) np <= 1;
end

always @ (posedge clk) begin
	if (rst) np_ctr <= 0;
	else np_ctr <= (rxv) ? 0 : np_ctr + 1;
end

assign rst_np  = (np_ctr == RESET_NP_TICKS); // deassert np if no new bytes were received for RESET_NP_TICKS parameter
assign crc_rst = (np_ctr == RESET_NP_TICKS || crc_ok);

// Request FSM
modbus_fsm_rx_t RX_FSM;

always @ (posedge clk) begin
	if (fsm_rst) begin 
		RX_FSM   <= RX_IDLE_S;
		to_ctr   <= 0;
		crc_ok   <= 0;
		byte_ctr <= 0;
		ex_send  <= 0;
	end
	else begin
		if (!rxv && RX_FSM != RX_IDLE_S) begin
			to_ctr     <= to_ctr + 1; // if the FSM isn't in idle state, increment timeout counter
			rx_buf.w_v <= 0;
			if (to_ctr == TIMEOUT_CLOCK_TICKS) begin 
				pkt.ex_code <= UNEXPECTED_PACKET_END;
				ex_send <= 1;
				RX_FSM  <= RX_IDLE_S;
				//crc_rst <= 1;
			end
		end
		if (rxv) begin
			to_ctr <= 0; // как пришел новый байт - счетчик времени ожидания обнуляется
			case (RX_FSM)
				RX_IDLE_S : begin // if the first received byte in ~3 uart frames is our slave id, start the state machine
					if (!np && (rxd == SLAVE_ADDR)) RX_FSM <= RX_FUNC_CODE_S;
				end
				RX_FUNC_CODE_S : begin
					pkt.func_code <= func_code_t'(rxd);
					case (rxd)
						READ_HOLDING_REGISTERS        : RX_FSM <= RX_START_ADDR_R_HI_S;
						WRITE_SINGLE_REGISTER         : RX_FSM <= RX_START_ADDR_W_HI_S;
						WRITE_MULTIPLE_REGISTERS      : RX_FSM <= RX_START_ADDR_W_HI_S;
						READ_WRITE_MULTIPLE_REGISTERS : RX_FSM <= RX_START_ADDR_R_HI_S;
						default : begin
							ex_send <= 1'b1;
							pkt.ex_code <= BAD_FUNC_CODE;
							RX_FSM  <= RX_IDLE_S;
						end
					endcase
				end
				RX_START_ADDR_R_HI_S : begin
					pkt.start_addr_r[15-:8] <= rxd;
					RX_FSM <= RX_START_ADDR_R_LO_S;
				end
				RX_START_ADDR_R_LO_S : begin
					pkt.start_addr_r[7-:8] <= rxd;
					case ( pkt.func_code )
						READ_HOLDING_REGISTERS        : RX_FSM <= RX_QUANTITY_R_HI_S;
						READ_WRITE_MULTIPLE_REGISTERS : RX_FSM <= RX_QUANTITY_R_HI_S;
					endcase
				end
				RX_START_ADDR_W_HI_S : begin
					pkt.start_addr_w[15-:8] <= rxd;
					RX_FSM <= RX_START_ADDR_W_LO_S;
				end
				RX_START_ADDR_W_LO_S : begin
					pkt.start_addr_w[7-:8] <= rxd;
					case ( pkt.func_code )
						WRITE_MULTIPLE_REGISTERS      : RX_FSM <= RX_QUANTITY_W_HI_S;
						READ_WRITE_MULTIPLE_REGISTERS : RX_FSM <= RX_QUANTITY_W_HI_S;
						WRITE_SINGLE_REGISTER : begin
							RX_FSM         <= RX_DATA_S;
							pkt.quantity_w <= 1;
							pkt.byte_count <= 2;
						end
					endcase
				end
				RX_QUANTITY_R_HI_S : begin
					pkt.quantity_r[15-:8] <= rxd;
					RX_FSM <= RX_QUANTITY_R_LO_S;
				end
				RX_QUANTITY_R_LO_S : begin
					pkt.quantity_r[7-:8] <= rxd;
					case (pkt.func_code) 
						READ_WRITE_MULTIPLE_REGISTERS : RX_FSM <= RX_START_ADDR_W_HI_S;
						READ_HOLDING_REGISTERS        : begin
							RX_FSM <= RX_CRC_HI_S;
							pkt.byte_count <= { pkt.quantity_r[15-:8], rxd[7:0] } << 1;
						end
					endcase
				end
				RX_QUANTITY_W_HI_S : begin
					pkt.quantity_w[15-:8] <= rxd;
					RX_FSM <= RX_QUANTITY_W_LO_S;
				end
				RX_QUANTITY_W_LO_S : begin
					pkt.quantity_w[7-:8] <= rxd;
					case (pkt.func_code) 
						WRITE_MULTIPLE_REGISTERS      : RX_FSM <= RX_BYTE_COUNT_S;
						READ_WRITE_MULTIPLE_REGISTERS : RX_FSM <= RX_BYTE_COUNT_S;
					endcase
				end
				RX_BYTE_COUNT_S : begin
					pkt.byte_count <= rxd;
					RX_FSM     <= RX_DATA_S;
				end
				RX_DATA_S : begin
					rx_buf.w_v <= byte_ctr[0]; // Write received bytes to 16-bit wide FIFO when byte counter is odd (every 2 bytes)
					if (byte_ctr[0]) rx_buf.w_d[7:0] <= rxd; // place received byte in lsbyte of msbyte position
					else rx_buf.w_d[15-:8] <= rxd;
					byte_ctr <= byte_ctr + 1;
					if (byte_ctr == pkt.byte_count - 1) RX_FSM <= RX_CRC_HI_S;
				end
				RX_CRC_HI_S : begin
					crc_value_rx <= crc_value;
					rx_buf.w_v   <= 0;
					crc_lo_rx    <= rxd;
					RX_FSM       <= RX_CRC_LO_S;
				end
				RX_CRC_LO_S : begin
					if ({rxd[7:0], crc_lo_rx[7:0]} == crc_value_rx) begin
						crc_ok <= 1;
					end
					else begin
						pkt.ex_code <= BAD_CRC;
						ex_send <= 1;
					end
				end
			endcase
		end
	end
end

// This small FSM passes control to the response FSM
mem_fsm_t MEM_FSM;

always @ ( posedge clk ) begin
	if ( fsm_rst ) begin
		MEM_FSM <= MEM_IDLE_S;
		hr_valid <= 0;
		resp_send <= 0;
		buf_req <= 0;
	end
	else begin
		case ( MEM_FSM )
			MEM_IDLE_S : begin
				if (crc_ok || ex_send) begin         // request fsm arrerts either crc_ok or ex_send
					buf_req <= 1;                    // assert signal to read data from fifo, but check that fifo isn't empty
					hr_addr <= pkt.start_addr_w - 1; // sets starting addr for local RAM (minus one since there will be a +1 at next state) 
					MEM_FSM <= WRITE_REG_S;
				end
			end
			WRITE_REG_S : begin
				hr_addr  <= hr_addr + 1;   // increment address every clock tick
				hr_valid <= !rx_buf.empty; // and write received data to local RAM until all data is read out from FIFO
				if ( rx_buf.empty ) resp_send <= 1; // when all data is written to local ram, begin responding
			end
		endcase
	end
end

assign hr_data = rx_buf.r_q; // data read from FIFO is directrly written to local RAM
assign rx_buf.r_v = buf_req & (!rx_buf.empty); // check that fifo isn't empty when reading from it

endmodule
