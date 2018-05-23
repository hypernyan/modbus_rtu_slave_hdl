import types::*;

module crc_16 (
	input logic         clk,
	input logic         rst,

	input logic         vi,
	input logic [7:0]   di,

	output logic [15:0] crc,
	output logic        done,

	output CRC_FSM_t    CRC_FSM
);


localparam [15:0] CRC_POLY = 16'ha001;

logic        crc_lsb;
logic [15:0] crc_xor;
logic [2:0]  shifts;
logic [15:0] val;

assign crc = val;
assign crc_xor = val ^ di;

always @ ( posedge clk ) begin
	if ( rst ) begin
		val <= 0;
		crc_lsb <= 0;
		shifts  <= 0;
		done    <= 0;
		CRC_FSM <= CRC_INIT_S;
	end
	else begin
		case ( CRC_FSM )
			CRC_INIT_S : begin // инициализизация регистра CRC единицами
				shifts  <= 0;
				if ( vi ) begin
					CRC_FSM <= CRC_XOR_SHIFT_S;
					val <= 16'hffff ^ di;
				end
			end
			CRC_XOR_SHIFT_S : begin // XOR содержимого регистра CRC и нового байта, сдвиг вправо на 1 бит
				val <= val >> 1;
				crc_lsb <= val[0];
				CRC_FSM <= CRC_LSB_S;
			end
			CRC_LSB_S : begin // Сравнение lsb регистра CRC
				val      <= ( crc_lsb ) ? ( val ^ CRC_POLY ) : val;
				shifts   <= shifts + 1;
				CRC_FSM  <= ( shifts == 7 ) ? CRC_WAIT_S : CRC_XOR_SHIFT_S; // Если сдвиги повторялись 8 раз, ждем следующего байта
				done     <= ( shifts == 7 ) ? 1 : 0;
			end
			CRC_WAIT_S : begin
				done    <= 0;
				if ( vi ) begin
					CRC_FSM <= CRC_XOR_SHIFT_S;
					val <= val ^ di;
					shifts  <= 0;
				end
			end
		endcase
	end
end

endmodule