import types::*;

module modbus_rtu #(
	parameter [7:0]   SLAVE_ADDR = 2,
	parameter integer PRESCALER  = 100 // UART prescaler
)
(
	input  logic        clk,
	input  logic        rst,
	// Interface with UART
	input  logic        rxv,
	input  logic [7:0]  rxd,
	output logic        txv,
	output logic [7:0]  txd,
	input  logic        cts,   // Clear to send from UART
	output logic        empty, // TX buffer empty flag
	// Received data
	output logic [15:0] data,
	output logic [15:0] addr,
	output logic        valid,
	// Second memory port access
	input  logic [15:0] ext_d,
	input  logic [15:0] ext_a,
	input  logic        ext_v,
	output logic [15:0] ext_q
);

localparam integer RAM_DEPTH = 10;

crc_master_t    state;
modbus_packet_t pkt;

logic        ex_send;
logic        crc_done;
logic        fsm_rst;

logic        hr_write;
logic [15:0] hr_addr;
logic [15:0] hr_data;

logic        rx_buf_empty;
logic        rx_buf_read;
logic [15:0] rx_buf_data;

logic [15:0] hr_a_rx;
logic [15:0] hr_d_rx;
logic        hr_v_rx;

logic [15:0] hr_a_tx;

logic        crc_req;
logic [7:0]  crc_dat_tx;
logic [7:0]  crc_dat_rx;
logic        crc_req_rx;
logic        crc_req_tx;
logic [7:0]  crc_di;
logic        crc_vi;
logic [15:0] crc_value;

ram_if #(RAM_DEPTH,16) hr(.*);
true_dpram_sclk #(RAM_DEPTH,16) HR_RAM ( .mem_if ( hr ) );

modbus_rx #(
	.SLAVE_ADDR ( SLAVE_ADDR ),
	.PRESCALER	( PRESCALER ) )
modbus_rx_inst (
	.clk          ( clk ),
	.rst          ( rst ),

	.rxv          ( rxv ),
	.rxd          ( rxd ),

	.pkt          ( pkt ),

	.resp_send    ( resp_send ),
	.ex_send      ( ex_send ),

	.hr_addr      ( hr_a_rx ),
	.hr_data      ( hr_d_rx ),
	.hr_valid     ( hr_v_rx ),
	.crc_rst      ( crc_rst_rx ),
	.crc_req      ( crc_req_rx ),
	.crc_dat      ( crc_dat_rx ),
	.crc_value    ( crc_value ),
	.fsm_rst      ( fsm_rst )
);

modbus_tx #(
	.SLAVE_ADDR ( SLAVE_ADDR ) )
modbus_tx_inst (
	.clk        ( clk ),
	.rst        ( rst ),

	.txv        ( txv ),
	.txd        ( txd ),
	
	.cts        ( cts ),
	.empty      ( empty ),

	.pkt        ( pkt ),

	.resp_send  ( resp_send ),
	.ex_send    ( ex_send ),

	.fsm_rst    ( fsm_rst ),
	.crc_rst    ( crc_rst_tx ),
	.crc_req    ( crc_req_tx ),
	.crc_dat    ( crc_dat_tx ),
	.crc_value  ( crc_value ),
	.crc_done   ( crc_done ),
	.hr_a       ( hr_a_tx ),
	.hr_q       ( hr.q_a )
);

crc_16 crc_16_inst (
	.clk  ( clk ),
	.rst  ( crc_rst ),

	.vi   ( crc_vi ),
	.di   ( crc_di ),

	.crc  ( crc_value ),
	.done ( crc_done )
);

logic state_rst;
assign state_rst = fsm_rst || rst;

always @ ( posedge clk ) begin
	if ( state_rst ) state <= rx;
	else if ( resp_send ) state <= tx;
end

assign crc_di    = ( state == rx ) ? crc_dat_rx : crc_dat_tx;
assign crc_vi    = ( state == rx ) ? crc_req_rx : crc_req_tx;
assign crc_rst = ( ( state == rx ) ? crc_rst_rx : crc_rst_tx ) || rst;

assign hr.clk = clk;
assign hr.rst = rst;

assign hr.a_a = ( state == rx ) ? hr_a_rx : hr_a_tx;
assign hr.d_a = hr_d_rx;
assign hr.w_a = hr_v_rx;

assign hr.a_b = ext_a;
assign hr.d_b = ext_d;
assign hr.w_b = ext_v;
assign ext_q  = hr.q_b;
assign data  = hr.d_a;
assign addr  = hr.a_a;
assign valid = hr.w_a;

endmodule