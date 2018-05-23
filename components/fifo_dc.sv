`timescale 1 ns / 1 ps

interface fifo_if
#( 
	parameter ADDR_WIDTH = 16,
	parameter DATA_WIDTH = 16 )
();
logic                  w_rst;
logic                  w_clk;
logic                  w_v  ;
logic [DATA_WIDTH-1:0] w_d  ;

logic                  r_rst;
logic                  r_clk;
logic                  r_v  ;
logic [DATA_WIDTH-1:0] r_q  ;

logic                  full ;
logic                  empty;

modport fifo ( input w_rst, w_clk, w_v, w_d, r_rst, r_clk, r_v, output r_q, full, empty );
modport sys  ( input r_q, full, empty, output w_rst, w_clk, w_v, w_d, r_rst, r_clk, r_v );
modport tb   ( output r_q, full, empty, w_rst, w_clk, w_v, w_d, r_rst, r_clk, r_v );

endinterface

module fifo_dc #(
	parameter ADDR_WIDTH = 16,
	parameter DATA_WIDTH = 16
)
(
	fifo_if.fifo ifc
);

reg [ADDR_WIDTH-1:0]	wr_addr;
reg [ADDR_WIDTH-1:0]	wr_addr_gray;
reg [ADDR_WIDTH-1:0]	wr_addr_gray_rd;
reg [ADDR_WIDTH-1:0]	wr_addr_gray_rd_r;
reg [ADDR_WIDTH-1:0]	rd_addr;
reg [ADDR_WIDTH-1:0]	rd_addr_gray;
reg [ADDR_WIDTH-1:0]	rd_addr_gray_wr;
reg [ADDR_WIDTH-1:0]	rd_addr_gray_wr_r;

function [ADDR_WIDTH-1:0] gray_conv;
	input [ADDR_WIDTH-1:0] in;
	begin
		gray_conv = {in[ADDR_WIDTH-1], in[ADDR_WIDTH-2:0] ^ in[ADDR_WIDTH-1:1]};
	end
endfunction

always @ ( posedge ifc.w_clk ) begin
	if ( ifc.w_rst ) begin
		wr_addr <= 0;
		wr_addr_gray <= 0;
	end else if ( ifc.w_v ) begin
		wr_addr <= wr_addr + 1'b1;
		wr_addr_gray <= gray_conv(wr_addr + 1'b1);
	end
end

// synchronize read address to write clock domain
always @ ( posedge ifc.w_clk ) begin
	if ( ifc.w_rst ) begin
		rd_addr_gray_wr   <= 0;
		rd_addr_gray_wr_r <= 0;
	end
	else begin
		rd_addr_gray_wr   <= rd_addr_gray;
		rd_addr_gray_wr_r <= rd_addr_gray_wr;
	end
end

always @ ( posedge ifc.w_clk )
	if ( ifc.w_rst )
		ifc.full <= 0;
	else if ( ifc.w_v )
		ifc.full <= gray_conv ( wr_addr + 2 ) == rd_addr_gray_wr_r;
	else
		ifc.full <= ifc.full & ( gray_conv ( wr_addr + 1'b1 ) == rd_addr_gray_wr_r);

always @ ( posedge ifc.r_clk ) begin
	if ( ifc.r_rst ) begin
		rd_addr      <= 0;
		rd_addr_gray <= 0;
	end else if ( ifc.r_v ) begin
		rd_addr      <= rd_addr + 1'b1;
		rd_addr_gray <= gray_conv(rd_addr + 1'b1);
	end
end

// synchronize write address to read clock domain
always @ ( posedge ifc.r_clk ) begin
	if ( ifc.r_rst ) begin
		wr_addr_gray_rd   <= 0;
		wr_addr_gray_rd_r <= 0;
	end
	else begin
		wr_addr_gray_rd   <= wr_addr_gray;
		wr_addr_gray_rd_r <= wr_addr_gray_rd;
	end
end

always @( posedge ifc.r_clk )
	if ( ifc.r_rst )
		ifc.empty <= 1'b1;
	else if ( ifc.r_v )
		ifc.empty <= gray_conv ( rd_addr + 1 ) == wr_addr_gray_rd_r;
	else
		ifc.empty <= ifc.empty & ( gray_conv ( rd_addr ) == wr_addr_gray_rd_r );

// generate dual clocked memory
reg [DATA_WIDTH-1:0] mem[(1<<ADDR_WIDTH)-1:0];

always @( posedge ifc.r_clk )
	if ( ifc.r_v )
		ifc.r_q <= mem[ rd_addr ];

always @(posedge ifc.w_clk )
	if (ifc.w_v)
		mem[wr_addr] <= ifc.w_d;
endmodule