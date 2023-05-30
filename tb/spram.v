module spram #(parameter ADDRWIDTH=8, DATAWIDTH=8, NUMWORDS=1<<ADDRWIDTH)
(
	input	                 clock,
	input	 [ADDRWIDTH-1:0] address,
	input	 [DATAWIDTH-1:0] data,
	input	                 wren,
	output reg [DATAWIDTH-1:0] q
);

reg [DATAWIDTH-1:0] mem[NUMWORDS];

always @(posedge clock) begin
	q <= mem[address];
	if (wren) mem[address] <= data;
end

endmodule
