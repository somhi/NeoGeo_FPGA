//
// data_io_neogeo.v
//
// data_io for the MiST board (Neo-Geo CD extensions)
// https://github.com/mist-devel/mist-board
//
// Copyright (c) 2014 Till Harbaum <till@harbaum.org>
// Copyright (c) 2015-2017 Sorgelig
// Copyright (c) 2019-2023 György Szombathelyi
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
///////////////////////////////////////////////////////////////////////

module data_io_neogeo
(
	input             clk_sys,

	// Global SPI clock from ARM. 24MHz
	input             SPI_SCK,
	input             SPI_SS2,
	input             SPI_DI,
	output reg        SPI_DO,
	input             reset,

	// CD IO
	input       [1:0] CD_SPEED,
	output reg        CD_DATA_DOWNLOAD,
	output reg        CD_DATA_WR,
	input             CD_DATA_WR_READY,
	output reg        CDDA_WR,
	input             CDDA_WR_READY,
	output reg [15:0] CD_DATA_DIN,
	output     [11:1] CD_DATA_ADDR,
	output reg [39:0] CDD_STATUS_IN,
	output reg        CDD_STATUS_LATCH,
	input      [39:0] CDD_COMMAND_DATA,
	input             CDD_COMMAND_SEND
);

///////////////////////////////   DOWNLOADING   ///////////////////////////////

localparam CD_STAT_GET      = 8'h60;
localparam CD_STAT_SEND     = 8'h61;
localparam CD_COMMAND_GET   = 8'h62;
localparam CD_DATA_SEND     = 8'h64;
localparam CD_AUDIO_SEND    = 8'h65;

// SPI receiver IO -> FPGA

reg       spi_receiver_strobe_r = 0;
reg       spi_transfer_end_r = 1;
reg [7:0] spi_byte_in;
reg [7:0] cmd;
reg [2:0] bit_cnt;
reg [7:0] byte_cnt;

// data_io has its own SPI interface to the io controller
// Read at spi_sck clock domain, assemble bytes for transferring to clk_sys
always@(posedge SPI_SCK or posedge SPI_SS2) begin : data_input

	reg  [6:0] sbuf;

	if(SPI_SS2) begin
		spi_transfer_end_r <= 1;
		bit_cnt <= 0;
		cmd <= 0;
		byte_cnt <= 0;
	end else begin
		spi_transfer_end_r <= 0;

		bit_cnt <= bit_cnt + 1'd1;

		if(bit_cnt != 7)
			sbuf[6:0] <= { sbuf[5:0], SPI_DI };

		// finished reading a byte, prepare to transfer to clk_sys
		if(bit_cnt == 7) begin
			if (~&byte_cnt) byte_cnt <= byte_cnt + 1'd1;
			if (cmd == 0) cmd <= { sbuf, SPI_DI};
			spi_byte_in <= { sbuf, SPI_DI};
			spi_receiver_strobe_r <= ~spi_receiver_strobe_r;
		end
	end
end

reg cd_command_pending = 0;

wire [7:0] cd_status = {2'd0, reset, CD_SPEED, cd_command_pending, CDDA_WR_READY, CD_DATA_WR_READY};

// SPI transmitter FPGA -> IO
always@(negedge SPI_SCK or posedge SPI_SS2) begin : data_output

	if(SPI_SS2) begin
		SPI_DO <= 1'bZ;
	end else begin
		if(cmd == CD_COMMAND_GET)
			SPI_DO <= CDD_COMMAND_DATA[{ byte_cnt - 1'd1, ~bit_cnt }];
		else
			SPI_DO <= cd_status[~bit_cnt];
	end
end

reg [11:0] abyte_cnt;   // counts bytes
assign CD_DATA_ADDR = abyte_cnt[11:1] - 1'd1;

always @(posedge clk_sys) begin

	reg        spi_receiver_strobe;
	reg        spi_transfer_end;
	reg        spi_receiver_strobeD;
	reg        spi_transfer_endD;
	reg  [7:0] acmd;

	CDD_STATUS_LATCH <= 0;
	CD_DATA_WR <= 0;
	CDDA_WR <= 0;

	if (CDD_COMMAND_SEND)
		cd_command_pending <= 1;

	//synchronize between SPI and sys clock domains
	spi_receiver_strobeD <= spi_receiver_strobe_r;
	spi_receiver_strobe <= spi_receiver_strobeD;
	spi_transfer_endD <= spi_transfer_end_r;
	spi_transfer_end <= spi_transfer_endD;

	if (spi_transfer_end) begin
		abyte_cnt <= 0;
		CD_DATA_DOWNLOAD <= 0;
	end else if (spi_receiver_strobeD ^ spi_receiver_strobe) begin
		if(~&abyte_cnt) abyte_cnt <= abyte_cnt + 1'd1;

		if(abyte_cnt == 0) begin
			acmd <= spi_byte_in;
		end else begin
			case (acmd)
				CD_COMMAND_GET:
					cd_command_pending <= 0;

				CD_STAT_SEND:
					if (abyte_cnt == 1) CDD_STATUS_IN[ 7: 0] <= spi_byte_in; else
					if (abyte_cnt == 2) CDD_STATUS_IN[15: 8] <= spi_byte_in; else
					if (abyte_cnt == 3) CDD_STATUS_IN[23:16] <= spi_byte_in; else
					if (abyte_cnt == 4) CDD_STATUS_IN[31:24] <= spi_byte_in; else
					if (abyte_cnt == 5) begin
						CDD_STATUS_IN[39:32] <= spi_byte_in;
						CDD_STATUS_LATCH <= 1;
					end

				CD_DATA_SEND: begin
					CD_DATA_DOWNLOAD <= 1;
					if (!abyte_cnt[0]) begin
						CD_DATA_DIN[15:8] <= spi_byte_in;
						CD_DATA_WR <= 1;
					end else
						CD_DATA_DIN[7:0] <= spi_byte_in;
				end

				CD_AUDIO_SEND: begin
					if (!abyte_cnt[0]) begin
						CD_DATA_DIN[15:8] <= spi_byte_in;
						CDDA_WR <= 1;
					end else
						CD_DATA_DIN[7:0] <= spi_byte_in;
				end

			endcase
		end
	end

end

endmodule
