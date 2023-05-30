module NeoGeo_MiST(
	output        LED,
	output  [5:0] VGA_R,
	output  [5:0] VGA_G,
	output  [5:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        AUDIO_L,
	output        AUDIO_R,
	input         SPI_SCK,
	inout         SPI_DO,
	input         SPI_DI,
	input         SPI_SS2,
	input         SPI_SS3,
	input         SPI_SS4,
	input         CONF_DATA0,
	input         CLOCK_27,

	output [12:0] SDRAM_A,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nWE,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nCS,
	output  [1:0] SDRAM_BA,
	output        SDRAM_CLK,
	output        SDRAM_CKE
);

`include "build_id.v" 

wire [6:0] core_mod;

localparam CONF_STR = {
	"NEOGEO;;",
	"F,ROM,Load BIOS;",
	"F,NEO,Load Cart;",
	"O1,System Type,Console(AES),Arcade(MVS);",
	"O3,Video Mode,NTSC,PAL;",
	"O45,Scanlines,Off,25%,50%,75%;",
	"O6,Swap Joystick,Off,On;",
	"O7,Blending,Off,On;",
	"O8,[DIP] Settings,OFF,ON;",
	"O9,[DIP] Freeplay,OFF,ON;",
	"OA,[DIP] Freeze,OFF,ON;",
	"T0,Reset;",
	"V,v1.0.",`BUILD_DATE
};

wire  [1:0] scanlines = status[5:4];
wire        joyswap   = status[6];
wire        blend     = status[7];
wire  [1:0] orientation = 2'b00;
wire        rotate = 1'b0;
wire        oneplayer = 1'b0;
wire  [2:0] dipsw = status[10:8];
wire        systype = status[1];
wire        vmode = status[3];

assign LED = ~ioctl_downl;
assign SDRAM_CKE = 1; 

wire CLK_120M, CLK_24M;
wire pll_locked;
pll_mist pll(
	.inclk0(CLOCK_27),
	.c0(SDRAM_CLK),
	.c1(CLK_120M),
	.c2(CLK_24M),
	.locked(pll_locked)
	);

wire [31:0] status;
wire  [1:0] buttons;
wire  [1:0] switches;
wire [15:0] joystick_0;
wire [15:0] joystick_1;
wire        scandoublerD;
wire        ypbpr;
wire        no_csync;
wire        key_pressed;
wire  [7:0] key_code;
wire        key_strobe;

user_io #(
	.STRLEN(($size(CONF_STR)>>3)),
	.ROM_DIRECT_UPLOAD(1'b1))
user_io(
	.clk_sys        (CLK_24M        ),
	.conf_str       (CONF_STR       ),
	.SPI_CLK        (SPI_SCK        ),
	.SPI_SS_IO      (CONF_DATA0     ),
	.SPI_MISO       (SPI_DO         ),
	.SPI_MOSI       (SPI_DI         ),
	.buttons        (buttons        ),
	.switches       (switches       ),
	.scandoubler_disable (scandoublerD	  ),
	.ypbpr          (ypbpr          ),
	.no_csync       (no_csync       ),
	.core_mod       (core_mod       ),
	.key_strobe     (key_strobe     ),
	.key_pressed    (key_pressed    ),
	.key_code       (key_code       ),
	.joystick_0     (joystick_0     ),
	.joystick_1     (joystick_1     ),
	.status         (status         )
	);

wire        ioctl_downl;
wire  [7:0] ioctl_index;
wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;

data_io #(.ROM_DIRECT_UPLOAD(1'b1)) data_io(
	.clk_sys       ( CLK_24M      ),
	.SPI_SCK       ( SPI_SCK      ),
	.SPI_SS2       ( SPI_SS2      ),
	.SPI_SS4       ( SPI_SS4      ),
	.SPI_DI        ( SPI_DI       ),
	.SPI_DO        ( SPI_DO       ),
	.clkref_n      ( 1'b0         ),
	.ioctl_download( ioctl_downl  ),
	.ioctl_index   ( ioctl_index  ),
	.ioctl_wr      ( ioctl_wr     ),
	.ioctl_addr    ( ioctl_addr   ),
	.ioctl_dout    ( ioctl_dout   )
);

// reset signal generation
reg reset;
always @(posedge CLK_24M) begin
	reg [15:0] reset_count;

	if (status[0] | buttons[1] | ioctl_downl) reset_count <= 16'hffff;
	else if (reset_count != 0) reset_count <= reset_count - 1'd1;

	reset <= reset_count != 16'h0000;

end

wire        SYSTEM_ROMS;

wire [23:0] P2ROM_ADDR;
wire [15:0] PROM_DATA;
wire        PROM_DATA_READY;
wire        ROM_RD;
wire        PORT_RD;
wire        SROM_RD;

wire [15:0] RAM_ADDR;
wire [15:0] RAM_DATA;
wire        RAM_DATA_READY;
wire [15:0] RAM_Q;
wire  [1:0] WRAM_WE;
wire  [1:0] WRAM_RD;
wire  [1:0] SRAM_WE;
wire  [1:0] SRAM_RD;

reg         sdr_vram_req;
wire        sdr_vram_ack;
reg  [14:0] sdr_vram_addr;
reg         sdr_vram_we;
reg  [15:0] sdr_vram_d;
reg         sdr_vram_sel;

wire [14:0] SLOW_VRAM_ADDR;
wire [15:0] SLOW_VRAM_DATA_IN = SLOW_VRAM_DATA_IN_CPU;
wire [31:0] SLOW_VRAM_DATA_IN_SPR;
wire [15:0] SLOW_VRAM_DATA_IN_CPU;
wire [15:0] SLOW_VRAM_DATA_OUT;
wire        SLOW_VRAM_RD;
wire        SLOW_VRAM_WE;

wire [14:0] SPRMAP_ADDR;
wire        SPRMAP_RD;
wire [31:0] SPRMAP_DATA = SLOW_VRAM_DATA_IN_SPR;

reg         sfix_req;
wire [17:0] SFIX_ADDR;
wire [15:0] SFIX_DATA;
wire        SFIX_RD;

reg         lo_rom_req;
wire [15:0] LO_ROM_ADDR;
wire [15:0] LO_ROM_DATA;

reg         sp_req;       
wire [26:0] CROM_ADDR;
wire [63:0] CROM_DATA;
wire        CROM_RD;

wire [18:0] Z80_ROM_ADDR;
wire        Z80_ROM_RD;
wire [15:0] Z80_ROM_DATA;
wire        Z80_ROM_READY;

wire [19:0] ADPCMA_ADDR;
wire  [3:0] ADPCMA_BANK;
wire        ADPCMA_RD;
reg   [7:0] ADPCMA_DATA;
wire        ADPCMA_DATA_READY;
wire [23:0] ADPCMB_ADDR;
wire        ADPCMB_RD;
reg   [7:0] ADPCMB_DATA;
wire        ADPCMB_DATA_READY;

wire        sample_roma_req;
wire        sample_roma_ack;
wire [25:0] sample_roma_addr;
wire [63:0] sample_roma_dout;
wire        sample_romb_req;
wire        sample_romb_ack;
wire [25:0] sample_romb_addr;
wire [63:0] sample_romb_dout;

// Cart download control
wire        system_rom_write = ioctl_downl && (ioctl_index == 0 || ioctl_index == 1);
wire        cart_rom_write = ioctl_downl && ioctl_index == 2;
reg         port1_req, port1_ack;
reg         port2_req, port2_ack;
reg  [31:0] PSize, SSize, MSize, V1Size, V2Size, CSize;
wire [31:0] V1Mask = V1Size - 1'd1;
wire [31:0] V2Mask = V2Size - 1'd1;
wire [31:0] CMask = CSize - 1'd1;
reg   [2:0] region;
reg  [25:0] offset;
reg  [25:0] region_size;

always @(*) begin
	case (region)
		0: region_size = PSize[25:0];
		1: region_size = SSize[25:0];
		2: region_size = MSize[25:0];
		3: region_size = V1Size[25:0];
		4: region_size = V2Size[25:0];
		5: region_size = CSize[25:0];
		default: region_size = 0;
	endcase
end

always @(posedge CLK_24M) begin
	reg [1:0] written = 0;
	if (ioctl_wr) begin
		if (system_rom_write) begin
			port1_req <= ~port1_req;
			written <= 0;
		end
		if (cart_rom_write) begin
			if (ioctl_addr[24:12] == 0) begin
				/*
				Header
				struct NeoFile
				{
					uint8_t header1, header2, header3, version;
					uint32_t PSize, SSize, MSize, V1Size, V2Size, CSize;
					uint32_t Year;
					uint32_t Genre;
					uint32_t Screenshot;
					uint32_t NGH;
					uint8_t Name[33];
					uint8_t Manu[17];
					uint8_t Filler[128 + 290];	//fill to 512
					uint8_t Filler2[4096 - 512];	//fill to 4096
				}
				*/
				region <= 0;
				offset <= 0;
				if (ioctl_addr >= 4 && ioctl_addr < 28)
					{CSize, V2Size, V1Size, MSize, SSize, PSize} <= {ioctl_dout, CSize, V2Size, V1Size, MSize, SSize, PSize[31:8]};

				if (&ioctl_addr[11:0]) begin
					written <= 2;
					if (V2Size == 0) begin // Hack? Neobuilder merges ADPCMA+ADPCMB
						V1Size <= V1Size >> 1;
						V2Size <= V1Size >> 1;
					end
				end
			end else begin
				// ROM data
				if (region <= 2)
					port1_req <= ~port1_req;
				else
					port2_req <= ~port2_req;
				written <= 1;
			end
		end
	end
	case (written)
		1: // write acked, advance offset
		if ((region <= 2 && port1_req == port1_ack) || (region > 2 && port2_req == port2_ack)) begin
			offset <= offset + 1'd1;
			written <= 2;
		end
		2: // check end of the region, advance until a region with >0 size found, or it's the last region
		if (offset == region_size) begin
			offset <= 0;
			region <= region + 1'd1;
			if (region == 5) written <= 0;
		end else begin
			written <= 0;
		end
		default: ;
	endcase
end

wire [23:0] system_port1_addr = ioctl_addr[23:19] == 0 ? { 5'b1111_1, ioctl_addr[18:0] } : // system ROM
                                ioctl_addr[23:17] == 7'b0000100 ? { 8'b1101_1110, ioctl_addr[15:0] } : // LO ROM
                                ioctl_addr[23:17] == 7'b0000101 ? { 7'b1110_100, ioctl_addr[16:5],ioctl_addr[2:0],~ioctl_addr[4],ioctl_addr[3] } : // SFIX ROM
                                        { 6'b1110_11, ioctl_addr[17:0] }; // SM1

wire [23:0] cart_port1_addr = region == 1 ? { 5'b1110_0, offset[18:5],offset[2:0],~offset[4],offset[3] } : // FIX ROM
							  region == 2 ? { 5'b1111_0, offset[18:0] } : // MROM
							                offset[23:0]; // PROM

wire [23:0] port1_addr = system_rom_write ? system_port1_addr : cart_port1_addr;

wire [25:0] port2_addr = region == 3 ? CSize + offset : // V1 ROM
                         region == 4 ? CSize + V1Size + offset : // V2 ROM
						               {offset[25:7], ioctl_addr[5:2], ~ioctl_addr[6], ioctl_addr[0], ioctl_addr[1]}; // CROM

// VRAM->SDRAM control
always @(posedge CLK_24M) begin
	reg SLOW_VRAM_WE_OLD;
	reg SLOW_VRAM_RD_OLD;
	reg SPRMAP_RD_OLD;

	SLOW_VRAM_WE_OLD <= SLOW_VRAM_WE;
	SLOW_VRAM_RD_OLD <= SLOW_VRAM_RD;
	SPRMAP_RD_OLD <= SPRMAP_RD;

	if ((!SLOW_VRAM_WE_OLD && SLOW_VRAM_WE) || (!SLOW_VRAM_RD_OLD && SLOW_VRAM_RD)) begin
		sdr_vram_req <= ~sdr_vram_req;
		sdr_vram_addr <= SLOW_VRAM_ADDR;
		sdr_vram_we <= SLOW_VRAM_WE;
		sdr_vram_d <= SLOW_VRAM_DATA_OUT;
		sdr_vram_sel <= 1;
	end
	if (!SPRMAP_RD_OLD && SPRMAP_RD) begin
		sdr_vram_req <= ~sdr_vram_req;
		sdr_vram_addr <= SPRMAP_ADDR;
		sdr_vram_we <= 0;
		sdr_vram_sel <= 0;
	end
end

// SFIX->SDRAM control
always @(posedge CLK_24M) begin
	reg SFIX_RD_OLD;
	SFIX_RD_OLD <= SFIX_RD;
	if (SFIX_RD_OLD & !SFIX_RD) sfix_req <= ~sfix_req;
end

// LO ROM->SDRAM control
always @(posedge CLK_24M) begin
	reg [15:0] LO_ROM_ADDR_OLD;
	LO_ROM_ADDR_OLD <= LO_ROM_ADDR;
	if (LO_ROM_ADDR_OLD[15:1] != LO_ROM_ADDR[15:1]) lo_rom_req <= ~lo_rom_req;
end

// CROM->SDRAM control
always @(posedge CLK_24M) begin
	reg CROM_RD_OLD;
	CROM_RD_OLD <= CROM_RD;
	if (CROM_RD_OLD & !CROM_RD) sp_req <= ~sp_req;
end

// ADPCM->SDRAM control
reg [23:0] ADPCMA_ADDR_LATCH;
reg [23:0] ADPCMB_ADDR_LATCH;
always @(posedge CLK_24M) begin
	reg ADPCMA_RD_OLD, ADPCMB_RD_OLD;
	ADPCMA_RD_OLD <= ADPCMA_RD;
	ADPCMB_RD_OLD <= ADPCMB_RD;
	if (!ADPCMA_RD_OLD & ADPCMA_RD) begin
		if (ADPCMA_ADDR_LATCH[23:3] != {ADPCMA_BANK, ADPCMA_ADDR[19:3]}) sample_roma_req <= ~sample_roma_req;
		ADPCMA_ADDR_LATCH <= {ADPCMA_BANK, ADPCMA_ADDR} & V1Mask;
	end
	if (!ADPCMB_RD_OLD & ADPCMB_RD) begin
		if (ADPCMB_ADDR_LATCH[23:3] != ADPCMB_ADDR[23:3]) sample_romb_req <= ~sample_romb_req;
		ADPCMB_ADDR_LATCH <= ADPCMB_ADDR & V2Mask;
	end
end

assign sample_roma_addr = CSize + ADPCMA_ADDR_LATCH;
assign sample_romb_addr = CSize + V1Size + ADPCMB_ADDR_LATCH;
assign ADPCMA_DATA_READY = sample_roma_req == sample_roma_ack;
assign ADPCMB_DATA_READY = sample_romb_req == sample_romb_ack;

always @(*) begin
	case (ADPCMA_ADDR_LATCH[2:0])
	3'd0: ADPCMA_DATA = sample_roma_dout[ 7: 0];
	3'd1: ADPCMA_DATA = sample_roma_dout[15: 8];
	3'd2: ADPCMA_DATA = sample_roma_dout[23:16];
	3'd3: ADPCMA_DATA = sample_roma_dout[31:24];
	3'd4: ADPCMA_DATA = sample_roma_dout[39:32];
	3'd5: ADPCMA_DATA = sample_roma_dout[47:40];
	3'd6: ADPCMA_DATA = sample_roma_dout[55:48];
	3'd7: ADPCMA_DATA = sample_roma_dout[63:56];
	default: ;
	endcase
end

always @(*) begin
	case (ADPCMB_ADDR_LATCH[2:0])
	3'd0: ADPCMB_DATA = sample_romb_dout[ 7: 0];
	3'd1: ADPCMB_DATA = sample_romb_dout[15: 8];
	3'd2: ADPCMB_DATA = sample_romb_dout[23:16];
	3'd3: ADPCMB_DATA = sample_romb_dout[31:24];
	3'd4: ADPCMB_DATA = sample_romb_dout[39:32];
	3'd5: ADPCMB_DATA = sample_romb_dout[47:40];
	3'd6: ADPCMB_DATA = sample_romb_dout[55:48];
	3'd7: ADPCMB_DATA = sample_romb_dout[63:56];
	default: ;
	endcase
end

// Bank 0-1-2 address map
// CROM    (CSize)
// V1ROM   (V1Size)
// V2ROM   (V2Size)

// Bank 3 address map
// xxxx xxxx xxxx xxxx xxxx xxxx    P1/2 ROMs
// 1101 1100 xxxx xxxx xxxx xxxx    VRAM
// 1101 111x xxxx xxxx xxxx xxxx    LO ROM
// 1110 0xxx xxxx xxxx xxxx xxxx    FIX ROM
// 1110 100x xxxx xxxx xxxx xxxx    SFIX
// 1110 1010 xxxx xxxx xxxx xxxx    SRAM
// 1110 1011 xxxx xxxx xxxx xxxx    WRAM
// 1110 11xx xxxx xxxx xxxx xxxx    SM1 
// 1111 0xxx xxxx xxxx xxxx xxxx    Z80 Cart ROM
// 1111 1xxx xxxx xxxx xxxx xxxx    SROM

sdram_4w_cl3 #(120) sdram
(
  .*,
  .init_n        ( pll_locked    ),
  .clk           ( CLK_120M      ),

  // Bank 3 ops
  .port1_a       ( port1_addr[23:1] ),
  .port1_req     ( port1_req  ),
  .port1_ack     ( port1_ack ),
  .port1_we      ( system_rom_write | cart_rom_write ),
  .port1_ds      ( { port1_addr[0], ~port1_addr[0] } ),
  .port1_d       ( { ioctl_dout, ioctl_dout } ),
  .port1_q       (  ),

  // Main CPU
  .cpu1_rom_addr ( SROM_RD ? { 5'b11111, P2ROM_ADDR[18:1] } : ROM_RD ? P2ROM_ADDR[19:1] : 23'h80000 + P2ROM_ADDR[23:1] ),
  .cpu1_rom_cs   ( ROM_RD | PORT_RD | SROM_RD ),
  .cpu1_rom_q    ( PROM_DATA ),
  .cpu1_rom_valid( PROM_DATA_READY ),

  .cpu1_ram_addr ( { (|(WRAM_WE | WRAM_RD)) ? 8'b1110_1011 : 8'b1110_1010, RAM_ADDR[15:1] } ),
  .cpu1_ram_we   ( |(WRAM_WE | SRAM_WE) ),
  .cpu1_ram_d    ( RAM_DATA  ),
  .cpu1_ram_q    ( RAM_Q     ),
  .cpu1_ram_ds   ( WRAM_RD | WRAM_WE | SRAM_RD | SRAM_WE ),
  .cpu1_ram_valid( RAM_DATA_READY ),

  // Audio CPU
  .cpu2_rom_addr ( SYSTEM_ROMS ? { 6'b111011, Z80_ROM_ADDR[17:1] } : { 5'b11110, Z80_ROM_ADDR[18:1] } ),
  .cpu2_rom_cs   ( Z80_ROM_RD    ),
  .cpu2_rom_q    ( Z80_ROM_DATA  ),
  .cpu2_rom_valid( Z80_ROM_READY ),

  // FIX ROM
  .sfix_req      ( sfix_req ),
  .sfix_ack      ( ),
  .sfix_addr     ( SYSTEM_ROMS ? { 7'b1110100, SFIX_ADDR[15:0] } : { 5'b11100, SFIX_ADDR[17:0] } ),
  .sfix_q        ( SFIX_DATA ),

  // LO ROM
  .lo_rom_req    ( lo_rom_req ),
  .lo_rom_ack    ( ),
  .lo_rom_addr   ( { 8'b1101_1110, LO_ROM_ADDR[15:1] } ),
  .lo_rom_q      ( LO_ROM_DATA ),

  // VRAM
  .vram_addr     ( { 8'b1101_1100, sdr_vram_addr[14:0] } ),
  .vram_req      ( sdr_vram_req ),
  .vram_q1       ( SLOW_VRAM_DATA_IN_SPR ),
  .vram_q2       ( SLOW_VRAM_DATA_IN_CPU ),
  .vram_d        ( sdr_vram_d  ),
  .vram_we       ( sdr_vram_we ),
  .vram_ack      ( sdr_vram_ack ),
  .vram_sel      ( sdr_vram_sel ),

  // Bank 0-1-2 ops
  .port2_a       ( port2_addr[25:1] ),
  .port2_req     ( port2_req  ),
  .port2_ack     ( port2_ack ),
  .port2_we      ( cart_rom_write ),
  .port2_ds      ( { port2_addr[0], ~port2_addr[0] } ),
  .port2_d       ( { ioctl_dout, ioctl_dout } ),
  .port2_q       (  ),

  .samplea_addr  ( sample_roma_addr ),
  .samplea_q     ( sample_roma_dout ),
  .samplea_req   ( sample_roma_req  ),
  .samplea_ack   ( sample_roma_ack  ),

  .sampleb_addr  ( sample_romb_addr ),
  .sampleb_q     ( sample_romb_dout ),
  .sampleb_req   ( sample_romb_req  ),
  .sampleb_ack   ( sample_romb_ack  ),

  .sp_req        ( sp_req ),
  .sp_ack        (  ),
  .sp_addr       ( CROM_ADDR[25:3] & CMask[25:3] ),
  .sp_q          ( CROM_DATA )
);

wire [15:0] ch_left, ch_right;
wire  [7:0] R, G, B;
wire        HBlank, VBlank, HSync, VSync;
wire        ce_pix;
wire  [1:0] SYSTEM_TYPE = {1'b0, systype};
wire [15:0] joy0 = joyswap ? joystick_1 : joystick_0;
wire [15:0] joy1 = joyswap ? joystick_0 : joystick_1;
wire  [2:0] ps2_mouse;
wire        use_mouse = 0, ms_pos, ms_btn;
wire  [9:0] P1_IN = {(joy0[9:8]|ps2_mouse[2]), {use_mouse ? ms_pos : {joy0[7:4]|{3{joy0[11]}}, joy0[0], joy0[1], joy0[2], joy0[3]}}};
wire  [9:0] P2_IN = {(joy1[9:8]             ), {use_mouse ? ms_btn : {joy1[7:4]|{3{joy1[11]}}, joy1[0], joy1[1], joy1[2], joy1[3]}}};

neogeo_top neogeo_top (
	.CLK_24M       ( CLK_24M ),
	.RESET         ( reset   ),

	.VIDEO_MODE    ( vmode ),
	.SYSTEM_TYPE   ( SYSTEM_TYPE ),
	.COIN1         ( m_coin1 ),
	.COIN2         ( m_coin2 ),
	.P1_IN         ( P1_IN ),
	.P2_IN         ( P2_IN ),
	.DIPSW         ( dipsw ),

	.RED           ( R ),
	.GREEN         ( G ),
	.BLUE          ( B ),
	.HSYNC         ( HSync ),
	.VSYNC         ( VSync ),
	.HBLANK        ( HBlank ),
	.VBLANK        ( VBlank ),

	.LSOUND        ( ch_left ),
	.RSOUND        ( ch_right ),

	.CE_PIXEL      ( ce_pix ),

	.P2ROM_ADDR          ( P2ROM_ADDR ),
	.PROM_DATA           ( PROM_DATA  ),
	.PROM_DATA_READY     ( PROM_DATA_READY ),
	.ROM_RD              ( ROM_RD ),
	.PORT_RD             ( PORT_RD ),
	.SROM_RD             ( SROM_RD ),

	.RAM_ADDR            ( RAM_ADDR ),
	.RAM_DATA            ( RAM_DATA ),
	.RAM_DATA_READY      ( RAM_DATA_READY ),
	.RAM_Q               ( RAM_Q ),
	.WRAM_WE             ( WRAM_WE ),
	.WRAM_RD             ( WRAM_RD ),
	.SRAM_WE             ( SRAM_WE ),
	.SRAM_RD             ( SRAM_RD ),

	.SYSTEM_ROMS         ( SYSTEM_ROMS ),
	.SFIX_ADDR           ( SFIX_ADDR ),
	.SFIX_DATA           ( SFIX_DATA ),
	.SFIX_RD             ( SFIX_RD   ),

	.LO_ROM_ADDR         ( LO_ROM_ADDR ),
	.LO_ROM_DATA         ( LO_ROM_ADDR[0] ? LO_ROM_DATA[15:8] : LO_ROM_DATA[7:0] ),

	.CROM_ADDR           ( CROM_ADDR ),
	.CROM_DATA           ( CROM_DATA ),
	.CROM_RD             ( CROM_RD   ),

	.Z80_ROM_ADDR        ( Z80_ROM_ADDR ),
	.Z80_ROM_RD          ( Z80_ROM_RD   ),
	.Z80_ROM_DATA        ( Z80_ROM_ADDR[0] ? Z80_ROM_DATA[15:8] : Z80_ROM_DATA[7:0] ),
	.Z80_ROM_READY       ( Z80_ROM_READY ),

	.SLOW_SCB1_VRAM_ADDR      ( SLOW_VRAM_ADDR ),
	.SLOW_SCB1_VRAM_DATA_IN   ( SLOW_VRAM_DATA_IN ),
	.SLOW_SCB1_VRAM_DATA_OUT  ( SLOW_VRAM_DATA_OUT ),
	.SLOW_SCB1_VRAM_RD        ( SLOW_VRAM_RD ),
	.SLOW_SCB1_VRAM_WE        ( SLOW_VRAM_WE ),

	.SPRMAP_ADDR         ( SPRMAP_ADDR ),
	.SPRMAP_RD           ( SPRMAP_RD ),
	.SPRMAP_DATA         ( SPRMAP_DATA ),

	.ADPCMA_ADDR         ( ADPCMA_ADDR ),
	.ADPCMA_BANK         ( ADPCMA_BANK ),
	.ADPCMA_RD           ( ADPCMA_RD   ),
	.ADPCMA_DATA         ( ADPCMA_DATA ),
	.ADPCMA_DATA_READY   ( ADPCMA_DATA_READY ),
	.ADPCMB_ADDR         ( ADPCMB_ADDR ),
	.ADPCMB_RD           ( ADPCMB_RD   ),
	.ADPCMB_DATA         ( ADPCMB_DATA ),
	.ADPCMB_DATA_READY   ( ADPCMB_DATA_READY )
);

mist_video #(.COLOR_DEPTH(6), .SD_HCNT_WIDTH(10), .USE_BLANKS(1'b0)) mist_video(
	.clk_sys        ( CLK_24M          ),
	.SPI_SCK        ( SPI_SCK          ),
	.SPI_SS3        ( SPI_SS3          ),
	.SPI_DI         ( SPI_DI           ),
	.R              ( R[7:2]           ),
	.G              ( G[7:2]           ),
	.B              ( B[7:2]           ),
	.HBlank         ( HBlank           ),
	.VBlank         ( VBlank           ),
	.HSync          ( HSync            ),
	.VSync          ( VSync            ),
	.VGA_R          ( VGA_R            ),
	.VGA_G          ( VGA_G            ),
	.VGA_B          ( VGA_B            ),
	.VGA_VS         ( VGA_VS           ),
	.VGA_HS         ( VGA_HS           ),
	.rotate         ( { orientation[1], rotate } ),
	.ce_divider     ( 1'b1             ),
	.scandoubler_disable( scandoublerD ),
	.scanlines      ( scanlines        ),
	.blend          ( blend            ),
	.ypbpr          ( ypbpr            ),
	.no_csync       ( no_csync         )
	);

dac #(
	.C_bits(16))
dacl(
	.clk_i(CLK_24M),
	.res_n_i(1),
	.dac_i({~ch_left[15], ch_left[14:0]}),
	.dac_o(AUDIO_L)
	);

dac #(
	.C_bits(16))
dacr(
	.clk_i(CLK_24M),
	.res_n_i(1),
	.dac_i({~ch_right[15], ch_right[14:0]}),
	.dac_o(AUDIO_R)
	);
	
wire m_up, m_down, m_left, m_right, m_fireA, m_fireB, m_fireC, m_fireD, m_fireE, m_fireF;
wire m_up2, m_down2, m_left2, m_right2, m_fire2A, m_fire2B, m_fire2C, m_fire2D, m_fire2E, m_fire2F;
wire m_tilt, m_coin1, m_coin2, m_coin3, m_coin4, m_one_player, m_two_players, m_three_players, m_four_players;

arcade_inputs #(.START1(8), .START2(10), .COIN1(9)) inputs (
	.clk         ( CLK_24M     ),
	.key_strobe  ( key_strobe  ),
	.key_pressed ( key_pressed ),
	.key_code    ( key_code    ),
	.joystick_0  ( joystick_0  ),
	.joystick_1  ( joystick_1  ),
	.rotate      ( rotate      ),
	.orientation ( orientation ),
	.joyswap     ( joyswap     ),
	.oneplayer   ( oneplayer   ),
	.controls    ( {m_tilt, m_coin4, m_coin3, m_coin2, m_coin1, m_four_players, m_three_players, m_two_players, m_one_player} ),
	.player1     ( {m_fireF, m_fireE, m_fireD, m_fireC, m_fireB, m_fireA, m_up, m_down, m_left, m_right} ),
	.player2     ( {m_fire2F, m_fire2E, m_fire2D, m_fire2C, m_fire2B, m_fire2A, m_up2, m_down2, m_left2, m_right2} )
);

endmodule 