#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <verilated.h>
#include "Vlspc_tb.h"
#include "verilated_vcd_c.h"


static Vlspc_tb *tb;
static VerilatedVcdC *trace;
static int tickcount;

static uint16_t vram[0x8800];
static uint16_t palram[0x2000];
static uint8_t lorom[0x10000];
static uint8_t sfix_in[0x20000];
static uint8_t sfix_out[0x20000];
static uint16_t crom_even_in[0x400000]; // 8MB
static uint16_t crom_odd_in[0x400000]; // 8MB
static uint16_t crom_even_out[0x400000]; // 8MB
static uint16_t crom_odd_out[0x400000]; // 8MB

static void initmems() {
	FILE *file;
	file=fopen("mems/vram.bin", "rb");
	fread(&vram, 2, 0x8800, file);
	fclose(file);

	file=fopen("mems/palram.bin", "rb");
	fread(&palram, 2, 0x2000, file);
	fclose(file);

	file=fopen("mems/000-lo.lo", "rb");
	fread(&lorom, 1, 0x10000, file);
	fclose(file);

	file=fopen("mems/sfix.sfix", "rb");
	fread(&sfix_in, 1, 0x20000, file);
	fclose(file);

	for (uint32_t i = 0; i < 0x20000; i++) sfix_out[i] = sfix_in[(i & ~0x1F) | ((i >> 2) & 7) | ((i & 1) << 3) | (((i & 2) << 3) ^ 0x10)];

	file=fopen("mems/crom_even.bin", "rb");
	fread(&crom_even_in, 0x400000, 2, file);
	fclose(file);

	file=fopen("mems/crom_odd.bin", "rb");
	fread(&crom_odd_in, 0x400000, 2, file);
	fclose(file);

	for (uint32_t i = 0; i < 0x400000; i++) crom_even_out[i] = crom_even_in[(i & ~0x1F) | ((i >> 1) & 0xF) | (((i & 1) ^ 1) << 4)];
	for (uint32_t i = 0; i < 0x400000; i++) crom_odd_out[i] = crom_odd_in[(i & ~0x1F) | ((i >> 1) & 0xF) | (((i & 1) ^ 1) << 4)];
}

void tick(int c) {
	if (c) {
		tb->SFIX_DATA = sfix_out[tb->SFIX_ADDR << 1] | sfix_out[(tb->SFIX_ADDR << 1) + 1]<<8 ;
		uint32_t crom_addr = (tb->CROM_ADDR>>2) & 0x3FFFFF;
		tb->CR_DOUBLE = crom_even_out[crom_addr] & 0xff | (crom_odd_out[crom_addr] & 0xff) << 8 | 
		                (crom_even_out[crom_addr] & 0xff00) << 8 | (crom_odd_out[crom_addr] & 0xff00) << 16;
		tb->CR_DOUBLE |= uint64_t(crom_even_out[crom_addr+1] & 0xff | (crom_odd_out[crom_addr+1] & 0xff) << 8 | 
		                (crom_even_out[crom_addr+1] & 0xff00) << 8 | (crom_odd_out[crom_addr+1] & 0xff00) << 16) << 32;
		//tb->CR_DOUBLE = 0xaaaaaaaaaaaaaaaa;
		tb->LO_ROM_DATA = lorom[tb->LO_ROM_ADDR];

		tb->PAL_RAM_DATA = palram[0x1000 | tb->PAL_RAM_ADDR];
		tb->FAST_VRAM_DATA_IN = vram[0x8000+tb->FAST_VRAM_ADDR];
		tb->SLOW_VRAM_DATA_IN = vram[tb->SLOW_VRAM_ADDR];
	}

	tb->CLK_24M = c;
	tb->eval();
	trace->dump(tickcount++);
}

/*
void write_reg(int addr, int data)
{
	// S0
	while (!tb->MHZ8_EN1) {
		tick(1);
		tick(0);
	};

	// S1
	while (!tb->MHZ8_EN2) {
		tick(1);
		tick(0);
	};
	tb->A = addr >> 1;
	while (!tb->bus_free) {
		tick(1);
		tick(0);
	}
	tb->RW = 1;

	// S2
	while (!tb->MHZ8_EN1) {
		tick(1);
		tick(0);
	};
	tb->AS_N = 0;
	tb->RW = 0;

	// S3
	while (!tb->MHZ8_EN2) {
		tick(1);
		tick(0);
	};
	tb->DIN = data;

	// S4
	while (!tb->MHZ8_EN1) {
		tick(1);
		tick(0);
	};
	tb->UDS_N = 0;
	tb->LDS_N = 0;

	// S5
	while (true) {
		tick(1);
		tick(0);
		if (tb->MHZ8_EN2 && !(tb->DTACK_N && tb->BERR_N)) break;
	}

	// S6
	while (!tb->MHZ8_EN1) {
		tick(1);
		tick(0);
	}

	// S7
	while (!tb->MHZ8_EN2) {
		tick(1);
		tick(0);
	}
	tb->RW=1;
	tb->AS_N=1;
	tb->UDS_N=1;
	tb->LDS_N=1;
	tick(1);
	tick(0);
}

int read_reg(int addr)
{
	int dout;
	while (!tb->bus_free) {
		tick(1);
		tick(0);
	}
	// S0
	while (!tb->MHZ8_EN1) {
		tick(1);
		tick(0);
	};
	tb->RW = 1;

	// S1
	while (!tb->MHZ8_EN2) {
		tick(1);
		tick(0);
	};
	tb->A = addr >> 1;

	// S2
	while (!tb->MHZ8_EN1) {
		tick(1);
		tick(0);
	};
	tb->AS_N = 0;
	tb->UDS_N = 0;
	tb->LDS_N = 0;
	// S3
	while (!tb->MHZ8_EN2) {
		tick(1);
		tick(0);
	}
	// S4 - S5
	while (true) {
		tick(1);
		tick(0);
		if (tb->MHZ8_EN2 && !(tb->DTACK_N && tb->BERR_N)) break;
	}

	// S6
	while (!tb->MHZ8_EN1) {
		tick(1);
		tick(0);
	}

	//S7
	while (!tb->MHZ8_EN2) {
		tick(1);
		tick(0);
	}
	dout = tb->DOUT;
	tb->AS_N=1;
	tb->UDS_N=1;
	tb->LDS_N=1;
	tick(1);
	tick(0);
	return dout;
}
*/

int main(int argc, char **argv) {

	int frame = 0;
	int vsync_old=0;
	uint32_t rgb;

	initmems();
	// Initialize Verilators variables
	Verilated::commandArgs(argc, argv);
//	Verilated::debug(1);
	Verilated::traceEverOn(true);
	trace = new VerilatedVcdC;
	tickcount = 0;

	// Create an instance of our module under test
	tb = new Vlspc_tb;
	tb->trace(trace, 99);
	trace->open("lspc.vcd");

	tb->nRESET = 0;
	tick(1);
	tick(0);
	tb->nRESET = 1;
	tick(1);
	tick(0);
	tb->nAS = 1;
	tick(1);
	tick(0);
	tb->nAS = 0;
	tick(1);
	tick(0);
	tb->nAS = 1;
	FILE *vidfile=fopen("video.rgb", "wb");
	while(1) {
		tick(1);
		tick(0);
		tick(1);
		tick(0);
		if (tb->VSYNC && !vsync_old) {
			frame++;
			printf("Frame: %d\n", frame);
		}
		if (frame == 1) {
			if (!tb->VSYNC) rgb = 0xaaaaaaaa;
			else if (!tb->HSYNC) rgb = 0x00ff00ff;
			else rgb = tb->RED<<8 | tb->GREEN<<16 | tb->BLUE<<24 | 0xff;
			fwrite(&rgb, 1, sizeof(rgb), vidfile);
		}
		vsync_old = tb->VSYNC;
		if (frame == 2) break;
	}
	fclose(vidfile);
	trace->close();

}