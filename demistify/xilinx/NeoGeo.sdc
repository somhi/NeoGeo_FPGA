# Clock constraints

# Automatically constrain PLL and other generated clocks
# derive_pll_clocks -create_base_clocks

# Automatically calculate clock uncertainty to jitter and other effects.
# derive_clock_uncertainty



# Set pin definitions for downstream constraints
set RAM_CLK DRAM_CLK
set RAM_OUT {DRAM_DQ* DRAM_ADDR* DRAM_BA* DRAM_RAS_N DRAM_CAS_N DRAM_WE_N DRAM_*DQM DRAM_CS_N DRAM_CKE}
set RAM_IN {DRAM_D*}

set VGA_OUT {VGA_R[*] VGA_G[*] VGA_B[*] VGA_HS VGA_VS}

# non timing-critical pins would be in the "FALSE_IN/OUT" collection (IN inputs, OUT outputs)
set FALSE_OUT {LED* PWM_AUDIO_* PS2_* JOYX_SEL_O UART_TXD SD_CS_N_O SD_MOSI_O SD_SCLK_O}
set FALSE_IN  {SW* PS2_* JOY1* EAR UART_RXD SD_MISO_I}


#**************************************************************
# Create Clock
#**************************************************************
create_clock -name {clk_50} -period 20.000 -waveform {0.000 10.000} { CLK_50 }

create_generated_clock -name spiclk -source [get_ports {CLK_50}] -divide_by 16 [get_nets {controller/spi/sck_i_1_n_0}]

set hostclk { clk_50 }
set supportclk { clk_50 }

# set sdram_clk "guest/pll/clk_out1"
set sdram_clk "clk_out1_pll"
set mem_clk   "clk_out2_pll"
set vid_clk   "clk_out3_pll"
set game_clk  "clk_out3_pll"


#**************************************************************
# Set Input Delay
#**************************************************************
set_input_delay -clock [get_clocks $sdram_clk] -reference_pin [get_ports ${RAM_CLK}] -max 6.4 [get_ports ${RAM_IN}]
set_input_delay -clock [get_clocks $sdram_clk] -reference_pin [get_ports ${RAM_CLK}] -min 3.2 [get_ports ${RAM_IN}]


#**************************************************************
# Set Output Delay
#**************************************************************
set_output_delay -clock [get_clocks $sdram_clk] -reference_pin [get_ports ${RAM_CLK}] -max 1.5 [get_ports ${RAM_OUT}]
set_output_delay -clock [get_clocks $sdram_clk] -reference_pin [get_ports ${RAM_CLK}] -min -0.8 [get_ports ${RAM_OUT}]


#**************************************************************
# Set Clock Groups
#**************************************************************
set_clock_groups -asynchronous -group [get_clocks spiclk] -group [get_clocks {clk_out1_pll clk_out2_pll clk_out3_pll} ]
set_clock_groups -asynchronous -group [get_clocks ${game_clk}]  -group [get_clocks ${supportclk}]

# guest/pll/plle2_adv_inst/CLKOUT2]]
#**************************************************************
# Set False Path
#**************************************************************
set_false_path -to ${FALSE_OUT}
set_false_path -from ${FALSE_IN}
set_false_path -to ${VGA_OUT}


#**************************************************************
# Set Multicycle Path
#**************************************************************
set_multicycle_path -from [get_clocks $sdram_clk] -to [get_clocks $mem_clk] -setup 2
set_multicycle_path -from [get_cells guest/neogeo_top/M68KCPU/FX68K/excUnit/aob*]  -to [get_clocks $mem_clk] -setup 2
set_multicycle_path -from [get_cells guest/neogeo_top/M68KCPU/FX68K/excUnit/aob*]  -to [get_clocks $mem_clk] -hold 1
set_multicycle_path -from [get_cells guest/neogeo_top/Z80CPU/cpu/u0/A*]  -to [get_clocks $mem_clk] -setup 2
set_multicycle_path -from [get_cells guest/neogeo_top/Z80CPU/cpu/u0/A*]  -to [get_clocks $mem_clk] -hold 1

set_multicycle_path -from [get_cells guest/*Size*] -to [get_clocks $mem_clk] -setup 2
set_multicycle_path -from [get_cells guest/*Size*] -to [get_clocks $mem_clk] -hold 1
set_multicycle_path -from [get_cells guest/pcm_merged*] -to [get_clocks $mem_clk] -setup 2
set_multicycle_path -from [get_cells guest/pcm_merged*] -to [get_clocks $mem_clk] -hold 1


