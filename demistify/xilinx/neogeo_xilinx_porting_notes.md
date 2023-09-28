# Neogeo specific notes

### Changed files for Xilinx port

Those files have been duplicated and modified in the Xilinx folder: 

* demistify_config_pkg.vhd

* rtl/mem/dpram.v

* mist/mist-modules/data_io.v

* NeoGeo.sdc

  

### Todo

* Neogeo CD: Gyurco: probably a NO_CD define will needed for some boards - however BRAM usage increase is mostly from the CDDA buffer, so CD could work if CDDA is disabled, or smaller buffer can be used. The firmware part could be based on the existing pcecd.c.
* Gyurco: Another improvement would be for boards with large internal RAM: move the whole VRAM and LOROM into BRAM (and undefine VRAM32). That would be a fairly easy task, the original BRAM definitions are still in neogeo_top, just commented out.  Benefits: 
  - Good feeling because using the original behavior of the LSPC2 (LO ROM and VRAM address/data travels on PBUS)
  - Probably less wait states on CPU access
* Gyurco: I think the data_io problem can be solved by adding some `ifdefs (like DATAIO_SPLIT_BUS) to the standard one (and do the same in mist_top). The dpram should work on Altera, too, so file duplication can be eliminated.
* adapt constraints file eightthirtytwo_multicycles.sdc
* check errors from xilinx/NeoGeo.sdc



# General porting notes

### Porting steps from Quartus to Vivado

* start with original Quartus repository

* update mist-modules with latest version including this [commit](https://github.com/mist-devel/mist-modules/commit/63c9d42f00257cc65e489e24375ff902fff02f48) for Vivado

* create folder for new xilinx board (or copy it from previous project)

  * adapt xxxxx_top.vhd, usually only changing guest core instantiation
  * adapt defs.v to defs.vh
  * adapt PLLs, with a text editor, using this [calculation tool](https://github.com/hansfbaier/pll-calculator)

* create new project in Vivado, or copy it from another project but removing all filesets from .xpr project config file, or use this [template](https://github.com/somhi/xilinx_stuff/blob/main/a100t_cape%20(template).xpr).

* open Xilinx project

* add sources according to qsf/qip files from Quartus project

* Changes required to avoid errors in Vivado:

  * most errors can be avoided if defining Verilog files as System Verilog type          

  `<FileInfo SFType="SVerilog">  `

  * define general macros file  (defs.v)  as Verilog header type (defs.vh) and set global include property             `<Attr Name="IsGlobalInclude" Val="1"/>`

  * Procedural assignment to a non-register 'SDRAM_DQ' is not permitted  >   Add another register and a OE and do an assing to SDRAM_DQ [sdram_ws_cl2.sv].

  * Do not use direct FPGA clock (CLK_50) as input to modules. A buffer is needed [a100t_cape_top.vhd]

* Remove design checkpoint files before distributing sources (.dcp files). That is automatically added after synthesis if design run synthesis option "auto_incremental_checkpoint" is set.   Uncheck that property to avoid those .dcp files being added to the project.

* Run TCL sources before synthesis (build_id.tcl). Add tcl file to design run synthesis option "tcl.pre". 

  * comment out post_message  lines
  * adapt location of build_id.vh  (in Vivado those files with defines should have .vh extension)


### Common adaptations to code to work with Xilinx

- https://github.com/sy2002/MiSTer2MEGA65/wiki/7.-Get-the-core-to-synthesize

- [Xilinx Design Flow for Intel FPGA and SoC Users](https://docs.xilinx.com/v/u/en-US/ug1192-xilinx-design-for-intel)  (see how to adapt constraints)

- signals should be defined as reg if included in always blocks

- always begin : block_name    (this is not needed if Verilog files are defined as System Verilog)

- 'default_nettype none       comment out because is way more restrictive than Quartus IDE

- clocks cannot be connected directly, always though a buffer

  

```
constant HALF_RANGE: unsigned(RANGE_WIDTH - 1 downto 0) := (RANGE_WIDTH-1 => '1', others=>'0');
#was changed to
constant HALF_RANGE: unsigned(RANGE_WIDTH - 1 downto 0) := '1'&(RANGE_WIDTH - 2 downto 0 => '0');
```



### Creating a PLL in Vivado

Project manager > IP catalog > Clocking wizard

Primitive PLL

Input clock Clk_in1   50.000

Sources > IP Sources > search for the created PLL

PLL is generated in folder    *.gen/sources_1/ip/clk_wiz_0

