# [SNK Neo Geo](https://en.wikipedia.org/wiki/Neo_Geo_(system)) for [MiST FPGA](https://github.com/mist-devel/mist-board/wiki)

This is the port of the [NeoGeo FPGA implementation](https://github.com/MiSTer-devel/NeoGeo_MiSTer) by [Furrtek](https://www.patreon.com/furrtek/posts)

## Limitations
The original Neo Geo system has big RAM/ROM memories, which don't fit into the BRAM of the MiST's FPGA. A new SDRAM controller was written, which can
read one 64 bit and one 32 bit word simultaneously in just 12 cycles using bank interleaving, and running at 120MHz. Later on, it was replaced by
a 96MHz variant reading two 32 bit words in 8 cycles. Both 32MiB and 64MiB equipped MiSTs are supported, the later obviously can load more games.

The limitation of ROM sizes for both variants:

**32 MiB** - supports  ~6 MiB PROMS and 24 MiB CROM+VROMs (in any size combination)

**64 MiB** - supports ~14 MiB PROMS and 48 MiB CROM+VROMs (in any size combinaion. Note: PROM size is not a real limitiation in this case.)

## Usage

Internal ROMs (System ROM, SFIX, LO ROM and SM1 ROM) can be created from MAME's neogeo.zip with the help of the MRA files.

TerraOnion .NEO file format was choosen as the supported cart format, as it conveniently merges all the various ROMs in one file. The following utilities can be used to create such files:

[Original NeoBuilder tool](https://wiki.terraonion.com/index.php/Neobuilder_Guide)

[MAME to .neo conversion tool](https://github.com/city41/neosdconv)

[Darksoft to .neo conversion tool](https://gitlab.com/loic.petit/darksoft-to-neosd/)

Note: Core doesn't support encrypted ROMs. Make sure the ROM has no encrypted parts before use. MAME ROM pack includes many encrypted ROMs so it's not recommended for inexperienced users. Using the .neo conversion tool with a MAME ROM set will result in some ROMs still being encrypted. There is an alternate .neo conversion tool for the Darksoft ROM set that will give you a fully decrypted set.

## Controls

| NeoGeo | MiST    |
|--------|---------|
| A      | A       |
| B      | B       |
| C      | X       |
| D      | Y       |
| Start  | Start   |
| Select | Select  |
| Coin1  | L       |
| Coin2  | R       |

## Sidenotes:

The core is inherently unstable. While Furrtek (Sean Gonsalves) did a very good and tedious job reverse-engineering and documenting the original Neo-Geo chipset,
the resulting HDL is not very good for FPGAs. Probably MiSTer's Cyclone V FPGA can deal with it better, as it's built in a newer process, has smaller inner delays, or the more 
recent Quartus tool is better synthesizing such code, but it still broken. Translating old ASIC designs 1-1 into FPGA won't work, as there are dozens of generated signals
(even with combinatorial output) used as clocks, which are glitching, the compiler cannot check if flip-flops clocked by these signals meet setup and hold times, resulting
in very unstable cores.

As I (gyurco) already translated many such arcade cores into proper synchronous code, I can say it's a huge job. I normally don't require nor accept any donations for my hobby, but
in this case I choose a different path. Converting Neo-Geo to proper HDL could easily take some months, and frankly I don't have the motivation to do it. So **only** for this work,
I accept donations, then I'll know if others are really interested in this (as I won't do it for just myself). MiSTer users can also benefit from a more stable core.

So if you want to support and appreciate this work, you can send donations to:

Via [PayPal](https://paypal.me/gyorgyszombathelyi)

Via [ko-fi](https://ko-fi.com/szombathelyigyorgy)
