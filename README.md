# RVPC
Set up to run on linux. Tested on Linux Mint.
Requires:
 * minichlink: install ch32v003fun, then change path in RVPC Makefile to suit
 * riscv64-unknown-elf-??? : cross assembler toolkit

## Device
CH32V003J4M6(SOP8)
ISA: RV32EC_Zicsr
The WCH documentaion only refers to RV32EC, but the chip does have the CSR instructions (as expected).
 
## Pins
Pin | MCU use               | 
:-- | :-------------------- |
1   | PD6\T2CH3_3\VSync     |
2   | VSS                   |
3   | PA2\KBD_DAT           |
4   | VDD                   |
5   | PC1\T2CH4_1\HSync     |
6   | PC2\T2CH2_1\Video_Out |
7   | PC4\T1CH4_2\Audio     |
8   | PD1\SWIO\KBD_CLK      |
    | PGM/DBG1              |

## VGA
Lots of considerations and options to consider. 
### test1
#### Using DMA
There's no bit-masking in DMA transfer, and a byte is the minimum-sized chunk. So, we have to hit the whole port. Now, PC1=HSync and PC2=video, so can set/clear both bits with every DMA transfer. Port PC also has our audio line, but we'll choose to ignore it for the time being, and see how far we get with VGA.
So, how many bytes needed for a whole scan line? Time for a whole scan line is about 31.770uS. Maybe DMA is per cycle, so clocks per whole scan line is 48*10^6*31.77*10^-6 = 1524.96, say 1525. So need 1525 bytes, which must be consecutive and some need to change every active scan line (by an asynchronous process), so must be in RAM.
#### pseudocode
 # initialise memory for DMA to drive port PC, called PCBYTES
 # Default all bytes in PCBYTES to 0b010 (HSync=PC1 hi, video=PC2 lo)
 # Setup HSync pulse (lo) for 3.813uS (=183.024 at 48MHz, say 183) after 0.636us (=30.528, say 31)
 # white video in PC2, starting after 0.636+3.813+1.907uS (=305.088, say 305) to end 

 # enable GPIO peripheral so port PC works
 # PC1&2 Output mode, maximum speed 50MHz, push-pull

 # enable DMA peripheral
 # choose highest priority DMA since time critical => CH1
 # set source address (memory), target address (peripheral, GPIO port C), counter, and configure
 # enable DMA

 # infinite loop, with logging for test purposes

#### mods for testing
 # Put different values in each PCBYTE so can see changes.

#### Observations
Looks like DMA runs at 8MHz, with some wait states dropping effective speed to ~7.6MHz. 


