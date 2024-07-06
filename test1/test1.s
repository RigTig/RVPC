#  VGA for RVPC
##

.text

# RVPC map
.equ RAM_BASE, 0x20000000

.equ R32_GPIOC, 0x40011000      # GPIO Port C registers
 # GPIOC offsets
.equ CFGLR, 0x000
  # CFGLR bitmasks
.equ MODEy, 0b11<<0             # mode selection for pin input (0b00) or output (>0b00) with speed (0b01=max10MHz,0b10=max2MHz,0b11=max30MHz)
.equ CNFy, 0b11<<2              # configuration Output: push-pull (0b?0) /open-drain (0b?1), Universal (0b0?)/Multiplexed 0b1?)
                                #               Input: Analog (0b00)/Floating (0b01)/pull-up and pull-down (0b10)
.equ Pin0, (MODEy | CNFy)<<0
.equ Pin1, (MODEy | CNFy)<<1
.equ Pin2, (MODEy | CNFy)<<2
.equ Pin3, (MODEy | CNFy)<<3
.equ Pin4, (MODEy | CNFy)<<4
.equ Pin5, (MODEy | CNFy)<<5
.equ Pin6, (MODEy | CNFy)<<6
.equ Pin7, (MODEy | CNFy)<<7

 # 
.equ INDR, 0x008                # PC port input data register; must read 16 bits, but only lo8 are valid (offset from R32_GPIOC)
.equ OUTDR, 0x00c               # PC port output data register; output operated in 16-bit form (offset from R32_GPIOC)
.equ BSHR, 0x010                # PC port set/reset register (offset from R32_GPIOC)
 # BSHR bitmasks
.equ BSy, 0xff<<0               # in OUTDIR, 1=set, 0=no change; accessed in 16-bit form
.equ BRy, 0xff<<16              # in OUTDIR, 1=clear, 0=no change; accessed in 16-bit form

.equ BCR, 0x014                 # PC port reset register
 # BCR bitmasks
.equ BRy, 0xff<<0               # in OUTDIR, 1=clear, 0=no change; accessed in 16-bit form


.equ LCKR, 0x018                # PC port configuration lock register

.equ LCKy, 0xff<<0              # set bit to 1 to indicate locking the configuration of the corresponding port CFGLR
.equ LCKK, 1<<8                 # reads 0: no locking, 1: locked; to lock:write 1, 0, 1, read 0, 1

.equ R32_AFIO, 0x40010000       # Alternate Function IO Register base
 # AFIO offsets
.equ PCFR1, 0x004               # Remap Register 1 (offset from R32_AFIO)
  # PCFR1 bitmasks (see CH32V003 Reference Manual)
.equ SPI1_RM, 1<<0              # Remapping of SPI1
.equ I2C1_RM, 1<<1              # I2C1 remapping low bit
.equ USART1_RM, 1<<2            # USART1 mapping configuration low bit
.equ TIM1_RM, 0b11<<6           # Remap bits for timer 1
.equ TIM2_RM, 0b11<<8           # Remap bits for timer 2
.equ PA12_RM, 1<<15             # Pin PA1 & PA2 remapping bit (external crystal)
.equ ADC_ETRGINJ_RM, 1<<17      # Remap bit for ADC external trigger rule conversion
.equ ADC_ETRGREG_RM, 1<<18      # Remap bit for ADC external trigger rule conversion
.equ USART1_RM1, 1<<21          # USART1 mapping configuration high
.equ I2C1REMAP1, 1<<22          # I2C1 remapping high bit
.equ TIM1_IREMAP, 1<<23         # Control timer 1 channel 1 selection
.equ SWCFG, 0b111<<24           # configure the I/O ports for SW function and trace function; 
                                #  0b0xx: SWD (SDI) enabled, 100: Turn off SWD (SDI), which functions as a GPIO

.equ EXTICR, 0x008              # External interrupt configuration register 1 (offset from R32_AFIO)
 # EXTICR bitmasks
.equ EXTIx, 0b11<<0             # (x=0-7), external interrupt input pin configuration bit; 0b00:PA, 0b10:PC, 0b11:PD
  # EXTI5 is EXTIx<<(x*2) = EXTICR(11:10)

.equ R32_RCC, 0x40021000        # Reset and Clock Control (RCC) registers
 # RCC offsets
.equ AHBPCENR, 0x14             # HB peripheral clock enable register (offset from R32_RCC)
  # AHBPCENR bitmasks
.equ DMA1EN, 1<<0               # DMA1 module clock enable bit
.equ SRAMEN, 1<<2               # SRAM interface module clock enable bit

.equ APB2PCENR, 0x18            # PB2 peripheral clock enable register (offset from R32_RCC)
  # APB2PCENR bitmasks
.equ AFIOEN, 1<<0               # I/O auxiliary function module clock enable bit
.equ IOPAEN, 1<<2               # PA port module clock enable bit for I/O
.equ IOPCEN, 1<<4               # PC port module clock enable bit for I/O
.equ IOPDEN, 1<<5               # PD port module clock enable bit for I/O
.equ ADC1EN, 1<<9               # ADC1 module clock enable bit
.equ TIM1EN, 1<<11              # TIM1 module clock enable bit 
.equ SPI1EN, 1<<12              # SPI1 interface clock enable bit 
.equ USART1EN, 1<<14            # USART1 interface clock enable bit

.equ R32_DMA, 0x40020000        # DMA-related registers
 # DMA offsets
.equ INTFR, 0x00                # DMA interrupt status register (offset from R32_DMA)
  # INTFR bitmasks
.equ GIFx, 1<<0                 # Global interrupt flag for channel x (x=1/2/3/4/5/6/7)
.equ TCIFx, 1<<1                # Transmission completion flag for channel x (x=1/2/3/4/5/6/7)
.equ HTIFx, 1<<2                # Transmission halfway flag for channel x (x=1/2/3/4/5/6/7)
.equ TEIFx, 1<<3                # Transmission error flag for channel x (x=1/2/3/4/5/6/7)
   # address offset for each channel is 4
   # so TEIF3 is TEIFx<<(x*4)-4 = TEIFx<<(3*4)-4 = bit (3*4)-4+3 = bit 11

.equ INTFCR, 0x04               # DMA interrupt flag clear register (offset from R32_DMA)
  # INTFCR bitmasks
.equ CGIFx, 1<<0                # Clear the global interrupt flag for channel x
.equ CTCIFx, 1<<1               # Clear the transmission completion flag for channel x
.equ CHTIFx, 1<<2               # Clear the transmission halfway flag for channel x
.equ CTEIFx, 1<<3               # Clear the transmission error flag for channel x
   # address offset for each channel is 4
   # so CTCIF5 is CTCIFx<<(x*4)-4 = CTCIFx<<(5*4)-4 = bit (5*4)-4+1 = bit 17

.equ CFGRx, 0x08                # offset +(x-1)*20; DMA channel x configuration register (offset from R32_DMA)
  # CFGRx bitmasks
.equ EN, 1<<0                   # Channel enable control
.equ TCIE, 1<<1                 # Transmission completion interrupt enable control
.equ HTIE, 1<<2                 # Transmission over half interrupt enable control
.equ TEIE, 1<<3                 # Transmission error interrupt enable control
.equ DIR, 1<<4                  # Data transfer direction (0b0=Read from peripheral; 0b1=Read from memory)
.equ CIRC, 1<<5                 # DMA channel cyclic mode enable
.equ PINC, 1<<6                 # Peripheral address incremental incremental mode enable
.equ MINC, 1<<7                 # Memory address incremental incremental mode enable
.equ PSIZE, 0b11<<8             # Peripheral address data width setting (0b00: 8 bits; 0b01: 16 bits; 0b10: 32 bits)
.equ MSIZE, 0b11<<10            # Memory address data width setting (0b00: 8 bits; 0b01: 16 bits; 0b10: 32 bits)
.equ PL, 0b11<<12               # Channel priority setting (0b00: low; 0b01: medium; 0b10: high; 0b11: very high)
.equ MEM2MEM, 1<<14             # Memory-to-memory mode enable

.equ CNTRx, 0x0c                # DMA channel x number of data register (offset from R32_DMA)
  # CNTRx bitmasks
.equ NDT, 0xffff                # Number of data transfers, range 0-65535; only write when EN=0, else counter

.equ PADDRx, 0x10               # DMA channel x peripheral address register = peripheral base address, 
                                #  which serves as the source or destination address for peripheral data transfer  (offset from R32_DMA)

.equ MADDRx, 0x14               # DMA channel x memory address register = memory data address, 
                                #  which serves as the source or destination address for data transfers (offset from R32_DMA)
   # address offset for each channel is 20 (0x14)
   # so PADDR7 is PADDRx+(x-1)*20 = PADDRx+(7-1)*20 = PADDRx+120 = PADDRx+0x78 = 0x88

.equ CFGR1, 0x08                # DMA channel 1 configuration register
.equ CNTR1, 0x0c                # DMA channel 1 number of data register
.equ PADDR1, 0x10               # DMA channel 1 peripheral address register
.equ MADDR1, 0x14               # DMA channel 1 memory address register
.equ CFGR2, 0x1c                # DMA channel 2 configuration register
.equ CNTR2, 0x20                # DMA channel 2 number of data register
.equ PADDR2, 0x24               # DMA channel 2 peripheral address register
.equ MADDR2, 0x28               # DMA channel 2 memory address register
.equ CFGR3, 0x30                # DMA channel 3 configuration register
.equ CNTR3, 0x34                # DMA channel 3 number of data register
.equ PADDR3, 0x38               # DMA channel 3 peripheral address register
.equ MADDR3, 0x3c               # DMA channel 3 memory address register
.equ CFGR4, 0x44                # DMA channel 4 configuration register
.equ CNTR4, 0x48                # DMA channel 4 number of data register
.equ PADDR4, 0x4c               # DMA channel 4 peripheral address register
.equ MADDR4, 0x50               # DMA channel 4 memory address register
.equ CFGR5, 0x58                # DMA channel 5 configuration register
.equ CNTR5, 0x5c                # DMA channel 5 number of data register
.equ PADDR5, 0x60               # DMA channel 5 peripheral address register
.equ MADDR5, 0x64               # DMA channel 5 memory address register
.equ CFGR6, 0x6c                # DMA channel 6 configuration register
.equ CNTR6, 0x70                # DMA channel 6 number of data register
.equ PADDR6, 0x74               # DMA channel 6 peripheral address register
.equ MADDR6, 0x78               # DMA channel 6 memory address register
.equ CFGR7, 0x80                # DMA channel 7 configuration register
.equ CNTR7, 0x84                # DMA channel 7 number of data register
.equ PADDR7, 0x88               # DMA channel 7 peripheral address register
.equ MADDR7, 0x8c               # DMA channel 7 memory address register

 # System Timer (SysTick) - ref:QingKeV2 Microprocessor Manual
.equ STK_CTLR, 0xE000F000       # System count control register
 # STK_CTLR bitmasks
.equ STE, 1<<0                  # System counter enable control bit
.equ STIE, 1<<1                 # Counter interrupt enable control bit
.equ STCLK, 1<<2                # Counter clock source selection bit (0b0:HCLK/8; 0b1:HCLK)
.equ STRE, 1<<3                 # Auto-reload Count enable bit
.equ SWIE, 1<<31                # Software interrupt trigger enable (SWI)

.equ STK_SR, 0xE000F004         # System count status register
 #
.equ CNTIF, 1>>0                # Counting value comparison flag (write 0 = clear, read 1 = upward counting)

.equ STK_CNTR, 0xE000F008       # System counter register
.equ STK_CMPR, 0xE000F010       # System count comparison value register


# allocate bytes in RAM for horizontal scan line
.equ PCBYTES, RAM_BASE

# timings for 640x460 at 60Hz according to Javier Valcarce
# front porch = 0.636uS; count=30.528 at 48MHz, say 31
.equ HORFRONTPORCH, 31          # PCBYTES[0:30]
# horizontal sync = 3.813uS duration; count=183.024 at 48MHz, say 183
.equ HORSYNC, 183               # PCBYTES[31:213]
.equ HS_OFFSET, HORFRONTPORCH
# back porch = 1.907uS; count = 91.536, say 91
.equ HORBACKPORCH, 91           # PCBYTES[214:304]
.equ HBP_OFFSET, HS_OFFSET+HORSYNC
# active video = 25.422uS; count=1220.256 at 48MHz, say 1220
.equ HORVIDEO, 1220            # PCBYTES[305:1524]
.equ HV_OFFSET, HBP_OFFSET+HORBACKPORCH

.equ PCBYTES_SIZE, HORFRONTPORCH+HORSYNC+HORBACKPORCH+HORVIDEO   # =1525
# next avail RAM+1525

# use top 0x200 bytes for timing list
.equ LISTtiming, PCBYTES+0x600
.equ LISTentries, 0x100 # half-words
# each half-word with low 8 bits = low 8 bits of SYSTICK, and top 8 bits with OUTDR of port C

Init:
# initialise memory for DMA to drive port PC

# Default all bytes in PCBYTES to 0x010 (HSync hi, video lo)
    li a1, 0b010                    # a1 = HSync hi, video lo for Port C
    li a5, PCBYTES                  # a5 = scan buffer base address
    li a0, PCBYTES_SIZE             # a0 = offset to just past end
1:
    addi a0, a0, -1                 # decrement offset into PCBYTES
    add a2, a5, a0                  # a2 = sum of offset and scan buffer base address
#    sb a1, 0(a2)                    # set byte at a2 to the default for Port C
    sb a2, 0(a2) # temp: set scan line buffer to lo8 of buffer address
    bgtz a0, 1b                     # loop until back to start of PCBYTES

# Setup HSync pulse (lo) after front porch
    li a1, 0b000                    # a1 = HSync lo, video lo for Port C
    li a5, PCBYTES                  # a5 = scan buffer base address
    addi a5, a5, HS_OFFSET          # a5 = scan buffer start of HSync
    li a0, HORSYNC                  # a0 = size of HSync pulse
2:
    addi a0, a0, -1                 # decrement a0 to be offset into HSync pulse
    add a2, a5, a0                  # a2 = scan buffer address for HSync pulse
#    sb a1, 0(a2)                    # set the HSync pulse for Port C into scan line buffer
    sb a2, 0(a2) # temp: set scan line buffer to lo8 of buffer address
    bgtz a0, 2b                     # loop until back to start of horizontal sync

    
# white video in bit 2, starting after all blanking (=0.636+3.813+1.907uS) to end
    li a1, 0b110                    # a1 = HSync hi, video hi for Port C
    li a5, PCBYTES                  # a5 = scan buffer base address
    addi a5, a5, HV_OFFSET          # a5 = scan buffer start of active video
    li a0, HORVIDEO                 # a0 = size of active video
3:
    addi a0, a0, -1                 # decrement a0 to be offset into active video
    add a2, a5, a0                  # a2 = scan buffer address for active video
#    sb a1, 0(a2)                    # set the active video for Port C into scan line buffer
    sb a0, 0(a2) # temp: set scan line buffer to lo8 of buffer address
    bgtz a0, 3b                     # loop until back to start of active video

# enable GPIO peripheral so port PC works
# # set PC port module clock enable bit for I/O
    li a0, R32_RCC                  # a0 = base address for Reset and Clock Control registers
    lw t1, APB2PCENR(a0)            # t1 = read PB2 peripheral clock enable register
    ori t1, t1, IOPCEN              # set PC port module clock enable bit for I/O
    sw t1, APB2PCENR(a0)            # set APB2PCENR register with t1
    
# # set pin hardware interface, including speed
    li a0, R32_GPIOC                # a0 = base address for GPIO Port C registers
    li t1, 0x44444334               # t1 = value PC1&2 Output mode, maximum speed 30MHz, push-pull
    sw t1, CFGLR(a0)                # set CFGLR register with t1

# enable and set up DMA peripheral
 # set DMA1 module clock enable bit
    li a0, R32_RCC                  # a0 = base address for Reset and Clock Control registers
    lw t1, AHBPCENR(a0)             # read AHB Peripheral Clock Enable Register register into t1
    ori t1, t1, DMA1EN              # set PC port module clock enable bit for I/O
    sw t1, AHBPCENR(a0)             # set AHB Peripheral Clock Enable Register register with t1

 # choose highest priority DMA since time critical => CH1; so CFGR1 = CFGRx+(x-1)*20 = CFGRX+0
 # set DMA loop range 
    li a0, R32_DMA                  # a0 = base address for DMA-related registers
    li t1, PCBYTES_SIZE             # count of bytes to do in cycle
    sw t1, CNTR1(a0)                # 
 # set DMA memory base source address 
    li t1, PCBYTES                  # start of list of bytes
    sw t1, MADDR1(a0)               # 
 # set peripheral address for target of DMA 
    li t1, R32_GPIOC+OUTDR          # direct to output register
    sw t1, PADDR1(a0)               #
 # configure DMA 
  # 0 EN = 0b0, Channel off
  # 1 TCIE = 0b0, Disable the transmission completion interrupt
  # 2 HTIE = 0b0, Disable the transmission over half interrupt
  # 3 TEIE = 0b0, Disable transmission error interrupt
  # 4 DIR = 0b1, Read from memory
  # 5 CIRC = 0b0, Disable cyclic operation
  # 6 PINC = 0b0, Peripheral address remains unchanged operation
  # 7 MINC = 0b1, Enable incremental memory address increment operation
  # 9:8 PSIZE = 0b01 Peripheral address data width 16 bits
  # 11:10 MSIZE = 0b00 Memory address data width 8 bits
  # 13:12 PL = 0b11, Channel priority setting Very high
  # 14 MEM2MEM = 0b1, Enable memory-to-memory data transfer mode
      # bits 432109876543210
    li t1, 0b111000110010000        # match up the config bits
    sw t1, (CFGRx+0)(a0)            # highest priority DMA channel
  # start DMA
    ori t1, t1, EN                  # enable DMA CH1
    sw t1, (CFGRx+0)(a0)            # highest priority DMA channel

# store systick and port C output in a circular list for debugging and speed check

# set up system timer
 # Using HCLK (48MHz) means 32-bit timer overflows each 89.478485333 seconds
    li a0, STK_CTLR                 # In System count control register
    li t0, STE|STRE|STCLK                 #  set enable, reload; choose HCLK, no interrupts
    sw t0, 0(a0)                    # done

    li a0, STK_CNTR                 # a0 = set up address of systick counter
    li a1, R32_GPIOC                # a1 = set up base address port C
    li a2, LISTtiming               # a2 = base address of circular buffer
    li a3, LISTentries*2            # a3 = limit of circular list in bytes
    li a4, 0                        # a4 = current entry pointer, initialised at 0
9:
    lw t0, 0(a0)                    # t0 = systick counter
    lh t1, OUTDR(a1)                # t1 = output data register read as 16-bit
    add t2, a4, a2                  # t2 = current entry address
    sb t0, 0(t2)                    # store lo8 of systick
    sb t1, 1(t2)                    # store lo8 of PC OUTDR
    addi a4, a4, 2                  # increment current entry pointer by a half-word
    blt a4, a3, 9b                  # if not over the end, around again
    li a4, 0                        # else back to beginning of circular buffer
# maybe tickle DMA
    li t2, R32_DMA                  # t2 = base address for DMA-related registers
    lw t1, CNTR1(t2)                # t1 = counter of remaining DMA bytes to transfer
    sw t1,  -4(a2)                  # save counter before timing list
    bne t1, zero, 9b                # if DMA still enabled, let it run, otherwise
  # restart DMA
    lw t1, CFGR1(t2)
        # can only update counter when EN=0
    li t0, 0xfffffffe
    and t1, t1, t0                  # turn off DMA CH1
    sw t1,  -8(a2)                  # save t1 before timing list
    sw t1, (CFGR1)(t2)              #  in highest priority DMA channel
        # update counter
    li t1, PCBYTES_SIZE             # count of bytes to do in cycle
    sw t1, CNTR1(t2)                # reset counter 
        # re-enable DMA CH1
    lw t1, CFGR1(t2)
    li t0, EN|MEM2MEM
    or t1, t1, t0                   # enable DMA CH1
    sw t1, (CFGR1)(t2)              # highest priority DMA channel

    j 9b                            # just loop forever!

