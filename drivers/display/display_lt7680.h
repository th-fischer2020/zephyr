/*
 * Copyright (c) 2025 JUMO GmbH & Co. KG
 * SPDX-License-Identifier: Apache-2.0
 *
 * Levetop LT768x Display Controller Driver - Register definitions
 *
 * LT768 SPI wire protocol – bits[7:6] of the FIRST byte of each transaction
 * encode the access mode (lower 6 bits are irrelevant for the access type):
 *
 *   0x00 (bits[7:6]=00) : select register address   -> 2nd byte = reg addr
 *   0x40 (bits[7:6]=01) : read status               -> 2nd byte clocked back = status
 *   0x80 (bits[7:6]=10) : write data                -> 2nd+ bytes = data
 *   0xC0 (bits[7:6]=11) : read data                 -> 2nd byte clocked back = value
 *
 * Every register write is TWO separate CS transactions:
 *   TX1: [0x00, reg_addr]           (select register)
 *   TX2: [0x80, data_value]         (write data)
 *
 * Every register read is TWO separate CS transactions:
 *   TX1: [0x00, reg_addr]           (select register)
 *   TX2: [0xC0, 0xFF], RX: [_, val] (read data; second RX byte = value)
 *
 * Status read is ONE transaction:
 *   TX: [0x40, 0xFF], RX: [_, status] (second RX byte = status byte)
 *
 * Memory (pixel) burst write is TWO transaction:
 *   TX1: [0x00, LT7680_REG_MRWDP]   (select memory R/W data port)
 *   TX2: [0x80, p0, p1, p2, ...]     (data-write mode + continuous pixel stream)
 */

#ifndef ZEPHYR_DRIVERS_DISPLAY_LT7680_H_
#define ZEPHYR_DRIVERS_DISPLAY_LT7680_H_

#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// =========================================================================
// SPI access-mode bytes
// =========================================================================
#define LT7680_SPI_SEL_REG          0x00u
#define LT7680_SPI_STATUS_READ      0x40u
#define LT7680_SPI_DATA_WRITE       0x80u
#define LT7680_SPI_DATA_READ        0xC0u

// =========================================================================
// Register Map
// =========================================================================
// Software Reset Register
#define LT7680_REG_SWRESET           0x00u
#define LT7680_SWRESET_TRIG          0x01u
#define LT7680_SWRESET_PLL_TRIG      0x80u

// Chip Configuration Register
#define LT7680_REG_CCR               0x01u
#define LT7680_CCR_PLL_ENABLE        0x80u
#define LT7680_CCR_SLEEP             0x40u
#define LT7680_CCR_TFT_24BIT         0x00u
#define LT7680_CCR_TFT_18BIT         0x08u
#define LT7680_CCR_TFT_16BIT         0x10u
#define LT7680_CCR_TFT_LVDS          0x18u
#define LT7680_CCR_HOST_8BIT         0x00u
#define LT7680_CCR_HOST_16BIT        0x01u

// Memory Configuration Register
#define LT7680_REG_MCR               0x02u
// RGB panel bus width + bpp for 16-bit host bus:
#define LT7680_MCR_RGB16_16BPP       0x40u  // bits[7:6]=01: 16-bit bus, 16 bpp
#define LT7680_MCR_RGB16_24BPP_M1    0x00u  // bits[7:6]=00: 16-bit bus, 24 bpp mode 1
#define LT7680_MCR_RGB16_24BPP_M2    0xC0u  // bits[7:6]=11: 16-bit bus, 24 bpp mode 2
// Memory write direction bits[2:1]:
#define LT7680_MCR_MWRITE_LR_TB      0x00u  // left→right, top→bottom
#define LT7680_MCR_MWRITE_RL_TB      0x02u  // right→left, top→bottom
#define LT7680_MCR_MWRITE_TD_LR      0x04u  // top→down,   left→right
#define LT7680_MCR_MWRITE_DT_LR      0x06u  // down→top,   left→right

// Interface Control Register
#define LT7680_REG_ICR               0x03u
#define LT7680_ICR_GRAPHIC_MODE      0x00u  // bit2=0: graphic mode
#define LT7680_ICR_TEXT_MODE         0x04u  // bit2=1: text mode
#define LT7680_ICR_MEM_SDRAM         0x00u  // bits[1:0]=00: SDRAM
#define LT7680_ICR_MEM_CURSOR_RAM    0x02u  // bits[1:0]=10: graphic cursor RAM
#define LT7680_ICR_MEM_PALETTE_RAM   0x03u  // bits[1:0]=11: colour palette RAM

// Memory R/W Data Port
#define LT7680_REG_MRWDP             0x04u

// --- [05h-0Ah] PLL Registers ---
// Format for registers 0x05, 0x07, 0x09:
//   bits[7:6] = lpllOD  (output divider, actual_div = 2^(OD-1), OD=1..3)
//   bits[4:1] = lpllR   (input divider R, ≥1)
//   bit[0]    = lpllN[8] (MSB of 9-bit multiplier N)
// Registers 0x06, 0x08, 0x0A hold lpllN[7:0].
// PLL output frequency:  FOUT = XI_freq * N / (R * 2^(OD-1))
//   where XI_freq is the crystal frequency, N = 9-bit value,
//   R is the 4-bit input divider.
#define LT7680_REG_SCLK_PLL1         0x05u  // SCLK (pixel clock) PLL control
#define LT7680_REG_SCLK_PLL_N        0x06u  // SCLK PLL multiplier N[7:0]
#define LT7680_REG_MCLK_PLL1         0x07u  // MCLK (SDRAM clock) PLL control
#define LT7680_REG_MCLK_PLL_N        0x08u  // MCLK PLL multiplier N[7:0]
#define LT7680_REG_CCLK_PLL1         0x09u  // CCLK (core clock) PLL control
#define LT7680_REG_CCLK_PLL_N        0x0Au  // CCLK PLL multiplier N[7:0]

// Display Panel Control Register
#define LT7680_REG_DPCR              0x12u
#define LT7680_DPCR_PCLK_FALLING     0x80u  // PDAT driven at PCLK rising edge (panel clocks at falling)
#define LT7680_DPCR_PCLK_RISING      0x00u  // PDAT driven at PCLK falling edge
#define LT7680_DPCR_DISPLAY_ON       0x40u  // Display ON
#define LT7680_DPCR_DISPLAY_OFF      0x00u  // Display OFF
#define LT7680_DPCR_VSCAN_T_TO_B     0x00u   // vertical scan top→bottom
#define LT7680_DPCR_VSCAN_B_TO_T     0x08u   // vertical scan bottom→top
#define LT7680_DPCR_PDATA_RGB        0x00u  // RGB output order
#define LT7680_DPCR_PDATA_BGR        0x05u  // BGR output order

// Horizontal/Display Control Register
#define LT7680_REG_HDCR              0x13u
#define LT7680_HDCR_HSYNC_LOW_ACT    0x00u  // bit7=0: HSYNC active low
#define LT7680_HDCR_HSYNC_HIGH_ACT   0x80u  // bit7=1: HSYNC active high
#define LT7680_HDCR_VSYNC_LOW_ACT    0x00u  // bit6=0: VSYNC active low
#define LT7680_HDCR_VSYNC_HIGH_ACT   0x40u  // bit6=1: VSYNC active high
#define LT7680_HDCR_DE_HIGH_ACT      0x00u  // bit5=0: DE active high
#define LT7680_HDCR_DE_LOW_ACT       0x20u  // bit5=1: DE active low

// Panel horizontal/vertical size
// [14h] HDWR:  Horizontal display width = (HDWR + 1) * 8 + HDWFTR  pixels
// [15h] HDWFTR: fine tuning [3:0], 0..7 pixels
// [1Ah] VDHR0: Vertical display height (LSB) = VDHR + 1  lines
// [1Bh] VDHR1: Vertical display height [10:8]
#define LT7680_REG_HDWR              0x14u
#define LT7680_REG_HDWFTR            0x15u
#define LT7680_REG_VDHR0             0x1Au
#define LT7680_REG_VDHR1             0x1Bu

// Horizontal non-display period (back porch)
// pixels = (HNDR + 1) * 8 + HNDFTR
#define LT7680_REG_HNDR              0x16u
#define LT7680_REG_HNDFTR            0x17u

// HSYNC start position (front porch)
// pixels = (HSTR + 1) * 8
//
#define LT7680_REG_HSTR              0x18u

// HSYNC pulse width
// pixels = (HPW + 1) * 8
#define LT7680_REG_HPW               0x19u

// Vertical non-display period (back porch)
// lines = VNDR + 1
#define LT7680_REG_VNDR0             0x1Cu
#define LT7680_REG_VNDR1             0x1Du

// VSYNC start position (front porch)
// lines = VSTR + 1
#define LT7680_REG_VSTR              0x1Eu

// VSYNC pulse width
// lines = VPWR + 1
#define LT7680_REG_VPWR              0x1Fu

// Main Image Start Address
#define LT7680_REG_MISA0             0x20u
#define LT7680_REG_MISA1             0x21u
#define LT7680_REG_MISA2             0x22u
#define LT7680_REG_MISA3             0x23u

// Main Image Width (pixels, must be divisible by 4)
#define LT7680_REG_MIW0              0x24u
#define LT7680_REG_MIW1              0x25u

// Main Window Upper-Left corner XY
#define LT7680_REG_MWULX0            0x26u
#define LT7680_REG_MWULX1            0x27u
#define LT7680_REG_MWULY0            0x28u
#define LT7680_REG_MWULY1            0x29u

// Canvas Image Start Address
#define LT7680_REG_CVSSA0            0x50u
#define LT7680_REG_CVSSA1            0x51u
#define LT7680_REG_CVSSA2            0x52u
#define LT7680_REG_CVSSA3            0x53u

// Canvas Image Width
#define LT7680_REG_CVS_IMWTH0        0x54u
#define LT7680_REG_CVS_IMWTH1        0x55u

// Active Window Upper-Left XY
#define LT7680_REG_AWUL_X0           0x56u
#define LT7680_REG_AWUL_X1           0x57u
#define LT7680_REG_AWUL_Y0           0x58u
#define LT7680_REG_AWUL_Y1           0x59u

// Active Window Width / Height
#define LT7680_REG_AW_WTH0           0x5Au
#define LT7680_REG_AW_WTH1           0x5Bu
#define LT7680_REG_AW_HT0            0x5Cu
#define LT7680_REG_AW_HT1            0x5Du

// Cursor and addressing mode control
#define LT7680_REG_MPWCTR            0x5Eu
#define LT7680_MPWCTR_XY_MODE        0x00u  // bit2=0: block / XY addressing
#define LT7680_MPWCTR_LINEAR_MODE    0x04u  // bit2=1: linear addressing
#define LT7680_MPWCTR_8BPP           0x00u  // bits[1:0]=00: 8 bpp
#define LT7680_MPWCTR_16BPP          0x01u  // bits[1:0]=01: 16 bpp
#define LT7680_MPWCTR_24BPP          0x02u  // bits[1:0]=1x: 24 bpp

// Graphic Write Cursor (XY in block mode)
#define LT7680_REG_CURH0             0x5Fu
#define LT7680_REG_CURH1             0x60u
#define LT7680_REG_CURV0             0x61u
#define LT7680_REG_CURV1             0x62u

// SDRAM registers
#define LT7680_REG_SDRC              0xE0u  // SDRAM Control
#define LT7680_REG_SDRCAS            0xE1u  // SDRAM CAS latency (0x02=CAS2, 0x03=CAS3)
#define LT7680_REG_SDRITV0           0xE2u  // SDRAM refresh interval [7:0]
#define LT7680_REG_SDRITV1           0xE3u  // SDRAM refresh interval [9:8]
#define LT7680_REG_SDRINI            0xE4u  // SDRAM initialise: write 0x01

// SDRC default value (64 Mbit, 4 banks, row=13, col=9, 16-bit)
#define LT7680_SDRC_DEFAULT          0x29u  // 32MB SDRAM at 143 MHz
#define LT7680_SDRCAS_CAS3           0x03u

// =========================================================================
// Status register bit masks (read from SPI status byte)
// =========================================================================
#define LT7680_STATUS_MEM_WR_FIFO_FULL   0x80u
#define LT7680_STATUS_MEM_WR_FIFO_EMPTY  0x40u
#define LT7680_STATUS_2D_BUSY            0x08u
#define LT7680_STATUS_SDRAM_READY        0x04u
#define LT7680_STATUS_POWER_SAVING       0x02u
#define LT7680_STATUS_INTERRUPT          0x01u

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_DISPLAY_LT7680_H_ */
