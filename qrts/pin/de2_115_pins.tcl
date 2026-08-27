## ============================================================================
## de2_115_pins.tcl — DE2-115 pin assignments
##
## Sourced by dvcon.qsf. Kept as a separate file because the pinout belongs to
## the BOARD, not to this design: a different carrier reuses everything else in
## the project and replaces only this.
##
## ----------------------------------------------------------------------------
## SOURCE OF TRUTH: resources/DE2-115 Pin Assignments.csv
##
## Every location here is checked against that table by tools/check_pins.py,
## which the build runs before the fitter. Edit this file, not the .qsf -- if
## the Quartus GUI flattens these assignments back into dvcon.qsf, the pin
## check fails until the duplicate is removed.
##
## This file was previously transcribed by hand from the user manual and was
## wrong in nine places: DRAM_DQ[26:30] shifted one position (putting
## DRAM_DQ[26] on SRAM_ADDR[9]), DRAM_DQ[28] and [31] missing entirely, and
## three ENET0 pins landing on ENETCLK_25 and on PHY 1. Every one of those is
## a bidirectional or driven pin fighting a real external driver. Do not
## hand-edit a location without running the pin check.
## ----------------------------------------------------------------------------

## ---------------------------------------------------------------------------
## Device and global settings
## ---------------------------------------------------------------------------
set_global_assignment -name FAMILY "Cyclone IV E"
set_global_assignment -name DEVICE EP4CE115F29C7

## Unused pins: the DE2-115 has peripherals wired to pins this design does not
## drive. Leaving them as the default (output driving ground) fights whatever
## is on the other side; tri-stated with a weak pull-up is the safe choice.
set_global_assignment -name RESERVE_ALL_UNUSED_PINS "AS INPUT TRI-STATED WITH WEAK PULL-UP"

## 3.3 V LVTTL is the DE2-115's default bank voltage.
set_global_assignment -name STRATIX_DEVICE_IO_STANDARD "3.3-V LVTTL"

## ---------------------------------------------------------------------------
## Clocks and keys
## ---------------------------------------------------------------------------
set_location_assignment PIN_Y2  -to CLOCK_50

set_location_assignment PIN_M23 -to KEY[0]
set_location_assignment PIN_M21 -to KEY[1]
set_location_assignment PIN_N21 -to KEY[2]
set_location_assignment PIN_R24 -to KEY[3]

## ---------------------------------------------------------------------------
## SDRAM — IS42S16320D x2, 128 MB total, 32-bit wide
##
## The DE2-115 wires two 16-bit devices in parallel to make a 32-bit bus, which
## is why DRAM_DQ is 32 bits and there are four DQM lines. Both share address,
## bank, and control.
## ---------------------------------------------------------------------------
set_location_assignment PIN_R6  -to DRAM_ADDR[0]
set_location_assignment PIN_V8  -to DRAM_ADDR[1]
set_location_assignment PIN_U8  -to DRAM_ADDR[2]
set_location_assignment PIN_P1  -to DRAM_ADDR[3]
set_location_assignment PIN_V5  -to DRAM_ADDR[4]
set_location_assignment PIN_W8  -to DRAM_ADDR[5]
set_location_assignment PIN_W7  -to DRAM_ADDR[6]
set_location_assignment PIN_AA7 -to DRAM_ADDR[7]
set_location_assignment PIN_Y5  -to DRAM_ADDR[8]
set_location_assignment PIN_Y6  -to DRAM_ADDR[9]
set_location_assignment PIN_R5  -to DRAM_ADDR[10]
set_location_assignment PIN_AA5 -to DRAM_ADDR[11]
set_location_assignment PIN_Y7  -to DRAM_ADDR[12]

set_location_assignment PIN_U7  -to DRAM_BA[0]
set_location_assignment PIN_R4  -to DRAM_BA[1]

set_location_assignment PIN_V7  -to DRAM_CAS_N
set_location_assignment PIN_U6  -to DRAM_RAS_N
set_location_assignment PIN_V6  -to DRAM_WE_N
set_location_assignment PIN_T4  -to DRAM_CS_N
set_location_assignment PIN_AA6 -to DRAM_CKE
set_location_assignment PIN_AE5 -to DRAM_CLK

set_location_assignment PIN_W3  -to DRAM_DQ[0]
set_location_assignment PIN_W2  -to DRAM_DQ[1]
set_location_assignment PIN_V4  -to DRAM_DQ[2]
set_location_assignment PIN_W1  -to DRAM_DQ[3]
set_location_assignment PIN_V3  -to DRAM_DQ[4]
set_location_assignment PIN_V2  -to DRAM_DQ[5]
set_location_assignment PIN_V1  -to DRAM_DQ[6]
set_location_assignment PIN_U3  -to DRAM_DQ[7]
set_location_assignment PIN_Y3  -to DRAM_DQ[8]
set_location_assignment PIN_Y4  -to DRAM_DQ[9]
set_location_assignment PIN_AB1 -to DRAM_DQ[10]
set_location_assignment PIN_AA3 -to DRAM_DQ[11]
set_location_assignment PIN_AB2 -to DRAM_DQ[12]
set_location_assignment PIN_AC1 -to DRAM_DQ[13]
set_location_assignment PIN_AB3 -to DRAM_DQ[14]
set_location_assignment PIN_AC2 -to DRAM_DQ[15]
set_location_assignment PIN_M8  -to DRAM_DQ[16]
set_location_assignment PIN_L8  -to DRAM_DQ[17]
set_location_assignment PIN_P2  -to DRAM_DQ[18]
set_location_assignment PIN_N3  -to DRAM_DQ[19]
set_location_assignment PIN_N4  -to DRAM_DQ[20]
set_location_assignment PIN_M4  -to DRAM_DQ[21]
set_location_assignment PIN_M7  -to DRAM_DQ[22]
set_location_assignment PIN_L7  -to DRAM_DQ[23]
set_location_assignment PIN_U5  -to DRAM_DQ[24]
set_location_assignment PIN_R7  -to DRAM_DQ[25]
set_location_assignment PIN_R1  -to DRAM_DQ[26]
set_location_assignment PIN_R2  -to DRAM_DQ[27]
set_location_assignment PIN_R3  -to DRAM_DQ[28]
set_location_assignment PIN_T3  -to DRAM_DQ[29]
set_location_assignment PIN_U4  -to DRAM_DQ[30]
set_location_assignment PIN_U1  -to DRAM_DQ[31]

## ---------------------------------------------------------------------------
## DRAM_DQ[31:26] -- RESOLVED against the Terasic pin table.
##
## These six were transcribed wrong. resources/DE2-115 Pin Assignments.csv (the
## Terasic-supplied table, now in this tree) gives the whole group as
## R1/R2/R3/T3/U4/U1 for bits 26..31, all in bank 2 with their siblings.
##
## What was here before was not a two-pin gap. Bits 26..30 were shifted by one
## against the real table, so DRAM_DQ[26] sat on PIN_T7 -- which is SRAM_ADDR[9]
## on this board -- and [28] and [31] were left unassigned, which put them in
## bank 6. Every one of those drives a real net.
##
## The old note's "free pins are T1, T2, T5, T6, U1, U4, P3" was reasoning from
## what the fitter would accept, not from the board. T2 and T1 are rejected
## because they are not DRAM pins at all; U1 and U4 appeared in that list only
## by coincidence.
##
## check_pins.py now cross-checks every location in this file against that CSV,
## so a wrong-but-legal pin fails the build instead of reaching the bench.
## ---------------------------------------------------------------------------

set_location_assignment PIN_U2  -to DRAM_DQM[0]
set_location_assignment PIN_W4  -to DRAM_DQM[1]
set_location_assignment PIN_K8  -to DRAM_DQM[2]
set_location_assignment PIN_N8  -to DRAM_DQM[3]

## ---------------------------------------------------------------------------
## Ethernet PHY 0 — Marvell 88E1111, RGMII
##
## RGMII is the only gigabit option on this board: the header exposes 4-bit
## TX/RX data plus ENET0_GTX_CLK, not the 8-bit GMII bus.
## ---------------------------------------------------------------------------
set_location_assignment PIN_A17 -to ENET0_GTX_CLK
set_location_assignment PIN_A15 -to ENET0_RX_CLK
set_location_assignment PIN_C16 -to ENET0_RX_DATA[0]
set_location_assignment PIN_D16 -to ENET0_RX_DATA[1]
set_location_assignment PIN_D17 -to ENET0_RX_DATA[2]
set_location_assignment PIN_C15 -to ENET0_RX_DATA[3]
set_location_assignment PIN_C17 -to ENET0_RX_DV
set_location_assignment PIN_C18 -to ENET0_TX_DATA[0]
set_location_assignment PIN_D19 -to ENET0_TX_DATA[1]
set_location_assignment PIN_A19 -to ENET0_TX_DATA[2]
set_location_assignment PIN_B19 -to ENET0_TX_DATA[3]
set_location_assignment PIN_A18 -to ENET0_TX_EN
set_location_assignment PIN_C19 -to ENET0_RST_N
set_location_assignment PIN_B21 -to ENET0_MDIO
set_location_assignment PIN_C20 -to ENET0_MDC

## ---------------------------------------------------------------------------
## LEDs
## ---------------------------------------------------------------------------
set_location_assignment PIN_G19 -to LEDR[0]
set_location_assignment PIN_F19 -to LEDR[1]
set_location_assignment PIN_E19 -to LEDR[2]
set_location_assignment PIN_F21 -to LEDR[3]
set_location_assignment PIN_F18 -to LEDR[4]
set_location_assignment PIN_E18 -to LEDR[5]
set_location_assignment PIN_J19 -to LEDR[6]
set_location_assignment PIN_H19 -to LEDR[7]
set_location_assignment PIN_J17 -to LEDR[8]
set_location_assignment PIN_G17 -to LEDR[9]
set_location_assignment PIN_J15 -to LEDR[10]
set_location_assignment PIN_H16 -to LEDR[11]
set_location_assignment PIN_J16 -to LEDR[12]
set_location_assignment PIN_H17 -to LEDR[13]
set_location_assignment PIN_F15 -to LEDR[14]
set_location_assignment PIN_G15 -to LEDR[15]
set_location_assignment PIN_G16 -to LEDR[16]
set_location_assignment PIN_H15 -to LEDR[17]

set_location_assignment PIN_E21 -to LEDG[0]
set_location_assignment PIN_E22 -to LEDG[1]
set_location_assignment PIN_E25 -to LEDG[2]
set_location_assignment PIN_E24 -to LEDG[3]
set_location_assignment PIN_H21 -to LEDG[4]
set_location_assignment PIN_G20 -to LEDG[5]
set_location_assignment PIN_G22 -to LEDG[6]
set_location_assignment PIN_G21 -to LEDG[7]
set_location_assignment PIN_F17 -to LEDG[8]

## ---------------------------------------------------------------------------
## I/O standards
##
## Everything on this board's user I/O is 3.3 V LVTTL. Setting it explicitly
## rather than relying on the default keeps a bank voltage change from silently
## reassigning it.
## ---------------------------------------------------------------------------
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to CLOCK_50
## KEY[3:0] sit in banks 6 and 5, which are 2.5 V on this board -- not the
## 3.3 V these were declared as. Quartus accepts the mismatch and picks a
## 3.3 V input threshold against a 2.5 V driver.
set_instance_assignment -name IO_STANDARD "2.5 V" -to KEY[0]
set_instance_assignment -name IO_STANDARD "2.5 V" -to KEY[1]
set_instance_assignment -name IO_STANDARD "2.5 V" -to KEY[2]
set_instance_assignment -name IO_STANDARD "2.5 V" -to KEY[3]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to DRAM_ADDR[*]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to DRAM_BA[*]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to DRAM_DQ[*]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to DRAM_DQM[*]
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to DRAM_CAS_N
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to DRAM_RAS_N
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to DRAM_WE_N
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to DRAM_CS_N
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to DRAM_CKE
set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to DRAM_CLK
set_instance_assignment -name IO_STANDARD "2.5 V" -to ENET0_GTX_CLK
set_instance_assignment -name IO_STANDARD "2.5 V" -to ENET0_RX_CLK
set_instance_assignment -name IO_STANDARD "2.5 V" -to ENET0_RX_DATA[*]
set_instance_assignment -name IO_STANDARD "2.5 V" -to ENET0_RX_DV
set_instance_assignment -name IO_STANDARD "2.5 V" -to ENET0_TX_DATA[*]
set_instance_assignment -name IO_STANDARD "2.5 V" -to ENET0_TX_EN
set_instance_assignment -name IO_STANDARD "2.5 V" -to ENET0_RST_N
set_instance_assignment -name IO_STANDARD "2.5 V" -to ENET0_MDIO
set_instance_assignment -name IO_STANDARD "2.5 V" -to ENET0_MDC
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDR[*]
set_instance_assignment -name IO_STANDARD "2.5 V" -to LEDG[*]

## SDRAM outputs switch 32 data lines at once. The default drive strength rings
## on a bus that long; the DE2-115 reference designs use the fast slew setting
## with 8 mA, which is what the memory's timing was characterised against.
set_instance_assignment -name CURRENT_STRENGTH_NEW "8MA" -to DRAM_DQ[*]
set_instance_assignment -name CURRENT_STRENGTH_NEW "8MA" -to DRAM_ADDR[*]
set_instance_assignment -name CURRENT_STRENGTH_NEW "8MA" -to DRAM_CLK
