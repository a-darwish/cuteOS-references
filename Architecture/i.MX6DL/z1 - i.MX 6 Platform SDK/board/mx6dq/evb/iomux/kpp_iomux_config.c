/*
 * Copyright (c) 2012, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// File: kpp_iomux_config.c

/* ------------------------------------------------------------------------------
 * <auto-generated>
 *     This code was generated by a tool.
 *     Runtime Version:3.4.0.0
 *
 *     Changes to this file may cause incorrect behavior and will be lost if
 *     the code is regenerated.
 * </auto-generated>
 * ------------------------------------------------------------------------------
*/

#include "iomux_config.h"
#include "registers/regsiomuxc.h"

// Function to configure IOMUXC for kpp module.
void kpp_iomux_config(void)
{
    // Config kpp.KEY_COL5 to pad SD2_CLK(C21)
    // HW_IOMUXC_SW_MUX_CTL_PAD_SD2_CLK_WR(0x00000012);
    // HW_IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_WR(0x000170B0);
    // HW_IOMUXC_KEY_COL5_SELECT_INPUT_WR(0x00000003);
    // Mux Register:
    // IOMUXC_SW_MUX_CTL_PAD_SD2_CLK(0x020E0354)
    //   SION [4] - Software Input On Field Reset: DISABLED
    //              Force the selected mux mode Input path no matter of MUX_MODE functionality.
    //     DISABLED (0) - Input Path is determined by functionality of the selected mux mode (regular).
    //     ENABLED (1) - Force input path of pad.
    //   MUX_MODE [2:0] - MUX Mode Select Field Reset: ALT5
    //                    Select iomux modes to be used for pad.
    //     ALT0 (0) - Select instance: usdhc2 signal: SD2_CLK
    //     ALT1 (1) - Select instance: ecspi5 signal: ECSPI5_SCLK
    //     ALT2 (2) - Select instance: kpp signal: KEY_COL5
    //     ALT3 (3) - Select instance: audmux signal: AUD4_RXFS
    //     ALT5 (5) - Select instance: gpio1 signal: GPIO1_IO10
    HW_IOMUXC_SW_MUX_CTL_PAD_SD2_CLK_WR(
            BF_IOMUXC_SW_MUX_CTL_PAD_SD2_CLK_SION_V(ENABLED) | 
            BF_IOMUXC_SW_MUX_CTL_PAD_SD2_CLK_MUX_MODE_V(ALT2));
    // Pad Control Register:
    // IOMUXC_SW_PAD_CTL_PAD_SD2_CLK(0x020E073C)
    //   HYS [16] - Hysteresis Enable Field Reset: ENABLED
    //     DISABLED (0) - CMOS input
    //     ENABLED (1) - Schmitt trigger input
    //   PUS [15:14] - Pull Up / Down Config. Field Reset: 100K_OHM_PU
    //     100K_OHM_PD (0) - 100K Ohm Pull Down
    //     47K_OHM_PU (1) - 47K Ohm Pull Up
    //     100K_OHM_PU (2) - 100K Ohm Pull Up
    //     22K_OHM_PU (3) - 22K Ohm Pull Up
    //   PUE [13] - Pull / Keep Select Field Reset: PULL
    //     KEEP (0) - Keeper Enabled
    //     PULL (1) - Pull Enabled
    //   PKE [12] - Pull / Keep Enable Field Reset: ENABLED
    //     DISABLED (0) - Pull/Keeper Disabled
    //     ENABLED (1) - Pull/Keeper Enabled
    //   ODE [11] - Open Drain Enable Field Reset: DISABLED
    //              Enables open drain of the pin.
    //     DISABLED (0) - Output is CMOS.
    //     ENABLED (1) - Output is Open Drain.
    //   SPEED [7:6] - Speed Field Reset: 100MHZ
    //     TBD (0) - TBD
    //     50MHZ (1) - Low (50 MHz)
    //     100MHZ (2) - Medium (100 MHz)
    //     200MHZ (3) - Maximum (200 MHz)
    //   DSE [5:3] - Drive Strength Field Reset: 40_OHM
    //     HIZ (0) - HI-Z
    //     240_OHM (1) - 240 Ohm
    //     120_OHM (2) - 120 Ohm
    //     80_OHM (3) - 80 Ohm
    //     60_OHM (4) - 60 Ohm
    //     48_OHM (5) - 48 Ohm
    //     40_OHM (6) - 40 Ohm
    //     34_OHM (7) - 34 Ohm
    //   SRE [0] - Slew Rate Field Reset: SLOW
    //             Slew rate control.
    //     SLOW (0) - Slow Slew Rate
    //     FAST (1) - Fast Slew Rate
    HW_IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_WR(
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_HYS_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_PUS_V(47K_OHM_PU) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_PUE_V(PULL) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_PKE_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_ODE_V(DISABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_SPEED_V(100MHZ) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_DSE_V(40_OHM) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_SRE_V(SLOW));
    // Pad SD2_CLK is involved in Daisy Chain.
    // Input Select Register:
    // IOMUXC_KEY_COL5_SELECT_INPUT(0x020E08E8)
    //   DAISY [1:0] - MUX Mode Select Field Reset: GPIO00_ALT2
    //                 Selecting Pads Involved in Daisy Chain.
    //     GPIO00_ALT2 (0) - Select signal kpp KEY_COL5 as input from pad GPIO00(ALT2).
    //     GPIO19_ALT0 (1) - Select signal kpp KEY_COL5 as input from pad GPIO19(ALT0).
    //     CSI0_DATA04_ALT3 (2) - Select signal kpp KEY_COL5 as input from pad CSI0_DATA04(ALT3).
    //     SD2_CLK_ALT2 (3) - Select signal kpp KEY_COL5 as input from pad SD2_CLK(ALT2).
    HW_IOMUXC_KEY_COL5_SELECT_INPUT_WR(
            BF_IOMUXC_KEY_COL5_SELECT_INPUT_DAISY_V(SD2_CLK_ALT2));

    // Config kpp.KEY_COL6 to pad SD2_DATA3(B22)
    // HW_IOMUXC_SW_MUX_CTL_PAD_SD2_DATA3_WR(0x00000012);
    // HW_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_WR(0x000170B0);
    // HW_IOMUXC_KEY_COL6_SELECT_INPUT_WR(0x00000002);
    // Mux Register:
    // IOMUXC_SW_MUX_CTL_PAD_SD2_DATA3(0x020E035C)
    //   SION [4] - Software Input On Field Reset: DISABLED
    //              Force the selected mux mode Input path no matter of MUX_MODE functionality.
    //     DISABLED (0) - Input Path is determined by functionality of the selected mux mode (regular).
    //     ENABLED (1) - Force input path of pad.
    //   MUX_MODE [2:0] - MUX Mode Select Field Reset: ALT5
    //                    Select iomux modes to be used for pad.
    //     ALT0 (0) - Select instance: usdhc2 signal: SD2_DATA3
    //     ALT1 (1) - Select instance: ecspi5 signal: ECSPI5_SS3
    //     ALT2 (2) - Select instance: kpp signal: KEY_COL6
    //     ALT3 (3) - Select instance: audmux signal: AUD4_TXC
    //     ALT5 (5) - Select instance: gpio1 signal: GPIO1_IO12
    HW_IOMUXC_SW_MUX_CTL_PAD_SD2_DATA3_WR(
            BF_IOMUXC_SW_MUX_CTL_PAD_SD2_DATA3_SION_V(ENABLED) | 
            BF_IOMUXC_SW_MUX_CTL_PAD_SD2_DATA3_MUX_MODE_V(ALT2));
    // Pad Control Register:
    // IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3(0x020E0744)
    //   HYS [16] - Hysteresis Enable Field Reset: ENABLED
    //     DISABLED (0) - CMOS input
    //     ENABLED (1) - Schmitt trigger input
    //   PUS [15:14] - Pull Up / Down Config. Field Reset: 100K_OHM_PU
    //     100K_OHM_PD (0) - 100K Ohm Pull Down
    //     47K_OHM_PU (1) - 47K Ohm Pull Up
    //     100K_OHM_PU (2) - 100K Ohm Pull Up
    //     22K_OHM_PU (3) - 22K Ohm Pull Up
    //   PUE [13] - Pull / Keep Select Field Reset: PULL
    //     KEEP (0) - Keeper Enabled
    //     PULL (1) - Pull Enabled
    //   PKE [12] - Pull / Keep Enable Field Reset: ENABLED
    //     DISABLED (0) - Pull/Keeper Disabled
    //     ENABLED (1) - Pull/Keeper Enabled
    //   ODE [11] - Open Drain Enable Field Reset: DISABLED
    //              Enables open drain of the pin.
    //     DISABLED (0) - Output is CMOS.
    //     ENABLED (1) - Output is Open Drain.
    //   SPEED [7:6] - Speed Field Reset: 100MHZ
    //     TBD (0) - TBD
    //     50MHZ (1) - Low (50 MHz)
    //     100MHZ (2) - Medium (100 MHz)
    //     200MHZ (3) - Maximum (200 MHz)
    //   DSE [5:3] - Drive Strength Field Reset: 40_OHM
    //     HIZ (0) - HI-Z
    //     240_OHM (1) - 240 Ohm
    //     120_OHM (2) - 120 Ohm
    //     80_OHM (3) - 80 Ohm
    //     60_OHM (4) - 60 Ohm
    //     48_OHM (5) - 48 Ohm
    //     40_OHM (6) - 40 Ohm
    //     34_OHM (7) - 34 Ohm
    //   SRE [0] - Slew Rate Field Reset: SLOW
    //             Slew rate control.
    //     SLOW (0) - Slow Slew Rate
    //     FAST (1) - Fast Slew Rate
    HW_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_WR(
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_HYS_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_PUS_V(47K_OHM_PU) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_PUE_V(PULL) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_PKE_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_ODE_V(DISABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_SPEED_V(100MHZ) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_DSE_V(40_OHM) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_SRE_V(SLOW));
    // Pad SD2_DATA3 is involved in Daisy Chain.
    // Input Select Register:
    // IOMUXC_KEY_COL6_SELECT_INPUT(0x020E08EC)
    //   DAISY [1:0] - MUX Mode Select Field Reset: GPIO09_ALT2
    //                 Selecting Pads Involved in Daisy Chain.
    //     GPIO09_ALT2 (0) - Select signal kpp KEY_COL6 as input from pad GPIO09(ALT2).
    //     CSI0_DATA06_ALT3 (1) - Select signal kpp KEY_COL6 as input from pad CSI0_DATA06(ALT3).
    //     SD2_DATA3_ALT2 (2) - Select signal kpp KEY_COL6 as input from pad SD2_DATA3(ALT2).
    HW_IOMUXC_KEY_COL6_SELECT_INPUT_WR(
            BF_IOMUXC_KEY_COL6_SELECT_INPUT_DAISY_V(SD2_DATA3_ALT2));

    // Config kpp.KEY_COL7 to pad SD2_DATA1(E20)
    // HW_IOMUXC_SW_MUX_CTL_PAD_SD2_DATA1_WR(0x00000014);
    // HW_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA1_WR(0x000170B0);
    // HW_IOMUXC_KEY_COL7_SELECT_INPUT_WR(0x00000000);
    // Mux Register:
    // IOMUXC_SW_MUX_CTL_PAD_SD2_DATA1(0x020E004C)
    //   SION [4] - Software Input On Field Reset: DISABLED
    //              Force the selected mux mode Input path no matter of MUX_MODE functionality.
    //     DISABLED (0) - Input Path is determined by functionality of the selected mux mode (regular).
    //     ENABLED (1) - Force input path of pad.
    //   MUX_MODE [2:0] - MUX Mode Select Field Reset: ALT5
    //                    Select iomux modes to be used for pad.
    //     ALT0 (0) - Select instance: usdhc2 signal: SD2_DATA1
    //     ALT1 (1) - Select instance: ecspi5 signal: ECSPI5_SS0
    //     ALT2 (2) - Select instance: eim signal: EIM_CS2
    //     ALT3 (3) - Select instance: audmux signal: AUD4_TXFS
    //     ALT4 (4) - Select instance: kpp signal: KEY_COL7
    //     ALT5 (5) - Select instance: gpio1 signal: GPIO1_IO14
    HW_IOMUXC_SW_MUX_CTL_PAD_SD2_DATA1_WR(
            BF_IOMUXC_SW_MUX_CTL_PAD_SD2_DATA1_SION_V(ENABLED) | 
            BF_IOMUXC_SW_MUX_CTL_PAD_SD2_DATA1_MUX_MODE_V(ALT4));
    // Pad Control Register:
    // IOMUXC_SW_PAD_CTL_PAD_SD2_DATA1(0x020E0360)
    //   HYS [16] - Hysteresis Enable Field Reset: ENABLED
    //     DISABLED (0) - CMOS input
    //     ENABLED (1) - Schmitt trigger input
    //   PUS [15:14] - Pull Up / Down Config. Field Reset: 100K_OHM_PU
    //     100K_OHM_PD (0) - 100K Ohm Pull Down
    //     47K_OHM_PU (1) - 47K Ohm Pull Up
    //     100K_OHM_PU (2) - 100K Ohm Pull Up
    //     22K_OHM_PU (3) - 22K Ohm Pull Up
    //   PUE [13] - Pull / Keep Select Field Reset: PULL
    //     KEEP (0) - Keeper Enabled
    //     PULL (1) - Pull Enabled
    //   PKE [12] - Pull / Keep Enable Field Reset: ENABLED
    //     DISABLED (0) - Pull/Keeper Disabled
    //     ENABLED (1) - Pull/Keeper Enabled
    //   ODE [11] - Open Drain Enable Field Reset: DISABLED
    //              Enables open drain of the pin.
    //     DISABLED (0) - Output is CMOS.
    //     ENABLED (1) - Output is Open Drain.
    //   SPEED [7:6] - Speed Field Reset: 100MHZ
    //     TBD (0) - TBD
    //     50MHZ (1) - Low (50 MHz)
    //     100MHZ (2) - Medium (100 MHz)
    //     200MHZ (3) - Maximum (200 MHz)
    //   DSE [5:3] - Drive Strength Field Reset: 40_OHM
    //     HIZ (0) - HI-Z
    //     240_OHM (1) - 240 Ohm
    //     120_OHM (2) - 120 Ohm
    //     80_OHM (3) - 80 Ohm
    //     60_OHM (4) - 60 Ohm
    //     48_OHM (5) - 48 Ohm
    //     40_OHM (6) - 40 Ohm
    //     34_OHM (7) - 34 Ohm
    //   SRE [0] - Slew Rate Field Reset: SLOW
    //             Slew rate control.
    //     SLOW (0) - Slow Slew Rate
    //     FAST (1) - Fast Slew Rate
    HW_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA1_WR(
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA1_HYS_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA1_PUS_V(47K_OHM_PU) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA1_PUE_V(PULL) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA1_PKE_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA1_ODE_V(DISABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA1_SPEED_V(100MHZ) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA1_DSE_V(40_OHM) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA1_SRE_V(SLOW));
    // Pad SD2_DATA1 is involved in Daisy Chain.
    // Input Select Register:
    // IOMUXC_KEY_COL7_SELECT_INPUT(0x020E08F0)
    //   DAISY [1:0] - MUX Mode Select Field Reset: SD2_DATA1_ALT4
    //                 Selecting Pads Involved in Daisy Chain.
    //     SD2_DATA1_ALT4 (0) - Select signal kpp KEY_COL7 as input from pad SD2_DATA1(ALT4).
    //     GPIO04_ALT2 (1) - Select signal kpp KEY_COL7 as input from pad GPIO04(ALT2).
    //     CSI0_DATA08_ALT3 (2) - Select signal kpp KEY_COL7 as input from pad CSI0_DATA08(ALT3).
    HW_IOMUXC_KEY_COL7_SELECT_INPUT_WR(
            BF_IOMUXC_KEY_COL7_SELECT_INPUT_DAISY_V(SD2_DATA1_ALT4));

    // Config kpp.KEY_ROW5 to pad SD2_CMD(F19)
    // HW_IOMUXC_SW_MUX_CTL_PAD_SD2_CMD_WR(0x00000012);
    // HW_IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_WR(0x0001F0B0);
    // HW_IOMUXC_KEY_ROW5_SELECT_INPUT_WR(0x00000002);
    // Mux Register:
    // IOMUXC_SW_MUX_CTL_PAD_SD2_CMD(0x020E0358)
    //   SION [4] - Software Input On Field Reset: DISABLED
    //              Force the selected mux mode Input path no matter of MUX_MODE functionality.
    //     DISABLED (0) - Input Path is determined by functionality of the selected mux mode (regular).
    //     ENABLED (1) - Force input path of pad.
    //   MUX_MODE [2:0] - MUX Mode Select Field Reset: ALT5
    //                    Select iomux modes to be used for pad.
    //     ALT0 (0) - Select instance: usdhc2 signal: SD2_CMD
    //     ALT1 (1) - Select instance: ecspi5 signal: ECSPI5_MOSI
    //     ALT2 (2) - Select instance: kpp signal: KEY_ROW5
    //     ALT3 (3) - Select instance: audmux signal: AUD4_RXC
    //     ALT5 (5) - Select instance: gpio1 signal: GPIO1_IO11
    HW_IOMUXC_SW_MUX_CTL_PAD_SD2_CMD_WR(
            BF_IOMUXC_SW_MUX_CTL_PAD_SD2_CMD_SION_V(ENABLED) | 
            BF_IOMUXC_SW_MUX_CTL_PAD_SD2_CMD_MUX_MODE_V(ALT2));
    // Pad Control Register:
    // IOMUXC_SW_PAD_CTL_PAD_SD2_CMD(0x020E0740)
    //   HYS [16] - Hysteresis Enable Field Reset: ENABLED
    //     DISABLED (0) - CMOS input
    //     ENABLED (1) - Schmitt trigger input
    //   PUS [15:14] - Pull Up / Down Config. Field Reset: 100K_OHM_PU
    //     100K_OHM_PD (0) - 100K Ohm Pull Down
    //     47K_OHM_PU (1) - 47K Ohm Pull Up
    //     100K_OHM_PU (2) - 100K Ohm Pull Up
    //     22K_OHM_PU (3) - 22K Ohm Pull Up
    //   PUE [13] - Pull / Keep Select Field Reset: PULL
    //     KEEP (0) - Keeper Enabled
    //     PULL (1) - Pull Enabled
    //   PKE [12] - Pull / Keep Enable Field Reset: ENABLED
    //     DISABLED (0) - Pull/Keeper Disabled
    //     ENABLED (1) - Pull/Keeper Enabled
    //   ODE [11] - Open Drain Enable Field Reset: DISABLED
    //              Enables open drain of the pin.
    //     DISABLED (0) - Output is CMOS.
    //     ENABLED (1) - Output is Open Drain.
    //   SPEED [7:6] - Speed Field Reset: 100MHZ
    //     TBD (0) - TBD
    //     50MHZ (1) - Low (50 MHz)
    //     100MHZ (2) - Medium (100 MHz)
    //     200MHZ (3) - Maximum (200 MHz)
    //   DSE [5:3] - Drive Strength Field Reset: 40_OHM
    //     HIZ (0) - HI-Z
    //     240_OHM (1) - 240 Ohm
    //     120_OHM (2) - 120 Ohm
    //     80_OHM (3) - 80 Ohm
    //     60_OHM (4) - 60 Ohm
    //     48_OHM (5) - 48 Ohm
    //     40_OHM (6) - 40 Ohm
    //     34_OHM (7) - 34 Ohm
    //   SRE [0] - Slew Rate Field Reset: SLOW
    //             Slew rate control.
    //     SLOW (0) - Slow Slew Rate
    //     FAST (1) - Fast Slew Rate
    HW_IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_WR(
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_HYS_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_PUS_V(22K_OHM_PU) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_PUE_V(PULL) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_PKE_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_ODE_V(DISABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_SPEED_V(100MHZ) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_DSE_V(40_OHM) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_SRE_V(SLOW));
    // Pad SD2_CMD is involved in Daisy Chain.
    // Input Select Register:
    // IOMUXC_KEY_ROW5_SELECT_INPUT(0x020E08F4)
    //   DAISY [1:0] - MUX Mode Select Field Reset: GPIO01_ALT2
    //                 Selecting Pads Involved in Daisy Chain.
    //     GPIO01_ALT2 (0) - Select signal kpp KEY_ROW5 as input from pad GPIO01(ALT2).
    //     CSI0_DATA05_ALT3 (1) - Select signal kpp KEY_ROW5 as input from pad CSI0_DATA05(ALT3).
    //     SD2_CMD_ALT2 (2) - Select signal kpp KEY_ROW5 as input from pad SD2_CMD(ALT2).
    HW_IOMUXC_KEY_ROW5_SELECT_INPUT_WR(
            BF_IOMUXC_KEY_ROW5_SELECT_INPUT_DAISY_V(SD2_CMD_ALT2));

    // Config kpp.KEY_ROW6 to pad SD2_DATA2(A23)
    // HW_IOMUXC_SW_MUX_CTL_PAD_SD2_DATA2_WR(0x00000014);
    // HW_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA2_WR(0x0001F0B0);
    // HW_IOMUXC_KEY_ROW6_SELECT_INPUT_WR(0x00000000);
    // Mux Register:
    // IOMUXC_SW_MUX_CTL_PAD_SD2_DATA2(0x020E0050)
    //   SION [4] - Software Input On Field Reset: DISABLED
    //              Force the selected mux mode Input path no matter of MUX_MODE functionality.
    //     DISABLED (0) - Input Path is determined by functionality of the selected mux mode (regular).
    //     ENABLED (1) - Force input path of pad.
    //   MUX_MODE [2:0] - MUX Mode Select Field Reset: ALT5
    //                    Select iomux modes to be used for pad.
    //     ALT0 (0) - Select instance: usdhc2 signal: SD2_DATA2
    //     ALT1 (1) - Select instance: ecspi5 signal: ECSPI5_SS1
    //     ALT2 (2) - Select instance: eim signal: EIM_CS3
    //     ALT3 (3) - Select instance: audmux signal: AUD4_TXD
    //     ALT4 (4) - Select instance: kpp signal: KEY_ROW6
    //     ALT5 (5) - Select instance: gpio1 signal: GPIO1_IO13
    HW_IOMUXC_SW_MUX_CTL_PAD_SD2_DATA2_WR(
            BF_IOMUXC_SW_MUX_CTL_PAD_SD2_DATA2_SION_V(ENABLED) | 
            BF_IOMUXC_SW_MUX_CTL_PAD_SD2_DATA2_MUX_MODE_V(ALT4));
    // Pad Control Register:
    // IOMUXC_SW_PAD_CTL_PAD_SD2_DATA2(0x020E0364)
    //   HYS [16] - Hysteresis Enable Field Reset: ENABLED
    //     DISABLED (0) - CMOS input
    //     ENABLED (1) - Schmitt trigger input
    //   PUS [15:14] - Pull Up / Down Config. Field Reset: 100K_OHM_PU
    //     100K_OHM_PD (0) - 100K Ohm Pull Down
    //     47K_OHM_PU (1) - 47K Ohm Pull Up
    //     100K_OHM_PU (2) - 100K Ohm Pull Up
    //     22K_OHM_PU (3) - 22K Ohm Pull Up
    //   PUE [13] - Pull / Keep Select Field Reset: PULL
    //     KEEP (0) - Keeper Enabled
    //     PULL (1) - Pull Enabled
    //   PKE [12] - Pull / Keep Enable Field Reset: ENABLED
    //     DISABLED (0) - Pull/Keeper Disabled
    //     ENABLED (1) - Pull/Keeper Enabled
    //   ODE [11] - Open Drain Enable Field Reset: DISABLED
    //              Enables open drain of the pin.
    //     DISABLED (0) - Output is CMOS.
    //     ENABLED (1) - Output is Open Drain.
    //   SPEED [7:6] - Speed Field Reset: 100MHZ
    //     TBD (0) - TBD
    //     50MHZ (1) - Low (50 MHz)
    //     100MHZ (2) - Medium (100 MHz)
    //     200MHZ (3) - Maximum (200 MHz)
    //   DSE [5:3] - Drive Strength Field Reset: 40_OHM
    //     HIZ (0) - HI-Z
    //     240_OHM (1) - 240 Ohm
    //     120_OHM (2) - 120 Ohm
    //     80_OHM (3) - 80 Ohm
    //     60_OHM (4) - 60 Ohm
    //     48_OHM (5) - 48 Ohm
    //     40_OHM (6) - 40 Ohm
    //     34_OHM (7) - 34 Ohm
    //   SRE [0] - Slew Rate Field Reset: SLOW
    //             Slew rate control.
    //     SLOW (0) - Slow Slew Rate
    //     FAST (1) - Fast Slew Rate
    HW_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA2_WR(
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA2_HYS_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA2_PUS_V(22K_OHM_PU) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA2_PUE_V(PULL) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA2_PKE_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA2_ODE_V(DISABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA2_SPEED_V(100MHZ) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA2_DSE_V(40_OHM) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA2_SRE_V(SLOW));
    // Pad SD2_DATA2 is involved in Daisy Chain.
    // Input Select Register:
    // IOMUXC_KEY_ROW6_SELECT_INPUT(0x020E08F8)
    //   DAISY [1:0] - MUX Mode Select Field Reset: SD2_DATA2_ALT4
    //                 Selecting Pads Involved in Daisy Chain.
    //     SD2_DATA2_ALT4 (0) - Select signal kpp KEY_ROW6 as input from pad SD2_DATA2(ALT4).
    //     GPIO02_ALT2 (1) - Select signal kpp KEY_ROW6 as input from pad GPIO02(ALT2).
    //     CSI0_DATA07_ALT3 (2) - Select signal kpp KEY_ROW6 as input from pad CSI0_DATA07(ALT3).
    HW_IOMUXC_KEY_ROW6_SELECT_INPUT_WR(
            BF_IOMUXC_KEY_ROW6_SELECT_INPUT_DAISY_V(SD2_DATA2_ALT4));

    // Config kpp.KEY_ROW7 to pad SD2_DATA0(A22)
    // HW_IOMUXC_SW_MUX_CTL_PAD_SD2_DATA0_WR(0x00000014);
    // HW_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_WR(0x0001F0B0);
    // HW_IOMUXC_KEY_ROW7_SELECT_INPUT_WR(0x00000000);
    // Mux Register:
    // IOMUXC_SW_MUX_CTL_PAD_SD2_DATA0(0x020E0054)
    //   SION [4] - Software Input On Field Reset: DISABLED
    //              Force the selected mux mode Input path no matter of MUX_MODE functionality.
    //     DISABLED (0) - Input Path is determined by functionality of the selected mux mode (regular).
    //     ENABLED (1) - Force input path of pad.
    //   MUX_MODE [2:0] - MUX Mode Select Field Reset: ALT5
    //                    Select iomux modes to be used for pad.
    //     ALT0 (0) - Select instance: usdhc2 signal: SD2_DATA0
    //     ALT1 (1) - Select instance: ecspi5 signal: ECSPI5_MISO
    //     ALT3 (3) - Select instance: audmux signal: AUD4_RXD
    //     ALT4 (4) - Select instance: kpp signal: KEY_ROW7
    //     ALT5 (5) - Select instance: gpio1 signal: GPIO1_IO15
    //     ALT6 (6) - Select instance: dcic2 signal: DCIC2_OUT
    HW_IOMUXC_SW_MUX_CTL_PAD_SD2_DATA0_WR(
            BF_IOMUXC_SW_MUX_CTL_PAD_SD2_DATA0_SION_V(ENABLED) | 
            BF_IOMUXC_SW_MUX_CTL_PAD_SD2_DATA0_MUX_MODE_V(ALT4));
    // Pad Control Register:
    // IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0(0x020E0368)
    //   HYS [16] - Hysteresis Enable Field Reset: ENABLED
    //     DISABLED (0) - CMOS input
    //     ENABLED (1) - Schmitt trigger input
    //   PUS [15:14] - Pull Up / Down Config. Field Reset: 100K_OHM_PU
    //     100K_OHM_PD (0) - 100K Ohm Pull Down
    //     47K_OHM_PU (1) - 47K Ohm Pull Up
    //     100K_OHM_PU (2) - 100K Ohm Pull Up
    //     22K_OHM_PU (3) - 22K Ohm Pull Up
    //   PUE [13] - Pull / Keep Select Field Reset: PULL
    //     KEEP (0) - Keeper Enabled
    //     PULL (1) - Pull Enabled
    //   PKE [12] - Pull / Keep Enable Field Reset: ENABLED
    //     DISABLED (0) - Pull/Keeper Disabled
    //     ENABLED (1) - Pull/Keeper Enabled
    //   ODE [11] - Open Drain Enable Field Reset: DISABLED
    //              Enables open drain of the pin.
    //     DISABLED (0) - Output is CMOS.
    //     ENABLED (1) - Output is Open Drain.
    //   SPEED [7:6] - Speed Field Reset: 100MHZ
    //     TBD (0) - TBD
    //     50MHZ (1) - Low (50 MHz)
    //     100MHZ (2) - Medium (100 MHz)
    //     200MHZ (3) - Maximum (200 MHz)
    //   DSE [5:3] - Drive Strength Field Reset: 40_OHM
    //     HIZ (0) - HI-Z
    //     240_OHM (1) - 240 Ohm
    //     120_OHM (2) - 120 Ohm
    //     80_OHM (3) - 80 Ohm
    //     60_OHM (4) - 60 Ohm
    //     48_OHM (5) - 48 Ohm
    //     40_OHM (6) - 40 Ohm
    //     34_OHM (7) - 34 Ohm
    //   SRE [0] - Slew Rate Field Reset: SLOW
    //             Slew rate control.
    //     SLOW (0) - Slow Slew Rate
    //     FAST (1) - Fast Slew Rate
    HW_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_WR(
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_HYS_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_PUS_V(22K_OHM_PU) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_PUE_V(PULL) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_PKE_V(ENABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_ODE_V(DISABLED) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_SPEED_V(100MHZ) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_DSE_V(40_OHM) | 
            BF_IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_SRE_V(SLOW));
    // Pad SD2_DATA0 is involved in Daisy Chain.
    // Input Select Register:
    // IOMUXC_KEY_ROW7_SELECT_INPUT(0x020E08FC)
    //   DAISY [1:0] - MUX Mode Select Field Reset: SD2_DATA0_ALT4
    //                 Selecting Pads Involved in Daisy Chain.
    //     SD2_DATA0_ALT4 (0) - Select signal kpp KEY_ROW7 as input from pad SD2_DATA0(ALT4).
    //     GPIO05_ALT2 (1) - Select signal kpp KEY_ROW7 as input from pad GPIO05(ALT2).
    //     CSI0_DATA09_ALT3 (2) - Select signal kpp KEY_ROW7 as input from pad CSI0_DATA09(ALT3).
    HW_IOMUXC_KEY_ROW7_SELECT_INPUT_WR(
            BF_IOMUXC_KEY_ROW7_SELECT_INPUT_DAISY_V(SD2_DATA0_ALT4));
}
