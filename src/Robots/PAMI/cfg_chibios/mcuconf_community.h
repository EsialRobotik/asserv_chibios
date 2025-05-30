/*
    ChibiOS/RT - Copyright (C) 2014 Uladzimir Pylinsky aka barthess

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * FSMC driver system settings.
 */
#define STM32_FSMC_USE_FSMC1                FALSE
#define STM32_FSMC_FSMC1_IRQ_PRIORITY       10
#define STM32_FSMC_DMA_CHN                  0x03010201

/*
 * FSMC NAND driver system settings.
 */
#define STM32_NAND_USE_NAND1                FALSE
#define STM32_NAND_USE_NAND2                FALSE
#define STM32_NAND_USE_EXT_INT              FALSE
#define STM32_NAND_DMA_STREAM               STM32_DMA_STREAM_ID(2, 7)
#define STM32_NAND_DMA_PRIORITY             0
#define STM32_NAND_DMA_ERROR_HOOK(nandp)    osalSysHalt("DMA failure")

/*
 * FSMC SRAM driver system settings.
 */
#define STM32_SRAM_USE_SRAM1                FALSE
#define STM32_SRAM_USE_SRAM2                FALSE
#define STM32_SRAM_USE_SRAM3                FALSE
#define STM32_SRAM_USE_SRAM4                FALSE

/*
 * FSMC SDRAM driver system settings.
 */
#define STM32_SDRAM_USE_SDRAM1              FALSE
#define STM32_SDRAM_USE_SDRAM2              FALSE

/*
 * TIMCAP driver system settings.
 */
#define STM32_TIMCAP_USE_TIM1                  FALSE
#define STM32_TIMCAP_USE_TIM2                  FALSE
#define STM32_TIMCAP_USE_TIM3                  FALSE
#define STM32_TIMCAP_USE_TIM4                  FALSE
#define STM32_TIMCAP_USE_TIM5                  FALSE
#define STM32_TIMCAP_USE_TIM8                  FALSE
#define STM32_TIMCAP_USE_TIM9                  FALSE
#define STM32_TIMCAP_TIM1_IRQ_PRIORITY         3
#define STM32_TIMCAP_TIM2_IRQ_PRIORITY         3
#define STM32_TIMCAP_TIM3_IRQ_PRIORITY         3
#define STM32_TIMCAP_TIM4_IRQ_PRIORITY         3
#define STM32_TIMCAP_TIM5_IRQ_PRIORITY         3
#define STM32_TIMCAP_TIM8_IRQ_PRIORITY         3
#define STM32_TIMCAP_TIM9_IRQ_PRIORITY         3

/*
 * COMP driver system settings.
 */
#define STM32_COMP_USE_COMP1                  FALSE
#define STM32_COMP_USE_COMP2                  FALSE
#define STM32_COMP_USE_COMP3                  FALSE
#define STM32_COMP_USE_COMP4                  FALSE
#define STM32_COMP_USE_COMP5                  FALSE
#define STM32_COMP_USE_COMP6                  FALSE
#define STM32_COMP_USE_COMP7                  FALSE

#define STM32_COMP_USE_INTERRUPTS             FALSE
#define STM32_COMP_1_2_3_IRQ_PRIORITY         5
#define STM32_COMP_4_5_6_IRQ_PRIORITY         5
#define STM32_COMP_7_IRQ_PRIORITY             5

#if STM32_COMP_USE_INTERRUPTS
#define STM32_DISABLE_EXTI21_22_29_HANDLER
#define STM32_DISABLE_EXTI30_32_HANDLER
#define STM32_DISABLE_EXTI33_HANDLER
#endif

/*
 * USBH driver system settings.
 */
#define STM32_OTG1_CHANNELS_NUMBER          8
#define STM32_OTG2_CHANNELS_NUMBER          12

#define STM32_USBH_USE_OTG1                 1
#define STM32_OTG1_RXFIFO_SIZE              1024
#define STM32_OTG1_PTXFIFO_SIZE             128
#define STM32_OTG1_NPTXFIFO_SIZE            128

#define STM32_USBH_USE_OTG2                 0
#define STM32_OTG2_RXFIFO_SIZE              2048
#define STM32_OTG2_PTXFIFO_SIZE             1024
#define STM32_OTG2_NPTXFIFO_SIZE            1024

#define STM32_USBH_MIN_QSPACE               4
#define STM32_USBH_CHANNELS_NP              4

/*
 * CRC driver system settings.
 */
#define STM32_CRC_USE_CRC1                  FALSE
#define STM32_CRC_CRC1_DMA_IRQ_PRIORITY     1
#define STM32_CRC_CRC1_DMA_PRIORITY         2
#define STM32_CRC_CRC1_DMA_STREAM           STM32_DMA1_STREAM2

#define CRCSW_USE_CRC1                      FALSE
#define CRCSW_CRC32_TABLE                   FALSE
#define CRCSW_CRC16_TABLE                   FALSE
#define CRCSW_PROGRAMMABLE                  FALSE

/*
 * LTDC driver system settings.
 */
#define STM32_LTDC_USE_LTDC                 FALSE
#define STM32_LTDC_EV_IRQ_PRIORITY          11
#define STM32_LTDC_ER_IRQ_PRIORITY          11

/*
 * QEI driver system settings.
 */
#define STM32_QEI_USE_TIM1                FALSE
#define STM32_QEI_USE_TIM2                TRUE
#define STM32_QEI_USE_TIM3                TRUE
#define STM32_QEI_TIM1_IRQ_PRIORITY         3
#define STM32_QEI_TIM2_IRQ_PRIORITY         3
#define STM32_QEI_TIM3_IRQ_PRIORITY         3
