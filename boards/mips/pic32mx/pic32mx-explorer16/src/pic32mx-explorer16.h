/****************************************************************************
 * boards/mips/pic32mx/pic32mx-explorer16/src/pic32mx-explorer16.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_MIPS_PIC32MX_PIC32MX_STARTERKIT_SRC_PIC32MX_EXPLORER16_H
#define __BOARDS_MIPS_PIC32MX_PIC32MX_STARTERKIT_SRC_PIC32MX_EXPLORER16_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* The Explorer 16 has 8 LEDs(D3-D10 on the schematic)
 *
 *   RA0          User LED D3 (high illuminates)
 *   RA1          User LED D4 (high illuminates)
 *   RA2          User LED D5 (high illuminates)
 *   RA3          User LED D6 (high illuminates)
 *   RA4          User LED D7 (high illuminates)
 *   RA5          User LED D8 (high illuminates)
 *   RA6          User LED D9 (high illuminates)
 *   RA7          User LED D10 (high illuminates)
 */

/* The Explorer 16 has 4 switches
 *
 *   RD6            Switch S3 (low when closed)
 *   RD7            Switch S6 (low when closed)
 *   RA7            Switch S5 (low when closed)
 *   RD13           Switch S4 (low when closed)
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: pic32mx_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PCB Logic board.
 *
 ****************************************************************************/

#if defined(CONFIG_PIC32MX_SPI2)
void weak_function pic32mx_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: pic32mx_led_initialize
 *
 * Description:
 *   Configure on-board LEDs if LED support has been selected.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void pic32mx_led_initialize(void);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_MIPS_PIC32MX_PIC32MX_STARTERKIT_SRC_PIC32MX_EXPLORER16_H */
