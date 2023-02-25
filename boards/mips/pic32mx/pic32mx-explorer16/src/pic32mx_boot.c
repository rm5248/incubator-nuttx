/****************************************************************************
 * boards/mips/pic32mx/pic32mx-explorer16/src/pic32mx_boot.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/adc.h>

#include <debug.h>

#include <arch/board/board.h>

#include "mips_internal.h"
#include "pic32mx.h"
#include "pic32mx-explorer16.h"
#include "pic32mx_adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_boardinitialize
 *
 * Description:
 *   All PIC32MX architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after
 *   all memory has been configured and mapped but before any devices have
 *   been initialized.
 *
 ****************************************************************************/

void pic32mx_boardinitialize(void)
{
  /* Configure SPI chip selects if 1) at least one SPI is enabled, and 2)
   * the weak function pic32mx_spidev_initialize() has been brought into
   * the link.
   */

#if defined(CONFIG_PIC32MX_SPI1) || defined(CONFIG_PIC32MX_SPI2) || \
    defined(CONFIG_PIC32MX_SPI3) || defined(CONFIG_PIC32MX_SPI4)
  if (pic32mx_spidev_initialize)
    {
      pic32mx_spidev_initialize();
    }
#endif

#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  pic32mx_led_initialize();
#endif

}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_initialize() is called and just before
 *   the initial application is started.  This additional initialization
 *   phae may be used, for example, to initialize board-specific device
 *   drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  struct adc_dev_s *adc;
  int ret;

  /* Perform board-specific initialization here if so configured */

  /* Configure the ADC */

  adc = pic32mx_adc_initialize();

  if (adc == NULL)
    {
      aerr("ERROR: Failed to get ADC interface\n");
    }

  /* Register the ADC driver at "/dev/adc0" */
ainfo("about to reg adc\n");

  ret = adc_register("/dev/adc0", adc);
  if (ret < 0)
    {
      aerr("ERROR: adc_register failed: %d\n", ret);
    }
ainfo("done register adc\n");

}
#endif
