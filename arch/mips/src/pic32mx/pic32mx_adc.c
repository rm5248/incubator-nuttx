/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx_adc.c
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

#include <stdbool.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include <arch/board/board.h>

#include "pic32mx_adc.h"
#include "mips_internal.h"

#ifdef CONFIG_ADC

/* ADC peripheral must be enabled */
#ifdef CONFIG_PIC32MX_ADC

/* Count the number of channels in use */

#define PIC32MX_CHAN0_INUSE    0
#define PIC32MX_CHAN1_INUSE    0
#define PIC32MX_CHAN2_INUSE    0
#define PIC32MX_CHAN3_INUSE    0
#define PIC32MX_CHAN4_INUSE    0
#define PIC32MX_CHAN5_INUSE    0
#define PIC32MX_CHAN6_INUSE    0
#define PIC32MX_CHAN7_INUSE    0
#define PIC32MX_CHAN8_INUSE    0
#define PIC32MX_CHAN9_INUSE    0
#define PIC32MX_CHAN10_INUSE   0
#define PIC32MX_CHAN11_INUSE   0

#ifdef CONFIG_PIC32MX_ADC_CHAN0
#  undef  PIC32MX_CHAN0_INUSE
#  define PIC32MX_CHAN0_INUSE  1
#endif
#ifdef CONFIG_PIC32MX_ADC_CHAN1
#  undef  PIC32MX_CHAN1_INUSE
#  define PIC32MX_CHAN1_INUSE  1
#endif
#ifdef CONFIG_PIC32MX_ADC_CHAN2
#  undef  PIC32MX_CHAN2_INUSE
#  define PIC32MX_CHAN2_INUSE  1
#endif
#ifdef CONFIG_PIC32MX_ADC_CHAN3
#  undef  PIC32MX_CHAN3_INUSE
#  define PIC32MX_CHAN3_INUSE  1
#endif
#ifdef CONFIG_PIC32MX_ADC_CHAN4
#  undef  PIC32MX_CHAN4_INUSE
#  define PIC32MX_CHAN4_INUSE  1
#endif
#ifdef CONFIG_PIC32MX_ADC_CHAN5
#  undef  PIC32MX_CHAN5_INUSE
#  define PIC32MX_CHAN5_INUSE  1
#endif
#ifdef CONFIG_PIC32MX_ADC_CHAN6
#  undef  PIC32MX_CHAN6_INUSE
#  define PIC32MX_CHAN6_INUSE  1
#endif
#ifdef CONFIG_PIC32MX_ADC_CHAN7
#  undef  PIC32MX_CHAN7_INUSE
#  define PIC32MX_CHAN7_INUSE  1
#endif
#ifdef CONFIG_PIC32MX_ADC_CHAN8
#  undef  PIC32MX_CHAN8_INUSE
#  define PIC32MX_CHAN8_INUSE  1
#endif
#ifdef CONFIG_PIC32MX_ADC_CHAN9
#  undef  PIC32MX_CHAN9_INUSE
#  define PIC32MX_CHAN9_INUSE  1
#endif
#ifdef CONFIG_PIC32MX_ADC_CHAN10
#  undef  PIC32MX_CHAN10_INUSE
#  define PIC32MX_CHAN10_INUSE 1
#endif
#ifdef CONFIG_PIC32MX_ADC_CHAN11
#  undef  PIC32MX_CHAN11_INUSE
#  define PIC32MX_CHAN11_INUSE 1
#endif

#define PIC32MX_NCHANNELS \
  (PIC32MX_CHAN0_INUSE + PIC32MX_CHAN1_INUSE  + PIC32MX_CHAN2_INUSE  + \
   PIC32MX_CHAN3_INUSE + PIC32MX_CHAN4_INUSE  + PIC32MX_CHAN5_INUSE  + \
   PIC32MX_CHAN6_INUSE + PIC32MX_CHAN7_INUSE  + PIC32MX_CHAN8_INUSE  + \
   PIC32MX_CHAN9_INUSE + PIC32MX_CHAN10_INUSE + PIC32MX_CHAN11_INUSE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the overall state of the ADC */

struct pic32mx_adc_s
{
  const struct adc_callback_s *cb;
  bool initialized;      /* The ADC driver is already initialized */
  struct adc_dev_s *dev; /* A reference to the outer, ADC device container */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  pic32mx_adc_bind(struct adc_dev_s *dev,
                             const struct adc_callback_s *callback);
static void pic32mx_adc_reset(struct adc_dev_s *dev);
static int  pic32mx_adc_setup(struct adc_dev_s *dev);
static void pic32mx_adc_shutdown(struct adc_dev_s *dev);
static void pic32mx_adc_rxint(struct adc_dev_s *dev, bool enable);
static int  pic32mx_adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC lower half device operations */

static const struct adc_ops_s g_adcops =
{
  .ao_bind     = pic32mx_adc_bind,
  .ao_reset    = pic32mx_adc_reset,
  .ao_setup    = pic32mx_adc_setup,
  .ao_shutdown = pic32mx_adc_shutdown,
  .ao_rxint    = pic32mx_adc_rxint,
  .ao_ioctl    = pic32mx_adc_ioctl,
};

/* ADC internal state */

static struct pic32mx_adc_s g_adcpriv;

/* ADC device instance */

static struct adc_dev_s g_adcdev;

/****************************************************************************
 * Initialization/Configuration
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_adc_channels
 *
 * Description:
 *   Configure the appropriate ADC channels depending on the configuration
 *
 ****************************************************************************/

static void pic32mx_adc_channels(struct pic32mx_adc_s *priv)
{
  uint32_t regval;

  ainfo("Entry\n");

  /* Enable channels. */

  regval = 0;

#ifdef CONFIG_PIC32MX_ADC_CHAN0
  regval |= ADC_CSSL(0);
#endif

#ifdef CONFIG_PIC32MX_ADC_CHAN1
  regval |= ADC_CSSL(1);
#endif

#ifdef CONFIG_PIC32MX_ADC_CHAN2
  regval |= ADC_CSSL(2);
#endif

#ifdef CONFIG_PIC32MX_ADC_CHAN3
  regval |= ADC_CSSL(3);
#endif

#ifdef CONFIG_PIC32MX_ADC_CHAN4
  regval |= ADC_CSSL(4);
#endif

#ifdef CONFIG_PIC32MX_ADC_CHAN5
  regval |= ADC_CSSL(5);
#endif

#ifdef CONFIG_PIC32MX_ADC_CHAN6
  regval |= ADC_CSSL(6);
#endif

#ifdef CONFIG_PIC32MX_ADC_CHAN7
  regval |= ADC_CSSL(7);
#endif

#ifdef CONFIG_PIC32MX_ADC_CHAN8
  regval |= ADC_CSSL(8);
#endif

#ifdef CONFIG_PIC32MX_ADC_CHAN9
  regval |= ADC_CSSL(9);
#endif

#ifdef CONFIG_PIC32MX_ADC_CHAN10
  regval |= ADC_CSSL(10);
#endif

#ifdef CONFIG_PIC32MX_ADC_CHAN11
  regval |= ADC_CSSL(11);
#endif

#ifdef CONFIG_PIC32MX_ADC_CHAN12
  regval |= ADC_CSSL(12);
#endif

#ifdef CONFIG_PIC32MX_ADC_CHAN13
  regval |= ADC_CSSL(13);
#endif

#ifdef CONFIG_PIC32MX_ADC_CHAN14
  regval |= ADC_CSSL(14);
#endif

#ifdef CONFIG_PIC32MX_ADC_CHAN15
  regval |= ADC_CSSL(15);
#endif

  putreg32(regval, PIC32MX_ADC_CSSL);
}

/****************************************************************************
 * ADC methods
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int pic32mx_adc_bind(struct adc_dev_s *dev,
                             const struct adc_callback_s *callback)
{
  struct pic32mx_adc_s *priv = (struct pic32mx_adc_s *)dev->ad_priv;
ainfo("bind\n");

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: pic32mx_adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before pic32mx_adc_setup() and on error conditions.
 *
 ****************************************************************************/

static void pic32mx_adc_reset(struct adc_dev_s *dev)
{
  struct pic32mx_adc_s *priv = (struct pic32mx_adc_s *)dev->ad_priv;
  uint32_t regval;

  ainfo("Resetting..\n");

  /* turn ADC off */ 
  
  putreg32(ADC_CON1_ON, PIC32MX_ADC_CON1CLR);

  /* disable all ADC input pins */

  putreg32(0, PIC32MX_ADC_CSSL);
  ainfo("Resetdone..\n");
}

/****************************************************************************
 * Name: pic32mx_adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.
 *   Interrupts are all disabled upon return.
 *
 ****************************************************************************/

static int pic32mx_adc_setup(struct adc_dev_s *dev)
{
  struct pic32mx_adc_s *priv = (struct pic32mx_adc_s *)dev->ad_priv;
  uint32_t regval;

  ainfo("Setup\n");

  /* Enable ADC channels */

  pic32mx_adc_channels(priv);

  /* Enable the ADC */

  putreg32(ADC_CON1_ON, PIC32MX_ADC_CON1SET);

  return 0;
}

/****************************************************************************
 * Name: pic32mx_adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void pic32mx_adc_shutdown(struct adc_dev_s *dev)
{
  struct pic32mx_adc_s *priv = (struct pic32mx_adc_s *)dev->ad_priv;

  ainfo("Shutdown\n");

  /* Reset the ADC peripheral */

  pic32mx_adc_reset(dev);
}

/****************************************************************************
 * Name: pic32mx_adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void pic32mx_adc_rxint(struct adc_dev_s *dev, bool enable)
{
  struct pic32mx_adc_s *priv = (struct pic32mx_adc_s *)dev->ad_priv;

  ainfo("enable=%d\n", enable);

}

/****************************************************************************
 * Name: pic32mx_adc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int pic32mx_adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  struct pic32mx_adc_s *priv = (struct pic32mx_adc_s *)dev->ad_priv;
  int ret = OK;

  ainfo("cmd=%d arg=%ld\n", cmd, arg);

  switch (cmd)
    {
      case ANIOC_GET_NCHANNELS:
        {
          /* Return the number of configured channels */

          ret = PIC32MX_NCHANNELS;
        }
        break;

      /* Unsupported or invalid command */

      default:
        {
          ret = -ENOTTY;
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_adc_initialize
 *
 * Description:
 *   Initialize the adc
 *
 * Returned Value:
 *   Valid can device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *pic32mx_adc_initialize(void)
{
  struct pic32mx_adc_s *priv = &g_adcpriv;

  if (!priv->initialized)
    {
      ainfo("Initializing...\n");

      /* Initialize the public ADC device data structure */

      g_adcdev.ad_ops  = &g_adcops;
      priv->dev = &g_adcdev;
      priv->cb  = NULL;

      g_adcdev.ad_priv = priv;

      /* Now we are initialized */

      priv->initialized = true;
    }

  /* Return a pointer to the device structure */

  ainfo("Returning %p\n", &g_adcdev);
  return &g_adcdev;
}

#endif /* CONFIG_PIC32MX_ADC */

#endif /* CONFIG_ADC */
