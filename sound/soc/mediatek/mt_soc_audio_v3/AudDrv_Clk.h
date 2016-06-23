/*
 * Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*******************************************************************************
 *
 * Filename:
 * ---------
 *   AudDrv_Clk.h
 *
 * Project:
 * --------
 *   MT6583  Audio Driver clock control
 *
 * Description:
 * ------------
 *   Audio clcok control
 *
 * Author:
 * -------
 *   Chipeng Chang (mtk02308)
 *
 *------------------------------------------------------------------------------
 * $Revision: #1 $
 * $Modtime:$
 * $Log:$
 *
 *
 *******************************************************************************/

#ifndef _AUDDRV_CLK_H_
#define _AUDDRV_CLK_H_

/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include "AudDrv_Common.h"
#include "AudDrv_Def.h"

/*****************************************************************************
 *                         D A T A   T Y P E S
 *****************************************************************************/


/*****************************************************************************
 *                         M A C R O
 *****************************************************************************/


/*****************************************************************************
 *                 FUNCTION       D E F I N I T I O N
 *****************************************************************************/
#if !defined(CONFIG_MTK_LEGACY)
#include <linux/clk.h>
#include <linux/platform_device.h>
#endif /* !defined(CONFIG_MTK_LEGACY) */

#if !defined(CONFIG_MTK_LEGACY)
typedef struct mt_aud_clk_t
{
   struct clk *aud_afe_clk;           /* main clock for AFE */
   struct clk *aud_i2s_clk;           /* main clock for I2S */
   struct clk *aud_adc_clk;           /* main clock for ADC */
   struct clk *aud_apll22m_clk;       /* main clock for APLL22M */
   struct clk *aud_apll24m_clk;       /* main clock for APLL24M */
   struct clk *aud_apll1_tuner_clk;       /* main clock for apll1_tuner */
   struct clk *aud_apll2_tuner_clk;       /* main clock for apll1_tuner */
   struct clk *aud_dac_clk;           /* main clock for DAC */
   struct clk *aud_dac_predis_clk;    /* main clock for dac_predis */
   struct clk *aud_tml_clk;           /* main clock for TML */
   struct clk *aud_infra_clk;         /* Audio Infra clock */   
   kal_int32 aud_afe_clk_status;        
   kal_int32 aud_i2s_clk_status;        
   kal_int32 aud_adc_clk_status;
   kal_int32 aud_apll22m_clk_status;        
   kal_int32 aud_apll24m_clk_status;   
   kal_int32 aud_apll1_tuner_clk_status;   
   kal_int32 aud_apll2_tuner_clk_status;   
   kal_int32 aud_dac_clk_status;   
   kal_int32 aud_dac_predis_clk_status;   
   kal_int32 aud_tml_clk_status;      
   kal_int32 aud_infra_clk_status;
}mt_aud_clk;

struct platform_device;
extern void AudDrv_Clk_probe(struct platform_device *pdev);
#endif

void AudDrv_Clk_AllOn(void);

void Auddrv_Bus_Init( void);

void AudDrv_Clk_Power_On(void);
void AudDrv_Clk_Power_Off(void);

void AudDrv_Clk_On(void);
void AudDrv_Clk_Off(void);

void AudDrv_ANA_Clk_On(void);
void AudDrv_ANA_Clk_Off(void);

void AudDrv_I2S_Clk_On(void);
void AudDrv_I2S_Clk_Off(void);

void AudDrv_Core_Clk_On(void);
void AudDrv_Core_Clk_Off(void);

void AudDrv_ADC_Clk_On(void);
void AudDrv_ADC_Clk_Off(void);
void AudDrv_ADC2_Clk_On(void);
void AudDrv_ADC2_Clk_Off(void);
void AudDrv_ADC3_Clk_On(void);
void AudDrv_ADC3_Clk_Off(void);

void AudDrv_HDMI_Clk_On(void);
void AudDrv_HDMI_Clk_Off(void);

void AudDrv_Suspend_Clk_On(void);
void AudDrv_Suspend_Clk_Off(void);

void AudDrv_APLL24M_Clk_On(void);
void AudDrv_APLL24M_Clk_Off(void);
void AudDrv_APLL22M_Clk_On(void);
void AudDrv_APLL22M_Clk_Off(void);

void AudDrv_APLL1Tuner_Clk_On(void);
void AudDrv_APLL1Tuner_Clk_Off(void);
void AudDrv_APLL2Tuner_Clk_On(void);
void AudDrv_APLL2Tuner_Clk_Off(void);

void AudDrv_Emi_Clk_On(void);
void AudDrv_Emi_Clk_Off(void);


#endif

