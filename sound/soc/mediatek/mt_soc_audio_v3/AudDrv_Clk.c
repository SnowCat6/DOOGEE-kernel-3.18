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
 *   AudDrv_Clk.c
 *
 * Project:
 * --------
 *   MT6583  Audio Driver clock control implement
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 * Chipeng Chang (MTK02308)
 *
 *------------------------------------------------------------------------------
 * $Revision: #1 $
 * $Modtime:$
 * $Log:$
 *
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/
#include <mach/mt_clkmgr.h>
#include <mach/mt_pm_ldo.h>
#include <mach/pmic_mt6325_sw.h>
#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>

#include "AudDrv_Common.h"
#include "AudDrv_Clk.h"
#include "AudDrv_Afe.h"
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <mach/mt_idle.h>
#include <linux/err.h>
#include <linux/platform_device.h>

/*****************************************************************************
 *                         D A T A   T Y P E S
 *****************************************************************************/

int        Aud_Core_Clk_cntr   = 0;
int        Aud_AFE_Clk_cntr    = 0;
int        Aud_I2S_Clk_cntr    = 0;
int        Aud_ADC_Clk_cntr    = 0;
int        Aud_ADC2_Clk_cntr    = 0;
int        Aud_ADC3_Clk_cntr    = 0;
int        Aud_ANA_Clk_cntr    = 0;
int        Aud_HDMI_Clk_cntr = 0;
int        Aud_APLL22M_Clk_cntr    = 0;
int        Aud_APLL24M_Clk_cntr    = 0;
int        Aud_APLL1_Tuner_cntr = 0;
int        Aud_APLL2_Tuner_cntr = 0;
static int Aud_EMI_cntr = 0;

static DEFINE_SPINLOCK(auddrv_Clk_lock);

// amp mutex lock
static DEFINE_MUTEX(auddrv_pmic_mutex);
static DEFINE_MUTEX(audEMI_Clk_mutex);
#if !defined(CONFIG_MTK_LEGACY)
static mt_aud_clk *paudclk = NULL;
#endif

//extern void disable_dpidle_by_bit(int id);
//extern void disable_soidle_by_bit(int id);
//extern void enable_dpidle_by_bit(int id);
//extern void enable_soidle_by_bit(int id);

void AudDrv_Clk_AllOn(void)
{
    unsigned long flags;
    pr_debug("AudDrv_Clk_AllOn \n");
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    Afe_Set_Reg(AUDIO_TOP_CON0, 0x00004000, 0xffffffff);
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void Auddrv_Bus_Init(void)
{
    unsigned long flags;
    pr_debug("%s  \n", __func__);
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    Afe_Set_Reg(AUDIO_TOP_CON0, 0x00004000, 0x00004000); //must set, system will default set bit14 to 0
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

#if !defined(CONFIG_MTK_LEGACY)

void AudDrv_Clk_probe(struct platform_device *pdev)
{
    paudclk =  kzalloc(sizeof(mt_aud_clk), GFP_KERNEL);

    paudclk->aud_afe_clk = devm_clk_get(&pdev->dev, "aud_afe_clk");
    if (IS_ERR(paudclk->aud_afe_clk)) {
        pr_err("cannot get aud_afe_clk clock\n");
        paudclk->aud_afe_clk_status = 0;
        BUG();
        return;
    }
    else
    {
        paudclk->aud_afe_clk_status = 1;
    }
    
    paudclk->aud_i2s_clk = devm_clk_get(&pdev->dev, "aud_i2s_clk");
    if (IS_ERR(paudclk->aud_i2s_clk)) {
        pr_err("cannot get aud_i2s_clk clock\n");
        paudclk->aud_i2s_clk_status = 0;
        BUG();
        return;     
    }
    else
    {
        paudclk->aud_i2s_clk_status = 1; 
    }   
    
    paudclk->aud_adc_clk = devm_clk_get(&pdev->dev, "aud_adc_clk");
    if (IS_ERR(paudclk->aud_adc_clk)) {
        pr_err("cannot get aud_adc_clk clock\n");
        paudclk->aud_adc_clk_status = 0;
        BUG();
        return;
    }
    else
    {
        paudclk->aud_adc_clk_status = 1;
    }
    
    paudclk->aud_apll22m_clk = devm_clk_get(&pdev->dev, "aud_apll22m_clk");
    if (IS_ERR(paudclk->aud_apll22m_clk)) {
        pr_err("cannot get aud_apll22m_clk clock\n");
        paudclk->aud_apll22m_clk_status = 0;
        BUG();
        return;        
    }
    else
    {
        paudclk->aud_apll22m_clk_status = 1;
    }
    
    paudclk->aud_apll24m_clk = devm_clk_get(&pdev->dev, "aud_apll24m_clk");
    if (IS_ERR(paudclk->aud_apll24m_clk)) {
        pr_debug("cannot get aud_apll24m_clk clock\n");
        paudclk->aud_apll24m_clk_status = 0;
        BUG();
        return;        
    }
    else
    {
        paudclk->aud_apll24m_clk_status = 1;
    }
        
    paudclk->aud_apll1_tuner_clk = devm_clk_get(&pdev->dev, "aud_apll1_tuner_clk");
    if (IS_ERR(paudclk->aud_apll1_tuner_clk)) {
        pr_err("cannot get aud_apll1_tuner_clk clock\n");
        paudclk->aud_apll1_tuner_clk_status = 0;
        BUG();
        return;        
    }
    else
    {
        paudclk->aud_apll1_tuner_clk_status = 1;
    }    
    
    paudclk->aud_apll2_tuner_clk = devm_clk_get(&pdev->dev, "aud_apll2_tuner_clk");
    if (IS_ERR(paudclk->aud_apll2_tuner_clk)) {
        pr_err("cannot get aud_apll2_tuner_clk clock\n");
        paudclk->aud_apll2_tuner_clk_status = 0;
        BUG();
        return;        
    }
    else
    {
        paudclk->aud_apll2_tuner_clk_status = 1;
    }

    paudclk->aud_dac_clk = devm_clk_get(&pdev->dev, "aud_dac_clk");
    if (IS_ERR(paudclk->aud_dac_clk)) {
        pr_err("cannot get aud_dac_clk clock\n");
        paudclk->aud_dac_clk_status = 0;
        BUG();
        return;        
    }
    else
    {
        paudclk->aud_dac_clk_status = 1;
    }

    paudclk->aud_dac_predis_clk = devm_clk_get(&pdev->dev, "aud_dac_predis_clk");
    if (IS_ERR(paudclk->aud_dac_predis_clk)) {
        pr_err("cannot get aud_dac_predis_clk clock\n");
        paudclk->aud_dac_predis_clk_status = 0;
        BUG();
        return;        
    }
    else
    {
        paudclk->aud_dac_predis_clk_status = 1;
    }    

    paudclk->aud_tml_clk = devm_clk_get(&pdev->dev, "aud_tml_clk");
    if (IS_ERR(paudclk->aud_tml_clk)) {
        pr_err("cannot get aud_tml_clk clock\n");
        paudclk->aud_tml_clk_status = 0;
        BUG();
        return;        
    }
    else
    {
        paudclk->aud_tml_clk_status = 1;
    }    

    paudclk->aud_infra_clk = devm_clk_get(&pdev->dev, "aud_infra_clk");
    if (IS_ERR(paudclk->aud_infra_clk)) {
        pr_err("cannot get aud_infra_clk clock\n");
        paudclk->aud_infra_clk_status = 0;
        BUG();
        return;        
    }
    else
    {
        paudclk->aud_infra_clk_status = 1;
    }      
}
#endif


/*****************************************************************************
 * FUNCTION
 *  AudDrv_Clk_Power_On / AudDrv_Clk_Power_Off
 *
 * DESCRIPTION
 *  Power on this function , then all register can be access and  set.
 *
 *****************************************************************************
 */

void AudDrv_Clk_Power_On(void)
{
    volatile uint32 *AFE_Register = (volatile uint32 *)Get_Afe_Powertop_Pointer();
    volatile uint32 val_tmp;
    pr_debug("%s", __func__);
    val_tmp = 0xd;
    mt_reg_sync_writel(val_tmp, AFE_Register);
}

void AudDrv_Clk_Power_Off(void)
{
}


/*****************************************************************************
 * FUNCTION
 *  AudDrv_Clk_On / AudDrv_Clk_Off
 *
 * DESCRIPTION
 *  Enable/Disable PLL(26M clock) \ AFE clock
 *
 *****************************************************************************
 */
void AudDrv_Clk_On(void)
{
    unsigned long flags;
#if !defined(CONFIG_MTK_LEGACY)  
    int ret = 0;
#endif
    PRINTK_AUD_CLK("+AudDrv_Clk_On, Aud_AFE_Clk_cntr:%d \n",Aud_AFE_Clk_cntr);
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    if (Aud_AFE_Clk_cntr == 0)
    {
        pr_err("-----------AudDrv_Clk_On, Aud_AFE_Clk_cntr:%d \n", Aud_AFE_Clk_cntr);
#ifdef PM_MANAGER_API
#if defined(CONFIG_MTK_LEGACY)
        if (enable_clock(MT_CG_INFRA_AUDIO, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s Aud enable_clock MT_CG_INFRA_AUDIO fail", __func__);
        }
        if (enable_clock(MT_CG_AUDIO_AFE, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s Aud enable_clock MT_CG_AUDIO_AFE fail", __func__);
        }
        if (enable_clock(MT_CG_AUDIO_DAC, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s MT_CG_AUDIO_DAC fail", __func__);
        }
        if (enable_clock(MT_CG_AUDIO_DAC_PREDIS, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s MT_CG_AUDIO_DAC_PREDIS fail", __func__);
        }
#else
        pr_err("-----------[CCF]AudDrv_Clk_On, paudclk->aud_infra_clk_status:%d \n", paudclk->aud_infra_clk_status);

        if (paudclk->aud_infra_clk_status)
        {
            ret = clk_prepare_enable(paudclk->aud_infra_clk);
            if (!ret)
            {
                pr_err("%s Aud enable_clock MT_CG_INFRA_AUDIO fail", __func__);
                BUG();
                return;                        
            }
        }
        else
        {
            pr_err("%s clk_status error Aud enable_clock MT_CG_INFRA_AUDIO fail", __func__);
            BUG();
            return;                                    
        }
        
        if (paudclk->aud_afe_clk_status)
        {
            ret = clk_prepare_enable(paudclk->aud_afe_clk);
            if (!ret)
            {
                pr_err("%s Aud enable_clock MT_CG_AUDIO_AFE fail", __func__);
                BUG();
                return;                  
            }
        }
        else
        {
            pr_err("%s clk_status error Aud enable_clock MT_CG_AUDIO_AFE fail", __func__);
            BUG();
            return;                                    
        }
    
        if (paudclk->aud_dac_clk_status)
        {
            ret = clk_prepare_enable(paudclk->aud_dac_clk);
            if (!ret)
            {
                pr_err("%s Aud enable_clock MT_CG_AUDIO_DAC fail", __func__);
                BUG();
                return;                  
            }
        }
        else
        {
            pr_err("%s clk_status error Aud enable_clock MT_CG_AUDIO_DAC fail", __func__);
            BUG();
            return;                                    
        }
        
        if (paudclk->aud_dac_predis_clk_status)
        {
            ret = clk_prepare_enable(paudclk->aud_dac_predis_clk);
            if (!ret)
            {
                pr_err("%s Aud enable_clock MT_CG_AUDIO_DAC_PREDIS fail", __func__);
                BUG();
                return;                  
            }
        }
        else
        {
            pr_err("%s clk_status error Aud enable_clock MT_CG_AUDIO_DAC_PREDIS fail", __func__);
            BUG();
            return;                                    
        }        

#endif
#else
        SetInfraCfg(AUDIO_CG_CLR, 0x2000000, 0x2000000); //bit 25=0, without 133m master and 66m slave bus clock cg gating
        Afe_Set_Reg(AUDIO_TOP_CON0, 0x4000, 0x06004044);
#endif
    }
    Aud_AFE_Clk_cntr++;
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
    PRINTK_AUD_CLK("-AudDrv_Clk_On, Aud_AFE_Clk_cntr:%d \n",Aud_AFE_Clk_cntr);
}

void AudDrv_Clk_Off(void)
{
    unsigned long flags;
    PRINTK_AUD_CLK("+!! AudDrv_Clk_Off, Aud_AFE_Clk_cntr:%d \n",Aud_AFE_Clk_cntr);
    spin_lock_irqsave(&auddrv_Clk_lock, flags);

    Aud_AFE_Clk_cntr--;
    if (Aud_AFE_Clk_cntr == 0)
    {
        pr_err("------------AudDrv_Clk_Off, Aud_AFE_Clk_cntr:%d \n", Aud_AFE_Clk_cntr);
        {
            // Disable AFE clock
#ifdef PM_MANAGER_API
#if defined(CONFIG_MTK_LEGACY)
            if (disable_clock(MT_CG_AUDIO_AFE, "AUDIO"))
            {
                PRINTK_AUD_CLK("%s disable_clock MT_CG_AUDIO_AFE fail", __func__);
            }
            if (disable_clock(MT_CG_AUDIO_DAC, "AUDIO"))
            {
                PRINTK_AUD_CLK("%s MT_CG_AUDIO_DAC fail", __func__);
            }
            if (disable_clock(MT_CG_AUDIO_DAC_PREDIS, "AUDIO"))
            {
                PRINTK_AUD_CLK("%s MT_CG_AUDIO_DAC_PREDIS fail", __func__);
            }
            if (disable_clock(MT_CG_INFRA_AUDIO, "AUDIO"))
            {
                PRINTK_AUD_CLK("%s disable_clock MT_CG_INFRA_AUDIO fail", __func__);
            }
#else
            pr_err("-----------[CCF]AudDrv_Clk_Off, paudclk->aud_infra_clk_status:%d \n", paudclk->aud_infra_clk_status);

            if (paudclk->aud_afe_clk_status)
            {
                clk_disable_unprepare(paudclk->aud_afe_clk);
            }
            if (paudclk->aud_dac_clk_status)
            {
                clk_disable_unprepare(paudclk->aud_dac_clk);
            }
            if (paudclk->aud_dac_predis_clk_status)
            {
                clk_disable_unprepare(paudclk->aud_dac_predis_clk);
            }
            if (paudclk->aud_infra_clk_status)
            {
                clk_disable_unprepare(paudclk->aud_infra_clk);
            }
#endif

#else
            Afe_Set_Reg(AUDIO_TOP_CON0, 0x06000044, 0x06000044);
            SetInfraCfg(AUDIO_CG_SET, 0x2000000, 0x2000000); //bit25=1, with 133m mastesr and 66m slave bus clock cg gating
#endif
        }
    }
    else if (Aud_AFE_Clk_cntr < 0)
    {
        PRINTK_AUD_ERROR("!! AudDrv_Clk_Off, Aud_AFE_Clk_cntr<0 (%d) \n", Aud_AFE_Clk_cntr);
        AUDIO_ASSERT(true);
        Aud_AFE_Clk_cntr = 0;
    }
    PRINTK_AUD_CLK("-!! AudDrv_Clk_Off, Aud_AFE_Clk_cntr:%d \n",Aud_AFE_Clk_cntr);
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

/*****************************************************************************
 * FUNCTION
 *  AudDrv_ANA_Clk_On / AudDrv_ANA_Clk_Off
 *
 * DESCRIPTION
 *  Enable/Disable analog part clock
 *
 *****************************************************************************/
void AudDrv_ANA_Clk_On(void)
{
    mutex_lock(&auddrv_pmic_mutex);
    if (Aud_ANA_Clk_cntr == 0)
    {
        PRINTK_AUD_CLK("+AudDrv_ANA_Clk_On, Aud_ANA_Clk_cntr:%d \n", Aud_ANA_Clk_cntr);
    }
    Aud_ANA_Clk_cntr++;
    mutex_unlock(&auddrv_pmic_mutex);
    //PRINTK_AUD_CLK("-AudDrv_ANA_Clk_Off, Aud_ANA_Clk_cntr:%d \n",Aud_ANA_Clk_cntr);
}

void AudDrv_ANA_Clk_Off(void)
{
    //PRINTK_AUD_CLK("+AudDrv_ANA_Clk_Off, Aud_ADC_Clk_cntr:%d \n",  Aud_ANA_Clk_cntr);
    mutex_lock(&auddrv_pmic_mutex);
    Aud_ANA_Clk_cntr--;
    if (Aud_ANA_Clk_cntr == 0)
    {
        PRINTK_AUD_CLK("+AudDrv_ANA_Clk_Off disable_clock Ana clk(%x)\n", Aud_ANA_Clk_cntr);
        // Disable ADC clock
#ifdef PM_MANAGER_API

#else
        // TODO:: open ADC clock....
#endif
    }
    else if (Aud_ANA_Clk_cntr < 0)
    {
        PRINTK_AUD_ERROR("!! AudDrv_ANA_Clk_Off, Aud_ADC_Clk_cntr<0 (%d) \n", Aud_ANA_Clk_cntr);
        AUDIO_ASSERT(true);
        Aud_ANA_Clk_cntr = 0;
    }
    mutex_unlock(&auddrv_pmic_mutex);
    //PRINTK_AUD_CLK("-AudDrv_ANA_Clk_Off, Aud_ADC_Clk_cntr:%d \n", Aud_ANA_Clk_cntr);
}

/*****************************************************************************
 * FUNCTION
  *  AudDrv_ADC_Clk_On / AudDrv_ADC_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable analog part clock
  *
  *****************************************************************************/

void AudDrv_ADC_Clk_On(void)
{
    //PRINTK_AUDDRV("+AudDrv_ADC_Clk_On, Aud_ADC_Clk_cntr:%d \n", Aud_ADC_Clk_cntr);
#if !defined(CONFIG_MTK_LEGACY)  
    int ret = 0;
#endif
    mutex_lock(&auddrv_pmic_mutex);

    if (Aud_ADC_Clk_cntr == 0)
    {
        PRINTK_AUDDRV("+AudDrv_ADC_Clk_On enable_clock ADC clk(%x)\n", Aud_ADC_Clk_cntr);
//        Afe_Set_Reg(AUDIO_TOP_CON0, 0 << 24 , 1 << 24);
#ifdef PM_MANAGER_API
#if defined(CONFIG_MTK_LEGACY)
        if (enable_clock(MT_CG_AUDIO_ADC, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s fail", __func__);
        }
#else
        if (paudclk->aud_adc_clk_status)
        {
            ret = clk_prepare_enable(paudclk->aud_adc_clk);
            if (!ret)
            {
                pr_err("%s Aud enable_clock enable_clock ADC fail", __func__);
                BUG();
                return;
            }
        }
        else
        {
            pr_err("%s clk_status error Aud enable_clock ADC fail", __func__);
            BUG();
            return;                                    
        }        

#endif
#else
        Afe_Set_Reg(AUDIO_TOP_CON0, 0 << 24 , 1 << 24);
#endif
    }
    Aud_ADC_Clk_cntr++;
    mutex_unlock(&auddrv_pmic_mutex);
}

void AudDrv_ADC_Clk_Off(void)
{
    //PRINTK_AUDDRV("+AudDrv_ADC_Clk_Off, Aud_ADC_Clk_cntr:%d \n", Aud_ADC_Clk_cntr);
    mutex_lock(&auddrv_pmic_mutex);
    Aud_ADC_Clk_cntr--;
    if (Aud_ADC_Clk_cntr == 0)
    {
        PRINTK_AUDDRV("+AudDrv_ADC_Clk_On disable_clock ADC clk(%x)\n", Aud_ADC_Clk_cntr);
        //Afe_Set_Reg(AUDIO_TOP_CON0, 1 << 24 , 1 << 24);
#ifdef PM_MANAGER_API
#if defined(CONFIG_MTK_LEGACY)

        if (disable_clock(MT_CG_AUDIO_ADC, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s fail", __func__);
        }
#else
        if (paudclk->aud_adc_clk_status)
        {
            clk_disable_unprepare(paudclk->aud_adc_clk);
        }
#endif
#else
        Afe_Set_Reg(AUDIO_TOP_CON0, 1 << 24 , 1 << 24);
#endif
    }
    if (Aud_ADC_Clk_cntr < 0)
    {
        PRINTK_AUDDRV("!! AudDrv_ADC_Clk_Off, Aud_ADC_Clk_cntr<0 (%d) \n", Aud_ADC_Clk_cntr);
        Aud_ADC_Clk_cntr = 0;
    }
    mutex_unlock(&auddrv_pmic_mutex);
    //PRINTK_AUDDRV("-AudDrv_ADC_Clk_Off, Aud_ADC_Clk_cntr:%d \n", Aud_ADC_Clk_cntr);
}

/*****************************************************************************
 * FUNCTION
  *  AudDrv_ADC2_Clk_On / AudDrv_ADC2_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable clock
  *
  *****************************************************************************/

void AudDrv_ADC2_Clk_On(void)
{
    PRINTK_AUD_CLK("+%s %d \n", __func__, Aud_ADC2_Clk_cntr);
    mutex_lock(&auddrv_pmic_mutex);

    if (Aud_ADC2_Clk_cntr == 0)
    {
        PRINTK_AUDDRV("+%s  enable_clock ADC2 clk(%x)\n", __func__, Aud_ADC2_Clk_cntr);
#if 0 //6752 removed
#ifdef PM_MANAGER_API
        if (enable_clock(MT_CG_AUDIO_ADDA2, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s fail", __func__);
        }
#else
        Afe_Set_Reg(AUDIO_TOP_CON0, 0 << 23 , 1 << 23); //temp hard code setting, after confirm with enable clock usage, this could be removed.
#endif
#endif
    }
    Aud_ADC2_Clk_cntr++;
    mutex_unlock(&auddrv_pmic_mutex);
}

void AudDrv_ADC2_Clk_Off(void)
{
    //PRINTK_AUDDRV("+%s %d \n", __func__,Aud_ADC2_Clk_cntr);
    mutex_lock(&auddrv_pmic_mutex);
    Aud_ADC2_Clk_cntr--;
    if (Aud_ADC2_Clk_cntr == 0)
    {
        PRINTK_AUDDRV("+%s disable_clock ADC clk(%x)\n", __func__, Aud_ADC2_Clk_cntr);
#if 0 //6752 removed
#ifdef PM_MANAGER_API
        if (disable_clock(MT_CG_AUDIO_ADDA2, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s fail", __func__);
        }
#else
        Afe_Set_Reg(AUDIO_TOP_CON0, 1 << 23 , 1 << 23); //temp hard code setting, after confirm with enable clock usage, this could be removed.
#endif
#endif
    }
    if (Aud_ADC2_Clk_cntr < 0)
    {
        PRINTK_AUDDRV("%s  <0 (%d) \n", __func__, Aud_ADC2_Clk_cntr);
        Aud_ADC2_Clk_cntr = 0;
    }
    mutex_unlock(&auddrv_pmic_mutex);
    //PRINTK_AUDDRV("-AudDrv_ADC_Clk_Off, Aud_ADC_Clk_cntr:%d \n", Aud_ADC_Clk_cntr);
}


/*****************************************************************************
 * FUNCTION
  *  AudDrv_ADC3_Clk_On / AudDrv_ADC3_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable clock
  *
  *****************************************************************************/

void AudDrv_ADC3_Clk_On(void)
{
    PRINTK_AUD_CLK("+%s %d \n", __func__, Aud_ADC3_Clk_cntr);
    mutex_lock(&auddrv_pmic_mutex);

    if (Aud_ADC3_Clk_cntr == 0)
    {
        PRINTK_AUDDRV("+%s  enable_clock ADC clk(%x)\n", __func__, Aud_ADC3_Clk_cntr);
#if 0 //6752 removed
#ifdef PM_MANAGER_API
        if (enable_clock(MT_CG_AUDIO_ADDA3, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s fail", __func__);
        }
#endif
#endif
    }
    Aud_ADC3_Clk_cntr++;
    mutex_unlock(&auddrv_pmic_mutex);
}

void AudDrv_ADC3_Clk_Off(void)
{
    //PRINTK_AUDDRV("+%s %d \n", __func__,Aud_ADC2_Clk_cntr);
    mutex_lock(&auddrv_pmic_mutex);
    Aud_ADC3_Clk_cntr--;
    if (Aud_ADC3_Clk_cntr == 0)
    {
        PRINTK_AUDDRV("+%s disable_clock ADC clk(%x)\n", __func__, Aud_ADC3_Clk_cntr);
#if 0 //6752 removed
#ifdef PM_MANAGER_API
        if (disable_clock(MT_CG_AUDIO_ADDA3, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s fail", __func__);
        }
#endif
#endif
    }
    if (Aud_ADC3_Clk_cntr < 0)
    {
        PRINTK_AUDDRV("%s  <0 (%d) \n", __func__, Aud_ADC3_Clk_cntr);
        Aud_ADC3_Clk_cntr = 0;
    }
    mutex_unlock(&auddrv_pmic_mutex);
    //PRINTK_AUDDRV("-AudDrv_ADC_Clk_Off, Aud_ADC_Clk_cntr:%d \n", Aud_ADC_Clk_cntr);
}


/*****************************************************************************
 * FUNCTION
  *  AudDrv_APLL22M_Clk_On / AudDrv_APLL22M_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable clock
  *
  *****************************************************************************/

void AudDrv_APLL22M_Clk_On(void)
{
    pr_debug("+%s %d \n", __func__, Aud_APLL22M_Clk_cntr);
#if !defined(CONFIG_MTK_LEGACY)  
    int ret = 0;
#endif    
    mutex_lock(&auddrv_pmic_mutex);

    if (Aud_APLL22M_Clk_cntr == 0)
    {
        PRINTK_AUDDRV("+%s  enable_clock ADC clk(%x)\n", __func__, Aud_APLL22M_Clk_cntr);
#ifdef PM_MANAGER_API
        pr_debug("+%s  enable_mux ADC \n", __func__);
        enable_mux(MT_MUX_AUD1, "AUDIO");//MT_MUX_AUD1	CLK_CFG_6 => [7]: pdn_aud_1 [15]: ,MT_MUX_AUD2: pdn_aud_2

        clkmux_sel(MT_MUX_AUD1, 1 , "AUDIO"); //select APLL1 ,hf_faud_1_ck is mux of 26M and APLL1_CK

        pll_fsel(APLL1, 0xb7945ea6); //APLL1 90.3168M

        //pdn_aud_1 => power down hf_faud_1_ck, hf_faud_1_ck is mux of 26M and APLL1_CK
        //pdn_aud_2 => power down hf_faud_2_ck, hf_faud_2_ck is mux of 26M and APLL2_CK (D1 is WHPLL)
#if defined(CONFIG_MTK_LEGACY)
        if (enable_clock(MT_CG_AUDIO_22M, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s fail", __func__);
        }
        if (enable_clock(MT_CG_AUDIO_APLL_TUNER, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s fail", __func__);
        }
#else
        if (paudclk->aud_apll22m_clk_status)
        {
            ret = clk_prepare_enable(paudclk->aud_apll22m_clk);
            if (!ret)
            {
                pr_err("%s Aud enable_clock enable_clock aud_apll22m_clk fail", __func__);
                BUG();
                return;                
            }
        }
        else
        {
            pr_err("%s clk_status error Aud enable_clock aud_apll22m_clk fail", __func__);
            BUG();
            return;                                    
        }
        
        if (paudclk->aud_apll1_tuner_clk_status)
        {
            ret = clk_prepare_enable(paudclk->aud_apll1_tuner_clk);
            if (!ret)
            {
                pr_err("%s Aud enable_clock enable_clock aud_apll1_tuner_clk fail", __func__);
                BUG();
                return;                
            }
        }  
        else
        {
            pr_err("%s clk_status error Aud enable_clock aud_apll1_tuner_clk fail", __func__);
            BUG();
            return;                                    
        }           
#endif
        
        
#endif
    }
    Aud_APLL22M_Clk_cntr++;
    mutex_unlock(&auddrv_pmic_mutex);
}

void AudDrv_APLL22M_Clk_Off(void)
{
    pr_debug("+%s %d \n", __func__, Aud_APLL22M_Clk_cntr);
    mutex_lock(&auddrv_pmic_mutex);
    Aud_APLL22M_Clk_cntr--;
    if (Aud_APLL22M_Clk_cntr == 0)
    {
        PRINTK_AUDDRV("+%s disable_clock ADC clk(%x)\n", __func__, Aud_APLL22M_Clk_cntr);
#ifdef PM_MANAGER_API

#if defined(CONFIG_MTK_LEGACY)
        if (disable_clock(MT_CG_AUDIO_22M, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s fail", __func__);
        }
        if (disable_clock(MT_CG_AUDIO_APLL_TUNER, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s fail", __func__);
        }
#else
        if (paudclk->aud_apll22m_clk_status)
        {
            clk_disable_unprepare(paudclk->aud_apll22m_clk);
        }
        if (paudclk->aud_apll1_tuner_clk_status)
        {
            clk_disable_unprepare(paudclk->aud_apll1_tuner_clk);
        }        
#endif
        clkmux_sel(MT_MUX_AUD1, 0, "AUDIO"); //select 26M
        disable_mux(MT_MUX_AUD1, "AUDIO");
#endif
    }
    if (Aud_APLL22M_Clk_cntr < 0)
    {
        PRINTK_AUDDRV("%s  <0 (%d) \n", __func__, Aud_APLL22M_Clk_cntr);
        Aud_APLL22M_Clk_cntr = 0;
    }
    mutex_unlock(&auddrv_pmic_mutex);
}


/*****************************************************************************
 * FUNCTION
  *  AudDrv_APLL24M_Clk_On / AudDrv_APLL24M_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable clock
  *
  *****************************************************************************/

void AudDrv_APLL24M_Clk_On(void)
{
    pr_debug("+%s %d \n", __func__, Aud_APLL24M_Clk_cntr);
#if !defined(CONFIG_MTK_LEGACY)  
    int ret = 0;
#endif     
    mutex_lock(&auddrv_pmic_mutex);
    if (Aud_APLL24M_Clk_cntr == 0)
    {
        PRINTK_AUDDRV("+%s  enable_clock ADC clk(%x)\n", __func__, Aud_APLL24M_Clk_cntr);
#ifdef PM_MANAGER_API
        enable_mux(MT_MUX_AUD1, "AUDIO");

        clkmux_sel(MT_MUX_AUD1, 1, "AUDIO"); //hf_faud_1_ck apll1_ck

        pll_fsel(APLL1, 0xbc7ea932); //ALPP1 98.304M
#if defined(CONFIG_MTK_LEGACY)
        if (enable_clock(MT_CG_AUDIO_24M, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s fail", __func__);
        }
        if (enable_clock(MT_CG_AUDIO_APLL2_TUNER, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s fail", __func__);
        }
#else
        if (paudclk->aud_apll24m_clk_status)
        {
            ret = clk_prepare_enable(paudclk->aud_apll24m_clk);
            if (!ret)
            {
                pr_err("%s Aud enable_clock enable_clock aud_apll24m_clk fail", __func__);
                BUG();
                return;                                
            }
        }
        else
        {
            pr_err("%s clk_status error Aud enable_clock aud_apll24m_clk fail", __func__);
            BUG();
            return;                                    
        } 
        
        if (paudclk->aud_apll2_tuner_clk_status)
        {
            ret = clk_prepare_enable(paudclk->aud_apll2_tuner_clk);
            if (!ret)
            {
                pr_err("%s Aud enable_clock enable_clock aud_apll2_tuner_clk fail", __func__);
                BUG();
                return;                                
            }
        } 
        else
        {
            pr_err("%s clk_status error Aud enable_clock aud_apll2_tuner_clk fail", __func__);
            BUG();
            return;                                    
        }         

#endif
#endif
    }
    Aud_APLL24M_Clk_cntr++;
    mutex_unlock(&auddrv_pmic_mutex);
}

void AudDrv_APLL24M_Clk_Off(void)
{
    pr_debug("+%s %d \n", __func__, Aud_APLL24M_Clk_cntr);
    mutex_lock(&auddrv_pmic_mutex);
    Aud_APLL24M_Clk_cntr--;
    if (Aud_APLL24M_Clk_cntr == 0)
    {
        PRINTK_AUDDRV("+%s disable_clock ADC clk(%x)\n", __func__, Aud_APLL24M_Clk_cntr);
#ifdef PM_MANAGER_API
#if defined(CONFIG_MTK_LEGACY)

        if (disable_clock(MT_CG_AUDIO_24M, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s fail", __func__);
        }
        if (disable_clock(MT_CG_AUDIO_APLL2_TUNER, "AUDIO"))
        {
            PRINTK_AUD_CLK("%s fail", __func__);
        }
#else
        if (paudclk->aud_apll24m_clk_status)
        {
            clk_disable_unprepare(paudclk->aud_apll24m_clk);
        }
        if (paudclk->aud_apll2_tuner_clk_status)
        {
            clk_disable_unprepare(paudclk->aud_apll2_tuner_clk);
        }        
#endif
        clkmux_sel(MT_MUX_AUD1, 0, "AUDIO"); //select 26M
        disable_mux(MT_MUX_AUD1, "AUDIO");
#endif
    }
    if (Aud_APLL24M_Clk_cntr < 0)
    {
        PRINTK_AUDDRV("%s  <0 (%d) \n", __func__, Aud_APLL24M_Clk_cntr);
        Aud_APLL24M_Clk_cntr = 0;
    }
    mutex_unlock(&auddrv_pmic_mutex);
}


/*****************************************************************************
  * FUNCTION
  *  AudDrv_I2S_Clk_On / AudDrv_I2S_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable analog part clock
  *
  *****************************************************************************/
void AudDrv_I2S_Clk_On(void)
{
    unsigned long flags;
#if !defined(CONFIG_MTK_LEGACY)  
    int ret = 0;
#endif     
    //PRINTK_AUD_CLK("+AudDrv_I2S_Clk_On, Aud_I2S_Clk_cntr:%d \n", Aud_I2S_Clk_cntr);
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    if (Aud_I2S_Clk_cntr == 0)
    {
#ifdef PM_MANAGER_API
#if defined(CONFIG_MTK_LEGACY)
        if (enable_clock(MT_CG_AUDIO_I2S, "AUDIO"))
        {
            PRINTK_AUD_ERROR("Aud enable_clock MT65XX_PDN_AUDIO_I2S fail !!!\n");
        }
#else
        if (paudclk->aud_i2s_clk_status)
        {
            ret = clk_prepare_enable(paudclk->aud_i2s_clk);
            if (!ret)
            {
                pr_err("%s Aud enable_clock enable_clock aud_i2s_clk fail", __func__);
                BUG();
                return;
            }
        }
        else
        {
            pr_err("%s clk_status error Aud enable_clock aud_i2s_clk fail", __func__);
            BUG();
            return;                                    
        }        
#endif
#else
        Afe_Set_Reg(AUDIO_TOP_CON0, 0x00000000, 0x00000040);  //power on I2S clock
#endif
    }
    Aud_I2S_Clk_cntr++;
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_I2S_Clk_Off(void)
{
    unsigned long flags;
    //PRINTK_AUD_CLK("+AudDrv_I2S_Clk_Off, Aud_I2S_Clk_cntr:%d \n", Aud_I2S_Clk_cntr);
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    Aud_I2S_Clk_cntr--;
    if (Aud_I2S_Clk_cntr == 0)
    {
#ifdef PM_MANAGER_API
#if defined(CONFIG_MTK_LEGACY)
        if (disable_clock(MT_CG_AUDIO_I2S, "AUDIO"))
        {
            PRINTK_AUD_ERROR("disable_clock MT_CG_AUDIO_I2S fail");
        }
#else
        if (paudclk->aud_i2s_clk_status)
        {
            clk_disable_unprepare(paudclk->aud_i2s_clk);
        }
#endif
#else
        Afe_Set_Reg(AUDIO_TOP_CON0, 0x00000040, 0x00000040);  //power off I2S clock
#endif
    }
    else if (Aud_I2S_Clk_cntr < 0)
    {
        PRINTK_AUD_ERROR("!! AudDrv_I2S_Clk_Off, Aud_I2S_Clk_cntr<0 (%d) \n", Aud_I2S_Clk_cntr);
        AUDIO_ASSERT(true);
        Aud_I2S_Clk_cntr = 0;
    }
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
    //PRINTK_AUD_CLK("-AudDrv_I2S_Clk_Off, Aud_I2S_Clk_cntr:%d \n",Aud_I2S_Clk_cntr);
}

/*****************************************************************************
  * FUNCTION
  *  AudDrv_Core_Clk_On / AudDrv_Core_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable analog part clock
  *
  *****************************************************************************/

void AudDrv_Core_Clk_On(void)
{
    //PRINTK_AUD_CLK("+AudDrv_Core_Clk_On, Aud_Core_Clk_cntr:%d \n", Aud_Core_Clk_cntr);
    unsigned long flags;
#if !defined(CONFIG_MTK_LEGACY)  
    int ret = 0;
#endif      
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    if (Aud_Core_Clk_cntr == 0)
    {
#ifdef PM_MANAGER_API
#if defined(CONFIG_MTK_LEGACY)
        if (enable_clock(MT_CG_AUDIO_AFE, "AUDIO"))
        {
            PRINTK_AUD_ERROR("AudDrv_Core_Clk_On Aud enable_clock MT_CG_AUDIO_AFE fail !!!\n");
        }
#else
        if (paudclk->aud_afe_clk_status)
        {
            ret = clk_prepare_enable(paudclk->aud_afe_clk);
            if (!ret)
            {
                pr_err("%s Aud enable_clock enable_clock aud_afe_clk fail", __func__);
                BUG();
                return;                  
            }
        }
        else
        {
            pr_err("%s clk_status error Aud enable_clock aud_afe_clk fail", __func__);
            BUG();
            return;                                    
        }           
#endif
        
#endif
    }
    Aud_Core_Clk_cntr++;
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
    //PRINTK_AUD_CLK("-AudDrv_Core_Clk_On, Aud_Core_Clk_cntr:%d \n", Aud_Core_Clk_cntr);
}


void AudDrv_Core_Clk_Off(void)
{
    //PRINTK_AUD_CLK("+AudDrv_Core_Clk_On, Aud_Core_Clk_cntr:%d \n", Aud_Core_Clk_cntr);
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    if (Aud_Core_Clk_cntr == 0)
    {
#ifdef PM_MANAGER_API
#if defined(CONFIG_MTK_LEGACY)
        if (disable_clock(MT_CG_AUDIO_AFE, "AUDIO"))
        {
            PRINTK_AUD_ERROR("AudDrv_Core_Clk_On Aud disable_clock MT_CG_AUDIO_AFE fail !!!\n");
        }
#else
        if (paudclk->aud_afe_clk_status)
        {
            clk_disable_unprepare(paudclk->aud_afe_clk);
        }
#endif        
#endif
    }
    Aud_Core_Clk_cntr++;
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
    //PRINTK_AUD_CLK("-AudDrv_Core_Clk_On, Aud_Core_Clk_cntr:%d \n", Aud_Core_Clk_cntr);
}

void AudDrv_APLL1Tuner_Clk_On(void)
{
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    if (Aud_APLL1_Tuner_cntr == 0)
    {
        PRINTK_AUD_CLK("+AudDrv_APLLTuner_Clk_On, Aud_APLL1_Tuner_cntr:%d \n", Aud_APLL1_Tuner_cntr);
        Afe_Set_Reg(AUDIO_TOP_CON0, 0x0 << 19, 0x1 << 19);
    }
    Aud_APLL1_Tuner_cntr++;
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_APLL1Tuner_Clk_Off(void)
{
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);

    Aud_APLL1_Tuner_cntr--;
    if (Aud_APLL1_Tuner_cntr == 0)
    {
        Afe_Set_Reg(AUDIO_TOP_CON0, 0x1 << 19, 0x1 << 19);
        Afe_Set_Reg(AFE_APLL1_TUNER_CFG, 0x00000033, 0x1 << 19);
    }
    // handle for clock error
    else if (Aud_APLL1_Tuner_cntr < 0)
    {
        PRINTK_AUD_ERROR("!! AudDrv_APLLTuner_Clk_Off, Aud_APLL1_Tuner_cntr<0 (%d) \n", Aud_APLL1_Tuner_cntr);
        Aud_APLL1_Tuner_cntr = 0;
    }
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}


void AudDrv_APLL2Tuner_Clk_On(void)
{
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    if (Aud_APLL2_Tuner_cntr == 0)
    {
        PRINTK_AUD_CLK("+Aud_APLL2_Tuner_cntr, Aud_APLL2_Tuner_cntr:%d \n", Aud_APLL2_Tuner_cntr);
        Afe_Set_Reg(AUDIO_TOP_CON0, 0x0 << 18, 0x1 << 18);
        Afe_Set_Reg(AFE_APLL2_TUNER_CFG, 0x00000033, 0x1 << 19);
    }
    Aud_APLL2_Tuner_cntr++;
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}
void AudDrv_APLL2Tuner_Clk_Off(void)
{
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);

    Aud_APLL2_Tuner_cntr--;
    if (Aud_APLL2_Tuner_cntr == 0)
    {
        Afe_Set_Reg(AUDIO_TOP_CON0, 0x1 << 18, 0x1 << 18);
    }
    // handle for clock error
    else if (Aud_APLL2_Tuner_cntr < 0)
    {
        PRINTK_AUD_ERROR("!! AudDrv_APLL2Tuner_Clk_Off, Aud_APLL1_Tuner_cntr<0 (%d) \n", Aud_APLL2_Tuner_cntr);
        Aud_APLL2_Tuner_cntr = 0;
    }
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

/*****************************************************************************
  * FUNCTION
  *  AudDrv_HDMI_Clk_On / AudDrv_HDMI_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable analog part clock
  *
  *****************************************************************************/

void AudDrv_HDMI_Clk_On(void)
{
    PRINTK_AUD_CLK("+AudDrv_HDMI_Clk_On, Aud_I2S_Clk_cntr:%d \n", Aud_HDMI_Clk_cntr);
    if (Aud_HDMI_Clk_cntr == 0)
    {
        AudDrv_ANA_Clk_On();
        AudDrv_Clk_On();
    }
    Aud_HDMI_Clk_cntr++;
}

void AudDrv_HDMI_Clk_Off(void)
{
    PRINTK_AUD_CLK("+AudDrv_HDMI_Clk_Off, Aud_I2S_Clk_cntr:%d \n", Aud_HDMI_Clk_cntr);
    Aud_HDMI_Clk_cntr--;
    if (Aud_HDMI_Clk_cntr == 0)
    {
        AudDrv_ANA_Clk_Off();
        AudDrv_Clk_Off();
    }
    else if (Aud_HDMI_Clk_cntr < 0)
    {
        PRINTK_AUD_ERROR("!! AudDrv_Linein_Clk_Off, Aud_I2S_Clk_cntr<0 (%d) \n", Aud_HDMI_Clk_cntr);
        AUDIO_ASSERT(true);
        Aud_HDMI_Clk_cntr = 0;
    }
    PRINTK_AUD_CLK("-AudDrv_I2S_Clk_Off, Aud_I2S_Clk_cntr:%d \n", Aud_HDMI_Clk_cntr);
}

/*****************************************************************************
* FUNCTION
*  AudDrv_Suspend_Clk_Off / AudDrv_Suspend_Clk_On
*
* DESCRIPTION
*  Enable/Disable AFE clock for suspend
*
*****************************************************************************
*/

void AudDrv_Suspend_Clk_Off(void)
{
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    if (Aud_Core_Clk_cntr > 0)
    {
#ifdef PM_MANAGER_API
        if (Aud_AFE_Clk_cntr  > 0)
        {
#if defined(CONFIG_MTK_LEGACY)        
            if (disable_clock(MT_CG_AUDIO_AFE, "AUDIO"))
            {
                printk("Aud enable_clock MT_CG_AUDIO_AFE fail !!!\n");
            }
#else
            if (paudclk->aud_afe_clk_status)
            {
                clk_disable_unprepare(paudclk->aud_afe_clk);
            }
#endif
        }
        if (Aud_I2S_Clk_cntr > 0)
        {
#if defined(CONFIG_MTK_LEGACY)               
            if (disable_clock(MT_CG_AUDIO_I2S, "AUDIO"))
            {
                PRINTK_AUD_ERROR("disable_clock MT_CG_AUDIO_I2S fail");
            }
#else
            if (paudclk->aud_i2s_clk_status)
            {
                clk_disable_unprepare(paudclk->aud_i2s_clk);
            }
#endif
        }
        if (Aud_ADC_Clk_cntr > 0)
        {
            Afe_Set_Reg(AUDIO_TOP_CON0, 1 << 24 , 1 << 24);

        }
        if (Aud_ADC2_Clk_cntr > 0)
        {
        	#if 0 //6752 removed
            if (disable_clock(MT_CG_AUDIO_ADDA2, "AUDIO"))
            {
                PRINTK_AUD_CLK("%s fail", __func__);
            }
			#endif
        }
        if (Aud_ADC3_Clk_cntr > 0)
        {
            #if 0 //6752 removed
            if (disable_clock(MT_CG_AUDIO_ADDA3, "AUDIO"))
            {
                PRINTK_AUD_CLK("%s fail", __func__);
            }
			#endif
        }
        if (Aud_ANA_Clk_cntr > 0)
        {

        }
        if (Aud_HDMI_Clk_cntr > 0)
        {

        }
        if (Aud_APLL22M_Clk_cntr  > 0)
        {
#if defined(CONFIG_MTK_LEGACY)                      
            if (disable_clock(MT_CG_AUDIO_22M, "AUDIO"))
            {
                PRINTK_AUD_CLK("%s fail", __func__);
            }
            if (disable_clock(MT_CG_AUDIO_APLL_TUNER, "AUDIO"))
            {
                PRINTK_AUD_CLK("%s fail", __func__);
            }
#else
            if (paudclk->aud_apll22m_clk_status)
            {
                clk_disable_unprepare(paudclk->aud_apll22m_clk);
            }
            if (paudclk->aud_apll1_tuner_clk_status)
            {
                clk_disable_unprepare(paudclk->aud_apll1_tuner_clk);
            }  
#endif
            clkmux_sel(MT_MUX_AUD1, 0, "AUDIO"); //select 26M
            disable_mux(MT_MUX_AUD1, "AUDIO");
        }
        if (Aud_APLL24M_Clk_cntr > 0)
        {
#if defined(CONFIG_MTK_LEGACY)                              
            if (disable_clock(MT_CG_AUDIO_24M, "AUDIO"))
            {
                PRINTK_AUD_CLK("%s fail", __func__);
            }
            if (disable_clock(MT_CG_AUDIO_APLL2_TUNER, "AUDIO"))
            {
                PRINTK_AUD_CLK("%s fail", __func__);
            }
            clkmux_sel(MT_MUX_AUD2, 0, "AUDIO"); //select 26M
            disable_mux(MT_MUX_AUD2, "AUDIO");
#else
            if (paudclk->aud_apll24m_clk_status)
            {
                clk_disable_unprepare(paudclk->aud_apll24m_clk);
            }
            if (paudclk->aud_apll2_tuner_clk_status)
            {
                clk_disable_unprepare(paudclk->aud_apll2_tuner_clk);
            }  
#endif
            
        }
#endif
    }
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_Suspend_Clk_On(void)
{
    unsigned long flags;
#if !defined(CONFIG_MTK_LEGACY)  
    int ret = 0;
#endif      
    
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    if (Aud_Core_Clk_cntr > 0)
    {
#ifdef PM_MANAGER_API
#if defined(CONFIG_MTK_LEGACY)
            if (enable_clock(MT_CG_AUDIO_AFE, "AUDIO"))
            {
                PRINTK_AUD_ERROR("Aud enable_clock MT_CG_AUDIO_AFE fail !!!\n");
            }
#else
            if (paudclk->aud_afe_clk_status)
            {
                ret = clk_prepare_enable(paudclk->aud_afe_clk);
                if (!ret)
                {
                    pr_err("%s Aud enable_clock enable_clock aud_afe_clk fail", __func__);
                    BUG();
                    return;                     
                }
            }
            else
            {
                pr_err("%s clk_status error Aud enable_clock aud_afe_clk fail", __func__);
                BUG();
                return;                                    
            }            
#endif                    

        if (Aud_I2S_Clk_cntr > 0)
        {
#if defined(CONFIG_MTK_LEGACY)       
            if (enable_clock(MT_CG_AUDIO_I2S, "AUDIO"))
            {
                PRINTK_AUD_ERROR("enable_clock MT_CG_AUDIO_I2S fail");
            }
#else
            if (paudclk->aud_i2s_clk_status)
            {
                ret = clk_prepare_enable(paudclk->aud_i2s_clk);
                if (!ret)
                {
                    pr_err("%s Aud enable_clock enable_clock aud_i2s_clk fail", __func__);
                    BUG();
                    return;                     
                }
            }
            else
            {
                pr_err("%s clk_status error Aud enable_clock aud_i2s_clk fail", __func__);
                BUG();
                return;                                    
            }            
#endif             
        }
        if (Aud_ADC_Clk_cntr > 0)
        {
            Afe_Set_Reg(AUDIO_TOP_CON0, 0 << 24 , 1 << 24);
        }
        if (Aud_ADC2_Clk_cntr > 0)
        {
        	#if 0 //6752 removed
            if (enable_clock(MT_CG_AUDIO_ADDA2, "AUDIO"))
            {
                PRINTK_AUD_CLK("%s fail", __func__);
            }
			#endif
        }
        if (Aud_ADC3_Clk_cntr > 0)
        {
        	#if 0 //6752 removed
            if (enable_clock(MT_CG_AUDIO_ADDA3, "AUDIO"))
            {
                PRINTK_AUD_CLK("%s fail", __func__);
            }
			#endif
        }
        if (Aud_ANA_Clk_cntr > 0)
        {

        }
        if (Aud_HDMI_Clk_cntr > 0)
        {

        }
        if (Aud_APLL22M_Clk_cntr  > 0)
        {

            enable_mux(MT_MUX_AUD1, "AUDIO");
            clkmux_sel(MT_MUX_AUD1, 1 , "AUDIO"); //select APLL1
            
#if defined(CONFIG_MTK_LEGACY)       
            if (enable_clock(MT_CG_AUDIO_22M, "AUDIO"))
            {
                PRINTK_AUD_CLK("%s fail", __func__);
            }
            if (enable_clock(MT_CG_AUDIO_APLL_TUNER, "AUDIO"))
            {
                PRINTK_AUD_CLK("%s fail", __func__);
            }
#else            
            if (paudclk->aud_apll22m_clk_status)
            {
                ret = clk_prepare_enable(paudclk->aud_apll22m_clk);
                if (!ret)
                {
                    pr_err("%s Aud enable_clock enable_clock aud_apll22m_clk fail", __func__);
                    BUG();
                    return;                     
                }
            }
            else
            {
                pr_err("%s clk_status error Aud enable_clock aud_apll22m_clk fail", __func__);
                BUG();
                return;                                    
            }  
            
            if (paudclk->aud_apll1_tuner_clk_status)
            {
                ret = clk_prepare_enable(paudclk->aud_apll1_tuner_clk);
                if (!ret)
                {
                    pr_err("%s Aud enable_clock enable_clock aud_apll1_tuner_clk fail", __func__);
                    BUG();
                    return;                     
                }
            }
            else
            {
                pr_err("%s clk_status error Aud enable_clock aud_apll1_tuner_clk fail", __func__);
                BUG();
                return;                                    
            }  
#endif        
        }
        if (Aud_APLL24M_Clk_cntr > 0)
        {
            enable_mux(MT_MUX_AUD2, "AUDIO");
            clkmux_sel(MT_MUX_AUD2, 1, "AUDIO"); //APLL2
#if defined(CONFIG_MTK_LEGACY)                   
            if (enable_clock(MT_CG_AUDIO_24M, "AUDIO"))
            {
                PRINTK_AUD_CLK("%s fail", __func__);
            }
            if (enable_clock(MT_CG_AUDIO_APLL2_TUNER, "AUDIO"))
            {
                PRINTK_AUD_CLK("%s fail", __func__);
            }
#else            
            if (paudclk->aud_apll24m_clk_status)
            {
                ret = clk_prepare_enable(paudclk->aud_apll24m_clk);
                if (!ret)
                {
                    pr_err("%s Aud enable_clock enable_clock aud_apll24m_clk fail", __func__);
                    BUG();
                    return;                     
                }
            }
            else
            {
                pr_err("%s clk_status error Aud enable_clock aud_apll24m_clk fail", __func__);
                BUG();
                return;                                    
            }


            if (paudclk->aud_apll2_tuner_clk_status)
            {
                ret = clk_prepare_enable(paudclk->aud_apll2_tuner_clk);
                if (!ret)
                {
                    pr_err("%s Aud enable_clock enable_clock aud_apll2_tuner_clk fail", __func__);
                    BUG();
                    return;                     
                }
            }
            else
            {
                pr_err("%s clk_status error Aud enable_clock aud_apll2_tuner_clk fail", __func__);
                BUG();
                return;                                    
            }    
#endif            
        }
#endif
    }
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_Emi_Clk_On(void)
{
    mutex_lock(&auddrv_pmic_mutex);
    if(Aud_EMI_cntr == 0)
    {
#ifndef CONFIG_FPGA_EARLY_PORTING //george early porting disable
        disable_dpidle_by_bit(MT_CG_AUDIO_AFE);
        disable_soidle_by_bit(MT_CG_AUDIO_AFE);
#endif
    }
    Aud_EMI_cntr++;
    mutex_unlock(&auddrv_pmic_mutex);
}
void AudDrv_Emi_Clk_Off(void)
{
    mutex_lock(&auddrv_pmic_mutex);
    Aud_EMI_cntr--;
    if(Aud_EMI_cntr ==0)
    {
#ifndef CONFIG_FPGA_EARLY_PORTING //george early porting disable
        enable_dpidle_by_bit(MT_CG_AUDIO_AFE);
        enable_soidle_by_bit(MT_CG_AUDIO_AFE);
#endif
    }

    if(Aud_EMI_cntr <0 )
    {
        Aud_EMI_cntr = 0;
        pr_warn("Aud_EMI_cntr = %d \n",Aud_EMI_cntr);
    }
    mutex_unlock(&auddrv_pmic_mutex);
}

// export symbol for other module use
EXPORT_SYMBOL(AudDrv_Clk_On);
EXPORT_SYMBOL(AudDrv_Clk_Off);
EXPORT_SYMBOL(AudDrv_ANA_Clk_On);
EXPORT_SYMBOL(AudDrv_ANA_Clk_Off);
EXPORT_SYMBOL(AudDrv_I2S_Clk_On);
EXPORT_SYMBOL(AudDrv_I2S_Clk_Off);

