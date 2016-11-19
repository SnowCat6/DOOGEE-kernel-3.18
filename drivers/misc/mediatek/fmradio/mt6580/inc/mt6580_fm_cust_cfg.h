#ifndef __FM_CUST_CFG_H__
#define __FM_CUST_CFG_H__

/* scan sort algorithm */
enum {
	FM_SCAN_SORT_NON = 0,
	FM_SCAN_SORT_UP,
	FM_SCAN_SORT_DOWN,
	FM_SCAN_SORT_MAX
};

/* ***************************************************************************************** */
/* ***********************************FM config for customer: start****************************** */
/* ***************************************************************************************** */
/* RX */
#define FM_RX_RSSI_TH_LONG_MT6580    -296	/* FM radio long antenna RSSI threshold(-4dBuV) */
#define FM_RX_RSSI_TH_SHORT_MT6580   -296	/* FM radio short antenna RSSI threshold(-4dBuV) */
#define FM_RX_DESENSE_RSSI_MT6580   -258
#define FM_RX_PAMD_TH_MT6580          -12
#define FM_RX_MR_TH_MT6580           -67
#define FM_RX_ATDC_TH_MT6580           3496
#define FM_RX_PRX_TH_MT6580           64
#define FM_RX_SMG_TH_MT6580          16421	/* FM soft-mute gain threshold */
#define FM_RX_DEEMPHASIS_MT6580       0	/* 0-50us, China Mainland; 1-75us China Taiwan */
#define FM_RX_OSC_FREQ_MT6580         0	/* 0-26MHz; 1-19MHz; 2-24MHz; 3-38.4MHz; 4-40MHz; 5-52MHz */

/* ***************************************************************************************** */
/* ***********************************FM config for customer:end ******************************* */
/* ***************************************************************************************** */

/*#define FM_SEEK_SPACE_MT6580 FM_RX_SEEK_SPACE_MT6580 */
/* max scan chl num */
/* #define FM_MAX_CHL_SIZ_MT6580E FM_RX_SCAN_CH_SIZE_MT6580 */
/* auto HiLo */
#define FM_AUTO_HILO_OFF_MT6580    0
#define FM_AUTO_HILO_ON_MT6580     1

/* seek threshold */
#define FM_SEEKTH_LEVEL_DEFAULT_MT6580 4

extern fm_cust_cfg mt6580_fm_config;

#endif /* __FM_CUST_CFG_H__ */
