/*****************************************************************************
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/types.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"


UINT32 S5K4ECGX_MIPI_Open(void)
{
     return ERROR_NONE;
} /* S5K4ECGXOpen() */

UINT32 S5K4ECGX_MIPI_GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
    MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
     return ERROR_NONE;
}  /* NSXC301HS5K4ECGXGetInfo() */



UINT32 S5K4ECGX_MIPI_GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    return ERROR_NONE;
} /* NSXC301HS5K4ECGXGetResolution() */

UINT32 S5K4ECGX_MIPI_FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
               UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
 
    return ERROR_NONE;
} /* S5K4ECGXFeatureControl() */

UINT32 S5K4ECGX_MIPI_Control(
    MSDK_SCENARIO_ID_ENUM ScenarioId,
    MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
   return ERROR_NONE;
} /* S5K4ECGXControl() */

UINT32 S5K4ECGX_MIPI_Close(void)
{

    return ERROR_NONE;
} /* S5K4ECGXClose() */

SENSOR_FUNCTION_STRUCT  SensorFuncS5K4ECGX_MIPI=
{
    S5K4ECGX_MIPI_Open,             // get sensor id, set initial setting to sesnor
    S5K4ECGX_MIPI_GetInfo,          // get sensor capbility,
    S5K4ECGX_MIPI_GetResolution,    // get sensor capure/preview resolution
    S5K4ECGX_MIPI_FeatureControl,   // set shutter/gain, set/read register
    S5K4ECGX_MIPI_Control,          // change mode to preview/capture/video
    S5K4ECGX_MIPI_Close             // close, do nothing currently
};

UINT32 S5K4ECGX_MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
  /* To Do : Check Sensor status here */
  if (pfFunc!=NULL)
     *pfFunc=&SensorFuncS5K4ECGX_MIPI;

  return ERROR_NONE;
} /* SensorInit() */

