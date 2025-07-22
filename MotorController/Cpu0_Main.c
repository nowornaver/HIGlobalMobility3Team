/***********************************************************************/
/*Include*/ 
/***********************************************************************/
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "Driver_Watchdog.h"
#include "Driver_Stm.h"
#include "Driver_Port.h"
#include "Driver_Adc.h"
#include "Driver_Asc.h"
#include "Driver_Gtm.h"
#include "DrvDio.h"
#include "AppScheduling.h"
typedef struct
{
    float fPllFreq;

    float fSourceFreq;

    float fSriFreq;
    float fSpbFreq;

    float fCpu0Freq;
    float fCpu1Freq;
    float fCpu2Freq;

    float fFsiFreq;
    float fFsi2Freq;

    float fStmFreq;
    float fGtmFreq;

    float fCanFreq;
}ClockSetting;

ClockSetting ClockSettingInfo;
/***********************************************************************/
/*Typedef*/ 
/***********************************************************************/

/***********************************************************************/
/*Static Function Prototype*/ 
/***********************************************************************/


/***********************************************************************/
/*Variable*/ 
/***********************************************************************/

/***********************************************************************/
/*Function*/ 
/***********************************************************************/
int core0_main(void)
{   

    ClockSettingInfo.fPllFreq = IfxScuCcu_getPllFrequency();
    ClockSettingInfo.fSourceFreq = IfxScuCcu_getSourceFrequency();

    ClockSettingInfo.fSriFreq = IfxScuCcu_getSriFrequency();
    ClockSettingInfo.fSpbFreq = IfxScuCcu_getSpbFrequency();

    ClockSettingInfo.fCpu0Freq = IfxScuCcu_getCpuFrequency(0);
    ClockSettingInfo.fCpu1Freq = IfxScuCcu_getCpuFrequency(1);
    ClockSettingInfo.fCpu2Freq = IfxScuCcu_getCpuFrequency(2);

    ClockSettingInfo.fFsiFreq = IfxScuCcu_getFsiFrequency();
    ClockSettingInfo.fFsi2Freq = IfxScuCcu_getFsi2Frequency();

    ClockSettingInfo.fStmFreq = IfxScuCcu_getStmFrequency();
    ClockSettingInfo.fGtmFreq = IfxScuCcu_getGtmFrequency();

    ClockSettingInfo.fCanFreq = IfxScuCcu_getCanFrequency();
    /*Driver Init*/
    Driver_Watchdog_Init();
//    Driver_Port_Init();
    Driver_Stm_Init();
    Driver_Adc_Init();
    Driver_Asc_Init();
    Driver_Gtm_Init();
    DrvDioInit();
    /*Global Interrupt Enable*/
    IfxCpu_enableInterrupts();

    while(1)
    {
        AppScheduling(); //여기함수안에
    }
    return (1);
}
