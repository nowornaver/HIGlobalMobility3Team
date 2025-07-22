#ifndef DRIVER_GTM
#define DRIVER_GTM

/***********************************************************************/
/*Include*/ 
/***********************************************************************/
#include "Ifx_Types.h"
#include "IfxGtm_Tom_Timer.h"
#include "IfxGtm_Tom_PwmHl.h"


/***********************************************************************/
/*Typedef*/ 
/***********************************************************************/
typedef struct
{
    struct
    {
        float32 gtmFreq;
        float32 gtmGclkFreq;
    }info;
    struct
    {
        IfxGtm_Tom_Timer timerOneMs;        /**< \brief Timer driver */
    }drivers;
    struct
    {
        uint32 slotOneMs;
    }isrCounter;
} App_GtmTomTimer;


/***********************************************************************/
/*Define*/ 
/***********************************************************************/

/***********************************************************************/
/*External Variable*/ 
/***********************************************************************/
typedef struct
{
        uint32_t CurrentSpeed ;
        uint32_t TargetSpeed ;
        boolean Dir;
}Motor;


/***********************************************************************/
/*Global Function Prototype*/ 
/***********************************************************************/
extern void Driver_Gtm_Init(void);
//extern void Driver_Gtm_Tom0Test(void);
extern void DrvGtmPwmTest(float32_t param_Ch12Duty, float32_t param_Ch13Duty, float32_t param_Ch14Duty, float32_t param_Ch15Duty);
extern void Driver_Tom0_Init(void);


#endif /* DRIVER_STM */
