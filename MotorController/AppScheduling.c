/***********************************************************************/
/*Include*/ 
/***********************************************************************/
#include "AppScheduling.h"
#include "Driver_Stm.h"
#include "Driver_Adc.h"
#include "Driver_Asc.h"
#include "Driver_Gtm.h"
#include "AppMode.h"
#include "MotorControl.h"
#include "AngularControl.h"
/***********************************************************************/
/*Define*/ 
/***********************************************************************/

/***********************************************************************/
/*Typedef*/ 
/***********************************************************************/
typedef struct
{
    uint32_t u32nuCnt1ms;
    uint32_t u32nuCnt10ms;
    uint32_t u32nuCnt100ms;
}TestCnt;


//typedef struct
//{
//        int32_t TargetPot ;
//
//}Angle;
/***********************************************************************/
/*Static Function Prototype*/ 
/***********************************************************************/
static void AppTask1ms(void);
static void AppTask10ms(void);
static void AppTask100ms(void);
int32_t getPotFromAngle(void);
void AngularFeedbackController(void);
/***********************************************************************/
/*Variable*/ 
/***********************************************************************/
TestCnt stTestCnt;
//extern uint32_t gu32nuAscRxData;
//Angle ang;

/***********************************************************************/
/*Function*/ 
/***********************************************************************/
static void AppTask1ms(void)
{
    stTestCnt.u32nuCnt1ms++;

}
//motorRPM a ;
static void AppTask10ms(void)
{
//    MotorFeedbackController();
    stTestCnt.u32nuCnt10ms++;
//    Unit_Mode();

//    float32_t fDuty = 0.025f;
//    MotorFeedbackController();
////    float32_t fSenseMotorRpm = 0.0f;
////   a= ((float32_t)ulPulseCnt*60.0f*10.0f)/(9.36f*103.0f); //현재 모터 RPM
//    a.motor11 = ((float32_t)ulPulseCnt*60.0f*10.0f)/(103.0f);
////    a.motor11=0;
//    DrvGtmPwmTest(fDuty,fDuty,fDuty,fDuty); //이 값이 Duty 값 조정하는거
//    Unit_TestControl();
//    ulPulseCnt=0;

    /*ADC Test*/
//    Driver_Adc0_ConvStart();
//
//    /*ASC Test*/

    /*PWM Test*/
//    Driver_Gtm_Tom0Test();


//
//    float32_t fDuty = 0.4f;
//        MotorFeedbackController();
//
////    DrvGtmPwmTest(fDuty,fDuty,fDuty,fDuty);
//    Unit_TestControl();
    Unit_Mode();

    /*ADC Test*/
    Driver_Adc0_ConvStart();

    Driver_Adc0_DataObtain();
    getPotFromAngle();

    AngularFeedbackController();
    Driver_Asc0_TxTest();

}

static void AppTask100ms(void)
{


//    float32_t fDuty = 1.0f;
//    MotorFeedbackController();
////    float32_t fSenseMotorRpm = 0.0f;
////   a= ((float32_t)ulPulseCnt*60.0f*10.0f)/(9.36f*103.0f); //현재 모터 RPM
//    a.motor11 = ((float32_t)ulPulseCnt*60.0f*10.0f)/(9.36f*103.0f);
//    DrvGtmPwmTest(fDuty,fDuty,fDuty,fDuty); //이 값이 Duty 값 조정하는거

//    Driver_Gtm_Tom0Test();
    stTestCnt.u32nuCnt100ms++;
}

void AppScheduling(void)
{
    if(stSchedulingInfo.u8nuScheduling1msFlag == 1u)
    {
        stSchedulingInfo.u8nuScheduling1msFlag = 0u;
        AppTask1ms();
        
        if(stSchedulingInfo.u8nuScheduling10msFlag == 1u)
        {
//            IfxCpu_delay(100); // 대략 몇 ms 정도 (정확하지 않음)
            stSchedulingInfo.u8nuScheduling10msFlag = 0u;
            AppTask10ms();
        }

        if(stSchedulingInfo.u8nuScheduling100msFlag == 1u)
        {
            stSchedulingInfo.u8nuScheduling100msFlag = 0u;
            AppTask100ms();
        }
    }
}
