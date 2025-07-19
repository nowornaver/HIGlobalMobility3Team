/*----------------------------------------------------------------*/
/*                        Include Header File                          */
/*----------------------------------------------------------------*/
#include "MotorControl.h"
#include "DrvDio.h"
#include "Driver_Gtm.h"

/*----------------------------------------------------------------*/
/*                        Define                                        */
/*----------------------------------------------------------------*/


/*----------------------------------------------------------------*/
/*                        Typedefs                                    */
/*----------------------------------------------------------------*/


/*----------------------------------------------------------------*/
/*                        Static Function Prototype                  */
/*----------------------------------------------------------------*/




/*----------------------------------------------------------------*/
/*                        Variables                                    */
/*----------------------------------------------------------------*/
uint8_t u8nuTestInput = 0u;

/*----------------------------------------------------------------*/
/*                        Functions                                    */
/*----------------------------------------------------------------*/
#if 1
float32_t fPwmDuty = 0.5f;
float32_t fSenseMotorRpm = 100.0f; //currentRPM

uint32_t ulRpmRef =50u; // TargetRPM
uint32_t ulPGain = 3u; //0.3
uint32_t ulIGain = 12u;
int32_t static previous_pos = 0;
//float32_t esp = 0.1;               // 적분 감쇠 계수


extern uint32_t gu32nuAscRxData;
extern uint32_t ulPulseCnt;
extern uint32_t gu32nuRxFlag;

void MotorFeedbackController(void)
{


    static int32_t lProportionalControlInput = 0;
    static int32_t lIntegralControlInput = 0;
    static int32_t lIntegralControlOld = 0;
    static uint32_t ulSamplingFrequency = 10u; /*100ms*/
    int32_t current_pos = GTM_TIM0_CH0_CNT.U;
    int32_t delta = current_pos - previous_pos;
    previous_pos = current_pos;
    int32_t g_nError = 0;
    int32_t g_nControlInput = 0;

//    a.TargetSpeed = ulRpmRef;

    fSenseMotorRpm = ((float)delta / (float)(300 * 4)) * (60.0f / 0.01f); //Current RPM
//    a.CurrentSpeed = fSenseMotorRpm;
    ulPulseCnt = 0u;

    /*PID Contorller*/
    g_nError =((int32_t)ulRpmRef- (int32_t)fSenseMotorRpm);
    lProportionalControlInput = (int32_t)ulPGain*g_nError/10;   // Not to express the gain as float -> g_nPGain/10
//
//    lIntegralControlInput = lIntegralControlOld + ((int32_t)ulIGain*g_nError/(int32_t)ulSamplingFrequency/10);
//    g_nControlInput = (lProportionalControlInput + lIntegralControlInput);  //PID control input
    g_nControlInput= lProportionalControlInput;
//    lIntegralControlOld = lIntegralControlInput;
//
//    if(lIntegralControlOld > 80u)
//    {
//        lIntegralControlOld = 80u; //Limit accumulated value by I gain
//    }
//
//    if(g_nControlInput >= 80u)
//    {
//        g_nControlInput =80u;  //Limit control input (0<=contorl input<100)
//    }
//    else if(g_nControlInput <= 0u)
//    {
//        g_nControlInput = 0u;
//    }
//    else
//    {
//        /*No Code*/
//    }

    /*Gtm PWM Test*/
//    fPwmDuty = (float32_t)g_nControlInput/100.0f;
//    DrvGtmPwmTest(fPwmDuty,fPwmDuty,fPwmDuty,fPwmDuty);
}

void Unit_WirelessControl(void)
{
    uint8_t ucWirelessCmd = 0u;

    ucWirelessCmd = (uint8_t)gu32nuAscRxData;

    if(ucWirelessCmd == 'w')    /*Forward*/
    {
        Unit_MotorFrontDirectionCtl(MOTOR_FWD);
//        Unit_MotorRearDirectionCtl(MOTOR_FWD);
    }
    else if(ucWirelessCmd == 'd') /*TurnRight*/
    {
        Unit_MotorFrontDirectionCtl(MOTOR_TURN_RIGHT);
//        Unit_MotorRearDirectionCtl(MOTOR_TURN_RIGHT);
    }
    else if(ucWirelessCmd == 'a') /*TurnLeft*/
    {
        Unit_MotorFrontDirectionCtl(MOTOR_TURN_LEFT);
//        Unit_MotorRearDirectionCtl(MOTOR_TURN_LEFT);
    }
    else if(ucWirelessCmd == 'x') /*Reverse*/
    {
        Unit_MotorFrontDirectionCtl(MOTOR_REVERSE);
//        Unit_MotorRearDirectionCtl(MOTOR_REVERSE);
    }
    else if(ucWirelessCmd == 's') /*Stop*/
    {
        Unit_MotorFrontDirectionCtl(MOTOR_STOP);
//        Unit_MotorRearDirectionCtl(MOTOR_STOP);
    }
    else if((ucWirelessCmd == 'u') && (gu32nuRxFlag ==1)) /*Stop*/
    {
        ulRpmRef = ulRpmRef + 5u;
        if(ulRpmRef >=80u)
        {
            ulRpmRef = 80u;
        }
        gu32nuRxFlag = 0u;
    }
    else if((ucWirelessCmd == 'y') && (gu32nuRxFlag ==1)) /*Stop*/
    {
        ulRpmRef = ulRpmRef - 5u;
        if(ulRpmRef <= 0u)
        {
            ulRpmRef = 0u;
        }
        gu32nuRxFlag = 0u;
    }
    else
    {
        /*No Code*/
    }
}
#endif
void Unit_MotorFrontDirectionCtl(MOTOR_CMD_TYPE param_DirectionType)
{
    switch(param_DirectionType)
    {
        case MOTOR_STOP: /*Stop*/
        {

            DrvDio_SetPinHigh(IfxPort_P00_1);
            DrvGtmPwmTest(0,0,0,0);


            //break 핀 활성화


            break;
        }
        case MOTOR_FWD: /*Forward*/
        {
            DrvDio_SetPinLow(IfxPort_P00_1);   // 브레이크 OFF (핀 LOW)
            DrvDio_SetPinLow(IfxPort_P00_2);   // DIR 핀 LOW (정방향)
            DrvGtmPwmTest(1.0f,1.0f,1.0f,1.0f);
//            a.Dir = 0; //정방향 회전
//            MidDio_SetFrontIn1(1u);
//            MidDio_SetFrontIn2(0u);
//            MidDio_SetFrontIn3(1u);
//            MidDio_SetFrontIn4(0u);
            break;
        }
//        case MOTOR_TURN_RIGHT: /*TurnRight*/
//        {
//            MidDio_SetFrontIn1(1u);
//            MidDio_SetFrontIn2(0u);
//            MidDio_SetFrontIn3(0u);
//            MidDio_SetFrontIn4(1u);
//            break;
//        }
//        case MOTOR_TURN_LEFT: /*TurnLeft*/
//        {
//            MidDio_SetFrontIn1(0u);
//            MidDio_SetFrontIn2(1u);
//            MidDio_SetFrontIn3(1u);
//            MidDio_SetFrontIn4(0u);
//            break;
//        }
        case MOTOR_REVERSE: /*Reverse*/
        {
            DrvDio_SetPinLow(IfxPort_P00_1);   // 브레이크 OFF
            DrvDio_SetPinHigh(IfxPort_P00_2);  // DIR 핀 HIGH (역방향)
//            MidDio_SetFrontIn1(0u);
//            MidDio_SetFrontIn2(1u);
//            MidDio_SetFrontIn3(0u);
//            MidDio_SetFrontIn4(1u);
            //dir 핀 사용 1 로 바꾼다거나
//            a.Dir=1; //역뱡향 회전
            break;
        }
        default:
        break;
    }
}
//
//
//void Unit_MotorRearDirectionCtl(MOTOR_CMD_TYPE param_DirectionType)
//{
//    switch(param_DirectionType)
//    {
//        case MOTOR_STOP: /*Stop*/
//        {
//            MidDio_SetRearIn1(0u);
//            MidDio_SetRearIn2(0u);
//            MidDio_SetRearIn3(0u);
//            MidDio_SetRearIn4(0u);
//            break;
//        }
//        case MOTOR_FWD: /*Forward*/
//        {
//            MidDio_SetRearIn1(1u);
//            MidDio_SetRearIn2(0u);
//            MidDio_SetRearIn3(1u);
//            MidDio_SetRearIn4(0u);
//            break;
//        }
//        case MOTOR_TURN_RIGHT: /*TurnRight*/
//        {
//            MidDio_SetRearIn1(1u);
//            MidDio_SetRearIn2(0u);
//            MidDio_SetRearIn3(0u);
//            MidDio_SetRearIn4(1u);
//            break;
//        }
//        case MOTOR_TURN_LEFT: /*TurnLeft*/
//        {
//            MidDio_SetRearIn1(0u);
//            MidDio_SetRearIn2(1u);
//            MidDio_SetRearIn3(1u);
//            MidDio_SetRearIn4(0u);
//            break;
//        }
//        case MOTOR_REVERSE: /*Reverse*/
//        {
//            MidDio_SetRearIn1(0u);
//            MidDio_SetRearIn2(1u);
//            MidDio_SetRearIn3(0u);
//            MidDio_SetRearIn4(1u);
//            break;
//        }
//        default:
//        break;
//    }
//}

void Unit_TestControl(void)
{
    if(u8nuTestInput == 1u)    /*Forward*/
    {
        Unit_MotorFrontDirectionCtl(MOTOR_FWD);
//        Unit_MotorRearDirectionCtl(MOTOR_FWD);
    }
    else if(u8nuTestInput == 2u) /*TurnRight*/
    {
        Unit_MotorFrontDirectionCtl(MOTOR_TURN_RIGHT);
//        Unit_MotorRearDirectionCtl(MOTOR_TURN_RIGHT);
    }
    else if(u8nuTestInput == 3u) /*TurnLeft*/
    {
        Unit_MotorFrontDirectionCtl(MOTOR_TURN_LEFT);
//        Unit_MotorRearDirectionCtl(MOTOR_TURN_LEFT);
    }
    else if(u8nuTestInput == 4u) /*Reverse*/
    {

        Unit_MotorFrontDirectionCtl(MOTOR_REVERSE);
        //여기서 DIR 방향에 대한 변수 바꿔주기
//        Unit_MotorRearDirectionCtl(MOTOR_REVERSE);
    }
    else if(u8nuTestInput == 5u) /*Stop*/
    {
        Unit_MotorFrontDirectionCtl(MOTOR_STOP);
//        Unit_MotorRearDirectionCtl(MOTOR_STOP);
    }
    else
    {
        /*No Code*/
    }
}

