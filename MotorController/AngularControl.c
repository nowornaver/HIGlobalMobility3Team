/*----------------------------------------------------------------*/
/*                        Include Header File                          */
/*----------------------------------------------------------------*/
#include "AngularControl.h"
#include "DrvDio.h"
#include "Driver_Gtm.h"
#include "Driver_Adc.h"
#include <math.h>
/*----------------------------------------------------------------*/
/*                        Define                                        */
/*----------------------------------------------------------------*/


/*----------------------------------------------------------------*/
/*                        Typedefs                                    */
/*----------------------------------------------------------------*/


/*----------------------------------------------------------------*/
/*                        Static Function Prototype                  */
/*----------------------------------------------------------------*/

const int angles[] = {-20, 24}; //<< 튜닝을 해봐야함.
const int potValues[] = {4095, 0}; //<< 튜닝해야함

/*----------------------------------------------------------------*/
/*                        Variables                                    */
/*----------------------------------------------------------------*/
//Angle angle;
Potential po ;
//po.kp = 0.1f;  // 적절히 튜닝 필요

/*----------------------------------------------------------------*/
/*                        Functions                                    */
/*----------------------------------------------------------------*/
#if 1

int32_t getPotFromAngle(void) {
    const int angle0 = -17;
    const int angle1 = 22;
    const int pot0 = 4095;
    const int pot1 = 520;
    po.TargetPot = pot0 + (po.targetangle - angle0) * (float)(pot1 - pot0) / (angle1 - angle0);
    return pot0 + (po.targetangle - angle0) * (float)(pot1 - pot0) / (angle1 - angle0);


}
void AngularFeedbackController(void)
{

    float dt = 0.01f;

    po.CurrentPot = adcDataResult[1];
    float error = (float)(po.TargetPot - po.CurrentPot);

    po.Integral += error * dt;
    if (po.Integral > 1000.0f) po.Integral = 1000.0f;
    else if (po.Integral < -1000.0f) po.Integral = -1000.0f;

    float derivative = (error - po.PrevError) / dt;

    float output = po.kp * error + po.ki * po.Integral + po.kd * derivative;

    // 출력 스케일 조정 (필요하면)
         output /= 100.0f;
//         output = roundf(output);


    // 출력 클램핑
    if (output > 1.0f) output = 1.0f;
    else if (output < -1.0f) output = -1.0f;

    po.control = output;
    po.PrevError = error;

    float duty = fabsf(po.control);
    if (duty < 0.01f)
        duty = 0.0f;
    if (duty < 0.01f)
        duty = 0.0f;
    if (po.control <0) {
        DrvDio_SetPinHigh(IfxPort_P00_2);  // DIR 핀 HIGH (역방향)
        DrvGtmPwmTest(duty,duty,duty,duty);

    }
    else {

        DrvDio_SetPinLow(IfxPort_P00_2);  // DIR 핀 HIGH (역방향)
        float duty = fabsf(po.control);

        DrvGtmPwmTest(duty,duty,duty,duty);

    }


}


#endif
//void Unit_AngularFrontDirectionCtl(MOTOR_CMD_TYPE param_DirectionType)
//{
//    switch(param_DirectionType)
//    {
//        case MOTOR_STOP: /*Stop*/
//        {
//
//            DrvDio_SetPinHigh(IfxPort_P00_1);
//            DrvGtmPwmTest(0,0,0,0);
//
//
//            //break 핀 활성화
//
//
//            break;
//        }
//        case MOTOR_FWD: /*Forward*/
//        {
//            DrvDio_SetPinLow(IfxPort_P00_1);   // 브레이크 OFF (핀 LOW)
//            DrvDio_SetPinLow(IfxPort_P00_2);   // DIR 핀 LOW (정방향)
//            DrvGtmPwmTest(1.0f,1.0f,1.0f,1.0f);
////            a.Dir = 0; //정방향 회전
////            MidDio_SetFrontIn1(1u);
////            MidDio_SetFrontIn2(0u);
////            MidDio_SetFrontIn3(1u);
////            MidDio_SetFrontIn4(0u);
//            break;
//        }
////        case MOTOR_TURN_RIGHT: /*TurnRight*/
////        {
////            MidDio_SetFrontIn1(1u);
////            MidDio_SetFrontIn2(0u);
////            MidDio_SetFrontIn3(0u);
////            MidDio_SetFrontIn4(1u);
////            break;
////        }
////        case MOTOR_TURN_LEFT: /*TurnLeft*/
////        {
////            MidDio_SetFrontIn1(0u);
////            MidDio_SetFrontIn2(1u);
////            MidDio_SetFrontIn3(1u);
////            MidDio_SetFrontIn4(0u);
////            break;
////        }
//        case MOTOR_REVERSE: /*Reverse*/
//        {
//            DrvDio_SetPinLow(IfxPort_P00_1);   // 브레이크 OFF
//            DrvDio_SetPinHigh(IfxPort_P00_2);  // DIR 핀 HIGH (역방향)
////            MidDio_SetFrontIn1(0u);
////            MidDio_SetFrontIn2(1u);
////            MidDio_SetFrontIn3(0u);
////            MidDio_SetFrontIn4(1u);
//            //dir 핀 사용 1 로 바꾼다거나
////            a.Dir=1; //역뱡향 회전
//            break;
//        }
//        default:
//        break;
//    }
//}
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

//void Unit_TestControl(void)
//{
//    if(u8nuTestInput == 1u)    /*Forward*/
//    {
//        Unit_MotorFrontDirectionCtl(MOTOR_FWD);
////        Unit_MotorRearDirectionCtl(MOTOR_FWD);
//    }
//    else if(u8nuTestInput == 2u) /*TurnRight*/
//    {
//        Unit_MotorFrontDirectionCtl(MOTOR_TURN_RIGHT);
////        Unit_MotorRearDirectionCtl(MOTOR_TURN_RIGHT);
//    }
//    else if(u8nuTestInput == 3u) /*TurnLeft*/
//    {
//        Unit_MotorFrontDirectionCtl(MOTOR_TURN_LEFT);
////        Unit_MotorRearDirectionCtl(MOTOR_TURN_LEFT);
//    }
//    else if(u8nuTestInput == 4u) /*Reverse*/
//    {
//
//        Unit_MotorFrontDirectionCtl(MOTOR_REVERSE);
//        //여기서 DIR 방향에 대한 변수 바꿔주기
////        Unit_MotorRearDirectionCtl(MOTOR_REVERSE);
//    }
//    else if(u8nuTestInput == 5u) /*Stop*/
//    {
//        Unit_MotorFrontDirectionCtl(MOTOR_STOP);
////        Unit_MotorRearDirectionCtl(MOTOR_STOP);
//    }
//    else
//    {
//        /*No Code*/
//    }
//}

