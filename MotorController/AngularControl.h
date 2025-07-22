#ifndef ANGULARCONTROL_H
#define ANGULARCONTROL_H

/*----------------------------------------------------------------*/
/*                        Include Header File                          */
/*----------------------------------------------------------------*/
#include "Ifx_Types.h"

/*----------------------------------------------------------------*/
/*                        Define                                        */
/*----------------------------------------------------------------*/


/*----------------------------------------------------------------*/
/*                        Typedefs                                    */
/*----------------------------------------------------------------*/
//typedef enum t_MOTOR_CMD_TYPE
//{
//    MOTOR_STOP = 0u,
//    MOTOR_FWD,
//    MOTOR_TURN_RIGHT,
//    MOTOR_TURN_LEFT,
//    MOTOR_REVERSE,
//    MOTOR_CMD_MAX
//}MOTOR_CMD_TYPE;


typedef struct
{
        int32_t TargetPot ;
        int32_t CurrentPot;
        int32_t targetangle;
        float control;
        float kp;
        float ki;
        float kd;
        float Integral;
        float PrevError;
        float Control;   // 최종 PWM duty (0.0 ~ 1.0)



}Potential;

extern Potential po;

/*----------------------------------------------------------------*/
/*                        Variables                                    */
/*----------------------------------------------------------------*/


/*----------------------------------------------------------------*/
/*                        Global Function Prototype                  */
/*----------------------------------------------------------------*/
extern void AngularFeedbackController(void);
//extern void Unit_AngularFrontDirectionCtl(MOTOR_CMD_TYPE param_DirectionType);
//extern void Unit_MotorRearDirectionCtl(MOTOR_CMD_TYPE param_DirectionType);
//extern void Unit_TestControl(void);
extern int32_t getPotFromAngle(void);


#endif

