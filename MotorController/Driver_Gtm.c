/***********************************************************************/
/*Include*/ 
/***********************************************************************/
#include "Driver_Gtm.h"
#include "IfxCpu.h"
#define TOM_BASE_FREQ    (100000000.0f/256.0f)
//#define TOM_BASE_FREQ    (100000000.0f)
#define PWM_HZ           (200.0f) //이 값을 조절하면 PWM 주기가 바뀜
#define PWM_PERIOD_CNT   TOM_BASE_FREQ/PWM_HZ

/***********************************************************************/
/*Define*/ 
/***********************************************************************/

/***********************************************************************/
/*Typedef*/ 
/***********************************************************************/



/***********************************************************************/
/*Static Function Prototype*/ 
/***********************************************************************/
static void GtmTom1Init(void);
static void GtmTim0Init(void);
static void GtmTim1Init(void) ;

/***********************************************************************/
/*Variable*/ 
/***********************************************************************/
App_GtmTomTimer g_GtmTomTimer; /**< \brief Demo information */
uint32_t u32nuMyTestPwmDuty = 500u; /*Unit: 0.1%, 500 -> 50.0% duty*/
float32_t fMyTestPwmDuty = 0.5f;
uint32_t ulPulseCnt = 0u;
uint32_t ulPulseCnt1 = 0u;
Motor a;
/***********************************************************************/
/*Function*/ 
/***********************************************************************/
//IFX_INTERRUPT(GTM_Tom0IntHandler, 0, 150u);
IFX_INTERRUPT(TIM0_IntHandler, 0, 200);
//IFX_INTERRUPT(TIM1_IntHandler, 0, 50);
//
void TIM0_IntHandler(void)
{
//    IfxCpu_enableInterrupts();
//    if (a.Dir == 0) {
    ulPulseCnt++; //디버깅해서 이 값이 올라가면 값 맞게 받고있는것.
//    }

//    else {
        ulPulseCnt--;
//    }

}

//void TIM1_IntHandler(void)
//{
////    IfxCpu_enableInterrupts();
//    ulPulseCnt1++; //디버깅해서 이 값이 올라가면 값 맞게 받고있는것.
//
//
//}

//static void Driver_Tom0_Init(void)
//{
//    {   /* GTM TOM configuration */
//        IfxGtm_Tom_Timer_Config timerConfig;
//        IfxGtm_Tom_Timer_initConfig(&timerConfig, &MODULE_GTM);
//        timerConfig.base.frequency       = 100;
//        timerConfig.base.isrPriority     = 0u;
//        timerConfig.base.minResolution   = (1.0 / timerConfig.base.frequency) / 1000;
//        timerConfig.base.trigger.enabled = FALSE;
//        timerConfig.tom                  = IfxGtm_Tom_0;
//        timerConfig.timerChannel         = IfxGtm_Tom_Ch_0;
//        timerConfig.clock                = IfxGtm_Tom_Ch_ClkSrc_cmuFxclk1;
//
//        timerConfig.triggerOut                      = &IfxGtm_TOM0_0_TOUT26_P33_4_OUT;
//        timerConfig.base.trigger.enabled            = TRUE;
//        timerConfig.base.trigger.outputEnabled      = TRUE;
//        timerConfig.base.trigger.triggerPoint       = 0u;   /*Default 0 duty*/
//        timerConfig.base.trigger.risingEdgeAtPeriod = TRUE;
//
//        IfxGtm_Tom_Timer_init(&g_GtmTomTimer.drivers.timerOneMs, &timerConfig);
//
//
//        IfxGtm_Tom_Timer_run(&g_GtmTomTimer.drivers.timerOneMs);
//    }



void DrvGtmPwmTest(float32_t param_Ch12Duty, float32_t param_Ch13Duty, float32_t param_Ch14Duty, float32_t param_Ch15Duty)
{
    float32_t fPeriodCnt = PWM_PERIOD_CNT;
    uint32_t ulPeriodCnt = (uint32_t)fPeriodCnt;

    GTM_TOM1_TGC1_GLB_CTRL.U = 0xAA000000u;

    GTM_TOM1_CH12_SR0.B.SR0 = ulPeriodCnt;
    GTM_TOM1_CH12_SR1.B.SR1 = (uint32_t)(fPeriodCnt*param_Ch12Duty);

    GTM_TOM1_CH13_SR0.B.SR0 = ulPeriodCnt;
    GTM_TOM1_CH13_SR1.B.SR1 = (uint32_t)(fPeriodCnt*param_Ch13Duty);

    GTM_TOM1_CH14_SR0.B.SR0 = ulPeriodCnt;
    GTM_TOM1_CH14_SR1.B.SR1 = (uint32_t)(fPeriodCnt*param_Ch14Duty);

    GTM_TOM1_CH15_SR0.B.SR0 = ulPeriodCnt;
    GTM_TOM1_CH15_SR1.B.SR1 = (uint32_t)(fPeriodCnt*param_Ch15Duty);

    GTM_TOM1_TGC1_GLB_CTRL.U = 0xAA000000u;
}

void Driver_Gtm_Init(void)
{/*disable interrupts*/
    /*disable interrupts*/
    boolean  interruptState = IfxCpu_disableInterrupts();

    /*obtain GTM clock frequency*/
    Ifx_GTM *gtm = &MODULE_GTM;
    g_GtmTomTimer.info.gtmFreq = IfxGtm_Cmu_getModuleFrequency(gtm);

    /*Enable GTM*/
    IfxGtm_enable(gtm);

    /*setting the global clock to 100MHz*/
    IfxGtm_Cmu_setGclkFrequency(&MODULE_GTM, g_GtmTomTimer.info.gtmFreq);
    /*get Global clock frequency*/
    g_GtmTomTimer.info.gtmGclkFreq = IfxGtm_Cmu_getGclkFrequency(gtm);

    /*setting CMU0 frequency*/
    IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, g_GtmTomTimer.info.gtmFreq);

    /*Tom1 Init*/
    GtmTom1Init();
    GtmTim0Init();
    GtmTim1Init();
    /*enable interrupts again*/
    IfxCpu_restoreInterrupts(interruptState);

    /*enable Cmu clock*/
    IfxGtm_Cmu_enableClocks(gtm, IFXGTM_CMU_CLKEN_FXCLK | IFXGTM_CMU_CLKEN_CLK0);
}
static void GtmTom1Init(void)
{
    IfxGtm_PinMap_setTomTout(&IfxGtm_TOM1_12_TOUT4_P02_4_OUT, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed4); /*Rear Left*/
//    IfxGtm_PinMap_setTomTout(&IfxGtm_TOM1_13_TOUT5_P02_5_OUT, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed4); /*Rear Right*/
//    IfxGtm_PinMap_setTomTout(&IfxGtm_TOM1_14_TOUT6_P02_6_OUT, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed4);  /*Front Left*/
//    IfxGtm_PinMap_setTomTout(&IfxGtm_TOM1_15_TOUT7_P02_7_OUT, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed4);  /*Front Right*/

    GTM_TOM1_CH12_CTRL.B.RST_CCU0 = 0u;
    GTM_TOM1_CH12_CTRL.B.TRIGOUT = 1u;
    GTM_TOM1_CH12_CTRL.B.SL = 1u;

    GTM_TOM1_CH13_CTRL.B.RST_CCU0 = 1u;
    GTM_TOM1_CH13_CTRL.B.TRIGOUT = 0u;
    GTM_TOM1_CH13_CTRL.B.SL = 1u;

    GTM_TOM1_CH14_CTRL.B.RST_CCU0 = 1u;
    GTM_TOM1_CH14_CTRL.B.TRIGOUT = 0u;
    GTM_TOM1_CH14_CTRL.B.SL = 1u;

    GTM_TOM1_CH15_CTRL.B.RST_CCU0 = 1u;
    GTM_TOM1_CH15_CTRL.B.TRIGOUT = 0u;
    GTM_TOM1_CH15_CTRL.B.SL = 1u;

    GTM_TOM1_TGC1_GLB_CTRL.U = 0xAA000000u;
    GTM_TOM1_TGC1_ENDIS_CTRL.U = 0xAA00u;
    GTM_TOM1_TGC1_OUTEN_CTRL.U = 0xAA00u;

    GTM_TOM1_CH12_CTRL.B.CLK_SRC_SR = IfxGtm_Tom_Ch_ClkSrc_cmuFxclk2;
    GTM_TOM1_CH13_CTRL.B.CLK_SRC_SR = IfxGtm_Tom_Ch_ClkSrc_cmuFxclk2;
    GTM_TOM1_CH14_CTRL.B.CLK_SRC_SR = IfxGtm_Tom_Ch_ClkSrc_cmuFxclk2;
    GTM_TOM1_CH15_CTRL.B.CLK_SRC_SR = IfxGtm_Tom_Ch_ClkSrc_cmuFxclk2;

    GTM_TOM1_TGC1_GLB_CTRL.B.HOST_TRIG = 1u;
}
#if 1
static void GtmTim0Init(void)
{
    float32_t temp = 0.0f;

    IfxGtm_PinMap_setTimTin(&IfxGtm_TIM0_0_TIN26_P33_4_IN, IfxPort_InputMode_pullDown); //33.4 핀과 DC모터의 A상 혹은 B상을 연결해야함.

//    GTM_TIM2_CH3_CTRL.B.TIM_MODE = 2u;
    GTM_TIM0_CH0_CTRL.B.TIM_MODE = 7u;
    GTM_TIM0_CH0_CNT.U= 0 ;
    GTM_TIM0_CH0_CTRL.B.ISL = 0u;
    GTM_TIM0_CH0_CTRL.B.DSL = 0u;
    GTM_TIM0_CH0_CTRL.B.FLT_EN = 1;

//    IfxGtm_enable(&MODULE_GTM);

    temp = 2.0f * TOM_BASE_FREQ;

    GTM_TIM0_CH0_FLT_RE.B.FLT_RE = (uint32_t)temp;

    GTM_TIM0_CH0_CTRL.B.FLT_MODE_RE = (uint8_t)IfxGtm_Tim_FilterMode_immediateEdgePropagation;

    SRC_GTMTIM00.B.SRE = 1;
    SRC_GTMTIM00.B.SRPN = 200u;
    SRC_GTMTIM00.B.TOS = 1u; // CPU0에 할당

    GTM_TIM0_CH0_CNT.U = 0;
    GTM_TIM0_CH0_IRQ_EN.U = 0x1;
    GTM_TIM0_CH0_CTRL.B.TIM_EN = 1;
    GTM_IRQ_MODE.B.IRQ_MODE = 2;
    GTM_TIM0_CH0_IRQ_MODE.B.IRQ_MODE = 2;
    GTM_TIM0_CH0_IRQ_NOTIFY.U = 0x1;  // IRQ notify 플래그 클리어 (A상)
//    GTM_TIM0_CH0_CTRL.B.TIM_MODE = 7;   // Quad


}
#endif

#if 1
static void GtmTim1Init(void)
{
    float32_t temp = 0.0f;

    IfxGtm_PinMap_setTimTin(&IfxGtm_TIM0_1_TIN27_P33_5_IN, IfxPort_InputMode_pullDown); //33.5 핀과 DC모터의 A상 혹은 B상을 연결해야함.
    //DC


//    GTM_TIM0_CH1_CTRL.B.TIM_MODE = 2u;
    GTM_TIM0_CH1_CTRL.B.TIM_MODE = 7u;   // Quad pair

    GTM_TIM0_CH1_CTRL.B.ISL = 1u;
    GTM_TIM0_CH1_CTRL.B.DSL = 1u;
    GTM_TIM0_CH1_CTRL.B.FLT_EN = 1;

    temp = 2.0f * TOM_BASE_FREQ;

    GTM_TIM0_CH1_FLT_RE.B.FLT_RE = (uint32_t)temp;

    GTM_TIM0_CH1_CTRL.B.FLT_MODE_RE = (uint8_t)IfxGtm_Tim_FilterMode_immediateEdgePropagation;




    SRC_GTMTIM01.B.SRE = 0;
    SRC_GTMTIM01.B.SRPN = 200u;
    SRC_GTMTIM01.B.TOS = 0; // CPU0에 할당

    GTM_TIM0_CH1_CNT.U = 0;
    GTM_TIM0_CH1_IRQ_EN.U = 0x1;
    GTM_TIM0_CH1_CTRL.B.TIM_EN = 1;
//    GTM_IRQ_MODE.B.IRQ_MODE = 2;
    GTM_TIM0_CH1_IRQ_MODE.B.IRQ_MODE = 0;
    GTM_TIM0_CH1_IRQ_EN.U = 0x0;

}
#endif


