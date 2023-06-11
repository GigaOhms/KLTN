#define EPWM1_TIMER_TBPRD  1000U  // Period register
#define D_BAT               445U
#define D_PFC               000U

#define DACA    1 // 2 3

#define SQRT3   1.732050808             /* sqrt(3.) */
#define SQRT2   1.414213562             /* sqrt(2.) */
#define PI      3.1415926535897932384626433832795
#define f       50
#define w       2*PI*f
#define Ts      0.00002      // Sample time

#define KP_PFC_VL  5.0
#define KI_PFC_VL  0.01
#define KP_PFC_IL  80.0
#define KI_PFC_IL  0.002
#define VBUSref    390.0
#define LOOP1_MAX   5.0
#define LOOP1_MIN   0.0
#define D_PFC_MAX   0.95
#define D_PFC_MIN   0.0

#define KP_BAT     5.0
#define KI_BAT     0.01
#define VBATref    42.0
#define IBATref    3.3
#define D_BAT_MAX   0.45
#define D_BAT_MIN   0.10

#define VoffsetVAC  1.05
#define VoffsetBAT  (1.5059 + 0.02)
#define VoffsetPFC  (1.5 + 0.01)
#define IoffsetBAT  2.2270
#define IoffsetIL   2.2333
#define IoffsetBAT  2.2270
#define IoffsetIL   2.2333

#define VACgain     (8*1200.0/(1200.0 + 2000000.0))
#define VPFCgain    0.0026
#define VBATgain    0.0281
#define ILgain      0.17909
#define IBATgain    0.17788

#define LED1    58      // Xanh
#define LED2    59      // Vang
#define LED3    124     // Do

#define     VAC_READ1   AdcaResultRegs.ADCRESULT0
#define     VAC_READ2   AdcaResultRegs.ADCRESULT2
#define     VAC_READ3   AdcaResultRegs.ADCRESULT4

#define     VPFC_READ1  AdccResultRegs.ADCRESULT1
#define     VPFC_READ2  AdccResultRegs.ADCRESULT3
#define     VPFC_READ3  AdccResultRegs.ADCRESULT5

#define     VBAT_READ1  AdcbResultRegs.ADCRESULT0
#define     VBAT_READ2  AdcbResultRegs.ADCRESULT2
#define     VBAT_READ3  AdcbResultRegs.ADCRESULT4

#define     IL_READ1    AdcaResultRegs.ADCRESULT1
#define     IL_READ2    AdcaResultRegs.ADCRESULT3
#define     IL_READ3    AdcaResultRegs.ADCRESULT5

#define     IBAT_READ1  AdccResultRegs.ADCRESULT0
#define     IBAT_READ2  AdccResultRegs.ADCRESULT2
#define     IBAT_READ3  AdccResultRegs.ADCRESULT4

typedef struct PI_VAR{
    float ERROR;
    float INTEGRAL;
    float KP;
    float KI;
    float DMAX;
    float DMIN;
    float OUT;
} PI_VAR;

//volatile PI_VAR PFC;
volatile PI_VAR BAT;
volatile PI_VAR PFC1;
volatile PI_VAR PFC2;


volatile float  ILmeas, IBATmeas, VACmeas, VPFCmeas, VBATmeas;
volatile float  VACpeak = 0, VACrms = 0, sumVAC = 0, VAC = 0;
volatile float  VPFC = 0, VPFCavg = 0, sumVPFC = 0;
volatile float  VBAT = 0, VBATavg = 0, sumVBAT = 0;
volatile float  IBAT = 0, IBATavg = 0, sumIBAT = 0;
volatile float  IL = 0, ILavg = 0, sumIL = 0;
volatile int    count = 0;


void setup_gpio(void);
void InitEPwm1(void);
void InitEPwm2(void);
void setup_ADC(void);
void setup_DAC(void);
void setupVAR(void);

void PFC_Control(void);
void BAT_Control(void);
void BAT_CC(void);
void BAT_CV(void);
void isFULL(void);
void peakDETECT(void);


//float PI_CONTROL(float FEEDBACK, float SETPOINT, float KP, float KI, float UPPER, float LOWER);

__interrupt void epwm1_isr(void);

inline void readSensor(void){
    VACmeas     = (float)(VAC_READ1  + VAC_READ2  + VAC_READ3)  /4095.0; // / VACgain  * VACcalib;
    VPFCmeas    = (float)(VPFC_READ1 + VPFC_READ2 + VPFC_READ3) /4095.0; // / VPFCgain * VPFCcalib;
    VBATmeas    = (float)(VBAT_READ1 + VBAT_READ2 + VBAT_READ3) /4095.0; // / VBATgain * VBATcalib;
    ILmeas      = (float)(IL_READ1   + IL_READ2   + IL_READ3)   /4095.0; // / ILgain   * ILcalib;
    IBATmeas    = (float)(IBAT_READ1 + IBAT_READ2 + IBAT_READ3) /4095.0; // / IBATgain * IBATcalib;
}

void setupVAR(void){
    BAT.KP = KP_BAT;
    BAT.KI = KI_BAT;
    BAT.DMIN = D_BAT_MIN;
    BAT.DMAX = D_BAT_MAX;

    PFC1.KP = KP_PFC_VL;
    PFC1.KI = KI_PFC_VL;
    PFC1.DMAX = LOOP1_MAX;
    PFC1.DMIN = LOOP1_MIN;

    PFC2.KP = KP_PFC_IL;
    PFC2.KI = KI_PFC_IL;
    PFC2.DMAX = D_PFC_MAX;
    PFC2.DMIN = D_PFC_MIN;
}

void CalibVAC(void);
void CalibPFC(void);
void CalibBAT(void);
//#include "Calib.h"

void setupLEDGPIO(void)
{
    GPIO_SetupPinOptions(LED1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(LED1, 0, 0);

    GPIO_SetupPinOptions(LED2, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(LED2, 0, 0);

    GPIO_SetupPinOptions(LED3, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(LED3, 0, 0);
}

void setup_gpio(void)
{
    EALLOW;

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // ePWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // ePWM1B
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)

    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // ePWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // ePWM2B
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3 (EPWM2B)
    
    EDIS;
}

void setup_ADC(void)
{
    EALLOW;                 // enable ADCA_1 -------------------------------------------------------------------------
    //write configurations
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;          //end of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;            //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;          //make sure INT1 flag is cleared
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;              // set ADCCLK divider to /1
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;              // set ADCCLK divider to /1
    AdccRegs.ADCCTL2.bit.PRESCALE = 6;              // set ADCCLK divider to /1

    AdcaRegs.ADCCTL2.bit.RESOLUTION =  0;           // 12-bit resolution
    AdcbRegs.ADCCTL2.bit.RESOLUTION =  0;           // 12-bit resolution
    AdccRegs.ADCCTL2.bit.RESOLUTION =  0;           // 12-bit resolution

    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0;            // Single-ended channel conversions (12-bit mode only)
    AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0;            // Single-ended channel conversions (12-bit mode only)
    AdccRegs.ADCCTL2.bit.SIGNALMODE = 0;            // Single-ended channel conversions (12-bit mode only)

    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;           // Set pulse positions to late
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;           // Set pulse positions to late
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;           // Set pulse positions to late

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;              //power up the ADC
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;              //power up the ADC
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;              //power up the ADC

    DELAY_US(1000);         //delay for 1ms to allow ADC time to power up



    // --------------------------------------------- ADCB -------------------------------------
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 14;              // SOC0 will convert pin A0 (A0 = 0; A1 = 1; A2 = 2..........; A15 = 15)
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;             // sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;            // trigger on ePWM1 SOCA/C
                                    //  0       is software only
                                    //  1 -> 3  is cputimer 0, tint0 -> 2, tint 2
                                    //  4       is GPIO, ADCEXTSOC
                                    //  5 -> 28 is ePWM1A, 1B -> ePWM12A, 12B
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 14;              // SOC0 will convert pin A0 (A0 = 0; A1 = 1; A2 = 2..........; A15 = 15)
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 14;             // sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5;

    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 14;              // SOC0 will convert pin A0 (A0 = 0; A1 = 1; A2 = 2..........; A15 = 15)
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = 14;             // sample window is 100 SYSCLK cycles
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 5;


    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5;

    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 3;
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = 14;
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 5;

    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 3;
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = 14;
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = 5;
    // --------------------------------------------- ADCB -------------------------------------
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 3;
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14;
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 3;
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 14;
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 5;

    AdcbRegs.ADCSOC4CTL.bit.CHSEL = 3;
    AdcbRegs.ADCSOC4CTL.bit.ACQPS = 14;
    AdcbRegs.ADCSOC4CTL.bit.TRIGSEL = 5;

    //---------------------------------------------- ADCC -------------------------------------
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 14;
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

    AdccRegs.ADCSOC2CTL.bit.CHSEL = 2;
    AdccRegs.ADCSOC2CTL.bit.ACQPS = 14;
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 5;

    AdccRegs.ADCSOC4CTL.bit.CHSEL = 2;
    AdccRegs.ADCSOC4CTL.bit.ACQPS = 14;
    AdccRegs.ADCSOC4CTL.bit.TRIGSEL = 5;


    AdccRegs.ADCSOC1CTL.bit.CHSEL = 3;
    AdccRegs.ADCSOC1CTL.bit.ACQPS = 14;
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 5;

    AdccRegs.ADCSOC3CTL.bit.CHSEL = 3;
    AdccRegs.ADCSOC3CTL.bit.ACQPS = 14;
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = 5;

    AdccRegs.ADCSOC5CTL.bit.CHSEL = 3;
    AdccRegs.ADCSOC5CTL.bit.ACQPS = 14;
    AdccRegs.ADCSOC5CTL.bit.TRIGSEL = 5;

    EDIS;
}

void InitEPwm1()
{
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
    // Setup TBCLK
    EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD;       // Set timer period 801 TBCLKs
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                  // Clear counter

    // Set Compare values
    EPwm1Regs.CMPA.bit.CMPA = D_BAT;    // Set compare A value
    EPwm1Regs.CMPB.bit.CMPB = D_BAT;    // Set Compare B value

    // Setup counter mode
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;   // Sync down-stream module
    EPwm1Regs.TBCTL.bit.PHSDIR = TB_UP;           // MODE up AFTER SYNC
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadowing
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set actions
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Set PWM1A on event A, up count
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;          // Clear PWM1A on event A, down count
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Set PWM1B on event B, up count
    EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;          // Clear PWM1B on event B, down count

    // Trigger ADC SOC
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = 3;
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;       // Generate pulse on 1st event

    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 3rd event
}

void InitEPwm2(void)
{
    CpuSysRegs.PCLKCR2.bit.EPWM2=1;
    // Setup TBCLK
    EPwm2Regs.TBPRD = EPWM1_TIMER_TBPRD;       // Set timer period 801 TBCLKs
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                  // Clear counter
    // Set Compare values
    EPwm2Regs.CMPA.bit.CMPA = D_PFC;    // Set compare A value
    EPwm2Regs.CMPB.bit.CMPB = D_PFC;    // Set Compare B value
    // Setup counter mode
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;   // Sync down-stream module
    EPwm2Regs.TBCTL.bit.PHSDIR=TB_UP;           // MODE up AFTER SYNC
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    // Setup shadowing
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    // Set actions
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Set PWM1A on event A, up count
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;          // Clear PWM1A on event A, down count
    EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Set PWM1B on event B, up count
    EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;          // Clear PWM1B on event B,down count


    // Trigger ADC SOC
     EPwm2Regs.ETSEL.bit.SOCAEN=1;
     EPwm2Regs.ETSEL.bit.SOCASEL=3;
     EPwm2Regs.ETPS.bit.SOCAPRD = 1;       // Generate pulse on 1st event
}

void setup_DAC(void){
    EALLOW;
    DacaRegs.DACCTL.bit.DACREFSEL = 1;
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;
    DacaRegs.DACVALS.all = 0;
    DELAY_US(10); // Delay for buffered DAC to power up
    EDIS;
}

void CalibBAT(void){
    if (VBATavg <= 2.356)
        VBAT = (VBATavg - VoffsetBAT + 0.0119) /VBATgain;
    else if (VBATavg <= 2.9)
        VBAT = (VBATavg - VoffsetBAT + 0.0059) /VBATgain;
    else if (VBATavg <= 2.438)
        VBAT = (VBATavg - VoffsetBAT)          /VBATgain;
    else if (VBATavg <= 2.608)
        VBAT = (VBATavg - VoffsetBAT - 0.0041) /VBATgain;
    else if (VBATavg <= 2.646)
        VBAT = (VBATavg - VoffsetBAT)          /VBATgain;
    else
        VBAT = (VBATavg - VoffsetBAT + 0.0039) /VBATgain;
}

void CalibVAC(void)
{
    if (VACmeas <= 1.179 + 0.00)
        VAC = (VACmeas - VoffsetVAC) * 157.2327044;
    else if (VACmeas <= 1.32 + 0.00)
        VAC = (VACmeas - VoffsetVAC) * 166.6666667;
    else if (VACmeas <= 1.58 + 0.00)
        VAC = (VACmeas - VoffsetVAC) * 178.5714286;
    else if (VACmeas <= 1.71 + 0.000)
        VAC = (VACmeas - VoffsetVAC) * 181.1594203;
    else if (VACmeas <= 1.81 + 0.00)
        VAC = (VACmeas - VoffsetVAC) * 189.8734177;
    else if (VACmeas <= 1.94 + 0.00)
        VAC = (VACmeas - VoffsetVAC) * 190.2173913;
    else if (VACmeas <= 2.06 + 0.00)
        VAC = (VACmeas - VoffsetVAC) * 192.3076923;
    else if (VACmeas <= 2.17 + 0.0)
        VAC = (VACmeas - VoffsetVAC) * 195.6521739;
    else if (VACmeas <= 2.26 + 0.0)
        VAC = (VACmeas - VoffsetVAC) * 201.6129032;
    else if (VACmeas <= 2.35 + 0.0)
        VAC = (VACmeas - VoffsetVAC) * 206.7669173;
    else if (VACmeas <= 2.40 + 0.0)
        VAC = (VACmeas - VoffsetVAC) * 212.7669173;
    else if (VACmeas <= 2.45 + 0.0)
        VAC = (VACmeas - VoffsetVAC) * 216.7832168;
    else if (VACmeas <= 2.46 + 0.0)
        VAC = (VACmeas - VoffsetVAC) * 218.75;
    else if (VACmeas <= 2.47 + 0.0)
        VAC = (VACmeas - VoffsetVAC) * 220.6896552;
    else if (VACmeas <= 2.48 + 0.0)
        VAC = (VACmeas - VoffsetVAC) * 222.6027397;
    else
        VAC = (VACmeas - VoffsetVAC) * 222.972973;
}

void CalibPFC(void)
{
    if (VPFCavg <= 2.293)
        VPFC = (VPFCavg - VoffsetPFC + 0.005) /VPFCgain;
    else if (VPFC <= 2.323)
        VPFC = (VPFCavg - VoffsetPFC)         /VPFCgain;
    else if (VPFCavg <= 2.381)
        VPFC = (VPFCavg - VoffsetPFC - 0.005) /VPFCgain;
    else if (VPFCavg <= 2.461)
        VPFC = (VPFCavg - VoffsetPFC - 0.01)  /VPFCgain;
    else if (VPFCavg <= 2.534)
        VPFC = (VPFCavg - VoffsetPFC - 0.007) /VPFCgain;
    else if (VPFCavg <= 2.568)
        VPFC = (VPFCavg - VoffsetPFC)         /VPFCgain;
    else
        VPFC = (VPFCavg - VoffsetPFC + 0.004) /VPFCgain;
}

