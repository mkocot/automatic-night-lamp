#include <ch32v00x.h>
#include <debug.h>

#define DEBUG_DATA0_ADDRESS  ((volatile uint32_t*)0xE00000F4)

void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

static void adc_init(void);
static void gpio_init(void);
static void loop(void);
static void setup(void);
static void set_led(BitAction action);

/*
 * WARNING:
 * if printf is used WITHOUT active SDI it will just
 * stall on printf command
*/
// #define DEBUG

#if SDI_PRINT
#define PRINTF printf
#else
#define PRINTF(...) do {} while(0)
#endif

static int counter = 0;

#define DUSK_THRESHOLD 512
#define LOW_BATTERY_THRESHOLD 900

int main()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    SystemInit();

    setup();
    while(1)
    {
        loop();
    }

    return 1;
}

void gpio_init()
{
    // PD4 -> EXTI when PIR is active
    // PD0 -> LED MOSFET-N
    // PD2 -> A3 photoresistor
    // PD5

    // Enable PDx and AFIO (required for EXTI)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);

    // OUTPUT for PD0
    GPIO_InitTypeDef conf = {
        .GPIO_Mode = GPIO_Mode_Out_PP, /* Push-Pull */
        .GPIO_Pin = GPIO_Pin_0, /* PD0 */
        .GPIO_Speed = GPIO_Speed_2MHz,
    };

    GPIO_Init(GPIOD, &conf);

    /* PD5, A5*/
    conf.GPIO_Mode = GPIO_Mode_AIN;
    conf.GPIO_Pin = GPIO_Pin_5;
    conf.GPIO_Speed = 0;
    GPIO_Init(GPIOD, &conf);

    /* PD4 */
    conf.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    conf.GPIO_Pin = GPIO_Pin_4;
    conf.GPIO_Speed = 0;
    GPIO_Init(GPIOD, &conf);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource4);

    EXTI_InitTypeDef EXTI_InitStructure = {0};
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Rising -> PIR activated, Falling -> Deactivated */
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable interrupts for GPIO */
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void adc_init()
{
    RCC_ADCCLKConfig(RCC_PCLK2_Div2);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_InitTypeDef conf = {0};
    conf.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    conf.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &conf);

    ADC_Cmd(ADC1, ENABLE);

    // ADC1->CTLR2 |= ADC_ADON | ADC_EXTSEL;

    // BUSY WAIT for ON
    while(!(ADC1->CTLR2 & ADC_ADON));

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));

    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

void adc_off()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);
    ADC_DeInit(ADC1);
    ADC_Cmd(ADC1, DISABLE);
}

uint16_t adc_get(uint8_t channel)
{
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_241Cycles);

    #define ADC_OVERSAMPLE 2

    // ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    // while(ADC_GetSoftwareStartConvStatus(ADC1));
    // // ADC_SoftwareStartConvCmd(ADC1, DISABLE);

    // return ADC_GetConversionValue(ADC1);
    #if 0
    ADC1->CTLR2 |= ADC_SWSTART;

    while(!(ADC1->STATR & ADC_EOC));

    return ADC1->RDATAR;
    #else 
    
    uint32_t average = 0;
    for (int i = 0; i < (1 << ADC_OVERSAMPLE); i++) {
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);

        while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

        average += ADC_GetConversionValue(ADC1);
    }

    return average >> ADC_OVERSAMPLE;

    #endif
}

static uint16_t ADC_GetRef()
{
    return adc_get(ADC_Channel_Vrefint);
}

static uint16_t ADC_GetPhotoresistor()
{
    // Configure A5 as rank 1
    // Lil confusing, but when there is more than one rank they are read
    // in sequence 1, 2, 3, 1, 2, 3
    // ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_241Cycles);
    return adc_get(ADC_Channel_5);
}

static void go_to_sleep()
{
    #define STANDBY
    #define SCTLR_DEEPSLEEP (1 << 2);  


    /* Deconfigure peripherals */
    adc_off();

    #ifdef STANDBY
        PWR_EnterSTANDBYMode(PWR_STANDBYEntry_WFI);
        // PWR->CTLR |= PWR_CTLR_PDDS; // Standby
        // PFIC->SCTLR |= (1 << 2); // enable deepsleep
    #else
        PWR->CTLR &= ~PWR_CTLR_PDDS; // Disable standby (enable sleep)
        PFIC->SCTLR &= ~SCTLR_DEEPSLEEP; // Disable deepsleep
    #endif

    /* resume ADC config */
    adc_init();
}

void setup()
{
    Delay_Init();

    /* enable SDI printf */
    *(DEBUG_DATA0_ADDRESS) = 0;

    gpio_init();

    for (int i = 0; i < 2; i++)
    {
        PRINTF("Prepare to sleep in %d second(s)!\n", (5 - i));
        Delay_Ms(1000);
    }

    PRINTF("SLEEP!\n");
    Delay_Ms(500);

    go_to_sleep();
}

static void set_led(BitAction action)
{
    PRINTF("LED: %d\n", action);
    GPIO_WriteBit(GPIOD, GPIO_Pin_0, action);
}

static BitAction get_pir_status()
{
    return GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4);
}

void loop()
{
    BitAction pirStatus = get_pir_status();
    uint16_t photoresistor = ADC_GetPhotoresistor();
    uint16_t battery = ADC_GetRef();

    PRINTF("PIR: %d\n", GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4));
    PRINTF("ADC: %d\n", photoresistor);
    PRINTF("BAT: %d\n", battery);

    if (photoresistor > DUSK_THRESHOLD && pirStatus == Bit_SET)
    {
        set_led(Bit_SET);
        if (battery > LOW_BATTERY_THRESHOLD)
        {
            uint8_t status = Bit_RESET;
            /* NOTE: Always use odd number for loop */
            for (uint8_t i = 0; i < 5; ++i)
            {
                Delay_Ms(200);
                set_led(status);
                status = (status + 1) & 1;
            }

            // 0 off -> on
            // 1 on -> off
            // 2 off -> on
            // 3 on -> off
            // 4 off -> on
        }

        // check for state again and power down led if needed
        pirStatus = get_pir_status();
        photoresistor = ADC_GetPhotoresistor();
        if (!(photoresistor > DUSK_THRESHOLD && pirStatus == Bit_SET))
        {
            set_led(Bit_RESET);
        }
    }
    else
    {
        set_led(Bit_RESET);
    }


    go_to_sleep();

    // if (counter--)
    // {
    //     /* reading ADC before init == stall */
    //     printf("PIR: %d\n", GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4));
    //     printf("ADC: %d\n", photoresistor);
    //     Delay_Ms(200);
    // }
    // else
    // {
    //     counter = 10;
    // }

}

void EXTI7_0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line4)!=RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line4);     /* Clear Flag */
    }
}