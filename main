#include "debug.h"
#define TICK_NS 20.833f
uint8_t button_was_pressed = 0;
void USARTx_CFG(void);
void GPIO_INIT(void);
void tim2_init_compare_pulses(void);
uint16_t ns_to_ticks(float ns)
{
    return (uint16_t)(((float)ns/TICK_NS)+0.5f);
}
// Параметры импульсов (в микросекундах)
volatile float checkpoint_start_ns = 100000.0f;
volatile float checkpoint_pulse1_ns = 200000.0f;// завершение импульса PD4
volatile float checkpoint_delay_ns = 210000.0f; // завершение задержки перед PC0
volatile float checkpoint_pulse2_ns = 211000.0f;// завершение импульса PC0
uint8_t is_button_pressed(void);

/* TIM2 IRQ: обрабатываем CCR1–CCR4 */
attribute((interrupt("WCH-Interrupt-fast")))
void TIM2_IRQHandler(void)
{


    if ((TIM2->DMAINTENR & TIM_CC1IE) && (TIM2->INTFR & TIM_CC1IF))
    {
        TIM2->INTFR &= ~TIM_CC1IF;
        GPIO_SetBits(GPIOD, GPIO_Pin_4);   // PD4 = 1
    }

    if ((TIM2->DMAINTENR & TIM_CC2IE) && (TIM2->INTFR & TIM_CC2IF))
    {
        TIM2->INTFR &= ~TIM_CC2IF;
        GPIO_ResetBits(GPIOD, GPIO_Pin_4); // PD4 = 0
    }

    if ((TIM2->DMAINTENR & TIM_CC3IE) && (TIM2->INTFR & TIM_CC3IF))
    {
        TIM2->INTFR &= ~TIM_CC3IF;
        GPIO_SetBits(GPIOC, GPIO_Pin_0);   // PC0 = 1
    }

    if ((TIM2->DMAINTENR & TIM_CC4IE) && (TIM2->INTFR & TIM_CC4IF))
    {
        TIM2->INTFR &= ~TIM_CC4IF;
        GPIO_ResetBits(GPIOC, GPIO_Pin_0); // PC0 = 0

        // Остановить TIM2 и выключить все прерывания
        TIM2->CTLR1 &= ~TIM_CEN;
        TIM2->DMAINTENR &= ~(TIM_CC1IE|TIM_CC2IE|TIM_CC3IE|TIM_CC4IE);
    }
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();

    Delay_Init();
    USARTx_CFG();
    GPIO_INIT();

    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

    while (1)
    {
        if (is_button_pressed() && !button_was_pressed)
        {
            button_was_pressed = 1;
            tim2_init_compare_pulses();
            TIM2->CTLR1 |= TIM_CEN;  // запускаем отсчёт — внутри IRQ пойдёт всё по расписанию
        }

        if (!is_button_pressed() && button_was_pressed)
            button_was_pressed = 0;
    }
}

uint8_t is_button_pressed(void)
{
    return (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == 0);
}

/* Настроить TIM2 на 4 compare-эвента для двух импульсов + паузы */
void tim2_init_compare_pulses(void)
{
    // 1) тактирование TIM2 и GPIO
    RCC->APB1PCENR |= RCC_TIM2EN;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC, ENABLE);

    // 2) PD4 и PC0 — как выходы, сбрасываем оба светодиода в 0
    GPIO_InitTypeDef G = {0};
    G.GPIO_Pin   = GPIO_Pin_4;
    G.GPIO_Mode  = GPIO_Mode_Out_PP;
    G.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &G);
    GPIO_ResetBits(GPIOD, GPIO_Pin_4);

    G.GPIO_Pin   = GPIO_Pin_0;
    GPIO_Init(GPIOC, &G);
    GPIO_ResetBits(GPIOC, GPIO_Pin_0);

    // 3) Полный сброс TIM2 перед настройкой
    TIM2->CTLR1   = 0;      // остановить и сбросить режим
    TIM2->CCER    = 0;      // отключить все каналы
    TIM2->CHCTLR1 = 0;
    TIM2->CHCTLR2 = 0;
    TIM2->DMAINTENR = 0;    // снять все CCxIE
    TIM2->INTFR   = 0;      // сброс всех флагов

    // 4) PSC и ARR: 1 мкс tick, общий период 111 мкс
    TIM2->PSC    = 0;
    TIM2->ATRLR  = ns_to_ticks(checkpoint_pulse2_ns);

    // 5) CCR1 = 0 мкс → включить PD4
    TIM2->CH1CVR = ns_to_ticks(checkpoint_start_ns);
    TIM2->DMAINTENR |= TIM_CC1IE;

    // 6) CCR2 = 100 мкс → выключить PD4
    TIM2->CH2CVR = ns_to_ticks(checkpoint_pulse1_ns);
    TIM2->DMAINTENR |= TIM_CC2IE;

    // 7) CCR3 = 110 мкс → включить PC0
    TIM2->CH3CVR = ns_to_ticks(checkpoint_delay_ns);
    TIM2->DMAINTENR |= TIM_CC3IE;

    // 8) CCR4 = 111 мкс → выключить PC0 и стоп
    TIM2->CH4CVR = ns_to_ticks(checkpoint_pulse2_ns);
    TIM2->DMAINTENR |= TIM_CC4IE;

    // 9) однократный режим (OPM)
    TIM2->CTLR1 = TIM_OPM;

    // 10) обновление всех регистров вручную
    TIM2->SWEVGR |= TIM_UG;
    TIM2->INTFR   = 0; // обязательно очистить флаги
