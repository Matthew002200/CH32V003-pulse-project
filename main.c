#include "debug.h"

/*
 * CH32V003 @ 48 МГц
 * Один тик таймера = 1 / 48e6 сек ≈ 20.833 нс
 */
#define TICK_NS 20.833333f

/* Флаг антидребезга/антиповтора: чтобы запуск был только один раз на нажатие */
static volatile uint8_t button_was_pressed = 0;

/* Контрольные точки во времени (наносекунды) */
static volatile float checkpoint_start_ns  = 100000.0f;  // момент включения PD4
static volatile float checkpoint_pulse1_ns = 200000.0f;  // момент выключения PD4
static volatile float checkpoint_delay_ns  = 210000.0f;  // момент включения PC0
static volatile float checkpoint_pulse2_ns = 211000.0f;  // момент выключения PC0 (и остановка)

static void USARTx_CFG(void);
static void GPIO_INIT(void);
static void tim2_init_compare_pulses(void);
static uint8_t is_button_pressed(void);

/* Перевод наносекунд в тики TIM2 (с округлением) */
static uint16_t ns_to_ticks(float ns)
{
    float ticks_f = ns / TICK_NS;

    if (ticks_f < 0.0f) ticks_f = 0.0f;

    /* TIM2 на CH32V003 — 16-битный счётчик */
    if (ticks_f > 65535.0f) ticks_f = 65535.0f;

    return (uint16_t)(ticks_f + 0.5f);
}

/*
 * Обработчик прерывания TIM2.
 * Здесь мы ловим события сравнения CCR1..CCR4 и переключаем пины.
 */
__attribute__((interrupt("WCH-Interrupt-fast")))
void TIM2_IRQHandler(void)
{
    /* CCR1: включить PD4 */
    if ((TIM2->DMAINTENR & TIM_CC1IE) && (TIM2->INTFR & TIM_CC1IF))
    {
        TIM2->INTFR &= ~TIM_CC1IF;
        GPIO_SetBits(GPIOD, GPIO_Pin_4);
    }

    /* CCR2: выключить PD4 */
    if ((TIM2->DMAINTENR & TIM_CC2IE) && (TIM2->INTFR & TIM_CC2IF))
    {
        TIM2->INTFR &= ~TIM_CC2IF;
        GPIO_ResetBits(GPIOD, GPIO_Pin_4);
    }

    /* CCR3: включить PC0 */
    if ((TIM2->DMAINTENR & TIM_CC3IE) && (TIM2->INTFR & TIM_CC3IF))
    {
        TIM2->INTFR &= ~TIM_CC3IF;
        GPIO_SetBits(GPIOC, GPIO_Pin_0);
    }

    /* CCR4: выключить PC0 и остановить таймер */
    if ((TIM2->DMAINTENR & TIM_CC4IE) && (TIM2->INTFR & TIM_CC4IF))
    {
        TIM2->INTFR &= ~TIM_CC4IF;
        GPIO_ResetBits(GPIOC, GPIO_Pin_0);

        /* Остановить таймер */
        TIM2->CTLR1 &= ~TIM_CEN;

        /* Выключить compare-прерывания, чтобы дальше не дёргалось */
        TIM2->DMAINTENR &= ~(TIM_CC1IE | TIM_CC2IE | TIM_CC3IE | TIM_CC4IE);
    }
}

int main(void)
{
    /* Настройка приоритетов NVIC */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    SystemCoreClockUpdate();
    Delay_Init();

    /* UART для отладки и кнопка */
    USARTx_CFG();
    GPIO_INIT();

    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

    while (1)
    {
        /* Запуск по нажатию (один раз на нажатие) */
        if (is_button_pressed() && !button_was_pressed)
        {
            button_was_pressed = 1;

            /* Настроить TIM2 на события CCR1..CCR4 */
            tim2_init_compare_pulses();

            /*
             * ВАЖНО: стартуем всегда с CNT=0,
             * иначе можно пропустить ранние события CCR1 (если CNT уже > CCR1)
             */
            TIM2->CNT = 0;

            /* Запуск */
            TIM2->CTLR1 |= TIM_CEN;
        }

        /* Сброс флага, когда кнопку отпустили */
        if (!is_button_pressed() && button_was_pressed)
        {
            button_was_pressed = 0;
        }
    }
}

/* Кнопка на PC1 с подтяжкой вверх: нажатие = 0 */
static uint8_t is_button_pressed(void)
{
    return (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == 0);
}

/*
 * Настраиваем TIM2 так, чтобы получить:
 *  - CCR1: PD4 = 1
 *  - CCR2: PD4 = 0
 *  - CCR3: PC0 = 1
 *  - CCR4: PC0 = 0 и стоп
 */
static void tim2_init_compare_pulses(void)
{
    /* Включаем тактирование TIM2 и портов */
    RCC->APB1PCENR |= RCC_TIM2EN;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC, ENABLE);

    /* PD4 — выход */
    GPIO_InitTypeDef G = {0};
    G.GPIO_Pin   = GPIO_Pin_4;
    G.GPIO_Mode  = GPIO_Mode_Out_PP;
    G.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &G);
    GPIO_ResetBits(GPIOD, GPIO_Pin_4);

    /* PC0 — выход */
    G.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOC, &G);
    GPIO_ResetBits(GPIOC, GPIO_Pin_0);

    /* Полный сброс TIM2 */
    TIM2->CTLR1     = 0;
    TIM2->CTLR2     = 0;
    TIM2->SMCFGR    = 0;
    TIM2->DMAINTENR = 0;
    TIM2->INTFR     = 0;
    TIM2->CCER      = 0;
    TIM2->CHCTLR1   = 0;
    TIM2->CHCTLR2   = 0;

    /*
     * PSC=0 -> тик таймера = 1/48МГц
     * ARR = конечный момент (checkpoint_pulse2_ns)
     */
    TIM2->PSC   = 0;
    TIM2->ATRLR = ns_to_ticks(checkpoint_pulse2_ns);

    /* Моменты сравнения */
    TIM2->CH1CVR = ns_to_ticks(checkpoint_start_ns);
    TIM2->CH2CVR = ns_to_ticks(checkpoint_pulse1_ns);
    TIM2->CH3CVR = ns_to_ticks(checkpoint_delay_ns);
    TIM2->CH4CVR = ns_to_ticks(checkpoint_pulse2_ns);

    /* Разрешаем прерывания compare */
    TIM2->DMAINTENR |= (TIM_CC1IE | TIM_CC2IE | TIM_CC3IE | TIM_CC4IE);

    /* OPM: одноразовый режим (таймер сам остановится на событии update) */
    TIM2->CTLR1 |= TIM_OPM;

    /*
     * UG: принудительное обновление (чтобы PSC/ARR/CCR применились корректно)
     * После UG обязательно чистим флаги, иначе может прилететь ложное IRQ.
     */
    TIM2->SWEVGR |= TIM_UG;
    TIM2->INTFR = 0;

    /* Включаем TIM2 IRQ в NVIC (это у тебя было сломано/не оформлено) */
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);
}

/* UART1 TX на PD5 (для printf) */
static void USARTx_CFG(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitTypeDef G = {0};
    G.GPIO_Pin   = GPIO_Pin_5;
    G.GPIO_Mode  = GPIO_Mode_AF_PP;
    G.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOD, &G);

    USART_InitTypeDef U = {0};
    U.USART_BaudRate            = 115200;
    U.USART_WordLength          = USART_WordLength_8b;
    U.USART_StopBits            = USART_StopBits_1;
    U.USART_Parity              = USART_Parity_No;
    U.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    U.USART_Mode                = USART_Mode_Tx;

    USART_Init(USART1, &U);
    USART_Cmd(USART1, ENABLE);
}

/* Кнопка PC1 как вход с подтяжкой вверх */
static void GPIO_INIT(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef G = {0};
    G.GPIO_Pin  = GPIO_Pin_1;
    G.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &G);
}

