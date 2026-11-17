/* ============================================================
 *  COMUNICADOR MORSE IR — STM32F401RCT6
 *  Half-duplex: recebe string Morse via UART do PC,
 *  transmite por LED IR e decodifica sinais IR recebidos
 *  reconstruindo a string Morse para o PC.
 *
 *  HARDWARE ALVO: STM32F401RCT6
 *    Flash: 256 KB  |  SRAM: 64 KB  |  Clock max: 84 MHz
 *
 *  DEVICE DEFINE (Project Properties → Preprocessor):
 *    STM32F401xC        ← use ESTE (não STM32F401xE)
 *
 *  LINKER SCRIPT (gerado pelo CubeIDE ao selecionar RCT6):
 *    FLASH: ORIGIN = 0x08000000, LENGTH = 256K
 *    RAM  : ORIGIN = 0x20000000, LENGTH = 64K
 *
 *  PINAGEM:
 *    IR TX  : PB6  (saída push-pull, via transistor NPN + resistor)
 *    IR RX  : PA0  (entrada EXTI0, ativo alto — fototransistor/TSOP)
 *    UART TX: PA2  (USART2_TX — VCP do ST-Link na Nucleo)
 *    UART RX: PA3  (USART2_RX — VCP do ST-Link na Nucleo)
 *
 *  TEMPORIZACAO MORSE (T = 200 ms):
 *    Ponto  : 1T ON
 *    Traço  : 3T ON
 *    Gap intra-char : 1T OFF
 *    Gap inter-char : 3T OFF
 *    Gap inter-word : 7T OFF
 *    Gap fim de msg : 12T OFF  (dispara nova linha para o PC)
 * ============================================================ */

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/* ============================================================
 *  HANDLES GLOBAIS
 * ============================================================ */
UART_HandleTypeDef huart2;
TIM_HandleTypeDef  htim2;

/* ============================================================
 *  CONSTANTES DE CONFIGURACAO
 * ============================================================ */

/* UART */
#define UART_BAUDRATE          115200U   /* DEVE ser igual ao Python */

/* Pinos */
#define IR_TX_GPIO_PORT        GPIOB
#define IR_TX_PIN              GPIO_PIN_6   /* LED IR transmissor */

#define IR_RX_GPIO_PORT        GPIOA
#define IR_RX_PIN              GPIO_PIN_0   /* Receptor IR, ativo alto */

/* Unidade Morse base: T = 200 ms */
#define MORSE_UNIT_MS          200U
#define MORSE_UNIT_US          (MORSE_UNIT_MS * 1000UL)

/* Multiplos em unidades T */
#define DOT_UNITS              1U
#define DASH_UNITS             3U
#define INTRA_GAP_UNITS        1U
#define CHAR_GAP_UNITS         3U
#define WORD_GAP_UNITS         7U
#define END_GAP_UNITS          12U

/* Limiares de classificacao na recepcao (em microsegundos) */
#define MIN_VALID_PULSE_US     (MORSE_UNIT_US / 2U)           /* < 100ms: descarta ruido  */
#define DOT_DASH_THRESHOLD_US  (2U * MORSE_UNIT_US)           /* < 400ms: ponto; >= 400ms: traco */
#define MAX_VALID_PULSE_US     (4U * MORSE_UNIT_US)           /* > 800ms: descarta pulso invalido */

#define CHAR_GAP_MIN_US        (2U * MORSE_UNIT_US)           /* >= 400ms de silencio = fim de char */
#define WORD_GAP_MIN_US        (5U * MORSE_UNIT_US)           /* >= 1000ms de silencio = fim de palavra */
#define END_GAP_US             (END_GAP_UNITS * MORSE_UNIT_US)/* 2400ms = fim de mensagem */

/* ============================================================
 *  BUFFERS E FILAS
 * ============================================================ */

/* Linha UART recebida do PC */
#define UART_LINE_MAX          256U

static uint8_t  uart_rx_byte;
static volatile uint8_t  uart_line_ready = 0;
static volatile uint16_t uart_line_len   = 0;
static char     uart_line[UART_LINE_MAX];

/* Fila circular de saida UART (Morse reconstruido → PC) */
#define OUT_Q_SIZE             512U

static volatile uint16_t out_head = 0;
static volatile uint16_t out_tail = 0;
static char     out_queue[OUT_Q_SIZE];

/* ============================================================
 *  ESTADO DO RECEPTOR IR
 * ============================================================ */

static volatile bool ir_tx_busy       = false;  /* TRUE durante transmissao */
static volatile bool rx_started       = false;  /* Primeira borda recebida */
static volatile bool rx_line_high     = false;  /* Nivel atual do sinal IR */
static volatile bool rx_message_active= false;  /* Ha mensagem sendo recebida */
static volatile bool rx_newline_sent  = true;   /* Newline ja enviado ao PC */
static volatile uint32_t rx_last_edge_us = 0;   /* Timestamp da ultima borda */

#define RX_MORSE_CHAR_MAX      10U
static volatile char    rx_morse_char[RX_MORSE_CHAR_MAX];
static volatile uint8_t rx_morse_len  = 0;

/* ============================================================
 *  PROTOTIPOS
 * ============================================================ */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);

static uint32_t timer_us(void);
static void     delay_us_blocking(uint32_t us);
static void     morse_delay_units(uint32_t units);

static void     ir_led_on(void);
static void     ir_led_off(void);
static void     ir_pulse_units(uint32_t units);
static bool     is_morse_mark(char c);
static void     transmit_morse_line(char *line);

static void     uart_line_task(void);
static void     uart_output_task(void);

static void     rx_append_symbol(char symbol);
static void     rx_flush_current_char(void);
static void     rx_send_word_gap(void);
static void     rx_timeout_task(void);
static void     rx_reset_decoder(void);

static void     out_enqueue_char(char c);
static int      out_pop_char(char *c);

void Error_Handler(void);

/* ============================================================
 *  MAIN
 * ============================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();

    HAL_TIM_Base_Start(&htim2);   /* Inicia contador microsegundos */
    ir_led_off();                 /* Garante LED apagado na inicializacao */

    /* Habilita interrupcao de recepcao UART byte a byte */
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);

    /* Loop principal: cooperativo, sem RTOS */
    while (1)
    {
        uart_line_task();    /* Verifica se chegou linha Morse do PC para transmitir */
        rx_timeout_task();   /* Detecta fim de char/palavra/mensagem por timeout */
        uart_output_task();  /* Drena fila de saida UART para o PC */
    }
}

/* ============================================================
 *  TIMER MICROSEGUNDOS (TIM2 a 1 MHz)
 * ============================================================ */

/* Retorna contador livre de 32 bits em microsegundos */
static uint32_t timer_us(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
}

/* Espera ativa por 'us' microsegundos.
 * Funciona mesmo com IRQs desabilitadas (nao usa SysTick/HAL_Delay). */
static void delay_us_blocking(uint32_t us)
{
    uint32_t start = timer_us();
    while ((uint32_t)(timer_us() - start) < us)
    {
        /* espera ativa — aritmetica de modulo trata overflow do contador */
    }
}

static void morse_delay_units(uint32_t units)
{
    delay_us_blocking(units * MORSE_UNIT_US);
}

/* ============================================================
 *  TRANSMISSAO IR
 * ============================================================ */

static void ir_led_on(void)
{
    HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_PIN, GPIO_PIN_SET);
}

static void ir_led_off(void)
{
    HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_PIN, GPIO_PIN_RESET);
}

/* Emite um pulso IR de duração 'units * T' */
static void ir_pulse_units(uint32_t units)
{
    ir_led_on();
    morse_delay_units(units);
    ir_led_off();
}

static bool is_morse_mark(char c)
{
    return (c == '.' || c == '-');
}

/*
 * Transmite string Morse por IR.
 *
 * Formato aceito (gerado pelo Python):
 *   '.'  ponto
 *   '-'  traco
 *   ' '  espaco simples = gap inter-caractere (3T)
 *   '/'  separador de palavras (7T) — compativel com o Python
 *
 * Tambem interpreta 3+ espacos consecutivos como separador de palavras.
 *
 * Apos o ultimo simbolo emite END_GAP_UNITS para o receptor
 * detectar fim de mensagem e enviar '\n' ao PC.
 */
static void transmit_morse_line(char *line)
{
    size_t n = strlen(line);
    size_t i = 0;
    bool emitted         = false;
    bool final_delay_done = false;

    while (i < n)
    {
        char c = line[i];

        /* Ignora caracteres que nao sao marcas Morse */
        if (!is_morse_mark(c))
        {
            i++;
            continue;
        }

        /* Emite ponto ou traco */
        if (c == '.')
            ir_pulse_units(DOT_UNITS);
        else
            ir_pulse_units(DASH_UNITS);

        emitted = true;

        size_t j = i + 1;

        /* Fim da string: emite gap longo para sinalizar FIM */
        if (j >= n)
        {
            morse_delay_units(END_GAP_UNITS);
            final_delay_done = true;
            break;
        }

        /* Proximo caractere e outra marca: gap intra-char (1T) */
        if (is_morse_mark(line[j]))
        {
            morse_delay_units(INTRA_GAP_UNITS);
            i = j;
            continue;
        }

        /* Conta espacos e '/' ate a proxima marca ou fim */
        uint32_t spaces      = 0;
        bool     slash_found = false;

        while (j < n)
        {
            if (line[j] == ' ' || line[j] == '\t')
            {
                spaces++;
                j++;
            }
            else if (line[j] == '/')
            {
                slash_found = true;
                j++;
            }
            else if (line[j] == '\r' || line[j] == '\n')
            {
                j = n;
                break;
            }
            else
            {
                break;
            }
        }

        /* Chegou ao fim da string durante os espacos */
        if (j >= n)
        {
            morse_delay_units(END_GAP_UNITS);
            final_delay_done = true;
            break;
        }

        /* Decide qual gap emitir */
        if (slash_found || spaces >= 3U)
            morse_delay_units(WORD_GAP_UNITS);   /* 7T — separador de palavras */
        else if (spaces >= 1U)
            morse_delay_units(CHAR_GAP_UNITS);   /* 3T — separador de caracteres */
        else
            morse_delay_units(INTRA_GAP_UNITS);  /* 1T — intra-char (nao deveria ocorrer aqui) */

        i = j;
    }

    /* Garante que o gap final sempre e emitido */
    if (emitted && !final_delay_done)
        morse_delay_units(END_GAP_UNITS);

    ir_led_off();
}

/* ============================================================
 *  RECEPCAO IR VIA EXTI (ISR)
 *
 *  PA0 → EXTI0, ambas as bordas (RISING + FALLING).
 *  Logica ativa ALTA: HIGH = IR detectado, LOW = silencio.
 * ============================================================ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != IR_RX_PIN)
        return;

    /* Ignora reflexos locais enquanto estamos transmitindo */
    if (ir_tx_busy)
        return;

    uint32_t now   = timer_us();
    GPIO_PinState level = HAL_GPIO_ReadPin(IR_RX_GPIO_PORT, IR_RX_PIN);

    if (level == GPIO_PIN_SET)
    {
        /*
         * BORDA DE SUBIDA: comecou um pulso IR.
         * O periodo LOW anterior pode indicar gap inter-char ou inter-word.
         */
        if (rx_started && !rx_line_high)
        {
            uint32_t low_dur = now - rx_last_edge_us;

            if (low_dur >= WORD_GAP_MIN_US)
            {
                /* Silencio longo: fecha char atual E envia separador de palavra */
                rx_flush_current_char();
                rx_send_word_gap();
            }
            else if (low_dur >= CHAR_GAP_MIN_US)
            {
                /* Silencio medio: fecha apenas o char atual */
                rx_flush_current_char();
            }
            /* Silencio curto (<= CHAR_GAP_MIN_US): gap intra-char — nao faz nada */
        }

        rx_started        = true;
        rx_line_high      = true;
        rx_last_edge_us   = now;
        rx_message_active = true;
        rx_newline_sent   = false;
    }
    else
    {
        /*
         * BORDA DE DESCIDA: terminou um pulso IR HIGH.
         * Mede duracao HIGH para classificar ponto ou traco.
         */
        if (rx_started && rx_line_high)
        {
            uint32_t high_dur = now - rx_last_edge_us;

            if (high_dur >= MIN_VALID_PULSE_US && high_dur <= MAX_VALID_PULSE_US)
            {
                if (high_dur < DOT_DASH_THRESHOLD_US)
                    rx_append_symbol('.');
                else
                    rx_append_symbol('-');
            }
            /* Fora dos limites: descarta como ruido */
        }

        rx_line_high      = false;
        rx_last_edge_us   = now;
        rx_message_active = true;
        rx_newline_sent   = false;
    }
}

/*
 * Handler da linha EXTI0.
 * REMOVA esta funcao se o CubeIDE ja gerou EXTI0_IRQHandler
 * em stm32f4xx_it.c — nao pode haver duas definicoes.
 */
#ifndef EXTI0_IRQHANDLER_DEFINED
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(IR_RX_PIN);
}
#endif

/* ============================================================
 *  DECODIFICACAO MORSE (chamada da ISR e do loop principal)
 * ============================================================ */

/* Acrescenta '.' ou '-' ao buffer do caractere atual */
static void rx_append_symbol(char symbol)
{
    if (rx_morse_len < RX_MORSE_CHAR_MAX - 1U)
    {
        rx_morse_char[rx_morse_len++] = symbol;
        rx_morse_char[rx_morse_len]   = '\0';
    }
    rx_message_active = true;
    rx_newline_sent   = false;
}

/*
 * Envia o caractere Morse acumulado para a fila UART de saida
 * e limpa o buffer.
 * Ex.: buffer = "..." → envia "... " para o PC
 */
static void rx_flush_current_char(void)
{
    if (rx_morse_len == 0)
        return;

    for (uint8_t i = 0; i < rx_morse_len; i++)
        out_enqueue_char(rx_morse_char[i]);

    out_enqueue_char(' ');  /* Espaco inter-char no stream Morse */

    rx_morse_len        = 0;
    rx_morse_char[0]    = '\0';
    rx_message_active   = true;
    rx_newline_sent     = false;
}

/*
 * Envia separador de palavra '/ ' para o PC.
 * Compativel com o dicionario do Python: '/' = espaco entre palavras.
 */
static void rx_send_word_gap(void)
{
    out_enqueue_char('/');
    out_enqueue_char(' ');
    rx_message_active = true;
    rx_newline_sent   = false;
}

/*
 * Chamada no loop principal para detectar fim de char e fim de mensagem
 * por timeout (sem atividade IR por tempo suficiente).
 *
 * Nao usa delay — apenas verifica o tempo decorrido desde a ultima borda.
 */
static void rx_timeout_task(void)
{
    /* Nao processa durante transmissao ou se nunca houve sinal */
    if (ir_tx_busy || !rx_started || rx_line_high)
        return;

    uint32_t now      = timer_us();
    uint32_t idle_us  = now - rx_last_edge_us;

    /* Desabilita IRQs apenas pelo tempo minimo necessario */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* Fim de caractere: silencio >= CHAR_GAP e ainda ha simbolos no buffer */
    if (rx_morse_len > 0 && idle_us >= CHAR_GAP_MIN_US)
        rx_flush_current_char();

    /* Fim de mensagem: silencio >= END_GAP — envia '\n' e reseta decoder */
    if (rx_message_active && !rx_newline_sent && idle_us >= END_GAP_US)
    {
        out_enqueue_char('\n');
        rx_newline_sent   = true;
        rx_message_active = false;
        rx_started        = false;
        rx_line_high      = false;
        rx_morse_len      = 0;
        rx_morse_char[0]  = '\0';
    }

    if (primask == 0U)
        __enable_irq();
}

/* Reseta completamente o estado do decoder (chamado antes de nova transmissao) */
static void rx_reset_decoder(void)
{
    rx_started        = false;
    rx_line_high      = false;
    rx_message_active = false;
    rx_newline_sent   = true;
    rx_last_edge_us   = timer_us();
    rx_morse_len      = 0;
    rx_morse_char[0]  = '\0';
}

/* ============================================================
 *  UART — ENTRADA (PC → STM32)
 *  Acumula bytes ate receber '\n' ou '\r'.
 * ============================================================ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
        return;

    char c = (char)uart_rx_byte;

    if (!uart_line_ready)
    {
        if (c == '\n' || c == '\r')
        {
            if (uart_line_len > 0)
            {
                uart_line[uart_line_len] = '\0';
                uart_line_ready = 1;
            }
        }
        else
        {
            if (uart_line_len < UART_LINE_MAX - 1U)
                uart_line[uart_line_len++] = c;
            else
                uart_line_len = 0;  /* Overflow: descarta linha */
        }
    }

    /* Reabilita interrupcao para proximo byte */
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
}

/*
 * Verifica se ha linha Morse pronta e a transmite por IR.
 * Half-duplex: desabilita EXTI durante transmissao para evitar eco.
 */
static void uart_line_task(void)
{
    if (!uart_line_ready)
        return;

    /* Copia atomicamente para buffer local e libera o buffer UART */
    char local_line[UART_LINE_MAX];

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    strncpy(local_line, (char *)uart_line, UART_LINE_MAX);
    local_line[UART_LINE_MAX - 1U] = '\0';
    uart_line_len   = 0;
    uart_line_ready = 0;

    if (primask == 0U)
        __enable_irq();

    if (local_line[0] == '\0')
        return;

    /* Sinaliza ocupado e desabilita recepcao IR */
    ir_tx_busy = true;
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    ir_led_off();

    transmit_morse_line(local_line);

    /* Limpa estado do receptor e reabilita EXTI */
    rx_reset_decoder();
    __HAL_GPIO_EXTI_CLEAR_IT(IR_RX_PIN);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    ir_tx_busy = false;
}

/* ============================================================
 *  UART — SAIDA (STM32 → PC)
 *  Fila circular protegida contra acesso concorrente ISR/main.
 * ============================================================ */

/* Retorna 1 se estamos dentro de um contexto de interrupcao */
static uint8_t in_interrupt_context(void)
{
    return (__get_IPSR() != 0U);
}

/* Enfileira caractere — seguro para chamar de ISR ou do loop principal */
static void out_enqueue_char(char c)
{
    uint32_t primask = 0U;
    uint8_t  isr     = in_interrupt_context();

    if (!isr)
    {
        primask = __get_PRIMASK();
        __disable_irq();
    }

    uint16_t next = (uint16_t)((out_head + 1U) % OUT_Q_SIZE);

    if (next != out_tail)   /* Descarta silenciosamente se fila cheia */
    {
        out_queue[out_head] = c;
        out_head = next;
    }

    if (!isr && primask == 0U)
        __enable_irq();
}

/* Remove e retorna 1 caractere da fila; retorna 0 se vazia */
static int out_pop_char(char *c)
{
    int has_data = 0;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (out_tail != out_head)
    {
        *c       = out_queue[out_tail];
        out_tail = (uint16_t)((out_tail + 1U) % OUT_Q_SIZE);
        has_data = 1;
    }

    if (primask == 0U)
        __enable_irq();

    return has_data;
}

/* Drena a fila de saida, enviando cada byte via UART para o PC */
static void uart_output_task(void)
{
    char c;
    while (out_pop_char(&c))
        HAL_UART_Transmit(&huart2, (uint8_t *)&c, 1, 20);
}

/* ============================================================
 *  INICIALIZACOES HAL
 * ============================================================ */

/*
 * TIM2: timer livre de 32 bits rodando a 1 MHz (1 contagem = 1 µs).
 * APB1 = SYSCLK/2 = 42 MHz; TIM2 recebe 2x APB1 = 84 MHz quando PPRE1 != 1.
 * Prescaler = 84 - 1 → 84 MHz / 84 = 1 MHz.
 */
static void MX_TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 84U - 1U;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 0xFFFFFFFFU;  /* 32 bits → overflow em ~4295 s */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
        Error_Handler();
}

/*
 * USART2: 115200 8N1, sem controle de fluxo.
 * PA2 = TX, PA3 = RX (VCP do debugger na Nucleo / conector UART externo no bare chip).
 */
static void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = UART_BAUDRATE;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
        Error_Handler();

    /* Prioridade menor que EXTI0 para nao atrasar medicao de tempo */
    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/*
 * Handler USART2.
 * REMOVA se o CubeIDE ja gerou USART2_IRQHandler em stm32f4xx_it.c.
 */
#ifndef USART2_IRQHANDLER_DEFINED
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}
#endif

/*
 * GPIO:
 *   PB6 — IR TX (saída push-pull)
 *   PA0 — IR RX (entrada com interrupcao ambas as bordas, pulldown)
 *   PA2 — USART2_TX (funcao alternativa AF7)
 *   PA3 — USART2_RX (funcao alternativa AF7)
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB6 — LED IR transmissor */
    HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = IR_TX_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IR_TX_GPIO_PORT, &GPIO_InitStruct);

    /* PA0 — Receptor IR com interrupcao nas duas bordas */
    GPIO_InitStruct.Pin  = IR_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;  /* Garante nivel LOW na ausencia de sinal */
    HAL_GPIO_Init(IR_RX_GPIO_PORT, &GPIO_InitStruct);

    /* PA2/PA3 — USART2 */
    GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* EXTI0 — prioridade ALTA (5) para medicoes de tempo precisas */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/*
 * SystemClock_Config:
 *   HSI 16 MHz → PLL → SYSCLK = 84 MHz
 *   AHB = 84 MHz | APB1 = 42 MHz | APB2 = 84 MHz
 *
 *   Valido para STM32F401xC (256KB) e STM32F401xE (512KB).
 *   Identico entre RCT6 e RE: mesma frequencia maxima de 84 MHz.
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;   /* 16 MHz / 16 = 1 MHz para VCO */
    RCC_OscInitStruct.PLL.PLLN            = 336;  /* 1 MHz * 336 = 336 MHz VCO */
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4; /* 336 / 4 = 84 MHz SYSCLK */
    RCC_OscInitStruct.PLL.PLLQ            = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   /* 42 MHz — TIM2 recebe 84 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;   /* 84 MHz */

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
}

/*
 * Error_Handler: pisca LED IR rapidamente para indicar falha.
 * Em producao, substitua por log UART ou watchdog reset.
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(IR_TX_GPIO_PORT, IR_TX_PIN);
        HAL_Delay(100);
    }
}
