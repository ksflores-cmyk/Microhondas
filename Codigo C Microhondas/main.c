#include <stdint.h>
#include "stm32l053xx.h"

/* =================== DEFINICIONES GENERALES =================== */

/* ---- LCD 16x2 en 4 bits ----
 * RS -> PA0
 * E  -> PA1
 * D4 -> PA8
 * D5 -> PA10
 * D6 -> PA5
 * D7 -> PA6
 */
#define LCD_RS_PORT GPIOA
#define LCD_E_PORT  GPIOA
#define LCD_D_PORT  GPIOA

#define LCD_RS_PIN  0u
#define LCD_E_PIN   1u
#define LCD_D4_PIN  8u
#define LCD_D5_PIN  10u
#define LCD_D6_PIN  5u
#define LCD_D7_PIN  6u

/* ---- Keypad 4x4 ----
 * Filas  -> PB12..PB15 (salida)
 * Columnas -> PB8..PB11 (entrada con pull-up)
 */
#define KP_R1 (1u<<12)
#define KP_R2 (1u<<13)
#define KP_R3 (1u<<14)
#define KP_R4 (1u<<15)
#define KP_C1 (1u<<8)
#define KP_C2 (1u<<9)
#define KP_C3 (1u<<10)
#define KP_C4 (1u<<11)

static const uint32_t KP_ROWS[4] = { KP_R1, KP_R2, KP_R3, KP_R4 };
static const uint32_t KP_COLS[4] = { KP_C1, KP_C2, KP_C3, KP_C4 };

/* Mapa de teclas [fila][columna]:
 * fila 0 (PB12): 1 2 3 A
 * fila 1 (PB13): 4 5 6 B
 * fila 2 (PB14): 7 8 9 C
 * fila 3 (PB15): * 0 # D
 */
static const char keymap[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

/* ---- Display 7 segmentos ----
 * Segmentos -> PB0..PB7
 * Dígitos   -> PC5, PC6, PC8, PC9 (activo LOW)
 */
static const uint8_t seg_lut[10] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111  // 9
};

static volatile uint8_t seg_digits[4] = {0,0,0,0}; // lo que se muestra

static const uint8_t seg_digit_pins[4] = {5,6,8,9}; // PC5, PC6, PC8, PC9

/* ---- Stepper en PC0..PC3, half-step 8 fases ---- */

static const uint8_t step_seq[8] = {
    0b0001, // A -> PC0
    0b0011, // A+B
    0b0010, // B
    0b0110, // B+C
    0b0100, // C
    0b1100, // C+D
    0b1000, // D
    0b1001  // D+A
};

static volatile uint8_t step_phase_idx = 0;
static volatile int8_t  step_dir = 1;       // 1 = horario, -1 = antihorario
static volatile uint8_t step_running = 0;

/* =================== LCD (driver no bloqueante) =================== */

static inline void LCD_RS(uint8_t v) {
    if (v) LCD_RS_PORT->ODR |=  (1u<<LCD_RS_PIN);
    else   LCD_RS_PORT->ODR &= ~(1u<<LCD_RS_PIN);
}

static inline void LCD_E(uint8_t v) {
    if (v) LCD_E_PORT->ODR |=  (1u<<LCD_E_PIN);
    else   LCD_E_PORT->ODR &= ~(1u<<LCD_E_PIN);
}

static void lcd_bus_write_nibble(uint8_t n) {
    uint32_t clr = (1u<<LCD_D4_PIN)|(1u<<LCD_D5_PIN)
                 | (1u<<LCD_D6_PIN)|(1u<<LCD_D7_PIN);
    LCD_D_PORT->BSRR = (clr << 16);  // limpiar

    uint32_t set = 0;
    if (n & 0x1) set |= (1u<<LCD_D4_PIN);
    if (n & 0x2) set |= (1u<<LCD_D5_PIN);
    if (n & 0x4) set |= (1u<<LCD_D6_PIN);
    if (n & 0x8) set |= (1u<<LCD_D7_PIN);
    if (set) LCD_D_PORT->BSRR = set;
}

/* FSM de la LCD */
typedef enum {
    LCD_IDLE,
    LCD_PUT4_HI,
    LCD_PULSE_HI,
    LCD_PUT4_LO,
    LCD_PULSE_LO
} lcd_state_t;

#define LCD_QSIZE 64
static volatile uint8_t lcd_q[LCD_QSIZE];
static volatile uint8_t lcd_q_head = 0, lcd_q_tail = 0;
static volatile lcd_state_t lcd_state = LCD_IDLE;
static volatile uint8_t  lcd_cur_byte = 0, lcd_is_data = 0;
static volatile uint16_t lcd_wait_ms = 0;

static void lcd_enqueue(uint8_t data, uint8_t is_data) {
    uint8_t next = (uint8_t)((lcd_q_head + 2) & (LCD_QSIZE-1));
    if (next == lcd_q_tail) return; // cola llena, se descarta

    lcd_q[lcd_q_head] = is_data;
    lcd_q[(lcd_q_head+1) & (LCD_QSIZE-1)] = data;
    lcd_q_head = next;
}

static inline void lcd_cmd_nb(uint8_t cmd) {
    lcd_enqueue(cmd, 0);
}

static inline void lcd_data_nb(uint8_t ch) {
    lcd_enqueue(ch, 1);
}

static void lcd_print_nb(const char *s) {
    while (*s) lcd_data_nb((uint8_t)*s++);
}

static void lcd_gpio_init(void) {
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

    GPIOA->MODER &= ~((3u<<(LCD_RS_PIN*2))|(3u<<(LCD_E_PIN*2))|
                      (3u<<(LCD_D4_PIN*2))|(3u<<(LCD_D5_PIN*2))|
                      (3u<<(LCD_D6_PIN*2))|(3u<<(LCD_D7_PIN*2)));

    GPIOA->MODER |=  ((1u<<(LCD_RS_PIN*2))|(1u<<(LCD_E_PIN*2))|
                      (1u<<(LCD_D4_PIN*2))|(1u<<(LCD_D5_PIN*2))|
                      (1u<<(LCD_D6_PIN*2))|(1u<<(LCD_D7_PIN*2)));

    LCD_RS(0);
    LCD_E(0);
}

static void lcd_init_nb_begin(void) {
    /* Secuencia típica 4 bits */
    lcd_cmd_nb(0x33);
    lcd_cmd_nb(0x32);
    lcd_cmd_nb(0x28); // 4 bits, 2 líneas
    lcd_cmd_nb(0x0C); // display ON, cursor OFF
    lcd_cmd_nb(0x06); // auto-increment
    lcd_cmd_nb(0x01); // clear
}

/* Llamar cada 1ms desde SysTick */
static void lcd_tick_1ms(void) {
    if (lcd_wait_ms) {
        lcd_wait_ms--;
    } else {
        switch (lcd_state) {
        case LCD_IDLE:
            if (lcd_q_tail != lcd_q_head) {
                lcd_is_data = lcd_q[lcd_q_tail];
                lcd_q_tail = (uint8_t)((lcd_q_tail+1) & (LCD_QSIZE-1));
                lcd_cur_byte = lcd_q[lcd_q_tail];
                lcd_q_tail = (uint8_t)((lcd_q_tail+1) & (LCD_QSIZE-1));
                LCD_RS(lcd_is_data);
                lcd_state = LCD_PUT4_HI;
            }
            break;

        case LCD_PUT4_HI: {
            uint8_t hi = (lcd_cur_byte >> 4) & 0x0F;
            lcd_bus_write_nibble(hi);
            LCD_E(1);
            lcd_wait_ms = 1;
            lcd_state = LCD_PULSE_HI;
            } break;

        case LCD_PULSE_HI:
            LCD_E(0);
            lcd_wait_ms = 1;
            lcd_state = LCD_PUT4_LO;
            break;

        case LCD_PUT4_LO: {
            uint8_t lo = lcd_cur_byte & 0x0F;
            lcd_bus_write_nibble(lo);
            LCD_E(1);
            lcd_wait_ms = 1;
            lcd_state = LCD_PULSE_LO;
            } break;

        case LCD_PULSE_LO:
            LCD_E(0);
            if (!lcd_is_data && (lcd_cur_byte == 0x01u || lcd_cur_byte == 0x02u)) {
                lcd_wait_ms = 2; // clear/home necesitan más tiempo
            }
            lcd_state = LCD_IDLE;
            break;

        default:
            lcd_state = LCD_IDLE;
            break;
        }
    }
}





/* =================== STEPPER MOTOR (PC0..PC3) =================== */

static inline void stepper_apply(uint8_t pattern) {
    GPIOC->BSRR = (0x0Fu << 16); // apagar PC0..PC3
    uint32_t bsrr_val = 0;
    if (pattern & 0x01) bsrr_val |= (1u<<0);
    if (pattern & 0x02) bsrr_val |= (1u<<1);
    if (pattern & 0x04) bsrr_val |= (1u<<2);
    if (pattern & 0x08) bsrr_val |= (1u<<3);
    if (bsrr_val) GPIOC->BSRR = bsrr_val;
}

static inline void stepper_all_off(void) {
    GPIOC->BSRR = (0x0Fu << 16);
}

static inline void stepper_start(void) {
    step_running = 1;
    TIM21->CR1 |= TIM_CR1_CEN;
}

static inline void stepper_stop(void) {
    step_running = 0;
    stepper_all_off();
    TIM21->CR1 &= ~TIM_CR1_CEN;
}

static inline void stepper_set_dir(int8_t dir) {
    step_dir = (dir >= 0) ? 1 : -1;
}

static inline void stepper_toggle_dir(void) {
    step_dir = -step_dir;
}

/* =================== DISPLAY 7-SEG (multiplex) =================== */

static inline void seg_digits_all_off(void) {
    /* PC5, PC6, PC8, PC9 HIGH -> todos apagados */
    GPIOC->BSRR = (1u<<5) | (1u<<6) | (1u<<8) | (1u<<9);
}

static inline void seg_digit_on(uint8_t idx) {
    uint8_t pin = seg_digit_pins[idx];
    GPIOC->BSRR = (1u << (pin + 16)); // LOW = ON
}

static inline void seg_write_pattern(uint8_t pattern) {
    GPIOB->BSRR = (0xFFu << 16);     // limpiar PB0..PB7
    GPIOB->BSRR = (uint32_t)pattern; // cargar nuevo patrón
}

/* =================== LÓGICA DE MICROONDAS =================== */

typedef enum {
    MW_STATE_IDLE = 0,   // mostrando HH:MM
    MW_STATE_TYPING,     // ingresando MM:SS
    MW_STATE_RUNNING     // cuenta regresiva
} mw_state_t;

static volatile mw_state_t mw_state = MW_STATE_IDLE;

/* Tiempo de microondas en dígitos MM:SS */
static volatile uint8_t mw_digits[4] = {0,0,0,0};
static volatile uint16_t mw_seconds = 0;         // segundos restantes
static volatile uint16_t mw_initial_seconds = 0; // para referencia

/* Reloj HH:MM (24 horas) */
static volatile uint8_t clock_hours   = 12;
static volatile uint8_t clock_minutes = 0;
static volatile uint8_t clock_seconds = 0;
static volatile uint8_t clock_digits[4] = {1,2,0,0}; // 12:00

/* Ciclos pre-grabados */
typedef struct {
    const char *name;
    uint8_t m10, m1, s10, s1; // MM:SS
} mw_cycle_t;

static const mw_cycle_t mw_cycles[] = {
    { "Descongelar Carnes", 0,5,0,0 }, // 05:00
    { "Poporopos",          0,2,3,0 }, // 02:30
    { "Agua Caliente",      0,1,0,0 }, // 01:00
    { "Calentar Plato",     0,3,0,0 }  // 03:00
};
#define MW_CYCLE_COUNT (sizeof(mw_cycles)/sizeof(mw_cycles[0]))
static volatile uint8_t mw_cycle_index = 0;

/* Prototipos */
static void lcd_show_cycle(void);
static void update_clock_digits(void);
static void update_seg_digits_from_clock(void);

/* Mostrar ciclo actual en LCD */
static void lcd_show_cycle(void) {
    lcd_cmd_nb(0x01);      // clear
    lcd_cmd_nb(0x80);      // línea 1
    lcd_print_nb("Ciclo:");


    lcd_cmd_nb(0xC0);      // línea 2
    lcd_print_nb(mw_cycles[mw_cycle_index].name);
}

/* Actualiza clock_digits a partir de horas/minutos */
static void update_clock_digits(void) {
    uint8_t h = clock_hours;
    uint8_t m = clock_minutes;
    clock_digits[0] = h / 10;
    clock_digits[1] = h % 10;
    clock_digits[2] = m / 10;
    clock_digits[3] = m % 10;
}

/* Copia HH:MM al buffer que usa el 7-seg */
static void update_seg_digits_from_clock(void) {
    seg_digits[0] = clock_digits[0];
    seg_digits[1] = clock_digits[1];
    seg_digits[2] = clock_digits[2];
    seg_digits[3] = clock_digits[3];
}



static void lcd_show_time_entry(void) {
    char buf[6];

    buf[0] = '0' + mw_digits[0];
    buf[1] = '0' + mw_digits[1];
    buf[2] = ':';
    buf[3] = '0' + mw_digits[2];
    buf[4] = '0' + mw_digits[3];
    buf[5] = '\0';

    lcd_cmd_nb(0x01);      // clear
    lcd_cmd_nb(0x80);      // línea 1
    lcd_print_nb("Tiempo:");

    lcd_cmd_nb(0xC0);      // línea 2
    lcd_print_nb(buf);
}




/* =================== KEYPAD: lectura y debounce =================== */

/* Lectura “cruda” del keypad: SIN debounce.
 * Devuelve: char de '0'..'9','A','B','C','D','*','#', o 0 si nada.
 */
static inline char keypad_hw_read_once(void) {
    /* Todas las filas HIGH */
    GPIOB->BSRR = KP_R1 | KP_R2 | KP_R3 | KP_R4;

    for (int r = 0; r < 4; r++) {
        /* Bajar SOLO la fila r */
        GPIOB->BSRR = (KP_ROWS[r] << 16);
        __asm volatile ("nop;nop;nop;nop");

        for (int c = 0; c < 4; c++) {
            if ((GPIOB->IDR & KP_COLS[c]) == 0u) {
                /* restaurar fila antes de salir */
                GPIOB->BSRR = KP_ROWS[r];
                return keymap[r][c];
            }
        }
        /* restaurar fila y seguir */
        GPIOB->BSRR = KP_ROWS[r];
    }
    return 0; // nada
}

/* Lógica de tecla (evento de key DOWN ya debounced) */
static void process_key(char keychar) {
    /* 1) Dígitos 0-9 -> armar MM:SS */
    if (keychar >= '0' && keychar <= '9') {
        if (mw_state != MW_STATE_RUNNING) {
            mw_state = MW_STATE_TYPING;
            uint8_t d = (uint8_t)(keychar - '0');

            mw_digits[0] = mw_digits[1];
            mw_digits[1] = mw_digits[2];
            mw_digits[2] = mw_digits[3];
            mw_digits[3] = d;

            seg_digits[0] = mw_digits[0];
            seg_digits[1] = mw_digits[1];
            seg_digits[2] = mw_digits[2];
            seg_digits[3] = mw_digits[3];

            // NUEVO: mostrar tiempo que llevas en la LCD
            lcd_show_time_entry();
        }
        return;
    }

    /* 2) Otros botones */
    switch (keychar) {
    case '*': { // START
        if (mw_state != MW_STATE_RUNNING) {
            uint16_t minutes = (uint16_t)(mw_digits[0]*10 + mw_digits[1]);
            uint16_t seconds = (uint16_t)(mw_digits[2]*10 + mw_digits[3]);
            uint16_t total   = (uint16_t)(minutes*60 + seconds);

            if (total > 0) {
                mw_seconds         = total;
                mw_initial_seconds = total;
                mw_state           = MW_STATE_RUNNING;

                /* Motor: arranca sentido horario */
                stepper_set_dir(1);
                stepper_start();
            }
        }
        } break;

    case '#': // CANCEL
        mw_state = MW_STATE_IDLE;
        mw_seconds = 0;
        mw_digits[0] = mw_digits[1] = mw_digits[2] = mw_digits[3] = 0;

        stepper_stop();

        /* Volver a mostrar hora */
        update_seg_digits_from_clock();
        break;

    case 'A': // ciclo anterior
        if (mw_cycle_index == 0) mw_cycle_index = MW_CYCLE_COUNT - 1;
        else mw_cycle_index--;
        lcd_show_cycle();
        break;

    case 'B': // ciclo siguiente
        mw_cycle_index++;
        if (mw_cycle_index >= MW_CYCLE_COUNT) mw_cycle_index = 0;
        lcd_show_cycle();
        break;

    case 'C': {
        const mw_cycle_t *c = &mw_cycles[mw_cycle_index];
        mw_digits[0] = c->m10;
        mw_digits[1] = c->m1;
        mw_digits[2] = c->s10;
        mw_digits[3] = c->s1;

        seg_digits[0] = mw_digits[0];
        seg_digits[1] = mw_digits[1];
        seg_digits[2] = mw_digits[2];
        seg_digits[3] = mw_digits[3];

        mw_state = MW_STATE_TYPING;

        // NUEVO: mostrar en LCD el tiempo del ciclo
        lcd_show_time_entry();
        } break;

    case 'D':
        /* Aquí podrías implementar ajuste de hora, etc. Por ahora no hace nada. */
        break;

    default:
        break;
    }
}

/* Debounce: se llama cada ~5ms desde SysTick */
#define KP_DEBOUNCE_TICKS 3 // 3*5ms = 15ms

static void keypad_scan_5ms(void) {
    static char   sample_prev   = 0;
    static uint8_t stable_cnt   = 0;
    static char   stable_key    = 0;
    static uint8_t pressed_latched = 0;

    char k = keypad_hw_read_once();

    if (k == sample_prev) {
        if (stable_cnt < 0xFF) stable_cnt++;
    } else {
        sample_prev = k;
        stable_cnt  = 1;
    }

    if (stable_cnt >= KP_DEBOUNCE_TICKS) {
        if (k != stable_key) {
            stable_key      = k;
            pressed_latched = 0;
        }

        if (stable_key != 0 && !pressed_latched) {
            pressed_latched = 1;
            process_key(stable_key);
        }

        if (stable_key == 0) {
            pressed_latched = 0;
        }
    }
}

/* =================== INTERRUPCIONES =================== */

/* SysTick: 1ms -> LCD FSM + keypad debounce */
void SysTick_Handler(void) {
    lcd_tick_1ms();

    static uint8_t kp_div = 0;
    kp_div++;
    if (kp_div >= 5) { // cada 5ms
        kp_div = 0;
        keypad_scan_5ms();
    }
}

/* TIM22: multiplex 7 segmentos */
void TIM22_IRQHandler(void) {
    if (TIM22->SR & TIM_SR_UIF) {
        TIM22->SR &= ~TIM_SR_UIF;

        static uint8_t which = 0;

        /* Apagar todos los dígitos */
        seg_digits_all_off();

        /* Tomar valor del dígito actual */
        uint8_t val = seg_digits[which];

        if (val < 10) {
            seg_write_pattern(seg_lut[val]);
            seg_digit_on(which);
        } else {
            seg_write_pattern(0x00); // en blanco
        }

        which++;
        if (which >= 4) which = 0;
    }
}

/* TIM21: stepper motor */
void TIM21_IRQHandler(void) {
    if (TIM21->SR & TIM_SR_UIF) {
        TIM21->SR &= ~TIM_SR_UIF;

        if (!step_running) {
            stepper_all_off();
            return;
        }

        if (step_dir > 0) {
            step_phase_idx++;
            if (step_phase_idx > 7) step_phase_idx = 0;
        } else {
            if (step_phase_idx == 0) step_phase_idx = 7;
            else step_phase_idx--;
        }
        stepper_apply(step_seq[step_phase_idx]);
    }
}

/* TIM2: 1 Hz -> reloj + cuenta regresiva microondas */
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;

        /* ---- Reloj HH:MM (interna) ---- */
        clock_seconds++;
        if (clock_seconds >= 60) {
            clock_seconds = 0;
            clock_minutes++;
            if (clock_minutes >= 60) {
                clock_minutes = 0;
                clock_hours++;
                if (clock_hours >= 24) clock_hours = 0;
            }
            update_clock_digits();
            if (mw_state == MW_STATE_IDLE) {
                update_seg_digits_from_clock();
            }
        }

        /* ---- Cuenta regresiva del microondas ---- */
        if (mw_state == MW_STATE_RUNNING && mw_seconds > 0) {
            mw_seconds--;

            uint16_t total = mw_seconds;
            uint8_t minutes = (uint8_t)(total / 60);
            uint8_t seconds = (uint8_t)(total % 60);

            mw_digits[0] = minutes / 10;
            mw_digits[1] = minutes % 10;
            mw_digits[2] = seconds / 10;
            mw_digits[3] = seconds % 10;

            seg_digits[0] = mw_digits[0];
            seg_digits[1] = mw_digits[1];
            seg_digits[2] = mw_digits[2];
            seg_digits[3] = mw_digits[3];

            /* Cambiar sentido del motor cada 10 segundos */
            if ((mw_seconds % 10u) == 0u) {
                stepper_toggle_dir();
            }

            if (mw_seconds == 0) {
                /* Tiempo terminado */
                mw_state = MW_STATE_IDLE;
                stepper_stop();
                update_seg_digits_from_clock();
                /* Aquí podrías mandar mensaje a LCD tipo "Listo!" si quieres */
            }
        }
    }
}

/* =================== main() =================== */

int main(void) {
    /* Habilitar HSI16 como reloj principal (normalmente ya lo está) */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }
    RCC->CFGR |= RCC_CFGR_SW_HSI;

    /* Clocks GPIO A/B/C */
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN
                 | RCC_IOPENR_GPIOBEN
                 | RCC_IOPENR_GPIOCEN;

    /* ---- LCD GPIO ---- */
    lcd_gpio_init();

    /* ---- 7-seg: PB0..PB7 salida ---- */
    GPIOB->MODER &= ~((3u<<(0*2))|(3u<<(1*2))|(3u<<(2*2))|(3u<<(3*2))|
                      (3u<<(4*2))|(3u<<(5*2))|(3u<<(6*2))|(3u<<(7*2)));
    GPIOB->MODER |=  ((1u<<(0*2))|(1u<<(1*2))|(1u<<(2*2))|(1u<<(3*2))|
                      (1u<<(4*2))|(1u<<(5*2))|(1u<<(6*2))|(1u<<(7*2)));

    /* ---- Dígitos PC5,6,8,9 salida ---- */
    GPIOC->MODER &= ~((3u<<(5*2))|(3u<<(6*2))|(3u<<(8*2))|(3u<<(9*2)));
    GPIOC->MODER |=  ((1u<<(5*2))|(1u<<(6*2))|(1u<<(8*2))|(1u<<(9*2)));
    seg_digits_all_off();

    /* ---- Stepper PC0..PC3 salida ---- */
    GPIOC->MODER &= ~((3u<<(0*2))|(3u<<(1*2))|(3u<<(2*2))|(3u<<(3*2)));
    GPIOC->MODER |=  ((1u<<(0*2))|(1u<<(1*2))|(1u<<(2*2))|(1u<<(3*2)));
    stepper_all_off();

    /* ---- Keypad ---- */
    // Filas PB12..PB15 salida
    GPIOB->MODER &= ~(0xFFu << 24);
    GPIOB->MODER |=  (0x55u << 24);
    GPIOB->BSRR  =   KP_R1 | KP_R2 | KP_R3 | KP_R4; // HIGH idle

    // Columnas PB8..PB11 entrada con pull-up
    GPIOB->MODER &= ~(0xFFu << 16);
    GPIOB->PUPDR &= ~(0xFFu << 16);
    GPIOB->PUPDR |=  (0x55u << 16); // pull-up

    /* ---- SysTick 1ms ---- */
    SysTick->LOAD  = 16000u - 1u;  // 16 MHz / 16000 = 1 kHz -> 1ms
    SysTick->VAL   = 0;
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk
                   | SysTick_CTRL_TICKINT_Msk
                   | SysTick_CTRL_ENABLE_Msk;

    /* ---- TIM22: multiplex 7-seg (~1kHz) ---- */
    RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;
    TIM22->PSC = 160u - 1u;    // 16MHz / 160 = 100kHz
    TIM22->ARR = 100u - 1u;    // 100kHz / 100 = 1kHz
    TIM22->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM22_IRQn);
    TIM22->CR1 |= TIM_CR1_CEN;

    /* ---- TIM21: stepper (~400 Hz) ---- */
    RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
    TIM21->PSC = 160u - 1u;              // 100kHz base
    TIM21->ARR = (100000u/400u) - 1u;    // ~400 Hz
    TIM21->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM21_IRQn);
    // TIM21 se enciende con stepper_start()

    /* ---- TIM2: 1 Hz (reloj + cuenta regresiva) ---- */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 15999u;   // 16MHz / (15999+1) = 1000 Hz
    TIM2->ARR = 999u;     // 1000 / (999+1) = 1 Hz
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;

    /* ---- Inicialización de estado ---- */
    update_clock_digits();
    update_seg_digits_from_clock();
    lcd_init_nb_begin();
    lcd_show_cycle();

    __enable_irq();

    while (1) {
        /* Todo se maneja por interrupciones */
        __WFI(); // opcional: espera a interrupción para ahorrar energía
    }
}
