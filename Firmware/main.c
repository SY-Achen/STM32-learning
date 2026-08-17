/* ============================================================================
 * 吹雪机 (Snow Blower) STM32 控制系统 — 主程序 (v1.5 去看门狗版)
 * ----------------------------------------------------------------------------
 * MCU   : STM32F103C8T6 (Blue Pill 最小系统板)
 * 时钟  : HSE 8MHz x PLL9 = 72MHz
 *
 * 引脚分配 (与项目记录 MD 一致):
 *   PA0  : 光敏信号输入 (上拉输入; 高=环境亮=触发工作, 低=环境暗=停止)
 *   PA6  : TIM3_CH1 舵机 PWM 输出 (50Hz, 500~1833us, 270°舵机限幅)
 *   PA1  : 5A单路H桥 IN1 (推杆正转上桥)
 *   PA2  : 5A单路H桥 IN2 (推杆正转下桥)
 *   PA7  : 5A单路H桥 IN3 (推杆反转上桥)
 *   PA5  : 5A单路H桥 IN4 (推杆反转下桥)
 *   PC13 : 板载 LED 状态指示 (低电平点亮; 可选)
 *
 * 状态机: IDLE -> EXTENDING -> STABILIZE -> BLOWING -> STOPPING -> RETRACTING -> IDLE
 *         任意状态 -> FAULT (推杆超时等异常) -> 人工复位 -> IDLE
 *
 * v1.3 更新 (2026-08-09):
 *   - 换用 5A 单路 H 桥 (GZ-PMDC-120A7T), 4 信号真值表驱动:
 *       正转: IN1=1 IN2=0 IN3=1 IN4=0
 *       反转: IN1=0 IN2=1 IN3=0 IN4=1
 *       停止: 全 0
 *     (真值表警告: IN1&IN2 或 IN3&IN4 同时为 1 会烧 MOS, 代码已保证不会)
 *
 * v1.2 更新 (2026-08-09):
 *   - 修复 EXTENDING/RETRACTING 超时判断顺序 (超时分支不再被 3s 分支遮蔽)
 *   - 光敏触发加 30ms 消抖, 防灯闪误触发
 *   - 删除未使用的 servo_sweep_to() 和 led_blink_phase
 *   - 舵机脉宽限制为 500~1833us (270°舵机只用前 180°)
 *
 * 编译: Keil MDK 5.43 + ARMCLANG (AC6), 纯寄存器操作, 不依赖标准外设库
 * ========================================================================== */

/* ---------------------------- 寄存器地址定义 ----------------------------- */
#define RCC_BASE        0x40021000UL
#define GPIOA_BASE      0x40010800UL
#define GPIOB_BASE      0x40010C00UL
#define GPIOC_BASE      0x40011000UL
#define TIM3_BASE       0x40000400UL
#define SYSTICK_BASE    0xE000E010UL

/* RCC */
#define RCC_CR          (*(volatile unsigned long *)(RCC_BASE + 0x00))
#define RCC_CFGR        (*(volatile unsigned long *)(RCC_BASE + 0x04))
#define RCC_APB2ENR     (*(volatile unsigned long *)(RCC_BASE + 0x18))
#define RCC_APB1ENR     (*(volatile unsigned long *)(RCC_BASE + 0x1C))

/* GPIO */
#define GPIOA_CRL       (*(volatile unsigned long *)(GPIOA_BASE + 0x00))
#define GPIOA_CRH       (*(volatile unsigned long *)(GPIOA_BASE + 0x04))
#define GPIOA_IDR       (*(volatile unsigned long *)(GPIOA_BASE + 0x08))
#define GPIOA_ODR       (*(volatile unsigned long *)(GPIOA_BASE + 0x0C))
#define GPIOB_CRL       (*(volatile unsigned long *)(GPIOB_BASE + 0x00))
#define GPIOB_CRH       (*(volatile unsigned long *)(GPIOB_BASE + 0x04))
#define GPIOB_IDR       (*(volatile unsigned long *)(GPIOB_BASE + 0x08))
#define GPIOB_ODR       (*(volatile unsigned long *)(GPIOB_BASE + 0x0C))
#define GPIOC_CRH       (*(volatile unsigned long *)(GPIOC_BASE + 0x04))
#define GPIOC_ODR       (*(volatile unsigned long *)(GPIOC_BASE + 0x0C))

/* TIM3 */
#define TIM3_CR1        (*(volatile unsigned long *)(TIM3_BASE + 0x00))
#define TIM3_CCMR1      (*(volatile unsigned long *)(TIM3_BASE + 0x18))
#define TIM3_CCER       (*(volatile unsigned long *)(TIM3_BASE + 0x20))
#define TIM3_CNT        (*(volatile unsigned long *)(TIM3_BASE + 0x24))
#define TIM3_PSC        (*(volatile unsigned long *)(TIM3_BASE + 0x28))
#define TIM3_ARR        (*(volatile unsigned long *)(TIM3_BASE + 0x2C))
#define TIM3_CCR1       (*(volatile unsigned long *)(TIM3_BASE + 0x34))

/* SysTick */
#define STK_CTRL        (*(volatile unsigned long *)(SYSTICK_BASE + 0x00))
#define STK_LOAD        (*(volatile unsigned long *)(SYSTICK_BASE + 0x04))
#define STK_VAL         (*(volatile unsigned long *)(SYSTICK_BASE + 0x08))

/* ============================ 可配置参数 ================================= */
#define HSE_VALUE           8000000UL   /* 外部晶振 8MHz */
#define SYSCLK_FREQ         8000000UL   /* 系统时钟 8MHz (HSI, 无外部晶振) */

/* ---- 舵机 PWM (TIM3_CH1 @ PA6) ---- */
#define SERVO_FREQ_HZ       50UL        /* 舵机标准频率 50Hz */
#define PULSE_MIN_US        500UL       /* 0°  油门关闭 */
#define PULSE_MAX_US        1833UL      /* 180° 油门全开 (270°舵机限制: 只用前180°, 留45°安全余量) */
#define PULSE_WORK_US       1500UL      /* 工作角度脉宽 (默认约130°, 可调) */
#define PULSE_CENTER_US     1500UL      /* 90° 中位 (上电默认, 安全) */

/* ---- 状态机时序 ---- */
#define EXTEND_TIME_MS      3000UL      /* 推杆正常伸出时间 (250mm/90mm/s ≈ 2.8s) */
#define EXTEND_TIMEOUT_MS   5000UL      /* 推杆伸出超时保护 (卡死/限位失效 -> FAULT) */
#define RETRACT_TIME_MS     3000UL      /* 推杆正常缩回时间 */
#define RETRACT_TIMEOUT_MS  5000UL      /* 推杆缩回超时保护 */
#define STABILIZE_MS        500UL       /* 稳定等待时间 */
#define BLOWING_TIMEOUT_MS  1800000UL   /* 吹风最长工作时间 30min (0=不限) */
#define FAULT_RESET_MS      3000UL      /* FAULT 人工复位: 光敏恢复未触发持续此时间 */
#define SWEEP_TIME_MS       6000UL      /* 油门缓推/缓收时间 (6s, 防止油门突变熄火) */
#define LIGHT_DEBOUNCE_MS   30UL        /* 光敏触发消抖时间 */

/* ---- 光敏逻辑 ---- */
#define LIGHT_TRIGGER_LEVEL 1UL         /* PA0 电平: 1=高电平触发工作, 0=低电平触发 */

/* ---- 功能开关 ---- */
#define LED_INDICATOR_EN    1           /* 1=PC13 LED 状态指示, 0=关闭 */

/* ============================ 系统状态机 ================================= */
typedef enum {
    STATE_IDLE = 0,
    STATE_EXTENDING,
    STATE_STABILIZE,
    STATE_BLOWING,
    STATE_STOPPING,
    STATE_RETRACTING,
    STATE_FAULT
} SystemState;

static SystemState state = STATE_IDLE;
static volatile unsigned long sys_tick = 0;   /* 1ms 时基计数器 */
static unsigned long state_enter_tick = 0;

/* ============================ 基础函数 ================================== */

/* SysTick 1ms 中断服务函数 */
void SysTick_Handler(void)
{
    sys_tick++;
}

static unsigned long now_ms(void)
{
    return sys_tick;
}

static void state_enter(SystemState s)
{
    state = s;
    state_enter_tick = now_ms();
}

static unsigned long state_elapsed(void)
{
    return now_ms() - state_enter_tick;
}

/* 毫秒延时 (阻塞; 仅初始化用, 运行时不用) */
static void delay_ms(unsigned long ms)
{
    unsigned long start = now_ms();
    while ((now_ms() - start) < ms) { }
}

/* ============================ 硬件初始化 ================================= */

void SystemInit(void)
{
    /* ponytail: 用 HSI 内部 8MHz, 不依赖外部晶振(晶振虚焊/坏板会卡死 HSE 等待)
     * 若要 72MHz 精度(舵机 PWM 更准), 换好晶振的板子后恢复 HSE+PLL 配置 */
    RCC_CR   &= ~(1UL << 24);                   /* PLLON=0 (保持关闭) */
    RCC_CR   |=  (1UL << 0);                    /* HSION=1 */
    RCC_CFGR &= ~(0x3UL);                       /* SW=HSI */
    while ((RCC_CFGR & 0x3UL) != 0) { }         /* 等待 SWS=HSI */
}

static void hw_init(void)
{
    RCC_APB2ENR |= (1UL << 2) | (1UL << 3) | (1UL << 4);  /* IOPA IOPB IOPC */
    RCC_APB1ENR |= (1UL << 1);                             /* TIM3 */

    /* --- PA0: 光敏输入, 上拉 --- */
    GPIOA_CRL &= ~(0xFUL << 0);
    GPIOA_CRL |=  (0x8UL << 0);
    GPIOA_ODR |=  (1UL << 0);

    /* --- PA6: TIM3_CH1 复用推挽输出 50MHz --- */
    GPIOA_CRL &= ~(0xFUL << 24);
    GPIOA_CRL |=  (0xBUL << 24);

    /* --- PA5/PA7: H桥 IN4/IN3, 推挽输出 50MHz, 初始低 --- */
    GPIOA_CRL &= ~((0xFUL << 20) | (0xFUL << 28));  /* PA5 bit20-23, PA7 bit28-31 */
    GPIOA_CRL |=  (0x3UL << 20) | (0x3UL << 28);     /* CNF=00 MODE=11 (推挽 50MHz) */
    GPIOA_ODR &= ~((1UL << 5) | (1UL << 7));         /* 初始低 */

    /* --- PA1/PA2: H桥 IN1/IN2, 推挽输出 50MHz, 初始低 --- */
    GPIOA_CRL &= ~((0xFUL << 4) | (0xFUL << 8));    /* PA1 bit4-7, PA2 bit8-11 */
    GPIOA_CRL |=  (0x3UL << 4) | (0x3UL << 8);       /* CNF=00 MODE=11 (推挽 50MHz) */
    GPIOA_ODR &= ~((1UL << 1) | (1UL << 2));         /* 初始低 */

#if LED_INDICATOR_EN
    /* --- PC13: 板载 LED --- */
    GPIOC_CRH &= ~(0xFUL << 20);
    GPIOC_CRH |=  (0x3UL << 20);
    GPIOC_ODR |=  (1UL << 13);
#endif

    /* --- TIM3_CH1 PWM: 50Hz, 1us 计数 --- */
    TIM3_PSC  = 7;                               /* 8MHz/8 = 1MHz (HSI) */
    TIM3_ARR  = (SYSCLK_FREQ / (TIM3_PSC + 1UL)) / SERVO_FREQ_HZ - 1UL;  /* 1MHz/50Hz = 19999 */
    TIM3_CCMR1 = (0x6UL << 4) | (1UL << 3);      /* OC1M=110 PWM1, OC1PE=1 */
    TIM3_CCER  = (1UL << 0);                     /* CC1E=1 */
    TIM3_CCR1  = PULSE_CENTER_US;
    TIM3_CR1  |= (1UL << 7) | (1UL << 0);        /* ARPE=1, CEN=1 */

    /* --- SysTick: 1ms 中断 --- */
    STK_LOAD = (SYSCLK_FREQ / 1000UL) - 1UL;
    STK_VAL  = 0;
    STK_CTRL = (1UL << 2) | (1UL << 1) | (1UL << 0);
}

/* ============================ 舵机控制 =================================== */

static void servo_set_pulse(unsigned long us)
{
    if (us > PULSE_MAX_US) us = PULSE_MAX_US;
    if (us < PULSE_MIN_US) us = PULSE_MIN_US;
    TIM3_CCR1 = us;
}

/* ============================ H桥 / 推杆控制 ============================= */
/* 5A 单路 H 桥 (GZ-PMDC-120A7T) 4 信号真值表:
 *   正转: IN1=1 IN2=0 IN3=1 IN4=0
 *   反转: IN1=0 IN2=1 IN3=0 IN4=1
 *   停止: 全 0
 * ⚠ 真值表警告: IN1&IN2 同时为 1 或 IN3&IN4 同时为 1 会烧 MOS!
 *   (本驱动函数保证任意时刻最多只有一个方向桥导通) */
#define HBR_IN1   (1UL << 1)    /* PA1  -> 模块 IN1 */
#define HBR_IN2   (1UL << 2)    /* PA2  -> 模块 IN2 */
#define HBR_IN3   (1UL << 7)    /* PA7  -> 模块 IN3 */
#define HBR_IN4   (1UL << 5)    /* PA5  -> 模块 IN4 */

/* 推杆伸出 (正转): IN1=1 IN2=0 IN3=1 IN4=0 */
static void actuator_extend(void)
{
    GPIOA_ODR |=  HBR_IN1;               /* IN1 = 1 */
    GPIOA_ODR &= ~HBR_IN2;               /* IN2 = 0 */
    GPIOA_ODR |=  HBR_IN3;               /* IN3 = 1 */
    GPIOA_ODR &= ~HBR_IN4;               /* IN4 = 0 */
}

/* 推杆缩回 (反转): IN1=0 IN2=1 IN3=0 IN4=1 */
static void actuator_retract(void)
{
    GPIOA_ODR &= ~HBR_IN1;               /* IN1 = 0 */
    GPIOA_ODR |=  HBR_IN2;               /* IN2 = 1 */
    GPIOA_ODR &= ~HBR_IN3;               /* IN3 = 0 */
    GPIOA_ODR |=  HBR_IN4;               /* IN4 = 1 */
}

/* 推杆停止: 全部信号清 0 (先清方向, 防直通烧 MOS) */
static void actuator_stop(void)
{
    GPIOA_ODR &= ~(HBR_IN1 | HBR_IN2 | HBR_IN3 | HBR_IN4);
}

/* ============================ 光敏检测 (带消抖) ========================== */

static unsigned long light_sensor_read(void)
{
    return (GPIOA_IDR >> 0) & 1UL;
}

static unsigned long light_raw(void)
{
#if LIGHT_TRIGGER_LEVEL == 1
    return light_sensor_read() == 1UL;
#else
    return light_sensor_read() == 0UL;
#endif
}

/* 带 30ms 消抖的光敏触发检测 (连续 LIGHT_DEBOUNCE_MS 保持才认为有效) */
static unsigned long light_triggered(void)
{
    static unsigned long debounce_start = 0;
    static unsigned long last_state = 0;
    unsigned long cur = light_raw();

    if (cur != last_state) {
        last_state = cur;
        debounce_start = now_ms();
        return 0;
    }
    if (cur && ((now_ms() - debounce_start) >= LIGHT_DEBOUNCE_MS)) {
        return 1;
    }
    return 0;
}

/* ============================ LED 指示 =================================== */
#if LED_INDICATOR_EN
static void led_on(void)  { GPIOC_ODR &= ~(1UL << 13); }
static void led_off(void) { GPIOC_ODR |=  (1UL << 13); }
static void led_toggle(void) { GPIOC_ODR ^= (1UL << 13); }
#endif

/* ============================ 状态机处理 ================================= */

static void fsm_run(void)
{
    switch (state) {

    /* ---- IDLE: 待机, 等待光敏触发 (带消抖) ---- */
    case STATE_IDLE:
        actuator_stop();
        if (light_triggered()) {
            state_enter(STATE_EXTENDING);
        }
        break;

    /* ---- EXTENDING: 推杆伸出 ---- */
    case STATE_EXTENDING:
        actuator_extend();
        if (state_elapsed() >= EXTEND_TIMEOUT_MS) {
            /* 5s 超时 = 限位失效或推杆卡死 -> 异常 */
            actuator_stop();
            state_enter(STATE_FAULT);
        } else if (!light_triggered()) {
            /* 光敏中途解除: 取消本次循环 */
            actuator_stop();
            state_enter(STATE_IDLE);
        } else if (state_elapsed() >= EXTEND_TIME_MS) {
            /* 3s 正常伸出完成 (内置限位已自动断电) */
            actuator_stop();
            state_enter(STATE_STABILIZE);
        }
        break;

    /* ---- STABILIZE: 稳定等待 ---- */
    case STATE_STABILIZE:
        actuator_stop();
        if (state_elapsed() >= STABILIZE_MS) {
            state_enter(STATE_BLOWING);
        }
        break;

    /* ---- BLOWING: 缓推油门 (二次曲线: 起步最慢, 防发动机突然进油) ---- */
    case STATE_BLOWING:
        if (state_elapsed() < SWEEP_TIME_MS) {
            unsigned long t = state_elapsed();
            /* 二次曲线缓推: pulse = min + (work-min) * (t/T)^2
             * 起步阶段脉宽变化极小, 油门缓慢打开, 越接近目标越快收尾 */
            unsigned long progress = (t * t) / SWEEP_TIME_MS;  /* 0..T 线性 */
            unsigned long pulse = PULSE_MIN_US
                + (progress * (PULSE_WORK_US - PULSE_MIN_US)) / SWEEP_TIME_MS;
            servo_set_pulse(pulse);
        } else {
            servo_set_pulse(PULSE_WORK_US);
            if (!light_triggered()) {
                state_enter(STATE_STOPPING);
            } else if ((BLOWING_TIMEOUT_MS > 0) &&
                       (state_elapsed() >= BLOWING_TIMEOUT_MS)) {
                state_enter(STATE_STOPPING);
            }
        }
        break;

    /* ---- STOPPING: 缓收油门回零 (反二次曲线: 先快后慢, 最后平稳关死) ---- */
    case STATE_STOPPING:
        actuator_stop();
        if (state_elapsed() < SWEEP_TIME_MS) {
            unsigned long t = state_elapsed();
            /* pulse = min + (work-min) * ((T-t)/T)^2
             * 刚开始收得快一些, 接近关闭时变化变慢, 避免最后猛关熄火 */
            unsigned long remaining = SWEEP_TIME_MS - t;
            unsigned long progress = (remaining * remaining) / SWEEP_TIME_MS;
            unsigned long pulse = PULSE_MIN_US
                + (progress * (PULSE_WORK_US - PULSE_MIN_US)) / SWEEP_TIME_MS;
            servo_set_pulse(pulse);
        } else {
            servo_set_pulse(PULSE_MIN_US);
            state_enter(STATE_RETRACTING);
        }
        break;

    /* ---- RETRACTING: 推杆缩回 ---- */
    case STATE_RETRACTING:
        actuator_retract();
        if (state_elapsed() >= RETRACT_TIMEOUT_MS) {
            actuator_stop();
            state_enter(STATE_FAULT);
        } else if (state_elapsed() >= RETRACT_TIME_MS) {
            actuator_stop();
            state_enter(STATE_IDLE);
        }
        break;

    /* ---- FAULT: 故障, 必须人工复位 ---- */
    case STATE_FAULT:
    default:
        {
            static unsigned long fault_idle_start = 0;

            actuator_stop();
            servo_set_pulse(PULSE_MIN_US);

            if (!light_triggered()) {
                if (fault_idle_start == 0) {
                    fault_idle_start = now_ms();
                } else if ((now_ms() - fault_idle_start) >= FAULT_RESET_MS) {
                    fault_idle_start = 0;
                    state_enter(STATE_IDLE);
                }
            } else {
                fault_idle_start = 0;
            }
        }
        break;
    }
}

/* ============================ 主函数 ===================================== */

int main(void)
{
    hw_init();

#if LED_INDICATOR_EN
    unsigned long last_led_blink = 0;
#endif

    /* 上电安全: 油门回零, 推杆停止 */
    servo_set_pulse(PULSE_MIN_US);
    actuator_stop();
    delay_ms(100);


    while (1) {
        fsm_run();

#if LED_INDICATOR_EN
        /* LED: IDLE 慢闪 / BLOWING 常亮 / FAULT 快闪 */
        if (state == STATE_BLOWING) {
            led_on();
        } else if (state == STATE_FAULT) {
            if ((now_ms() - last_led_blink) >= 100UL) {
                last_led_blink = now_ms();
                led_toggle();
            }
        } else {
            if ((now_ms() - last_led_blink) >= 1000UL) {
                last_led_blink = now_ms();
                led_toggle();
            }
        }
#endif
    }
}

/* ============================ 未使用中断处理 ============================= */
void HardFault_Handler(void) { while (1) { } }
void MemManage_Handler(void) { while (1) { } }
void BusFault_Handler(void)  { while (1) { } }
void UsageFault_Handler(void){ while (1) { } }
void NMI_Handler(void)       { while (1) { } }
void SVC_Handler(void)       { }
void DebugMon_Handler(void)  { }
void PendSV_Handler(void)    { }
