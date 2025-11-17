#include "clock.h"

void Clock_Init(void)
{
    SET_BIT(RCC->CR, RCC_CR_HSION); // Включаем внутренний генератор HSI

    while ((READ_BIT(RCC->CR, RCC_CR_HSIRDY)) == 0U)
    {
    } // Ждём

    // Переключаем системное тактирование на HSI (SW = 00)
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);

    // Ждём, пока статус (SWS) покажет, что источник SYSCLK = HSI (00)
    while ((READ_BIT(RCC->CFGR, RCC_CFGR_SWS)) != 0)
    {
    } // ждём

    //* 1. Выключаем PLL и HSE, чтобы можно было безопасно перенастроить PLL

    // Выключаем PLL
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);

    // Ждем, пока флаг готовности PLLRDY сбросится (0 = PLL остановлен)
    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != 0U)
    {
    } // Ждём

    // Выключаем внешний генератор HSE и защиту CSS (если вдруг были включены)
    CLEAR_BIT(RCC->CR, RCC_CR_HSEON | RCC_CR_CSSON);

    // Ждем, пока HSE полностью выключится (HSERDY = 0)
    while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != 0U)
    {
        // Ждём
    }

    // Снимаем bypass HSE (на всякий случай)
    CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);

    // Для F429 обычно VOS[1:0] = 0b11 (макр PWR_CR_VOS может сразу поставить нужное)
    // На частотах до 168 МГц ядру нужно работать в Scale 1 по напряжению.
    // В референс-мануале для F42x/F43x написано: чтобы использовать максимальную частоту (168 МГц при 2.7–3.6 В), нужно выставить Scale 1.

    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
    // Для F429 обычно VOS[1:0] = 0b11 (макр PWR_CR_VOS может сразу поставить нужное)
    SET_BIT(PWR->CR, PWR_CR_VOS);

    //* 2. Настраиваем FLASH под 168 МГц. * Нужно включить кэши и выставить 5 тактов ожидания (LATENCY = 5)

    // Сначала очищаем регистр ACR (чтобы не наслаивать мусор)
    WRITE_REG(FLASH->ACR, 0U);
    // Включаем instruction cache
    SET_BIT(FLASH->ACR, FLASH_ACR_ICEN);
    // Включаем data cache
    SET_BIT(FLASH->ACR, FLASH_ACR_DCEN);
    // Включаем prefetch
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN);
    // Устанавливаем 5 тактов ожидания для FLASH при 168 МГц
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_5WS);

    /* =========================
     * 3. Настраиваем делители шин AHB, APB1, APB2
     * =========================
     * AHB  = SYSCLK / 1  = 168 МГц
     * APB1 = HCLK   / 4  = 42 МГц
     * APB2 = HCLK   / 2  = 84 МГц
     */

    // Очищаем поля делителей (HPRE, PPRE1, PPRE2) в CFGR
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2, 0U);

    // Устанавливаем делитель AHB = 1 (HPRE = 0000)
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);

    // Устанавливаем делитель APB1 = 4 (PPRE1 = 101)
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV4);

    // Устанавливаем делитель APB2 = 2 (PPRE2 = 100)
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV2);

    /* =========================
     * 4. Настраиваем PLL от HSI
     * =========================
     * Формула:
     *   f_in  = HSI / PLLM   = 16 / 16 = 1 МГц
     *   f_VCO = f_in * PLLN  = 1 * 336 = 336 МГц
     *   f_PLL = f_VCO / PLLP = 336 / 2 = 168 МГц (SYSCLK)
     *   f_48  = f_VCO / PLLQ = 336 / 7 ≈ 48 МГц
     */

    WRITE_REG(RCC->PLLCFGR, 0U);
    // Устанавливаем PLLM = 16 (биты [5:0])
    MODIFY_REG(RCC->PLLCFGR,
               RCC_PLLCFGR_PLLM,
               16U << RCC_PLLCFGR_PLLM_Pos);

    // Устанавливаем PLLN = 336 (биты [14:6])
    MODIFY_REG(RCC->PLLCFGR,
               RCC_PLLCFGR_PLLN,
               336U << RCC_PLLCFGR_PLLN_Pos);

    // Устанавливаем PLLP = 2 (код 00 в битах [17:16])
    // То есть просто гарантируем, что PLLP-биты = 00
    MODIFY_REG(RCC->PLLCFGR,
               RCC_PLLCFGR_PLLP,
               0U << RCC_PLLCFGR_PLLP_Pos);

    // Источник PLL = HSI (бит PLLSRC = 0, значит тут ничего не ставим)
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC);

    // Устанавливаем PLLQ = 7 (биты [27:24])
    MODIFY_REG(RCC->PLLCFGR,
               RCC_PLLCFGR_PLLQ,
               7U << RCC_PLLCFGR_PLLQ_Pos);
    //* 5. Включаем PLL и ждём готовности

    // Включаем PLL
    SET_BIT(RCC->CR, RCC_CR_PLLON);
    // Ждем, пока PLLRDY = 1 (PLL стабилизировался)
    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0U)
    {
        // Ждём
    }

    /* =========================
     * 6. Переключаем SYSCLK на PLL
     * ========================= */

    // Выбираем PLL как источник SYSCLK (SW = 10)
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
    // Ждем, пока статус (SWS) покажет, что SYSCLK = PLL (10)
    while ((READ_BIT(RCC->CFGR, RCC_CFGR_SWS)) != RCC_CFGR_SWS_PLL)
    {
        // Ждём
    }

    /* =========================
     * 7. (Опционально) Выводим SYSCLK на MCO2 (PC9) для проверки
     * =========================
     * MCO2 source = SYSCLK
     * MCO2 prescaler = /5 (чтобы частота не была слишком большой)
     * На PC9 получим 168 / 5 ≈ 33.6 МГц
     */

    // Включаем тактирование порта GPIOC (если вдруг выше не включили)
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);

    // Настраиваем PC9 в режим альтернативной функции (MODER9 = 10)
    MODIFY_REG(GPIOC->MODER,
               GPIO_MODER_MODER9,
               GPIO_MODER_MODER9_1); // 10b = AF

    // Максимальная скорость для PC9
    SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEED9);

    // Альтернативная функция AF0 для MCO2: AFRH9 = 0000 (AF0)
    MODIFY_REG(GPIOC->AFR[1],
               GPIO_AFRH_AFSEL9,
               0U << GPIO_AFRH_AFSEL9_Pos);

    // Настраиваем MCO2: источник SYSCLK, предделитель /5
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_MCO2 | RCC_CFGR_MCO2PRE,
               (0b00 << RCC_CFGR_MCO2_Pos) |         // MCO2 source: SYSCLK
                   (0b100 << RCC_CFGR_MCO2PRE_Pos)); // /5

    /* =========================
     * 8. Обновляем SystemCoreClock (если используешь CMSIS)
     * ========================= */

    SystemCoreClock = 168000000UL;
}