#include "clock.h"

void Clock_Init(void)
{
    /* 1. Включаем тактирование блока питания PWR и ставим Scale 1 (для частот до 168 МГц) */
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
    SET_BIT(PWR->CR, PWR_CR_VOS); // VOS = 11b → Scale 1

    /* 2. Включаем HSE и ждём стабилизации */
    SET_BIT(RCC->CR, RCC_CR_HSEON);
    while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0U)
    {
        /* ждём, пока HSE не стабилизируется */
    }
    /* 3. Настраиваем FLASH: кеши + 5 тактов ожидания (для 168 МГц)*/
    /* Сбрасываем LATENCY и включаем кэши/предвыборку, если нужно */
    MODIFY_REG(FLASH->ACR,
               FLASH_ACR_LATENCY,
               FLASH_ACR_LATENCY_5WS); // 5 wait states
    /* Можно дополнительно включить prefetch / I-cache / D-cache (по желанию):
       SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN); */
    /* 4. Настраиваем делители шин: AHB, APB1, APB2
       - AHB (HCLK)  = SYSCLK / 1  = 168 МГц  (максимум)
       - APB1 (PCLK1)= HCLK  / 4   = 42  МГц  (максимум для APB1)
       - APB2 (PCLK2)= HCLK  / 2   = 84  МГц  (максимум для APB2)
    */
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2,
               RCC_CFGR_HPRE_DIV1 |      // AHB = /1
                   RCC_CFGR_PPRE1_DIV4 | // APB1 = /4
                   RCC_CFGR_PPRE2_DIV2); // APB2 = /2

    /* 5. Настраиваем PLL под 168 МГц от HSE = 8 МГц:
           PLLM = 8, PLLN = 336, PLLP = 2, PLLQ = 7 */
    WRITE_REG(RCC->PLLCFGR,
              (8U << RCC_PLLCFGR_PLLM_Pos) |      // PLLM = 8
                  (336 << RCC_PLLCFGR_PLLN_Pos) | // PLLN = 336
                  (0U << RCC_PLLCFGR_PLLP_Pos) |  // PLLP = 2 (код 00)
                  RCC_PLLCFGR_PLLSRC_HSE |        // источник PLL = HSE
                  (7U << RCC_PLLCFGR_PLLQ_Pos));  // PLLQ = 7 (~48 МГц)

    /* 6. Включаем PLL и ждём готовности */
    SET_BIT(RCC->CR, RCC_CR_PLLON);
    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0U)
    {
        /* ждём, пока PLL не поднимется */
    }
    /* 7. Переключаем системное тактирование на PLL */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
    while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    {
        /* ждём, пока в статусе не появится "PLL как SYSCLK" */
    }
    /* 8. (Опционально) Выключить HSI, чтобы не жрать лишний ток */
    CLEAR_BIT(RCC->CR, RCC_CR_HSION);
    /* 9. Обновляем глобальную переменную SystemCoreClock */
    SystemCoreClock = 168000000U;
}

void MCO_init(void)
{

    // Для PC9
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
    /* MODER: сначала очищаем, потом ставим 10b (AF) */
    CLEAR_BIT(GPIOC->MODER, GPIO_MODER_MODE9_Msk);
    SET_BIT(GPIOC->MODER, GPIO_MODER_MODE9_1); // бит MODE9_1 = 1, MODE9_0 = 0 → 10b

    CLEAR_BIT(GPIOC->OTYPER, GPIO_OTYPER_OT9_Msk);
    /* OSPEEDR: 11b = very high speed */
    CLEAR_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEED9_Msk);
    SET_BIT(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEED9_Msk);
    CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPD9_Msk);

    CLEAR_BIT(GPIOC->AFR[1], 0xFU << ((9U - 8U) * 4U)); // (9-8)*4 = 4, поле для PC9
    /* Сначала очищаем поля MCO2 и MCO2PRE */ //там 0000

    CLEAR_BIT(RCC->CFGR, RCC_CFGR_MCO2 | RCC_CFGR_MCO2PRE);
    /* Источник MCO2 = PLLCLK (MCO2 = 11b) */
    SET_BIT(RCC->CFGR, RCC_CFGR_MCO2_0 | RCC_CFGR_MCO2_1);
    /* Предделитель MCO2PRE = /5 → 111b: ставлю все три бита */
    SET_BIT(RCC->CFGR, RCC_CFGR_MCO2PRE_0 |
                           RCC_CFGR_MCO2PRE_1 |
                           RCC_CFGR_MCO2PRE_2);
}