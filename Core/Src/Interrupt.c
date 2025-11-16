#include "Interrupt.h"

void EXTI15_10_IRQHandler()
{
    // Мы должны обязательно отключать pending бит после того, как прерыв. выполнено
    // 1 чтобы сбросит в 0
    SET_BIT(EXTI->PR, EXTI_PR_PR13);


}