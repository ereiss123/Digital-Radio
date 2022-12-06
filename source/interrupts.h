#ifndef INTERRUPT_H
#define INTERRUPT_H

void SysTick_Init(uint32_t ticks);
void SysTick_Handler(void);
void EXTI_Init(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);

#endif
