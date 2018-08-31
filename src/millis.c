#include "stm8s.h"
#include "millis.h"
#include "main.h"


/* Private defines -----------------------------------------------------------*/


#define TIM4_PERIOD 255// Interrupt every 15ms (16M/8*128*255)


__IO uint32_t current_millis = 0; //--IO: volatile read/write 



void Millis_Init(void)
{
  TIM4_DeInit();
  
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV8); //f_master = HSI = 2Mhz
  /* Time base configuration */
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, TIM4_PERIOD);
  /* Clear TIM4 update flag */
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  /* Enable update interrupt */
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  /* enable interrupts */
  enableInterrupts();
  /* Enable TIM4 */
  TIM4_Cmd(ENABLE);
}



uint32_t millis(void)
{
  return current_millis;
}


/*
void delay_us(uint16_t time)
{
  while(time--)
  {
    nop();
    nop();
    nop();
    nop();
    nop();
    nop();
    nop();
  }
}


void delay_ms(uint16_t time)
{
  while(time--)
  {
    delay_us(1000);
  }
}
*/



void delay_ms(uint16_t time)
{
uint32_t temp;
temp =millis();
while(millis()-temp<time);

}



//Interupt event, happens every 1 ms

INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23) 

{   
  //increase 1, for millis() function
  current_millis ++;
  TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
  Wakeup_Callback();
}


