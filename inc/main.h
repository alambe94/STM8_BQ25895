#ifndef _MAIN_H
#define _MAIN_H
#include "stm8l15x.h"

void BQ25_INT_Callback();

/* Private defines -----------------------------------------------------------*/

#define BQ25_INT_PIN                    GPIO_Pin_7
#define BQ25_INT_PIN_SOURCE             EXTI_Pin_7
#define BQ25_INT_PORT                   GPIOB
#define BQ25_INT_PORT_SOURCE            EXTI_Port_B
#define BQ25_INT_PIN_READ()             (BQ25_INT_PORT->IDR & BQ25_INT_PIN)


#define BQ25_STAT_PIN                   GPIO_Pin_6
#define BQ25_STAT_PORT                  GPIOB
#define BQ25_STAT_PIN_READ()            (BQ25_STAT_PORT->IDR & BQ25_STAT_PIN)

#define _SOFT_I2C_SDA_PIN		GPIO_Pin_0
#define _SOFT_I2C_SDA_PORT		GPIOC


#define _SOFT_I2C_SCL_PIN		GPIO_Pin_1
#define _SOFT_I2C_SCL_PORT		GPIOC

/*The T6 bit can be used to generate a software reset (the WDGA bit is set and then T6 bit is
cleared)*/
#define SOFTWARE_RST_MCU()             do{ WWDG->CR |= WWDG_CR_WDGA; \
                                           WWDG->CR &=~WWDG_CR_T6; \
                                         } while(0);


#endif 
