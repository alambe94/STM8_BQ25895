/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MILLIS_H
#define __MILLIS_H



/* Private function prototypes -----------------------------------------------*/
void Millis_Init(void);
void delay_ms(uint16_t time);

uint32_t millis(void);

void delay_us(uint16_t time);



#endif /* __MILLIS_H */
