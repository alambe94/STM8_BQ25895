
#include "i2c.h"

void I2C_Setup()
{
  CLK_PeripheralClockConfig(CLK_Peripheral_I2C1,ENABLE);
  

  
}
/*generate start condition*/
uint8_t I2C_Start() {
  uint8_t timeout=255;
  
  I2C1->CR2 |= (I2C_CR2_START);
  while (!(I2C1->SR1 & (I2C_SR1_SB)) && (--timeout));
    if(timeout)
  {
    return 1;
  }
    return 0;
}

/*generate stop condition*/
uint8_t I2C_Stop() {
  uint8_t timeout=255;

  I2C1->CR2 |= (I2C_CR2_STOP);
  while (!(I2C1->SR3 & (I2C_SR3_MSL)) && --timeout);
   if(timeout)
  {
    return 1;
  }
    return 0;
}

/*send byte */
uint8_t I2C_Write_Byte(uint8_t data) {

}
/*send device address */
uint8_t I2C_Write_Address(uint8_t addr) {

}

uint8_t I2C_Read_Byte() {

  
}

void I2C_Read_Buffer(uint8_t *buf, uint16_t len) {
  while (len-- > 1) {
    I2C1->CR2 |= (I2C_CR2_ACK);
    while (!(I2C1->SR1 & (I2C_SR1_RXNE)));
    *(buf++) = I2C1->DR;
  }
  *buf = I2C_Read_Byte();
}