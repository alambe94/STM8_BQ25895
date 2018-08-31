#include "soft_i2c.h"
#include "stm8l15x.h"

void Soft_I2C_Init(void) {

  GPIO_Init(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_Mode_Out_OD_HiZ_Fast);
  GPIO_Init(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_Mode_Out_OD_HiZ_Fast);
  SOFT_I2C_SDA_HIGH();
  SOFT_I2C_SCL_HIGH();
  
}

void Soft_I2C_Delay() {
uint16_t i = 5;
  while (i--);
}
  
uint8_t Soft_I2C_Start(void)
{
	SOFT_I2C_SDA_HIGH();
	Soft_I2C_Delay();
	SOFT_I2C_SCL_HIGH();
	Soft_I2C_Delay();
	if (!SOFT_I2C_SDA_READ())
	{
		return SOFT_I2C_ERR;
	}

	SOFT_I2C_SDA_LOW();

	Soft_I2C_Delay();
	if (SOFT_I2C_SDA_READ())
	{
		return SOFT_I2C_ERR;
	}

	return SOFT_I2C_OK;
}

void Soft_I2C_Stop(void)
{
	SOFT_I2C_SCL_LOW();
	Soft_I2C_Delay();
	SOFT_I2C_SDA_LOW();
	Soft_I2C_Delay();
	SOFT_I2C_SCL_HIGH();
	Soft_I2C_Delay();
	SOFT_I2C_SDA_HIGH();
	Soft_I2C_Delay();
}

uint8_t Soft_I2C_Wait_ACK()
{
	uint16_t timeOut = 5000;
	SOFT_I2C_SCL_LOW();
	Soft_I2C_Delay();
	SOFT_I2C_SDA_HIGH();
	Soft_I2C_Delay();
	SOFT_I2C_SCL_HIGH();
	Soft_I2C_Delay();

	while (SOFT_I2C_SDA_READ())
	{
		timeOut--;
		if (timeOut == 0)
		{
			Soft_I2C_Stop();
			return SOFT_I2C_ERR;
		}
	}

	SOFT_I2C_SCL_LOW();
	Soft_I2C_Delay();

	return SOFT_I2C_OK;

}

void Soft_I2C_ACK(void)
{

	SOFT_I2C_SCL_LOW();
	Soft_I2C_Delay();
	SOFT_I2C_SDA_LOW();
	Soft_I2C_Delay();
	SOFT_I2C_SCL_HIGH();
	Soft_I2C_Delay();
	SOFT_I2C_SCL_LOW();
	Soft_I2C_Delay();

}

void Soft_I2C_NACK(void)
{

	SOFT_I2C_SCL_LOW();
	Soft_I2C_Delay();
	SOFT_I2C_SDA_HIGH();
	Soft_I2C_Delay();
	SOFT_I2C_SCL_HIGH();
	Soft_I2C_Delay();
	SOFT_I2C_SCL_LOW();
	Soft_I2C_Delay();

}

uint8_t Soft_I2C_Send_Byte(uint8_t byte)
{

	uint8_t count = 8;

	SOFT_I2C_SCL_LOW();
	Soft_I2C_Delay();
	while (count--)
	{
		if (byte & 0x80)
		{
			SOFT_I2C_SDA_HIGH();
		}
		else
		{
			SOFT_I2C_SDA_LOW();
		}

		byte <<= 1;

		SOFT_I2C_SCL_HIGH();
		Soft_I2C_Delay();
		SOFT_I2C_SCL_LOW();
		Soft_I2C_Delay();
	}
	if (Soft_I2C_Wait_ACK() == SOFT_I2C_ERR)
	{
		return SOFT_I2C_ERR;
	}
	return SOFT_I2C_OK;
}

uint8_t Soft_I2C_Receive_Byte(void)
{

	uint8_t i = 8;
	uint8_t ReceivedByte = 0;

	SOFT_I2C_SDA_HIGH();
	while (i--)
	{
		ReceivedByte <<= 1;
		SOFT_I2C_SCL_LOW();
		Soft_I2C_Delay();
		SOFT_I2C_SCL_HIGH();
		Soft_I2C_Delay();
		if (SOFT_I2C_SDA_READ())
		{
			ReceivedByte |= 0x01;
		}
	}
	SOFT_I2C_SCL_LOW();
	return ReceivedByte;

}

uint8_t Soft_I2C_Write_Byte(uint8_t slave_address, uint8_t register_address, uint8_t *byte)
{

	if (Soft_I2C_Start() == SOFT_I2C_ERR)
	{
		Soft_I2C_Stop();
		return SOFT_I2C_ERR;
	}

	if (Soft_I2C_Send_Byte(slave_address) == SOFT_I2C_ERR)
	{
		return SOFT_I2C_ERR;
	}
	if (Soft_I2C_Send_Byte(register_address) == SOFT_I2C_ERR)
	{
		return SOFT_I2C_ERR;
	}

	if (byte)
	{
		if (Soft_I2C_Send_Byte(*byte) == SOFT_I2C_ERR)
		{
			return SOFT_I2C_ERR;
		}
	}

	Soft_I2C_Stop();

	return SOFT_I2C_OK;

}

uint8_t Soft_I2C_Read_Byte(uint8_t slave_address, uint8_t register_address, uint8_t *val)
{

	if (Soft_I2C_Start() == SOFT_I2C_ERR)
	{
		Soft_I2C_Stop();
		return SOFT_I2C_ERR;
	}

	if (Soft_I2C_Send_Byte(slave_address) == SOFT_I2C_ERR)
	{
		return SOFT_I2C_ERR;
	}

	if (Soft_I2C_Send_Byte(register_address) == SOFT_I2C_ERR)
	{
		return SOFT_I2C_ERR;
	}

	if (Soft_I2C_Start() == SOFT_I2C_ERR) //repeated start
	{
		return SOFT_I2C_ERR;
	}

	if (Soft_I2C_Send_Byte(slave_address + 1) == SOFT_I2C_ERR) //read bit
	{
		return SOFT_I2C_ERR;
	}

	*val = Soft_I2C_Receive_Byte();
	Soft_I2C_NACK();

	Soft_I2C_Stop();

	return SOFT_I2C_OK;

}

uint8_t Soft_I2C_Write_Bytes(uint8_t slave_address, uint8_t register_address, uint8_t *buf,
		uint8_t num)
{

	if (Soft_I2C_Start() == SOFT_I2C_ERR)
	{
		Soft_I2C_Stop();
		return SOFT_I2C_ERR;
	}

	if (Soft_I2C_Send_Byte(slave_address) == SOFT_I2C_ERR)
	{
		return SOFT_I2C_ERR;
	}

	if (Soft_I2C_Send_Byte(register_address) == SOFT_I2C_ERR)
	{
		return SOFT_I2C_ERR;
	}

	while (num--)
	{
		if (Soft_I2C_Send_Byte(*buf++) == SOFT_I2C_ERR)
		{
			return SOFT_I2C_ERR;
		}

	}

	Soft_I2C_Stop();

	return SOFT_I2C_OK;

}

uint8_t Soft_I2C_Read_Bytes(uint8_t slave_address, uint8_t register_address, uint8_t *buf,
		uint8_t num)
{

	if (Soft_I2C_Start() == SOFT_I2C_ERR)
	{
		Soft_I2C_Stop();
		return SOFT_I2C_ERR;
	}

	if (Soft_I2C_Send_Byte(slave_address) == SOFT_I2C_ERR)
	{
		return SOFT_I2C_ERR;
	}

	if (Soft_I2C_Send_Byte(register_address) == SOFT_I2C_ERR)
	{
		return SOFT_I2C_ERR;
	}

	if (Soft_I2C_Start() == SOFT_I2C_ERR) //repeated start
	{
		Soft_I2C_Stop();
		return SOFT_I2C_ERR;
	}

	if (Soft_I2C_Send_Byte(slave_address + 1) == SOFT_I2C_ERR) //read bit
	{
		return SOFT_I2C_ERR;
	}

	while (num--)
	{
		*buf++ = Soft_I2C_Receive_Byte();

		if (num == 0)
		{
			Soft_I2C_NACK();
		}
		else
		{
			Soft_I2C_ACK();
		}
	}

	Soft_I2C_Stop();

	return SOFT_I2C_OK;

}

uint8_t Soft_I2C_Scan(uint8_t slave_address)
{

	if (Soft_I2C_Start() == SOFT_I2C_ERR)
	{
		Soft_I2C_Stop();
		return SOFT_I2C_ERR;
	}

	if (Soft_I2C_Send_Byte(slave_address) == SOFT_I2C_ERR)
	{
		return SOFT_I2C_ERR;
	}
	Soft_I2C_Stop();
	return SOFT_I2C_OK;
}
