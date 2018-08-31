/**
******************************************************************************
* @file    Project/main.c 
* @author  MCD Application Team
* @version V2.2.0
* @date    30-September-2014
* @brief   Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software 
* distributed under the License is distributed on an "AS IS" BASIS, 
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************
*/ 


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm8l15x.h"
#include "bq2589x_reg.h"
#include "soft_i2c.h"



/* Private defines -----------------------------------------------------------*/


/* Private variables -----------------------------------------------------------*/

int revision;
bq2589x_part_no part_no;
uint8_t BQ25_ERR_Status=RESET;


/* Private function prototypes -----------------------------------------------*/
void delay_us(uint16_t time);
void delay_ms(uint16_t time);
void BQ25_INT_Callback();


/* Private functions ---------------------------------------------------------*/

void BQ25_INT_Callback()
{
  
}


void main(void)
{
  
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);//16Mhz
  Soft_I2C_Init();
  
  part_no=BQ25892;//clear
  bq2589x_detect_device(&part_no,&revision);
  if(part_no==BQ25895)
  {
    BQ25_ERR_Status=bq2589x_reset_chip();
    BQ25_ERR_Status=bq2589x_set_watchdog_timer(160);
    BQ25_ERR_Status=bq2589x_reset_watchdog_timer();
    BQ25_ERR_Status=bq2589x_set_charge_current(2500); //2.5A  //default 2A
    BQ25_ERR_Status=bq2589x_disable_otg();
    
    
    BQ25_ERR_Status=bq2589x_enable_max_charge(FALSE);
    BQ25_ERR_Status=bq2589x_use_absolute_vindpm(TRUE);// make VINDPM writeable
    BQ25_ERR_Status=bq2589x_set_input_volt_limit(4600);
    
    BQ25_ERR_Status=bq2589x_set_otg_volt(4900);
    BQ25_ERR_Status=bq2589x_set_otg_current(2100);
    
    /************** INPUT ***************************/
    GPIO_Init(BQ25_INT_PORT, BQ25_INT_PIN, GPIO_Mode_In_PU_No_IT);
    //EXTI_SetPinSensitivity(EXTI_Pin_7,EXTI_Trigger_Falling);
  }
  else
  {
    SOFTWARE_RST_MCU();//i2c communication error
  }  
  
  /* Infinite loop */
  while (1)
  {
    
    part_no=BQ25892;//clear
    BQ25_ERR_Status=bq2589x_detect_device(&part_no,&revision);
    if(part_no!=BQ25895)
    {
      SOFTWARE_RST_MCU();
    }
    else
    {
      BQ25_ERR_Status=bq2589x_reset_watchdog_timer();
    }
    
  }
  
}


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


#ifdef USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*   where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval : None
*/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
