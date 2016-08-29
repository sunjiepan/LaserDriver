#include <stdio.h>
#include "laser_ctl.h"
#include "stm32f0xx_hal.h"
#include "Queue.h"
#include "eeprom.h"

static uint16_t buzzer_interval = 50;
static uint16_t set_temp_value = 4000;
static uint16_t current_temp = 0;
static uint8_t switchOnOff = 1;

static QUEUE8_t m_QueueComRx         = {0};
static QUEUE8_t m_QueueComTx         = {0};
static uint8_t  m_ComRxBuf[COM_RX_BUF_SIZE]      = {0};     
static uint8_t  m_ComTxBuf[COM_TX_BUF_SIZE]      = {0}; 

	/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777};
uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0};
uint16_t VarValue = 0;

static PID_TypeDef pid={
	0,0,0,0,0,10,0,0.1,0,400
};



void Init(void)
{
	QUEUE_PacketCreate(&m_QueueComRx, m_ComRxBuf, sizeof(m_ComRxBuf));
  QUEUE_PacketCreate(&m_QueueComTx, m_ComTxBuf, sizeof(m_ComTxBuf));
	
//	/* Unlock the Flash Program Erase controller */
//  HAL_FLASH_Unlock();
//	
//	/* EEPROM Init */
//  EE_Init();

//  /* read the last stored variables data*/
//  EE_ReadVariable(VirtAddVarTab[0], &VarDataTab[0]);
//	
//	HAL_FLASH_Lock();
//	
//	set_temp_value = VarDataTab[0];
	

}

void SetBuzzer(uint16_t interval)
{
	buzzer_interval = interval;
}

	
void BuzzerOutPut(void)
{
	if(buzzer_interval)
	{
		//BuzzerON();
		/* Insert delay n ms */
		HAL_Delay(buzzer_interval);		
	}
	//BuzzerOFF();
}

void SetTemp(uint16_t temp)
{
	/* Unlock the Flash Program Erase controller */
//  HAL_FLASH_Unlock();
//  /* Store 0x1000 values of Variable1 in EEPROM */
//	EE_WriteVariable(VirtAddVarTab[0], temp);
//	HAL_FLASH_Lock();
	set_temp_value = temp;
}

uint16_t GetTemp(void)
{
	return set_temp_value;
}

uint16_t GetCurrentTemp(void)
{
	//current_temp = BSP_DS18B20_Get_Temp();
	return current_temp;
}

uint16_t InserAver(uint16_t *array, uint16_t n)
{
	int32_t sum = 0;
	uint16_t i = 0;
	int16_t max=0x8000;
	int16_t min=0x7FFF;
	
	for(i= 0; i<n; i++)
	{
		if(array[i]>max) max=array[i];
		if(array[i]<min) min=array[i];
		sum += array[i];
	}
	sum=(sum-max-min)/(n-2);
	
	return sum;
}

uint16_t ReadCurrentTemp(void)
{
	uint16_t temp[10];
	
	for(int i = 0; i < 10; i++)
	{
		//temp[i] = BSP_DS18B20_Get_Temp();
	}	
	current_temp = InserAver(temp,10);
	
	return current_temp;
}

void SwitchOnOff(uint8_t onOff)
{
	TIM_HandleTypeDef htim3;
	htim3.Instance = TIM3;
	switchOnOff = onOff;
	
	if(switchOnOff == 0)
		HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
	else
		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
}

void PWMOutPut(uint16_t duty_cycle)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
	TIM_HandleTypeDef htim3;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000 - 1; //F = SYSCLK/(( Prescaler + 1)*( Period + 1)); 8000K/(( 7  +  1) * ( 999 + 1))
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = duty_cycle;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
	
	if(switchOnOff == 0)
		HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
	else
		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
}

/*******************************************************************************
* Function Name : uint32_t RxRead(uint8_t *buffter, uint32_t buffterSize)
* Description   : 从接收缓存中读数据
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
uint32_t RxRead(uint8_t *buffter, uint32_t buffterSize)
{
	uint32_t tempLen;

	tempLen = QUEUE_PacketStartEndCharSplit(&m_QueueComRx, START_CHAR, buffter, buffterSize);
		
	return tempLen;
}
/*******************************************************************************
* Function Name : uint32_t RxWrite(uint8_t *buffter, uint32_t writeLen)
* Description   : 写数据到接收缓存中
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
uint32_t RxWrite(uint8_t *buffter, uint32_t writeLen)
{
    return QUEUE_PacketIn(&m_QueueComRx, buffter, writeLen);
}
/*******************************************************************************
* Function Name : uint32_t TxRead(uint8_t *buffter, uint32_t buffterSize)
* Description   : 从发送缓存中读数据
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
uint32_t TxRead(uint8_t *buffter, uint32_t buffterSize)
{
	uint32_t tempLen;
	
	tempLen = QUEUE_PacketLengthGet(&m_QueueComTx);
	QUEUE_PacketOut(&m_QueueComTx, buffter, tempLen);
	
	return tempLen;
}
/*******************************************************************************
* Function Name : uint32_t TxWrite(uint8_t *buffter, uint32_t writeLen)
* Description   : 写数据到发送缓存中
* Input         : 
* Output        : 
* Other         : 
* Date          : 2014.11.28
*******************************************************************************/
uint32_t TxWrite(uint8_t *buffter, uint32_t writeLen)
{
    return QUEUE_PacketIn(&m_QueueComTx, buffter, writeLen);
}



HAL_StatusTypeDef UartTransmit(uint8_t *aTxBuffer)
{
	return HAL_UART_Transmit_DMA(&huart1, (uint8_t*)aTxBuffer, sizeof(CustomProtocol_TypeDef));	
}
HAL_StatusTypeDef UartReceive(uint8_t *aRxBuffer)
{
	return HAL_UART_Receive_DMA(&huart1, (uint8_t*)aRxBuffer, sizeof(CustomProtocol_TypeDef));
}

uint16_t PID_Algorithm(uint16_t setValue, uint16_t feedbackValue)
{	
	pid.setValue = setValue; 
	pid.feedbackValue = feedbackValue;
	
	if(setValue < 1000)
	{
		setValue = 1000;
	}
	else if(setValue > 10000)
	{
		setValue = 10000;
	}
	
	if(feedbackValue <= 0)
	{
		feedbackValue = 0;
	}
	else if(feedbackValue > 10000)
	{
		feedbackValue = 10000;
	}
	pid.err = pid.setValue - pid.feedbackValue;
	if(pid.err > MAX_ERROR_VALUE)
	{
		pid.out = MAX_DUTY_CYCLE;
	}
	else if(pid.err > MIN_ERROR_VALUE && pid.err < MAX_ERROR_VALUE)
	{
		pid.out = pid.outNext + pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last);         
		pid.err_last = pid.err_next;     
		pid.err_next = pid.err;
		pid.outNext = pid.out;
		if(pid.out < MIN_DUTY_CYCLE)
		{
			pid.out = MIN_DUTY_CYCLE;
		}
		else if(pid.out > MAX_DUTY_CYCLE)
		{
			pid.out = MAX_DUTY_CYCLE;
		}
	}
	else if(pid.err < 0)
	{
		pid.out = MIN_DUTY_CYCLE;
	}
	
	return pid.out;
}




uint8_t getCrc8(uint8_t *data)
{
   uint8_t crc = 0;
   uint8_t length = 3;
   uint8_t i, j;
   for(i=0; i<length; i++) {
      crc ^= data[i];
      for(j=0; j<8; j++) {
         if(crc & 1) 
					 crc = (uint8_t)((crc >> 1) ^ ((1 << 7) + (1 << 6) + (1 << 5)));
         else crc >>= 1;
      }
   }
   return crc;
}


LaserCtl_TypeDef laser_ctl = {
	Init,
	SetTemp,
	GetTemp,
	GetCurrentTemp,
	ReadCurrentTemp,
	SwitchOnOff,
	PWMOutPut,
	UartTransmit,
	UartReceive,
	PID_Algorithm,
	getCrc8,
	{0,0,0,0,0},
	{0,0,0,0,0},
};

