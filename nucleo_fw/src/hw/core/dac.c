/*
 *  dac.c
 *
 *  Created on: 2016. 7. 13.
 *      Author: Baram
 */
#include "hw.h"
#include "dac.h"



#define DAC_BUFFER_MAX      (1024*2)





DAC_HandleTypeDef       DacHandle;
TIM_HandleTypeDef       htim;


typedef struct
{
  DAC_ChannelConfTypeDef  sConfig;
  uint32_t                channel;
  uint8_t                 resolution;
  uint8_t                 buffer[DAC_BUFFER_MAX];
} dac_t;



static ring_buf_t tx_buf;
static uint32_t   dac_hz = 0;

static dac_t dac_tbl[DAC_CH_MAX];



int dacCmdif(int argc, char **argv);
void dacInitTimer(uint32_t hz);



void dacInit(void)
{
  uint32_t i;
  uint32_t j;


  cmdifAdd("dac", dacCmdif);

  for (i=0; i<DAC_CH_MAX; i++)
  {
    for (j=0; j<DAC_BUFFER_MAX; j++)
    {
      dac_tbl[i].buffer[j] = 0;
    }
  }

  tx_buf.ptr_in  = 0;
  tx_buf.ptr_out = 0;
  tx_buf.p_buf   = (uint8_t *)dac_tbl[0].buffer;
  tx_buf.length  = DAC_BUFFER_MAX;

}


void dacSetup(uint32_t hz)
{
  dac_hz = hz;
}

void dacStart(void)
{
  DacHandle.Instance = DAC1;

  HAL_DAC_Init(&DacHandle);

  dac_tbl[0].channel    = DAC_CHANNEL_1;
  dac_tbl[0].resolution = 8;


  dac_tbl[0].sConfig.DAC_Trigger      = DAC_TRIGGER_T6_TRGO;
  dac_tbl[0].sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

  HAL_DAC_ConfigChannel(&DacHandle, &dac_tbl[0].sConfig, dac_tbl[0].channel);

  HAL_DAC_Start_DMA(&DacHandle, dac_tbl[0].channel, (uint32_t *)dac_tbl[0].buffer, DAC_BUFFER_MAX, DAC_ALIGN_8B_R);



  dac_tbl[1].channel    = DAC_CHANNEL_2;
  dac_tbl[1].resolution = 8;

  dac_tbl[1].sConfig.DAC_Trigger      = DAC_TRIGGER_T6_TRGO;
  dac_tbl[1].sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

  HAL_DAC_ConfigChannel(&DacHandle, &dac_tbl[1].sConfig, dac_tbl[1].channel);

  HAL_DAC_Start_DMA(&DacHandle, dac_tbl[1].channel, (uint32_t *)dac_tbl[1].buffer, DAC_BUFFER_MAX, DAC_ALIGN_8B_R);


  dacInitTimer(dac_hz);
}

void dacStop(void)
{
  //HAL_TIM_Base_Stop(&htim);
  HAL_DAC_DeInit(&DacHandle);

  tx_buf.ptr_in  = (tx_buf.length - 1) - DacHandle.DMA_Handle1->Instance->CNDTR;;
  tx_buf.ptr_out = (tx_buf.length - 1) - DacHandle.DMA_Handle1->Instance->CNDTR;;
}

void dacInitTimer(uint32_t hz)
{
  TIM_MasterConfigTypeDef sMasterConfig;

  htim.Instance = TIM6;


  htim.Init.Period            = 10-1;
  htim.Init.Prescaler         = (uint32_t)((SystemCoreClock / 1) / (hz*10)) - 1;
  htim.Init.ClockDivision     = 0;
  htim.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;

  HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);

  HAL_TIM_Base_Start(&htim);
}

uint32_t dacAvailable(void)
{
  uint32_t length = 0;


  tx_buf.ptr_in = (tx_buf.length - 1) - DacHandle.DMA_Handle1->Instance->CNDTR;

  //*
  length = ((tx_buf.length + tx_buf.ptr_out) - tx_buf.ptr_in) % tx_buf.length;
  //*/
  length = tx_buf.length - 1 - length;
  //length = DacHandle.DMA_Handle1->Instance->CNDTR;

  return length;
}

uint32_t dacGetDebug(void)
{
  return DacHandle.DMA_Handle1->Instance->CNDTR;
}

void dacPutch(uint8_t data)
{
  uint32_t index;
  volatile uint32_t next_index;


  index      = tx_buf.ptr_out;
  next_index = tx_buf.ptr_out + 1;

  tx_buf.p_buf[index] = data;
  tx_buf.ptr_out      = next_index % tx_buf.length;
}

void dacWrite(uint8_t *p_data, uint32_t length)
{
  uint32_t i;


  for (i=0; i<length; i++)
  {
    dacPutch(p_data[i]);
  }
}

//-- dacCmdif
//
int dacCmdif(int argc, char **argv)
{
  bool ret = true;
  //uint8_t number;


  if (argc == 3)
  {
    //number = (uint8_t) strtoul((const char * ) argv[2], (char **)NULL, (int) 0);

    if(strcmp("on", argv[1]) == 0)
    {
    }
    else if(strcmp("off", argv[1])==0)
    {
    }
    else if(strcmp("toggle", argv[1])==0)
    {
    }
    else if(strcmp("demo", argv[1])==0)
    {
      while(cmdifRxAvailable() == 0)
      {
      }
    }
    else
    {
      ret = false;
    }
  }
  else
  {
    ret = false;
  }

  if (ret == false)
  {
    cmdifPrintf( "dac on/off/toggle/demo number ...\n");
  }

  return 0;
}

volatile uint32_t dac_isr_count = 0;

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
  dac_isr_count++;
}

void DMA1_Channel3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(DacHandle.DMA_Handle1);
}

void DMA1_Channel4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(DacHandle.DMA_Handle2);
}


void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac)
{
  GPIO_InitTypeDef          GPIO_InitStruct;
  static DMA_HandleTypeDef  hdma_dac1;
  static DMA_HandleTypeDef  hdma_dac2;


  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock ****************************************/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /* DAC Periph clock enable */
  __HAL_RCC_DAC1_CLK_ENABLE();
  /* DMA1 clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* DAC Channel1 GPIO pin configuration */
  GPIO_InitStruct.Pin   = GPIO_PIN_4;
  GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*##-3- Configure the DMA ##########################################*/
  /* Set the parameters to be configured for DACx_DMA1_CHANNEL3 */
  hdma_dac1.Instance                  = DMA1_Channel3;

  hdma_dac1.Init.Request              = DMA_REQUEST_6;

  hdma_dac1.Init.Direction            = DMA_MEMORY_TO_PERIPH;
  hdma_dac1.Init.PeriphInc            = DMA_PINC_DISABLE;
  hdma_dac1.Init.MemInc               = DMA_MINC_ENABLE;
  hdma_dac1.Init.PeriphDataAlignment  = DMA_PDATAALIGN_BYTE;
  hdma_dac1.Init.MemDataAlignment     = DMA_MDATAALIGN_BYTE;
  hdma_dac1.Init.Mode                 = DMA_CIRCULAR;
  hdma_dac1.Init.Priority             = DMA_PRIORITY_HIGH;

  HAL_DMA_Init(&hdma_dac1);

  /* Associate the initialized DMA handle to the DAC handle */
  __HAL_LINKDMA(hdac, DMA_Handle1, hdma_dac1);

  /*##-4- Configure the NVIC for DMA #########################################*/
  /* Enable the DMA1_Channel3 IRQ Channel */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);






  /*##-2- Configure peripheral GPIO ##########################################*/
  /* DAC Channel1 GPIO pin configuration */
  GPIO_InitStruct.Pin   = GPIO_PIN_5;
  GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*##-3- Configure the DMA ##########################################*/
  /* Set the parameters to be configured for DACx_DMA1_CHANNEL3 */
  hdma_dac2.Instance                  = DMA1_Channel4;

  hdma_dac2.Init.Request              = DMA_REQUEST_5;

  hdma_dac2.Init.Direction            = DMA_MEMORY_TO_PERIPH;
  hdma_dac2.Init.PeriphInc            = DMA_PINC_DISABLE;
  hdma_dac2.Init.MemInc               = DMA_MINC_ENABLE;
  hdma_dac2.Init.PeriphDataAlignment  = DMA_PDATAALIGN_BYTE;
  hdma_dac2.Init.MemDataAlignment     = DMA_MDATAALIGN_BYTE;
  hdma_dac2.Init.Mode                 = DMA_CIRCULAR;
  hdma_dac2.Init.Priority             = DMA_PRIORITY_HIGH;

  HAL_DMA_Init(&hdma_dac2);

  /* Associate the initialized DMA handle to the DAC handle */
  __HAL_LINKDMA(hdac, DMA_Handle2, hdma_dac2);

  /*##-4- Configure the NVIC for DMA #########################################*/
  /* Enable the DMA1_Channel3 IRQ Channel */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief  DeInitializes the DAC MSP.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
void HAL_DAC_MspDeInit(DAC_HandleTypeDef *hdac)
{
  __HAL_RCC_DAC1_FORCE_RESET();
  __HAL_RCC_DAC1_RELEASE_RESET();

  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);

  HAL_DMA_DeInit(hdac->DMA_Handle1);
  HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);

  HAL_DMA_DeInit(hdac->DMA_Handle2);
  HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);

}


