
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"

/* USER CODE BEGIN Includes */
#include "printf.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
long i=0;
long j=0;
long k=0;
long l=0;
long m=0;
long n=0;
unsigned long a;

char buff[10];

unsigned long zero=33220;
int sw=0;
int flag=0;
int count=0;

unsigned long adc1[1024];//原始波形
float avg1[600];//滑动平均后的平滑波形
float cut1[200];//从过零点开始的一个周期
int position1[10]={0};//过零点位置


unsigned long adc0[1024];
float avg0[600];
float cut0[200];
int position0[10]={0};


float A=0.0;
float B=0.0;
float AB=0.0;
float cos2=0.0;//互相关系数平方
int start=0;
int har=0;
float AB1=0.0;
float AB2=0.0;
int tim_count=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_DIFFERENTIAL_ENDED);
  
  HAL_Delay(10000);
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	
                
        //float2char((float)test,buff,10);
                //for(i=0;i<10;i++){U2TXREG=buff[i];while(U2STAbits.TRMT==0);}U2TXREG='\r';while(U2STAbits.TRMT==0);U2TXREG='\n';while(U2STAbits.TRMT==0);
        //LATBbits.LATB7=~LATBbits.LATB7; //led1
		HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
        //if(PORTCbits.RC7==0 || PORTCbits.RC6==0){LATBbits.LATB1=0;}
        j=0;
        sw=!sw;//选择adc1还是adc0
        flag=1;//等待1024个点采样完成
        while(flag==1);
        j=0;

        for(i=0;i<600;i++){avg1[i]=(adc1[i]+adc1[i+1]+adc1[i+2]+adc1[i+3]+adc1[i+4])/5.0;}//滑动平均使波形平滑，放到avg1[]
        for(i=0;i<600;i++){avg0[i]=(adc0[i]+adc0[i+1]+adc0[i+2]+adc0[i+3]+adc0[i+4])/5.0;}//滑动平均使波形平滑，放到avg0[]
        
        m=0;
    if(sw==1)//若是adc1[]
        {
            for(l=4;l<420;l++)
            {
                if((avg1[l-2]<zero) && (avg1[l-1]<zero) && (avg1[l+5]>zero) && (avg1[l+6]>zero))
                {position1[m]=l;//记录过零点位置
                m++;
                l=l+40;}
                if(m==2){if((position1[1]-position1[0])<90){har=1;HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);} }//当过零点间距太小，则认为有较大谐波，har=1
                if(m==3) {m=0;break;}
            }
            for(i=0;i<200;i++){cut1[i]=avg1[position1[0]+i];}//从过零点开始截取avg1[]的一段，放入cut1[200]
        }
        else//若是adc0[]
        {
                for(l=4;l<420;l++)
            {
                if((avg0[l-2]<zero) && (avg0[l-1]<zero) && (avg0[l+5]>zero) && (avg0[l+6]>zero))
                {position0[m]=l;//记录过零点位置
                m++;
                l=l+40;}
                if(m==1) {m=0;break;}
            }
        for(i=0;i<200;i++){cut0[i]=avg0[position0[0]+i];}//从过零点开始截取avg0[]的一段，放入cut0[200]
    
    
    }


            for(j=0;j<1024;j++){adc1[j]=0;adc0[j]=0;}//清空adc[]
      
            for(i=0;i<200;i++){A+=(cut0[i]-zero)*(cut0[i]-zero);B+=(cut1[i]-zero)*(cut1[i]-zero);AB+=(cut0[i]-zero)*(cut1[i]-zero);}//求A、B、AB

        if(har==1)//若有较大谐波
        {
            for(i=0;i<(200-position1[1]+position1[0]);i++){AB1+=(cut0[i]-zero)*(cut1[i+position1[1]-position1[0]]-zero);}//移动cut1[]至第2个过零点再计算互相关系数AB1
            for(i=0;i<(position1[1]-position1[0]);i++){AB1+=(cut0[i+200-position1[1]+position1[0]]-zero)*(cut1[i]-zero);}

            for(i=0;i<(200-position1[2]+position1[0]);i++){AB1+=(cut0[i]-zero)*(cut1[i+position1[2]-position1[0]]-zero);}//移动cut1[]至第3个过零点再计算互相关系数AB2
            for(i=0;i<(position1[2]-position1[0]);i++){AB1+=(cut0[i+200-position1[2]+position1[0]]-zero)*(cut1[i]-zero);}
            
            if(AB1>AB)AB=AB1;//取AB、AB1和AB2中的最大值
            if(AB2>AB)AB=AB2;
        }

                
                cos2=AB*AB/A/B;
//                float2char(cos2,buff,10);
//                for(i=0;i<10;i++){U2TXREG=buff[i];while(U2STAbits.TRMT==0);}U2TXREG='\r';while(U2STAbits.TRMT==0);U2TXREG='\n';while(U2STAbits.TRMT==0);  //发送相似度平方
               
				printf("cos2:%f\r\n",cos2);

				if((cos2<0.7) && (start==1))//start表示有新旧数据，可以进行比较
                {
                    HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_SET); //LATBbits.LATB8=1;//led3
                    for(j=0;j<200;j++)//发送cut1[]
                    {      
//                        float2char(cut1[j],buff,10);
//                        for(i=0;i<9;i++){U2TXREG=buff[i];while(U2STAbits.TRMT==0);}
//                        U2TXREG='\r';while(U2STAbits.TRMT==0);
//                        U2TXREG='\n';while(U2STAbits.TRMT==0);
						
						printf("cut1[%l]:%f\t",j,cut1[j]);
						
                    } 
                    HAL_Delay(1000);//__delay32(150000000);
                    for(j=0;j<200;j++)//发送cut0[]
                    {      
//                        float2char(cut0[j],buff,10);
//                        for(i=0;i<9;i++){U2TXREG=buff[i];while(U2STAbits.TRMT==0);}
//                        U2TXREG='\r';while(U2STAbits.TRMT==0);
//                        U2TXREG='\n';while(U2STAbits.TRMT==0);
//						
						  printf("cut0[%l]:%f\t",j,cut1[j]);
						
                    } 
                    HAL_Delay(1000);// __delay32(150000000);
                    //发送A、B、AB、AB1、AB2
//                    float2char(A,buff,10);
//                    for(i=0;i<10;i++){U2TXREG=buff[i];while(U2STAbits.TRMT==0);}U2TXREG='\r';while(U2STAbits.TRMT==0);U2TXREG='\n';while(U2STAbits.TRMT==0);
//                    float2char(B,buff,10);
//                    for(i=0;i<10;i++){U2TXREG=buff[i];while(U2STAbits.TRMT==0);}U2TXREG='\r';while(U2STAbits.TRMT==0);U2TXREG='\n';while(U2STAbits.TRMT==0);
//                    float2char(AB,buff,10);
//                    for(i=0;i<10;i++){U2TXREG=buff[i];while(U2STAbits.TRMT==0);}U2TXREG='\r';while(U2STAbits.TRMT==0);U2TXREG='\n';while(U2STAbits.TRMT==0);
//                    __delay32(150000000);
//                    float2char(AB1,buff,10);
//                    for(i=0;i<10;i++){U2TXREG=buff[i];while(U2STAbits.TRMT==0);}U2TXREG='\r';while(U2STAbits.TRMT==0);U2TXREG='\n';while(U2STAbits.TRMT==0);
//                    float2char(AB2,buff,10);
//                    for(i=0;i<10;i++){U2TXREG=buff[i];while(U2STAbits.TRMT==0);}U2TXREG='\r';while(U2STAbits.TRMT==0);U2TXREG='\n';while(U2STAbits.TRMT==0);
//                    //发送cut1[]的过零点位置
//                    float2char((float)position1[0],buff,10);
//                    for(i=0;i<10;i++){U2TXREG=buff[i];while(U2STAbits.TRMT==0);}U2TXREG='\r';while(U2STAbits.TRMT==0);U2TXREG='\n';while(U2STAbits.TRMT==0);
//                    float2char((float)position1[1],buff,10);
//                    for(i=0;i<10;i++){U2TXREG=buff[i];while(U2STAbits.TRMT==0);}U2TXREG='\r';while(U2STAbits.TRMT==0);U2TXREG='\n';while(U2STAbits.TRMT==0); 
//                    float2char((float)position1[2],buff,10);
//                    for(i=0;i<10;i++){U2TXREG=buff[i];while(U2STAbits.TRMT==0);}U2TXREG='\r';while(U2STAbits.TRMT==0);U2TXREG='\n';while(U2STAbits.TRMT==0);
//                    float2char((float)position1[3],buff,10);
//                    for(i=0;i<10;i++){U2TXREG=buff[i];while(U2STAbits.TRMT==0);}U2TXREG='\r';while(U2STAbits.TRMT==0);U2TXREG='\n';while(U2STAbits.TRMT==0); 
					  printf("A:%f\tB:%f\tAB:%f\tAB1:%f\tAB2:%f\r\n",A,B,AB,AB1,AB2);	
					  printf("position[0]:%i\tposition[1]:%i\tposition[2]:%i\tposition[3]:%i\r\n",position1[0],position1[1],position1[2],position1[3]);
				}	
                                //float2char((float)position1[0],buff,10);
                //for(i=0;i<10;i++){U2TXREG=buff[i];while(U2STAbits.TRMT==0);}U2TXREG='\r';while(U2STAbits.TRMT==0);U2TXREG='\n';while(U2STAbits.TRMT==0);
                A=0.0;B=0.0;AB=0.0;cos2=0.0;//清零
                AB1=0.0;AB2=0.0;
                if(sw==0)har=0;
                start=1;//已有旧数据

    

	

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Supply configuration update enable 
    */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) 
  {
    
  }
    /**Macro to configure the PLL clock source 
    */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 10;
  PeriphClkInitStruct.PLL2.PLL2P = 1;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(SystemCoreClock/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.BoostMode = ENABLE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.NbrOfDiscConversion = 1;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.BoostMode = ENABLE;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC3 init function */
static void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 3;
  hadc3.Init.DiscontinuousConvMode = ENABLE;
  hadc3.Init.NbrOfDiscConversion = 1;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.BoostMode = ENABLE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 200;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.Prescaler = UART_PRESCALER_DIV1;
  huart2.Init.FIFOMode = UART_FIFOMODE_DISABLE;
  huart2.Init.TXFIFOThreshold = UART_TXFIFO_THRESHOLD_1_8;
  huart2.Init.RXFIFOThreshold = UART_RXFIFO_THRESHOLD_1_8;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.Prescaler = UART_PRESCALER_DIV1;
  huart3.Init.FIFOMode = UART_FIFOMODE_DISABLE;
  huart3.Init.TXFIFOThreshold = UART_TXFIFO_THRESHOLD_1_8;
  huart3.Init.RXFIFOThreshold = UART_RXFIFO_THRESHOLD_1_8;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BEE_GPIO_Port, BEE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED8_Pin|LED9_Pin|LED10_Pin|LED3_Pin 
                          |LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, KM1_Pin|KM2_Pin|KM3_Pin|KM4_Pin 
                          |KM5_Pin|KM6_Pin|KM7_Pin|KM8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : K1_Pin */
  GPIO_InitStruct.Pin = K1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(K1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : K2_Pin RE_Pin */
  GPIO_InitStruct.Pin = K2_Pin|RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : K3_Pin K4_Pin K5_Pin K6_Pin 
                           K7_Pin K8_Pin */
  GPIO_InitStruct.Pin = K3_Pin|K4_Pin|K5_Pin|K6_Pin 
                          |K7_Pin|K8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : BEE_Pin */
  GPIO_InitStruct.Pin = BEE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BEE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED8_Pin LED9_Pin LED10_Pin LED3_Pin 
                           LED4_Pin LED5_Pin LED6_Pin LED7_Pin */
  GPIO_InitStruct.Pin = LED8_Pin|LED9_Pin|LED10_Pin|LED3_Pin 
                          |LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : KM1_Pin KM2_Pin KM3_Pin KM4_Pin 
                           KM5_Pin KM6_Pin KM7_Pin KM8_Pin */
  GPIO_InitStruct.Pin = KM1_Pin|KM2_Pin|KM3_Pin|KM4_Pin 
                          |KM5_Pin|KM6_Pin|KM7_Pin|KM8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)		// 判断是由哪个定时器触发的中断
	{
	
		tim_count++;
		if(tim_count==1000){HAL_GPIO_TogglePin(LED10_GPIO_Port,LED10_Pin);tim_count=0;printf("*****");}
		 if(sw==1)
        {
            adc1[j]+=HAL_ADC_GetValue(&hadc1);
            count++;
            if(count==6)
            {
            count=0;
            adc1[j]/=6;
            j++;
            }
            if(j==1024)flag=0;
        }
        else
        {
        adc0[j]+=HAL_ADC_GetValue(&hadc1);
        count++;
        if(count==6)
        {
        count=0;
        adc0[j]/=6;
        j++;
        }
        if(j==1024)flag=0;
        }
	}

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
