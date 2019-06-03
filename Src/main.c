/* USER CODE BEGIN Header */
/**
 ******************************************************************************
                                                                  88
                                                                  ""

8b,dPPYba,  ,adPPYba, 8b       d8  ,adPPYba, 8b,dPPYba, ,adPPYba, 88
88P'   "Y8 a8P_____88 `8b     d8' a8P_____88 88P'   "Y8 I8[    "" 88
88         8PP"""""""  `8b   d8'  8PP""""""" 88          `"Y8ba,  88
88         "8b,   ,aa   `8b,d8'   "8b,   ,aa 88         aa    ]8I 88
88          `"Ybbd8"'     "8"      `"Ybbd8"' 88         `"YbbdP"' 88

 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f769i_discovery.h"
#include "stm32f769i_discovery_lcd.h"
#include "stm32f769i_discovery_ts.h"
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {MENU, SINGLE, MULTI,SCORE,RULES } states; //estados do switch do menu
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEMP_REFRESH_PERIOD    250    /* Internal temperature refresh period */
#define MAX_CONVERTED_VALUE   4095    /* Max converted value */
#define AMBIENT_TEMP            25    /* Ambient Temperature */
#define VSENS_AT_AMBIENT_TEMP  760    /* VSENSE value (mv) at ambient temperature */
#define AVG_SLOPE               25    /* Avg_Solpe multiply by 10 */
#define VREF                  3300

#define UPLEFT 					0	//variaveis de direção
#define UP 						1
#define UPRIGHT					2
#define LEFT 					3
#define RIGHT 					4
#define DOWNLEFT 				5
#define DOWN 					6
#define DOWNRIGHT 				7

#define DIM  					8 //dimensão da matriz

#define FALSE 0
#define TRUE  1

#define NoPossMov	         	LCD_COLOR_LIGHTGRAY //cor das jogadas nao possiveis
#define PossMov					LCD_COLOR_DARKGRAY  //cor das jogadas possiveis
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DMA2D_HandleTypeDef hdma2d;

DSI_HandleTypeDef hdsi;

LTDC_HandleTypeDef hltdc;

SD_HandleTypeDef hsd2;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim13;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
volatile uint8_t flagTemperatura = 0;
volatile uint8_t flagTouch=0; //flag da interrupção do touch screen
volatile uint8_t flagPrintMenu=0; // flag do push button que retorna ao menu
volatile uint8_t counter = 0; // contador para o tempo total de jogo
volatile uint8_t flagCounter20=0; // flag da contagem decrescente para passar jogada
volatile uint8_t counterPlay = 20; //inicialização do contador dos 20 segundos
volatile uint8_t numberOfPlays=0; // numero de jogadas
volatile uint8_t ScorePlayer1=0;
volatile uint8_t ScorePlayer2=0;
volatile uint8_t counterPossibleMove=0; // conta jogadas possiveis
volatile uint8_t counterPass1=0; //conta o numero de "passagens" do player 1
volatile uint8_t counterPass2=0;
long int JTemp = 0;
char string[100]; //"aloca" informação ao longo do programa
uint32_t ConvertedValue;
TS_StateTypeDef TS_State; //estrutura que indica posição do touch
volatile int player = 1;
unsigned int nBytes;
states stadus; // nome - enum
bool compass[8]; // array de boleanos que confirmam existencia de jogadas possiveis
uint16_t cX = 0;	//X coordinate
uint16_t cY = 0;	//Y coordinate
uint32_t color1 = LCD_COLOR_BLUE;
uint32_t color2 = LCD_COLOR_YELLOW;
bool victor; // se foi vitoria ou nao
volatile uint8_t updateDisplay=0;  //variavel para ocupar o processador, para fazer refresh do display
int board[DIM][DIM]={{0}}; //board virtual
char stringPlayer[30]; //jogador vencedor


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA2D_Init(void);
static void MX_DSIHOST_DSI_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_SDMMC2_SD_Init(void);
static void MX_TIM13_Init(void);
/* USER CODE BEGIN PFP */
static void LCD_Config();
static void LCD_Config2();
void printBoardGame();
void printScoreTable();
void TempCalc ();
void  putCircle(uint16_t *cX, uint16_t *cY);
void checkPlace(uint16_t*, uint16_t*);
void firstPlays ();
void printMenu();
void touchMenu();
void checkPossibleMoves();
void ReadFromSD ();
void printTemperatura ();
void printTotalTime ();
void writeGameInfoSD ();
void clearPrePossMov();
//void putplay(uint16_t *cX, uint16_t *cY, uint32_t color1, uint32_t color2);
void counterScore();
void blockMove (uint16_t *cX, uint16_t *cY);
void reverse();
void readRules();
void pass3times();
void setToMemory ();
void setMemoryToBoard();
void returnTimeZero();
void printToScreen(void);
void initBoard(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == GPIO_PIN_13){ // interrupt do touch screen
		BSP_TS_GetState(&TS_State); //recebe valor de touch
		flagTouch=1;			//flag "acende" quando há toque no ecrã
		HAL_Delay(100); // para impedir toques consecutivos
	}
	if(GPIO_Pin == GPIO_PIN_0){ // interrupt do botao
		flagPrintMenu=1;
	}
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM7)  // timer confg 2 segundos - temperatura
		flagTemperatura=1;

	if(htim->Instance == TIM6)
	{
		//timer confg a 1segundo - contagem global do tempo
		counter++;
		updateDisplay=1;
	}

	if(htim->Instance == TIM13) //timer confg a 1segundo - contagem tempo max da jogada
	{
		flagCounter20=1;
		counterPlay--;
		if(counterPlay==0)
		{
			counterPlay=20;
		}

	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */



  /* USER CODE END 1 */
  

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA2D_Init();
  MX_DSIHOST_DSI_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_SDMMC2_SD_Init();
  MX_FATFS_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
	BSP_LED_Init(LED_GREEN);
	BSP_LED_Init(LED_RED);
	HAL_ADC_Start(&hadc1);
	HAL_TIM_Base_Start_IT(&htim7);
	BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
	BSP_TS_ITConfig();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	//writeGameInfoSD();
	stadus= MENU;
	LCD_Config();
	printMenu();

	while (1)
	{
		if(flagTemperatura)
		{
			TempCalc ();
		}
		printTemperatura ();

		switch (stadus) {

			case SINGLE:
				break;

			case MULTI:
				if(updateDisplay==1)

				{
					updateDisplay=0;

					printScoreTable();
					firstPlays ();
					HAL_TIM_Base_Start_IT(&htim6);
					HAL_TIM_Base_Start_IT(&htim13);
					printTotalTime ();
					clearPrePossMov();
				}

				if (flagTouch)
				{
					flagTouch=0;

					checkPlace(&cX, &cY);
					blockMove(cX, cY);
					setToMemory ();
					reverse();
					setMemoryToBoard ();
					returnTimeZero();
				}

				//writeGameInfoSD ();
				break;

			case SCORE:
				//ReadFromSD ();
				break;

			case RULES:
				if(updateDisplay==1)

				{
					updateDisplay=0;

					BSP_LCD_Clear(LCD_COLOR_WHITE);
					readRules();
				}

				break;


			case MENU:
				if (flagTouch)
				{
					flagTouch = 0;
					touchMenu();
					updateDisplay=1;
					BSP_LCD_Clear(LCD_COLOR_WHITE);
					//FIXME: apagar isto daqui?
					printBoardGame();
				}
				break;
			}
		if(flagPrintMenu)
		{
			flagPrintMenu = 0;
			BSP_LCD_Clear(LCD_COLOR_WHITE);
			printMenu();
			printTemperatura();
			stadus=MENU;
		}


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	}//end of while


  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_SDMMC2
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc2ClockSelection = RCC_SDMMC2CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief DSIHOST Initialization Function
  * @param None
  * @retval None
  */
static void MX_DSIHOST_DSI_Init(void)
{

  /* USER CODE BEGIN DSIHOST_Init 0 */

  /* USER CODE END DSIHOST_Init 0 */

  DSI_PLLInitTypeDef PLLInit = {0};
  DSI_HOST_TimeoutTypeDef HostTimeouts = {0};
  DSI_PHY_TimerTypeDef PhyTimings = {0};
  DSI_LPCmdTypeDef LPCmd = {0};
  DSI_CmdCfgTypeDef CmdCfg = {0};

  /* USER CODE BEGIN DSIHOST_Init 1 */

  /* USER CODE END DSIHOST_Init 1 */
  hdsi.Instance = DSI;
  hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  hdsi.Init.TXEscapeCkdiv = 4;
  hdsi.Init.NumberOfLanes = DSI_ONE_DATA_LANE;
  PLLInit.PLLNDIV = 20;
  PLLInit.PLLIDF = DSI_PLL_IN_DIV1;
  PLLInit.PLLODF = DSI_PLL_OUT_DIV1;
  if (HAL_DSI_Init(&hdsi, &PLLInit) != HAL_OK)
  {
    Error_Handler();
  }
  HostTimeouts.TimeoutCkdiv = 1;
  HostTimeouts.HighSpeedTransmissionTimeout = 0;
  HostTimeouts.LowPowerReceptionTimeout = 0;
  HostTimeouts.HighSpeedReadTimeout = 0;
  HostTimeouts.LowPowerReadTimeout = 0;
  HostTimeouts.HighSpeedWriteTimeout = 0;
  HostTimeouts.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE;
  HostTimeouts.LowPowerWriteTimeout = 0;
  HostTimeouts.BTATimeout = 0;
  if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK)
  {
    Error_Handler();
  }
  PhyTimings.ClockLaneHS2LPTime = 28;
  PhyTimings.ClockLaneLP2HSTime = 33;
  PhyTimings.DataLaneHS2LPTime = 15;
  PhyTimings.DataLaneLP2HSTime = 25;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 0;
  if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigFlowControl(&hdsi, DSI_FLOW_CONTROL_BTA) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetLowPowerRXFilter(&hdsi, 10000) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigErrorMonitor(&hdsi, HAL_DSI_ERROR_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  LPCmd.LPGenShortWriteNoP = DSI_LP_GSW0P_DISABLE;
  LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_DISABLE;
  LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_DISABLE;
  LPCmd.LPGenShortReadNoP = DSI_LP_GSR0P_DISABLE;
  LPCmd.LPGenShortReadOneP = DSI_LP_GSR1P_DISABLE;
  LPCmd.LPGenShortReadTwoP = DSI_LP_GSR2P_DISABLE;
  LPCmd.LPGenLongWrite = DSI_LP_GLW_DISABLE;
  LPCmd.LPDcsShortWriteNoP = DSI_LP_DSW0P_DISABLE;
  LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_DISABLE;
  LPCmd.LPDcsShortReadNoP = DSI_LP_DSR0P_DISABLE;
  LPCmd.LPDcsLongWrite = DSI_LP_DLW_DISABLE;
  LPCmd.LPMaxReadPacket = DSI_LP_MRDP_DISABLE;
  LPCmd.AcknowledgeRequest = DSI_ACKNOWLEDGE_DISABLE;
  if (HAL_DSI_ConfigCommand(&hdsi, &LPCmd) != HAL_OK)
  {
    Error_Handler();
  }
  CmdCfg.VirtualChannelID = 0;
  CmdCfg.ColorCoding = DSI_RGB888;
  CmdCfg.CommandSize = 640;
  CmdCfg.TearingEffectSource = DSI_TE_EXTERNAL;
  CmdCfg.TearingEffectPolarity = DSI_TE_RISING_EDGE;
  CmdCfg.HSPolarity = DSI_HSYNC_ACTIVE_LOW;
  CmdCfg.VSPolarity = DSI_VSYNC_ACTIVE_LOW;
  CmdCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  CmdCfg.VSyncPol = DSI_VSYNC_FALLING;
  CmdCfg.AutomaticRefresh = DSI_AR_ENABLE;
  CmdCfg.TEAcknowledgeRequest = DSI_TE_ACKNOWLEDGE_DISABLE;
  if (HAL_DSI_ConfigAdaptedCommandMode(&hdsi, &CmdCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetGenericVCID(&hdsi, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DSIHOST_Init 2 */

  /* USER CODE END DSIHOST_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 654;
  hltdc.Init.AccumulatedActiveH = 485;
  hltdc.Init.TotalWidth = 660;
  hltdc.Init.TotalHeigh = 487;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SDMMC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC2_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC2_Init 0 */

  /* USER CODE END SDMMC2_Init 0 */

  /* USER CODE BEGIN SDMMC2_Init 1 */

  /* USER CODE END SDMMC2_Init 1 */
  hsd2.Instance = SDMMC2;
  hsd2.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd2.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd2.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd2.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd2.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC2_Init 2 */

  /* USER CODE END SDMMC2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 19999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 9999;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 9999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_32;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin : PI13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PI15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/*------------------------------------------------------------------
 * Função chamada na main, ocorre quando o timer 7 faz uma interrupção (a cada 2 segundos) e
 * flagTemperatura é colocada a 1.
 * zera novamente a flag e obtem o valor
 * Obtém o valor da temperatura interna e converte-o.
------------------------------------------------------------------- */
void TempCalc ()
{
	flagTemperatura = 0;
	ConvertedValue=HAL_ADC_GetValue(&hadc1);
	JTemp = ((((ConvertedValue * VREF)/MAX_CONVERTED_VALUE) - VSENS_AT_AMBIENT_TEMP) * 10 / AVG_SLOPE) + AMBIENT_TEMP;
}

/*----------------------------------------------------------------
 * imprime temperatura
 */
void printTemperatura ()
{
	sprintf(string, "Temp: %ld C", JTemp);
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 215, (uint8_t *)string, LEFT_MODE);
}

static void LCD_Config(void)
{
	uint32_t  lcd_status = LCD_OK;

	/* Initialize the LCD */
	lcd_status = BSP_LCD_Init();
	while(lcd_status != LCD_OK);

	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	/* Clear the LCD */
	BSP_LCD_Clear(LCD_COLOR_WHITE);

	/* Set LCD Example description */
	LCD_Config2();
}

/*---------------------------------------------------------
 * design do cabeçalho e do rodape
------------------------------------------------------------ */
static void LCD_Config2(void)
{
	/* Set LCD Example description */
	BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()- 20, (uint8_t *)"Copyright (c) Holy Fathers company", CENTER_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
	BSP_LCD_FillRect(0, 0, BSP_LCD_GetXSize(), 35);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_CYAN);
	BSP_LCD_SetFont(&Font24);

	BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"The Reversi", CENTER_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetFont(&Font24);
}

/*-------------------------------------------------------------
 * impressão da board de jogo
------------------------------------------------------------ */
void printBoardGame()
{
	LCD_Config2(); //design do cabeçalho e do rodape

	//desenha o fundo da tabela
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
	BSP_LCD_FillRect(BSP_LCD_GetXSize()/2.10, BSP_LCD_GetYSize()/10, 400, 400);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

	//desenha a grelha do tabuleiro
	for(int i = 0; i<=8; i++)
	{
		BSP_LCD_DrawVLine(BSP_LCD_GetXSize()/2.10 + (BSP_LCD_GetXSize()/16)*i, BSP_LCD_GetYSize()/10, 400);
	}
	for(int j = 0; j<=8; j++){
		BSP_LCD_DrawHLine(BSP_LCD_GetXSize()/2.10, BSP_LCD_GetYSize()/10 + (BSP_LCD_GetYSize()/9.6)*j, 400);
	}

	sprintf(string, "Player 1 turn");
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(60,100 , (uint8_t *)string, LEFT_MODE);
}


void returnTimeZero()
{
	counter=0;
	if(TS_State.touchX[0]>=20 && TS_State.touchX[0]<=350 && TS_State.touchY[0]>=350 && TS_State.touchY[0]<=400)
	{

			int h=counter/3600, m=(counter-3600*h)/60, s=(counter-3600*h-m*60);
			sprintf(string, "Time Total: %2d:%2d:%2d", h,m,s);
			BSP_LCD_SetFont(&Font20);
			BSP_LCD_DisplayStringAt(20, BSP_LCD_GetYSize()/2 + 120, (uint8_t *)string, LEFT_MODE);
	}
	flagTouch=0;
}

/*---------------------------------------------------------
 * imprime tabela dos scores
--------------------- */
void printScoreTable()
{

	BSP_LCD_DrawVLine(10, 50, 375);
	BSP_LCD_DrawHLine(10, 50, 350);
	BSP_LCD_DrawHLine(10, 85, 350);
	BSP_LCD_DrawHLine(10, 425, 350);
	BSP_LCD_DrawVLine(360, 50, 375);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

	sprintf(string, "Game Info");
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(90,55 , (uint8_t *)string, LEFT_MODE);

	sprintf(string, "Player 1 Score: %d", ScorePlayer1);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(60,155 , (uint8_t *)string, LEFT_MODE);

	sprintf(string, "Player 2 Score: %d", ScorePlayer2);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(60,200 , (uint8_t *)string, LEFT_MODE);

	sprintf(string, "N. de jogadas: %d", numberOfPlays);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(60,245 , (uint8_t *)string, LEFT_MODE);
}


/*-----------------------------------------------------------------
 * função chamada pela função blockMove.
 * define qual o player que joga, através da cor.
 * conta o numero de jogadas
 * chama a função que conta os scores
 * controla passagem da jogada para o player seguinte
------------------------------------------------------------------ */
void  putCircle(uint16_t *cX,uint16_t *cY)
{

	if(player%2==1){
		//player 1
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
		BSP_LCD_FillCircle(cX, cY, 15);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		sprintf(string, "Player 1 turn");
		BSP_LCD_SetFont(&Font24);
		BSP_LCD_DisplayStringAt(60,100 , (uint8_t *)string, LEFT_MODE);
		numberOfPlays++;
		counterScore();
		counterPlay=20;
		if(counterPlay == 0 )
		{
			player++;
		}
		player++;
	}
	else{
		//player 2
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW );
		BSP_LCD_FillCircle(cX, cY, 15);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		sprintf(string, "Player 2 turn");
		BSP_LCD_SetFont(&Font24);
		BSP_LCD_DisplayStringAt(60,100 , (uint8_t *)string, LEFT_MODE);
		numberOfPlays++;
		counterScore();
		counterPlay=20;
		if(counterPlay == 0 )
		{
			player++;
		}
		player++;
	}

}

/*-----------------------------------------------------------
 * Função que reconhece a seleção de celulas e centra-as.
 * chamada na main
-------------------------------------------------------------- */
void checkPlace(uint16_t* coordX, uint16_t* coordY)
{
	if(TS_State.touchX[0]>=380 && TS_State.touchX[0]<=780 && TS_State.touchY[0]>=50 && TS_State.touchY[0]<=450){ //limites da board no lcd

		for(int i=0; i<=8; i++)
		{
			if(TS_State.touchX[0] >= 380 + (50*i)  && TS_State.touchX[0]< (380 + (50*i)) +50 ) //limites de x
			{
				*coordX = 405+50*i;
				break;
			}
		}
		for(int j=0; j<=8; j++){
			if(TS_State.touchY[0] >= 50 +(50*j) && TS_State.touchY[0] < (50 +(50*j))+50){ // limites de y
				*coordY = 75+50*j;
				break;
			}
		}

		updateDisplay = 1; // flag que "ocupa" o processador, para fazer refresh do display
	}
}

/*---------------------------------------------------
 * coloca as primeiras 4 peças no tabuleiro
------------------------------------------------- */
void firstPlays ()
{
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_FillCircle(555, 225, 15);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_FillCircle(605, 225, 15);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_FillCircle(555, 275, 15);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_FillCircle(605, 275, 15);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
}


/*--------------------------------------------
 * design do menu.
 * chamada na main
 * chamada a quando a interrupção do push button (zera a flag)
----------------------------------------------- */
void printMenu()
{
	BSP_LCD_DrawVLine(250, 50, 375);
	BSP_LCD_DrawHLine(250, 50, 300);
	BSP_LCD_DrawHLine(250, 85, 300);
	BSP_LCD_DrawHLine(250, 425, 300);
	BSP_LCD_DrawVLine(550, 50, 375);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

	BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
	BSP_LCD_FillRect(250, 50, 300 , 35);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_DARKBLUE);



	sprintf(string, "Menu");
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(10,55 , (uint8_t *)string, CENTER_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	sprintf(string, "Single Player");
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(15,125 , (uint8_t *)string, CENTER_MODE);
	sprintf(string, "Multiplayer");
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(15,200 , (uint8_t *)string, CENTER_MODE);
	sprintf(string, "Score");
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(10,275 , (uint8_t *)string, CENTER_MODE);
	sprintf(string, "Rules");
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(10,350 , (uint8_t *)string, CENTER_MODE);

	LCD_Config2();

	flagPrintMenu=0;
}
/*--------------------------------------------
 * Função que adequa zonas do LCD á opção escolhida
 * activa pela flag do touch e coloca a flag a zero
------------------------------------------------- */
void touchMenu()
{

	if(TS_State.touchX[0]>=250 && TS_State.touchX[0]<=550 && TS_State.touchY[0]>=120 && TS_State.touchY[0]<=200)
	{
		stadus = SINGLE;
	}
	if(TS_State.touchX[0]>=250 && TS_State.touchX[0]<=550 && TS_State.touchY[0]>=201 && TS_State.touchY[0]<=260)
	{
		stadus = MULTI;
	}
	if(TS_State.touchX[0]>=250 && TS_State.touchX[0]<=550 && TS_State.touchY[0]>=261 && TS_State.touchY[0]<=310)
	{
		stadus = SCORE;
	}
	if(TS_State.touchX[0]>=250 && TS_State.touchX[0]<=550 && TS_State.touchY[0]>=311 && TS_State.touchY[0]<=360)
	{
		stadus = RULES;
	}

	flagTouch=0;
}

/*----------------------------------------------------
 * função lê do SD
------------------------------------------------------- */
void ReadFromSD ()
{
	if(f_mount (&SDFatFS, SDPath, 0)!=FR_OK) //activa o sistema
	{
		Error_Handler();
	}
	HAL_Delay(100);
	if(f_open (&SDFile, "Log.txt", FA_READ)!=FR_OK){ //cria ficheiro em modo escrita e testa se detectou SD
		Error_Handler();
	}
	HAL_Delay(100);

	if(f_read (&SDFile, string, sizeof(string), &nBytes)!=FR_OK){ // strlen(string) ->numero de caracteres sizeof->n de bytes
		Error_Handler();
	}

	BSP_LCD_DisplayStringAt(0, 100, (uint8_t *)string, CENTER_MODE);
	HAL_Delay(100);
	f_close (&SDFile);
}

/*--------------------------------------------------------------
 * faz conversão do tempo recebido pelo contador "counter" para hh:mm:ss
 * imprime valores de tempo total de jogo e o tempo de jogada restante
------------------------------------------------------------------ */
void printTotalTime ()
{
	int h=counter/3600, m=(counter-3600*h)/60, s=(counter-3600*h-m*60);
	sprintf(string, "Time Total: %2d:%2d:%2d", h,m,s);
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_DisplayStringAt(20, BSP_LCD_GetYSize()/2 + 120, (uint8_t *)string, LEFT_MODE);

	sprintf(string, "Time of Play: %d s", counterPlay);
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_DisplayStringAt(20, BSP_LCD_GetYSize()/2 + 145, (uint8_t *)string, LEFT_MODE);
}

/*-----------------------------------------------------------
 * Escreve informação para o cartão SD
-------------------------------------------------------- */
void writeGameInfoSD ()
{
	if(f_mount (&SDFatFS, SDPath, 0)!=FR_OK){ //activa o sistema
		Error_Handler();
	}
	HAL_Delay(100);
	if(f_open (&SDFile, "Log.txt", FA_OPEN_APPEND | FA_WRITE)!=FR_OK){ //cria ficheiro em modo escrita e testa se detectou SD
		Error_Handler();
	}
	HAL_Delay(100);

	sprintf(string,"Vencedor: %s, Jogador 1 Pontuação: %d, Jogador 1 Pontuação: %d, Tempo de jogo: %d, Numero de jogadas: %d\n", stringPlayer, ScorePlayer1, ScorePlayer2, counter, numberOfPlays);

	int x=strlen(string)*sizeof(char);

	if(f_write (&SDFile, string, strlen(x), &nBytes)!=FR_OK) // strlen(string) ->numero de caracteres sizeof->n de bytes
		Error_Handler();
	HAL_Delay(100);
	f_close (&SDFile);
}


/*-------------------------------------------------
 * função percorre tabela e verifica, através da função checkPossMov,
 * quais as jogadas possiveis e imprime um ponto na celula respectiva
------------------------------------------------------------------- */
void clearPrePossMov()
{
	int i=0;
	int j=0;


	for(i = 0; i < DIM; i++)
	{
		for(j = 0; j < DIM; j++)
		{
			if(BSP_LCD_ReadPixel(405+50*i,75+50*j)==PossMov)
			{
				BSP_LCD_SetTextColor(NoPossMov);
				BSP_LCD_FillCircle(405+50*i,75+50*j, 5);
			}
			checkPossMov(i,j);
		}
	}
}

/*------------------------------------
 * se passar 3 vezes, perde
------------------------------------ */
void pass3times(){
	if (counterPass1==3){
		counterPass1=0;
		victor=0;
		endgame();
	}
	if (counterPass2==3){
		counterPass2=0;
		victor=1;
		endgame();
	}
}
/*---------------------------------------------------
 * função chamada na função clearPrePossMov
 * verifica se existem jogadas válidas
--------------------------------------------------- */
void checkPossMov(int i, int j){

	uint32_t col1;
	uint32_t col2;
	int iaux, jaux;
	counterPossibleMove=0;

	//define cor de cada player para teste de jogadas possiveis
	if (player%2 == 1){ // se for player 1
		col1 = color1;
		col2 = color2;
	}
	else{
		col1 = color2;
		col2 = color1;
	}

	//Testing UP+LEFT
	iaux = i-1;
	jaux = j-1;
	while (BSP_LCD_ReadPixel(405+50*iaux,75+50*jaux) == col2) { 				//enquanto a posiçao que estamos a verificar estiver dentro dos
		iaux--;                                         //limites do tabuleiro e for igual ao simbolo do adversario
		jaux--;                                         //continuamos a movimentar o nosso estudo na direçao a testar

		if (BSP_LCD_ReadPixel(405+50*iaux,75+50*jaux) == col1)
		{
			if (BSP_LCD_ReadPixel(405+50*i,75+50*j) == NoPossMov)
			{     //substitui espaço em branco por jogada possivel
				BSP_LCD_SetTextColor(PossMov);
				BSP_LCD_FillCircle(405+50*i,75+50*j, 5);       //assinalada por *
				compass[UPLEFT] = TRUE;                 //True quando o teste de direçao funcionou, sera usado na funçao reverse
				counterPossibleMove++;
			}
		}
	}

	iaux = i - 1;       //Testing UP
	jaux = j;
	while (BSP_LCD_ReadPixel(405+50*iaux,75+50*jaux) == col2) {
		iaux--;

		if (BSP_LCD_ReadPixel(405+50*iaux,75+50*jaux) == col1) {
			if (BSP_LCD_ReadPixel(405+50*i,75+50*j) == NoPossMov) {
				BSP_LCD_SetTextColor(PossMov);
				BSP_LCD_FillCircle(405+50*i,75+50*j, 5);
				compass[UP] = TRUE;
				counterPossibleMove++;
			}
		}
	}


	iaux = i - 1;       //Testing UP+RIGHT
	jaux = j + 1;
	while (BSP_LCD_ReadPixel(405+50*iaux,75+50*jaux) == col2) {
		iaux--;
		jaux++;

		if (BSP_LCD_ReadPixel(405+50*iaux,75+50*jaux) == col1) {
			if (BSP_LCD_ReadPixel(405+50*i,75+50*j) == NoPossMov) {
				BSP_LCD_SetTextColor(PossMov);
				BSP_LCD_FillCircle(405+50*i,75+50*j, 5);
				compass[UPRIGHT] = TRUE;
				counterPossibleMove++;
			}
		}
	}


	iaux = i;       //Testing LEFT
	jaux = j - 1;
	while (BSP_LCD_ReadPixel(405+50*iaux,75+50*jaux) == col2) {
		jaux--;

		if (BSP_LCD_ReadPixel(405+50*iaux,75+50*jaux)== col1) {
			if (BSP_LCD_ReadPixel(405+50*i,75+50*j) == NoPossMov) {
				BSP_LCD_SetTextColor(PossMov);
				BSP_LCD_FillCircle(405+50*i,75+50*j, 5);
				compass[LEFT] = TRUE;
				counterPossibleMove++;
			}
		}
	}


	iaux = i;       //Testing RIGHT
	jaux = j + 1;
	while (BSP_LCD_ReadPixel(405+50*iaux,75+50*jaux) == col2) {
		jaux++;

		if (BSP_LCD_ReadPixel(405+50*iaux,75+50*jaux) == col1) {
			if (BSP_LCD_ReadPixel(405+50*i,75+50*j) == NoPossMov) {
				BSP_LCD_SetTextColor(PossMov);
				BSP_LCD_FillCircle(405+50*i,75+50*j, 5);
				compass[RIGHT] = TRUE;
				counterPossibleMove++;
			}
		}
	}


	iaux = i + 1;       //Testing DOWN+LEFT
	jaux = j - 1;
	while (BSP_LCD_ReadPixel(405+50*iaux,75+50*jaux) == col2) {
		iaux++;
		jaux--;

		if (BSP_LCD_ReadPixel(405+50*iaux,75+50*jaux)== col1) {
			if (BSP_LCD_ReadPixel(405+50*i,75+50*j)  == NoPossMov) {
				BSP_LCD_SetTextColor(PossMov);
				BSP_LCD_FillCircle(405+50*i,75+50*j, 5);
				compass[DOWNLEFT] = TRUE;
				counterPossibleMove++;
			}
		}
	}


	iaux = i + 1;       //Testing DOWN
	jaux = j;
	while (BSP_LCD_ReadPixel(405+50*iaux,75+50*jaux) == col2) {
		iaux++;

		if (BSP_LCD_ReadPixel(405+50*iaux,75+50*jaux)  == col1) {
			if (BSP_LCD_ReadPixel(405+50*i,75+50*j)  == NoPossMov) {
				BSP_LCD_SetTextColor(PossMov);
				BSP_LCD_FillCircle(405+50*i,75+50*j, 5);
				compass[DOWN] = TRUE;
				counterPossibleMove++;
			}
		}
	}


	iaux = i + 1;       //Testing DOWN+RIGHT
	jaux = j + 1;
	while (BSP_LCD_ReadPixel(405+50*iaux,75+50*jaux) == col2) {
		iaux++;
		jaux++;

		if (BSP_LCD_ReadPixel(405+50*iaux,75+50*jaux) == col1) {
			if (BSP_LCD_ReadPixel(405+50*i,75+50*j) == NoPossMov) {
				BSP_LCD_SetTextColor(PossMov);
				BSP_LCD_FillCircle(405+50*i,75+50*j, 5);
				compass[DOWNRIGHT] = TRUE;
				counterPossibleMove++;
			}
		}
	}
}
void blockMove (uint16_t *cX, uint16_t *cY)
{
	if(BSP_LCD_ReadPixel(cX,cY)==PossMov){
		putCircle(cX, cY);
	}
}

void reverse() {

	int iaux, jaux;
	int col1, col2;

	//define cor de cada player para teste de jogadas possiveis
	if (player%2 == 1){ // se for player 1
		col1 = 1;
		col2 = 2;
	}
	else{
		col1 = 2;
		col2 = 1;
	}

	if (compass[UPLEFT]) { //apenas testa direçao se esta direçao tinha sido assinalada como jogada possivel
		iaux = cX - 50;           //Testing UP+LEFT
		jaux = cY - 50;
		while (board[iaux][jaux] == col2) {
			board[iaux][jaux] = col1;
			iaux-=50;
			jaux-=50;
		}
	}


	if (compass[UP]) {
		iaux = cX - 50;           //Testing UP
		jaux = cY;
		while (board[iaux][jaux] == col2) {
			board[iaux][jaux] = col1;
			iaux-=50;
		}
	}


	if (compass[UPRIGHT]) {
		iaux = cX - 50;           //Testing UP+RIGHT
		jaux = cY+ 50;
		while (board[iaux][jaux] == col2) {
			board[iaux][jaux] = col1;
			iaux-=50;
			jaux+=50;
		}
	}


	if (compass[LEFT]) {
		iaux = cX;               //Testing LEFT
		jaux = cY- 50;
		while (board[iaux][jaux] == col2) {
			board[iaux][jaux] = col1;
			jaux-=50;
		}
	}


	if (compass[RIGHT]) {
		iaux = cX;               //Testing RIGHT
		jaux = cY + 50;
		while (board[iaux][jaux] == col2) {
			board[iaux][jaux] = col1;
			jaux+=50;
		}
	}


	if (compass[DOWNLEFT]) {
		iaux = cX +50;           //Testing DOWN+LEFT
		jaux = cY - 50;
		while (board[iaux][jaux] == col2) {
			board[iaux][jaux] = col1;
			iaux+=50;
			jaux-=50;
		}
	}


	if (compass[DOWN]) {
		iaux = cX + 50;           //Testing DOWN
		jaux = cY;
		while (board[iaux][jaux] == col2) {
			board[iaux][jaux] = col1;
			iaux+=50;
		}
	}


	if (compass[DOWNRIGHT]) {
		iaux = cX + 50;           //Testing DOWN+RIGHT
		jaux = cY + 50;
		while ( board[iaux][jaux] == col2) {
			board[iaux][jaux] = col1;
			iaux+=50;
			jaux+=50;
		}
	}

	for (int k = 0; k < 8; k++) {       //limpa rosa dos ventos, necessario para a funçao neighborhood e ciclo do jogo
		compass[k] = FALSE;
	}
}

void counterScore()
{
	int i=0;
	int j=0;

	ScorePlayer1=0;
	ScorePlayer2=0;
	for(i = 0; i < DIM; i++)
	{
		for(j = 0; j < DIM; j++)
		{
			if(BSP_LCD_ReadPixel(405+50*i,75+50*j)==color1)
			{
				ScorePlayer1++;
			}
			if(BSP_LCD_ReadPixel(405+50*i,75+50*j)==color2)
			{
				ScorePlayer2++;
			}
		}
	}
	if(ScorePlayer1>ScorePlayer2)
	{sprintf(string, "Player 1 is winnig");
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(60,280 , (uint8_t *)string, LEFT_MODE);
	}

	else if(ScorePlayer2>ScorePlayer1)
	{sprintf(string, "Player 2 is winnig");
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(60,280 , (uint8_t *)string, LEFT_MODE);
	}
	else if(ScorePlayer2==ScorePlayer1)
	{sprintf(string, "Empatados         ");
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(60,280 , (uint8_t *)string, LEFT_MODE);
	}
}

void readRules()
{
	char stringRules[200];
	sprintf(stringRules, "RULES");
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(15,40 , (uint8_t *)stringRules, CENTER_MODE);
	sprintf(stringRules, "Reversi is a strategy board game for two players,");
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_DisplayStringAt(8,80 , (uint8_t *)stringRules, LEFT_MODE);
	sprintf(stringRules, "played on an 8 x 8 uncheckered board.");
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_DisplayStringAt(8,100 , (uint8_t *)stringRules, LEFT_MODE);
	sprintf(stringRules, "There are sixty-four identical game pieces called disks (often spelled discs),");
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_DisplayStringAt(8,120 , (uint8_t *)stringRules, LEFT_MODE);
	sprintf(stringRules, "which are light on one side and dark on the other.");
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_DisplayStringAt(8,140 , (uint8_t *)stringRules, LEFT_MODE);
	sprintf(stringRules, "Players take turns placing disks on the board with their assigned color facing up.");
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_DisplayStringAt(8,160 , (uint8_t *)stringRules, LEFT_MODE);
	sprintf(stringRules, "During a play, any disks of the opponent's color that are in a straight line and bounded by the disk");
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_DisplayStringAt(8,180 , (uint8_t *)stringRules, LEFT_MODE);
	sprintf(stringRules, "just placed and another disk of the current player's color are turned over to the current player's color.");
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_DisplayStringAt(8,200 , (uint8_t *)stringRules, LEFT_MODE);
	sprintf(stringRules, "The object of the game is to have the majority of disks turned to display your color when the last playable empty square is filled.");
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_DisplayStringAt(8,220 , (uint8_t *)stringRules, LEFT_MODE);

	LCD_Config2();
}


void setToMemory ()
{
	for (int i=0; i<DIM; i++)
	{
		for (int j=0; j<DIM; j++)
		{
			if(BSP_LCD_ReadPixel((405+50*i), (75+50*j))== LCD_COLOR_LIGHTGRAY || BSP_LCD_ReadPixel((405+50*i), (75+50*j))== LCD_COLOR_DARKGRAY)
			{
				board[i][j] = 0;
			}
			else if(BSP_LCD_ReadPixel((405+50*i), (75+50*j))== LCD_COLOR_YELLOW)
			{
				board[i][j] = 1;
			}
			else if(BSP_LCD_ReadPixel((405+50*i), (75+50*j))== LCD_COLOR_BLUE)
			{
				board[i][j] = 2;
			}
		}
	}
}

void setMemoryToBoard ()
{
	for (int i=0; i<DIM; i++)
	{
		for (int j=0; j<DIM; j++)
		{
			if(board[i][j] == 0)
			{
				BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY); //altera a cor dessas cordenadas
				BSP_LCD_FillCircle(405+50*i, 75+50*j, 15);
			}
			else if(board[i][j] == 1)
			{
				BSP_LCD_SetTextColor(LCD_COLOR_YELLOW); //altera a cor dessas cordenadas
				BSP_LCD_FillCircle(405+50*i, 75+50*j, 15);
			}
			else if(board[i][j] == 2)
			{
				BSP_LCD_SetTextColor(LCD_COLOR_BLUE); //altera a cor dessas cordenadas
				BSP_LCD_FillCircle(405+50*i, 75+50*j, 15);
			}
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1) //para ficar dependurado
	{
		BSP_LED_Toggle(LED_GREEN);
		HAL_Delay(1000);

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
