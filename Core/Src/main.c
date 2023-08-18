/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
//Test Git sar
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ESP8266_Chelo.h"
#include "ModBUS_Chelo.h"
#include "STR_Chelo.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TOK 1
#define FIND 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define PIN_RESET_ON HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
#define PIN_RESET_OFF HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint32_t ms_ticks=0,
		 min_ticks=0,
		 _flag1_ticks=0;
int		 pasos=0;
uint8_t _flag1=0;		//Flag test envio;

uint32_t ADC_VAL[4];
uint32_t DAC_VAL[2];
uint32_t DAC_VAL2[2];

struct WIFI wf;
struct MBUS mb;
struct MBUS_FIFO FIFO[5];

uint8_t ESP_REinit=0,			//Conteo de intentos de incializacion
		ESP_InitF=0,			//Flag de error por no encontrar la sentencia
		ESP_HW_Init=0,
		EN_UART2_TMR=0,
		EN_UART1_TMR=0,
		FLAG_TIMEOUT=0,
		FLAG_UART2=0,
		resultado=0,
		error_rxdata=0,
		ITM_AT_debug=1,//ITM_AT_debug=1,
		AT_PROC_ON=1, //=1 Proceso AT recibido =0 NO
		esp_restart=0,
		conexion,
		asc=0,
		//--------debug----//
		CP_ready=0,
		CP_ai=0;

char	UART_RX_vect[384],
		datarx_uart1[384],
		AT_debug[384],
		datarx1[2],
		UART_RX_vect_hld[384],
		UART_RX_vect[384],
		RXdata_uart1[384],
		RXdata_DEBUG[384],
		WIFI_NET[]="Fibertel WiFi967 2.4GHz",//WIFI_NET[]="PLC_DEV",//
		WIFI_PASS[]="0042880756",//WIFI_PASS[]="12345678",//
		TCP_SERVER[]="192.168.0.181",//TCP_SERVER[]="192.168.0.102",//TCP_SERVER[]="192.168.0.47",
		TCP_PORT[]="503",
		TCP_SERVER_LOCAL[]="192.168.0.33",//TCP_SERVER[]="192.168.0.47",
		TCP_SERVER_LOCAL_GWY[]="192.168.0.1",//TCP_SERVER[]="192.168.0.47",
		TCP_SERVER_LOCAL_MSK[]="255.255.255.0",//TCP_SERVER[]="192.168.0.47",
		TCP_PORT_LOCAL[]="502",
		RX2[]="RX.",
		RX[384],
		CMP_VECT[]="\0",
	    TESTA[32],
		TESTB[32],
		TESTC[32],
		UART_RX_byte[2];

int UART_RX_items=0,
    ESP_ticks=0,
	MBUS_ticks=0,
	items_rx_debug=0,
    ticks=0,
	ntesta=17,
	ntestb=4,
	ntestc=0,
	postesta=0,
	funcion=0,
	uart1pass=0,
	UART1_ticks=0,
	FLAG_UART1=0,
	chr_pos=0,
	items_rx=0,

	UART_RX_pos=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t ESP8266_HW_Init(UART_HandleTypeDef *);
void ESP8266_HW_Reset(void);

void BorrarVect(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  /* USER CODE BEGIN 1 */
		  //----------------------- WIFI ------------------------//
	 	  	Inicializar(&wf); 									//Borra todos los registros de la estructura
			strcpy(wf._WF_Net, WIFI_NET);						//Nombre de la red WIFI  a conectar Fibertel WiFi967 2.4GHz
			strcpy(wf._WF_Pass, WIFI_PASS);						//Password de la red WIFI
			strcpy(wf._TCP_Remote_Server_IP, TCP_SERVER);		//char _TCP_Remote_Server_IP[16];		//IP del Servidor TCP
			strcpy(wf._TCP_Remote_Server_Port, TCP_PORT);		//char _TCP_Remote_Server_Port[16];			//Puerto del Servidor TCP
			strcpy(wf._TCP_Local_Server_IP, TCP_SERVER_LOCAL);
			strcpy(wf._TCP_Local_Server_GWY, TCP_SERVER_LOCAL_GWY);
			strcpy(wf._TCP_Local_Server_MSK, TCP_SERVER_LOCAL_MSK);
			strcpy(wf._TCP_Local_Server_Port, TCP_PORT_LOCAL);
			wf._TCP_Local_Server_EN=0;							//Habilito el Servidor Local en = 1
			wf._data2SND[0]=0x00;//strcpy(wf._data2SND,"01;03;00;00;00;0A;C5;CD");//strcpy(wf._data2SND,"20;352;52#");
			wf._data2SND[1]=0x00;
			wf._data2SND[2]=0x00;
			wf._data2SND[3]=0x00;
			wf._data2SND[4]=0x00;
			wf._data2SND[5]=0x05;
			wf._data2SND[6]=0x01;
			wf._data2SND[7]=0x03;
			wf._data2SND[8]=0x02;//strcpy(wf._data2SND,"01;03;00;00;00;0A;C5;CD");//strcpy(wf._data2SND,"20;352;52#");
			wf._data2SND[9]=0x00;
			wf._data2SND[10]=0x00;
			wf._data2SND[11]=0x0A;
			wf._data2SND[12]=0x00;
			wf._data2SND[13]=0x33;
			wf._data2SND[14]=0x34;
			wf._data2SND[15]=0x35;
			wf._n_D2SND=11;
			wf._estado_conexion=CAMBIAR_MODO_EN_CURSO;	//wf._estado_conexion=1;					//Arranco en WiFi Desconectado
		 //----------------------- WIFI ------------------------//

		 //---------------------- ModBUS -----------------------//

			ModBUS_Config(&mb);
			ModBUS_F03_Assign(&mb,3,0xAA55);


		 //---------------------- ModBUS -----------------------//
  /* USER CODE END 1 */

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
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  ESP8266_HW_Reset();		  //Reseteo el modulo desde el pin de RESET
  HAL_TIM_Base_Start(&htim6); //Timer como base de tiempo
  HAL_UART_Receive_IT(&huart1,(uint8_t *)UART_RX_byte,1); // DATO SERIAL
  HAL_UART_Receive_IT(&huart2,(uint8_t *)datarx1,1); //DATO MODULO WIFI
  //HAL_UART_Transmit(&huart1, "\r\n ESP Init . . \r\n",strlen("\r\n ESP Init . . \r\n"),1000);
  ITM0_Write(" ESP Init . . \r\n", strlen(" ESP Init . . \r\n"));
  if(ESP8266_HW_Init(&huart2)==1)
  {
	  ESP_HW_Init=1;
	  //HAL_UART_Transmit(&huart1, "\r\n ESP HW Init OK\r\n",strlen("\r\n ESP HW Init OK\r\n"),1000);
	  ITM0_Write("ESP HW Init OK\r", strlen(" ESP HW Init OK\r"));
  }
  else
  {
	  ESP8266_HW_Reset();
	  if(ESP8266_HW_Init(&huart2)==1)
	  {
		  ESP_HW_Init=1;
		  //HAL_UART_Transmit(&huart1, "\r\n ESP HW Init OK\r\n",strlen("\r\n ESP HW Init OK\r\n"),1000);
		  ITM0_Write("ESP HW Init OK\r", strlen(" ESP HW Init OK\r"));
	  }
	  else
	  {
		  ESP_HW_Init=0;
		  //HAL_UART_Transmit(&huart1, "\r\n ESP HW Init Fail\r\n",strlen("\r\n ESP HW Init Fail\r\n"),1000);
		  ITM0_Write("ESP HW Init Fail\r", strlen(" ESP HW Init Fail\r"));
	  }
  }

  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 /*if (_flag1){
	  HAL_UART_Transmit(&huart1, "test", 4, 100);
	  _flag1=0;
	  }*/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 //----------------INSTRUCCIONS POR PUERTO SERIE---------------------
	 		if (FLAG_UART1==1)
	 		{
	 			if (AT_PROC_ON==0)
	 			{
	 				HAL_UART_Transmit_IT(&huart2, datarx_uart1, items_rx);
	 			}
	 			else
	 			{
	 				//----- COMANDOS RECIBIDOS POR PUERTO SERIAL 1
	 			 	wf._n_fcomp=strlen("ATDEBUG:");
	 			 	wf._n_orig=items_rx;
	 			 	if(FT_String_ND(datarx_uart1,&wf._n_orig,"ATDEBUG:",&wf._n_fcomp,wf._uartRCVD_tok,&wf._n_tok,&ntestc,&wf._id_conn,FIND)==1)
	 				{
	 				 	wf._n_fcomp=strlen("ATDEBUG:");
	 				 	wf._n_orig=items_rx;
	 				 	FT_String_ND(datarx_uart1,&wf._n_orig,"ATDEBUG:",&wf._n_fcomp,wf._uartRCVD_tok,&wf._n_tok,&ntestc,&wf._id_conn,TOK);
	 				 	HAL_UART_Transmit(&huart1, datarx_uart1,wf._n_orig,100);
	 				 	//HAL_UART_Transmit(&huart2, RXdata_uart1,wf._n_orig);
	 				 	HAL_UART_Receive_IT(&huart2,(uint8_t *)UART_RX_byte,1);
	 				 	datarx_uart1[0]='\0';
	 				 	datarx_uart1[2]='\0';
	 					BorrarVect();
	 				}
	 					else
	 					{
	 				if(FT_String(datarx_uart1,"RESTART_SM",wf._uartRCVD_tok,&chr_pos,FIND)==1)
	 				{
	 					HAL_UART_Receive_IT(&huart2,(uint8_t *)UART_RX_byte,1);
	 					datarx_uart1[0]='\0';
	 					BorrarVect();
	 				}
	 					else
	 					{
	 					if(FT_String(datarx_uart1,"CNWF",wf._uartRCVD_tok,&chr_pos,FIND)==1)
	 					{
	 						ConectarWIFI(&wf);
	 						wf._estado_conexion=3;	//Cambio el estado de la maquina de estados a WiFi Conectando
	 						datarx_uart1[0]='\0';
	 						BorrarVect();
	 					}
	 						else
	 						{
	 						if((FT_String(datarx_uart1,"CNDWF",wf._uartRCVD_tok,&chr_pos,FIND)==1)&&((wf._estado_conexion==209)||(wf._estado_conexion>=309)))
	 						{
	 							//HAL_UART_Transmit(&huart2,"AT+CWQAP\r\n",strlen("AT+CWQAP\r\n"), 100);
	 							DesconectarWIFI(&wf);
	 							//wf._estado=0;
	 							//wf._estado_conexion=1;	//Cambio el estado de la maquina de estados a WiFi Desconectado
	 							datarx_uart1[0]='\0';
	 							BorrarVect();
	 							AT_ESP8266_ND(&wf);//Debe procesar el pedido de desconexión
	 						}
	 							else
	 							{
	 								if((FT_String(datarx_uart1,"CNDWFPWR",wf._uartRCVD_tok,&chr_pos,FIND)==1)&&(conexion==4))
	 								{
	 								HAL_UART_Transmit(&huart1,"AT+CWQAP\r\n",strlen("AT+CWQAP\r\n"), 100);
	 								}
	 									else
	 									{

	 							if(FT_String(datarx_uart1,"CNTCP",wf._uartRCVD_tok,&chr_pos,FIND)==1)
	 							{
	 								ConectarTCP(&wf);
	 								wf._estado_conexion=7;	//Cambio el estado de la maquina de estados a Conectando TCP
	 								wf._estado=0;			//Borramos los estados de alarma

	 								datarx_uart1[0]='\0';
	 								BorrarVect();
	 							}
	 								else
	 								{
	 								if(FT_String(datarx_uart1,"CNDTCP",wf._uartRCVD_tok,&chr_pos,FIND)==1)
	 								{
	 									DesconectarTCP(&wf);
	 									wf._estado_conexion=5;	//Cambio el estado de la maquina de estados a Conectando TCP
	 									wf._estado=0;			//Borramos los estados de alarma
	 									datarx_uart1[0]='\0';
	 									BorrarVect();
	 								}
	 									else
	 									{
	 										if(FT_String(datarx_uart1,"DEBUG_SER_OFF",wf._uartRCVD_tok,&chr_pos,FIND)==1)
	 										{
	 											ITM_AT_debug=0;
	 										}
	 											else
	 											{
	 												if(FT_String(datarx_uart1,"ESP_RESTART",wf._uartRCVD_tok,&chr_pos,FIND)==1)
	 												{
	 												 esp_restart=1;
	 												}
	 													else
	 													{

	 									if(FT_String(datarx_uart1,"WIFINET:",wf._uartRCVD_tok,&chr_pos,FIND)==1) //Detecto que me mandan la red
	 									{
	 										/*FT_String(datarx_uart1,"WIFINET:",wf._uartRCVD_tok,&chr_pos,TOK);
	 										strcpy(wf._WF_Net,datarx_uart1);

	 										I2C_E2PROM( &hi2c1, iic._addr_WF_Net, 2, wf._WF_Net, strlen(wf._WF_Net), iic._send, WRITE);
	 										HAL_Delay(1);
	 										I2C_E2PROM( &hi2c1, iic._addr_WF_Net, 2, iic._WF_Net, 32, iic._receive, READ);
	 										HAL_Delay(1);

	 										datarx_uart1[0]='\0';*/
	 									}
	 										else
	 										{
	 										if(FT_String(datarx_uart1,"WIFIPASS:",wf._uartRCVD_tok,&chr_pos,FIND)==1) //Detecto que me mandan la red
	 										{
	 											/*FT_String(datarx_uart1,"WIFIPASS:",wf._uartRCVD_tok,&chr_pos,TOK);
	 											strcpy(wf._WF_Pass,datarx_uart1);

	 											I2C_E2PROM( &hi2c1, iic._addr_WF_Net_WF_Pass, 2, wf._WF_Pass, strlen(wf._WF_Pass), iic._send, WRITE);
	 											HAL_Delay(1);
	 											I2C_E2PROM( &hi2c1, iic._addr_WF_Net_WF_Pass, 2, iic._WF_Pass, 11, iic._receive, READ);
	 											HAL_Delay(1);
	 											datarx_uart1[0]='\0';*/
	 										}
	 											else
	 											{
	 											if(FT_String(datarx_uart1,"TCP:",wf._uartRCVD_tok,&chr_pos,FIND)==1) //Detecto que me mandan la red
	 											{
	 												/*FT_String(datarx_uart1,"TCP:",wf._uartRCVD_tok,&chr_pos,TOK);
	 												strcpy(wf._TCP_Remote_Server_IP,datarx_uart1);

	 												I2C_E2PROM( &hi2c1, iic._addr_tcp, 2, wf._TCP_Remote_Server_IP, strlen(wf._TCP_Remote_Server_IP), iic._send, WRITE);
	 												HAL_Delay(1);
	 												I2C_E2PROM( &hi2c1, iic._addr_tcp, 2, iic._TCP_Remote_Server_IP, 16, iic._receive, READ);
	 												HAL_Delay(1);
	 												datarx_uart1[0]='\0';*/
	 											}
	 												else
	 												{
	 												if(FT_String(datarx_uart1,"TCP_PORT:",wf._uartRCVD_tok,&chr_pos,FIND)==1) //Detecto que me mandan la red
	 												{
	 													/*FT_String(datarx_uart1,"TCP_PORT:",wf._uartRCVD_tok,&chr_pos,TOK);
	 													strcpy(wf._TCP_Remote_Server_Port,datarx_uart1);

	 													I2C_E2PROM( &hi2c1, iic._addr_tcp_port, 2, wf._TCP_Remote_Server_Port, strlen(wf._TCP_Remote_Server_Port), iic._send, WRITE);
	 													HAL_Delay(1);
	 													I2C_E2PROM( &hi2c1, iic._addr_tcp_port, 2, iic._TCP_Remote_Server_Port, 5, iic._receive, READ);
	 													HAL_Delay(1);
	 													I2C_E2PROM( &hi2c1, iic._addr_WF_Net, 2, iic._TCP_Remote_Server_Port,41, iic._receive, READ);
	 													HAL_Delay(1);
	 													I2C_E2PROM( &hi2c1, iic._addr_tcp, 2, iic._TCP_Remote_Server_Port,32, iic._receive, READ);
	 													HAL_Delay(1);
	 													datarx_uart1[0]='\0';*/
	 												}
	 													else
	 													{
	 													if(FT_String(datarx_uart1,"WIFIRDNET:",wf._uartRCVD_tok,&chr_pos,FIND)==1) //Detecto que me mandan la red
	 													{
	 														//Leo el valor de la red
	 														/*I2C_E2PROM( &hi2c1, iic._addr_WF_Net, 2, iic._WF_Net, 32, iic._receive, READ);
	 														HAL_Delay(1);
	 														strncat(iic._serial,";WIFIRDNET;",strlen(";WIFIRDNET;"));
	 														strncat(iic._serial,iic._receive,strlen(iic._receive));
	 														strncat(iic._serial,";",1);
	 														datarx_uart1[0]='\0';
	 														HAL_UART_Transmit_IT(&huart1,(uint8_t*)iic._serial, strlen(iic._serial));
	 														iic._serial[0]='\0';*/
	 													}
	 														else
	 														{
	 														if(FT_String(datarx_uart1,"WIFIRDPASS:",wf._uartRCVD_tok,&chr_pos,FIND)==1) //Detecto que me mandan la red
	 														{
	 															//Leo el valor de la red
	 															/*I2C_E2PROM( &hi2c1, iic._addr_WF_Net_WF_Pass, 2, iic._WF_Net, 11, iic._receive, READ);
	 															HAL_Delay(1);
	 															strncat(iic._serial,";WIFIRDPASS;",strlen(";WIFIRDPASS;"));
	 															strncat(iic._serial,iic._receive,strlen(iic._receive));
	 															strncat(iic._serial,";",1);
	 															datarx_uart1[0]='\0';
	 															HAL_UART_Transmit_IT(&huart1,(uint8_t*)iic._serial, strlen(iic._serial));
	 															iic._serial[0]='\0';*/
	 														}
	 															else
	 															{
	 															if(FT_String(datarx_uart1,"TCPRD:",wf._uartRCVD_tok,&chr_pos,FIND)==1) //Detecto que me mandan la red
	 															{
	 																//Leo el valor de la red
	 																/*I2C_E2PROM( &hi2c1, iic._addr_tcp, 2, iic._WF_Net, 16, iic._receive, READ);
	 																HAL_Delay(1);
	 																strncat(iic._serial,";TCPRD;",strlen(";TCPRD;"));
	 																strncat(iic._serial,iic._receive,strlen(iic._receive));
	 																strncat(iic._serial,";",1);
	 																datarx_uart1[0]='\0';
	 																HAL_UART_Transmit_IT(&huart1,(uint8_t*)iic._serial, strlen(iic._serial));
	 																iic._serial[0]='\0';*/
	 															}
	 																else
	 																{
	 															if(FT_String(datarx_uart1,"TCPRDPORT:",wf._uartRCVD_tok,&chr_pos,FIND)==1) //Detecto que me mandan la red
	 															{
	 																//Leo el valor de la red
	 																/*I2C_E2PROM( &hi2c1, iic._addr_tcp_port, 2, iic._WF_Net, 5, iic._receive, READ);
	 																HAL_Delay(1);
	 																strncat(iic._serial,";TCPRDPORT;",strlen(";TCPRDPORT;"));
	 																strncat(iic._serial,iic._receive,strlen(iic._receive));
	 																strncat(iic._serial,";",1);
	 																datarx_uart1[0]='\0';
	 																HAL_UART_Transmit_IT(&huart1,(uint8_t*)iic._serial, strlen(iic._serial));
	 																iic._serial[0]='\0';*/
	 															}
	 															 else
	 																{
	 																if(FT_String(datarx_uart1,"ENDSND:",wf._uartRCVD_tok,&chr_pos,FIND)==1) //Detecto que me mandan la red
	 																{
	 																	//Deshabilita el envío de data recibida por el ESP8266 al puerto 1
	 																	//densnd=1;
	 																}
	 																 else
	 																	{
	 																	if(FT_String(datarx_uart1,"ENSND:",wf._uartRCVD_tok,&chr_pos,FIND)==1) //Detecto que me mandan la red
	 																	{
	 																		//Deshabilita el envío de data recibida por el ESP8266 al puerto 1
	 																		//densnd=0;
	 																	}

	 					}}}}}}}}}}}}}}}}}}}

	 				FLAG_UART1=0;
	 		}
	 //----------------INSTRUCCIONES POR PUERTO SERIE---------------------

	 		//HAL_Delay(500);
	 		//ITM_Port32(31) = 1;9
	 		//ITM0_Write("TESTeEE\r\n", strlen("TESTeEE\r\n"));
	 		//
	 		//_write(2);
	 		//printf("test \r\n");
	 		/*HAL_Delay(200);
	 		HAL_Delay(200);
	 		HAL_Delay(200);
	 		printf("PASO POR MAIN\r");
	 		pasos++;*/



	 		if ((FLAG_UART2==1)||(FLAG_TIMEOUT==1))  //Si recibí datos o me fui por TimeOUT
	 		{


	 			if(FLAG_UART2==1)
	 				{
	 					FLAG_UART2=0;

	 					if (error_rxdata==3)
	 					{
	 						error_rxdata=0;
	 					}
	 					if (error_rxdata==1)
	 					{
	 						error_rxdata=5;
	 						error_rxdata=0;
	 					}
	 				}

	 			if(FLAG_TIMEOUT==1)
	 				{
	 					FLAG_TIMEOUT=0;
	 					//strcpy(wf._uartRCVD, UART_RX_vect_hld);
	 				}

	 			if ((ESP_HW_Init==1)&&(AT_PROC_ON)) //Si el módulo se inició correctamente
	 			{
 				  	//--------------------------------------------------------------------------------------------------------------//
	 				//	Decodifco la info recibida por el puerto serie, separo lo +IPD en el caso que vengan concatenados en el 	//
	 				//	de datos recibidos, y se continua con la maquina de estados.												//
	 				//	Aquí no se envìa la info, se da la orden y se arma el string de acuerdo a la instruccion					//
 					//--------------------------------------------------------------------------------------------------------------//
	 				wf._n_orig=UART_RX_items;
	 				CopiaVector(wf._uartRCVD,UART_RX_vect_hld,UART_RX_items,1,CMP_VECT);
	 				resultado=AT_ESP8266_ND(&wf); // Lo proceso despues de haberlo recibido por uart no de toque.

 				  	//--------------------------------------------------------------------------------------------------------------//
// NO VA ACA		//	Se gestiona el ordenameint de la pila y la prioridad de los mensajes a responder							//
 					//--------------------------------------------------------------------------------------------------------------//
	 				//
	 				//	Solo se maneja el FIFO si se recibieron nuevos datos, no cada vez que se recibe un dato por puerto serie, pues
	 				//	mientras está recibiendo el FIFO debe indicar el proximo dato a enviar, o bien ir manejando el timeout de los
	 				//	datos que a los que no se les ha dado respuesta aún.
	 				//
	 				//	Con un dato nuevo se le asigna nueva ubicacion
	 				//  Las ubicaciones anuladas poseen 255 en FIFO[X].Conexion y no se les computa timeout, al resto si
	 				//	Una variable externa debe estar apuntando siempre al proximo dato a procesar, caundo se recibe uno nuevo se
	 				//	agrega sin alterar el estado de esa variable
	 				//  Un registro se anula cuando se verfica su respuesta o se excede el timeout
	 				//	Si se recibe una variable nueva y el regitro está lleno, la misma no se procesa y se desecha
	 				//
	 				if ((wf._new_data_rcv==1)&&(wf._estado_data==4)) // Si recibí nueva data y está ok
	 				{
	 					FIFO[0].IDRX=1;
	 					FIFO[0].Conexion=wf._id_conn;
	 					CopiaVector(FIFO[0].MODBUS_DATA,wf._dataRCV,wf._n_dataRCV,0,'A');
	 					FIFO[0]._n_MODBUS_DATA=wf._n_dataRCV;
	 					FIFO[0].timeout=234;

	 					if(ModBUS_Check(FIFO[0].MODBUS_DATA,FIFO[0]._n_MODBUS_DATA)) wf._send_data=1;
	 				}

 				  	//--------------------------------------------------------------------------------------------------------------//
// NO VA ACA		//	Se genera la respuesa MosBUS de acuerdo a la función recibida												//
 					//--------------------------------------------------------------------------------------------------------------//

	 				if (wf._send_data==1)// Si recibí nueva data
	 				{
	 				  	//-------------------------------------------------------//
	 					//				Genero informacion ModBUS				 //
	 					//-------------------------------------------------------//

	 					CopiaVector(mb._MBUS_RCVD,wf._dataRCV,wf._n_dataRCV,0,'A');
	 				  	mb._n_MBUS_RCVD=wf._n_dataRCV;
	 				  	//CopiaVector(mb._MBUS_RCVD,wf._dataRCV,5,0,'A');  //Solo copio la info si es correcta
	 				  	//-------- Proceso el ModBUS ----------------------------//
	 				  	ModBUS(&mb);
	 				 	//--------Copio vector ModBUS a data a enviar------------//
	 				  	CopiaVector(wf._data2SND,mb._MBUS_2SND,mb._n_MBUS_2SND,0,'A');
	 				 	wf._n_D2SND=mb._n_MBUS_2SND;
	 					//-------------------------------------------------------//
	 				}
	 			}
	 		}

	 		if ((ESP_HW_Init==1)&&(AT_PROC_ON)) //Si el módulo se inició correctamente
	 		{
			//--------------------------------------------------------------------------------------------------------------//
 			//	Se maneja la maquina de estados, se realiza el envío de informacion											//
			//--------------------------------------------------------------------------------------------------------------//

	 			conexion=WiFi_Conn_ND(&wf,&huart2,0);	//Tiene que ir en el main el chequeo es constante
	 		}
	 		if (esp_restart==1)
	 		{
	 			/*HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	 			HAL_Delay(2000);//210419
	 			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	 			HAL_Delay(5000);//210419
	 			esp_restart=0;*/
	 		}

	   }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x19;
  sTime.Minutes = 0x40;
  sTime.Seconds = 0x1;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x21;
  sDate.Year = 0x22;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 90;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_INACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim6.Init.Prescaler = 16800-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 12000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, WF_EN_Pin|LORA_EN_Pin|BT_EN_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NSS_SPI_Pin|RST_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : WF_EN_Pin LORA_EN_Pin BT_EN_Pin LED_Pin */
  GPIO_InitStruct.Pin = WF_EN_Pin|LORA_EN_Pin|BT_EN_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_SPI_Pin RST_SPI_Pin */
  GPIO_InitStruct.Pin = NSS_SPI_Pin|RST_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_BOARD_Pin */
  GPIO_InitStruct.Pin = BTN_BOARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_BOARD_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	  if (_flag1_ticks>=500){
			 if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15)){
				 HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0);
			 }
	  _flag1=1;
	  _flag1_ticks=0;
	  }
	  _flag1_ticks++;
		ms_ticks++;
	if (ESP_HW_Init==0){ESP_ticks++;}

	if (ms_ticks==500)//(ms_ticks==250)//(ms_ticks==50)
	  {

		  ms_ticks=0;
		  min_ticks++;


		  	if(MBUS_ticks==360) MBUS_ticks=0;

		  	if (asc==0)  MBUS_ticks++;
		  	if (MBUS_ticks==100) asc=1;
		  	if (asc==1) MBUS_ticks--;
		  	if (MBUS_ticks==0) asc=0;

			ModBUS_F03_Assign(&mb,5,MBUS_ticks);
			ModBUS_F03_Assign(&mb,6,MBUS_ticks+10);
			ModBUS_F03_Assign(&mb,7,MBUS_ticks+20);
			ModBUS_F03_Assign(&mb,8,MBUS_ticks+30);
			ModBUS_F03_Assign(&mb,9,MBUS_ticks+40);
			ModBUS_F03_Assign(&mb,10,MBUS_ticks+50);
			ModBUS_F03_Assign(&mb,11,MBUS_ticks+60);

			ModBUS_F04_Assign(&mb,5,MBUS_ticks);
			ModBUS_F04_Assign(&mb,6,MBUS_ticks+1);
			ModBUS_F04_Assign(&mb,7,MBUS_ticks+2);
			ModBUS_F04_Assign(&mb,8,MBUS_ticks+3);
			ModBUS_F04_Assign(&mb,9,MBUS_ticks+4);
			ModBUS_F04_Assign(&mb,10,MBUS_ticks+5);
			ModBUS_F04_Assign(&mb,11,MBUS_ticks+6);



		  DAC_VAL[0]+=128;
		  DAC_VAL2[0]+=256;
		  if(DAC_VAL[0]==4096) DAC_VAL[0]=0;
		  if(DAC_VAL2[0]==4096) DAC_VAL2[0]=0;
		  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
		  //------------ DAC MODO BLOQUEANTE --------------------//
		  //HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,DAC_VAL[0]);
		  //HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,DAC_VAL2[0]);
		  //------------ DAC MODO BLOQUEANTE --------------------//
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
		  if(min_ticks==2)//if(min_ticks==10)
			  {
			  	  min_ticks=0;
			  	  /*
			  	  CopiaVector(mb._MBUS_RCVD,wf._dataRCV,wf._n_dataRCV,0,'A');
			  	  mb._n_MBUS_RCVD=wf._n_dataRCV;
			  	  CopiaVector(mb._MBUS_RCVD,wf._dataRCV,5,0,'A');  //Solo copio la info si es correcta
			  	  ModBUS(&mb);
			 	  CopiaVector(wf._data2SND,mb._MBUS_2SND,mb._n_MBUS_2SND,0,'A');
			 	  wf._n_D2SND=mb._n_MBUS_2SND;*/

			  	/*  if (conexion>=4)
			  	  {
			  	  wf._estado_conexion=3;
			  	  EnviarDatos(&wf);
			  	 //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);

			  	  }*/
			  }
	  }

		if(EN_UART1_TMR==1) UART1_ticks++;

		if(UART1_ticks>=2)//if(UART1_ticks>=10)
		{
			UART1_ticks=0;
			FLAG_UART1=1;
			EN_UART1_TMR=0;
			items_rx=uart1pass;
			uart1pass=0;
		}

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	if(wf._estado_conexion==4)//if((wf._estado_conexion!=1)&&(wf._estado_conexion!=2)&&(resultado!=20)&&(resultado!=24)) //Solo cuento cuando no estahaciendo otra cosa
	{
		ticks++;
	}
	else
	{
		ticks=0;
	}

if(wf._ejecucion==1)
	{
		if (FLAG_TIMEOUT!=1)
		{
			if(wf._instruccion!=2) wf._ticks++;//-----------------------Solo cuento una vez reconcido el timeout, cuando entro al timeout no cuento
			if(wf._instruccion==2) wf._ticks2++;
		}


		if ((wf._instruccion!=2)&&(wf._ticks > 5500)) //if (wf._ticks > 5000)
		{
			FLAG_TIMEOUT=1;
			if(huart2.Instance->CR1 == 0x200C)  //--------------------Evito error UART colgado
			{
				HAL_UART_Receive_IT(&huart2,(uint8_t *)UART_RX_byte,1);
				EN_UART2_TMR=0; //OBS-VER Para que me vuelva a habilitar el timer
			}
			//wf._ticks=0;
		}
		if ((wf._instruccion==2)&&(wf._ticks2 > 20500)) //if (wf._ticks > 5000)
		{
			FLAG_TIMEOUT=1;
			if(huart2.Instance->CR1 == 0x200C)  //--------------------Evito error UART colgado
			{
				HAL_UART_Receive_IT(&huart2,(uint8_t *)UART_RX_byte,1);
				EN_UART2_TMR=0; //OBS-VER Para que me vuelva a habilitar el timer
			}
			//wf._ticks=0;
		}

	}
	else
	{
		wf._ticks=0;
	}
  /* USER CODE END SysTick_IRQn 1 */
}
void HAL_USART_ErrorCallback(UART_HandleTypeDef *huart2)
{
	 volatile int aore=0;
	 volatile int bore=0;

	// if ( UART_FLAG_ORE == HAL_UART_GetError(huart1))
	//{
	//Al leer los registros de esta forma SR y luego DR se resetean los errores de Framing Noise y Overrun FE NE ORE
	//}
	 	 wf._debug_count9++;
		aore=huart2->Instance->SR;
		bore=huart2->Instance->DR;


	//HAL_UART_Transmit_IT(&huart2,"U4",strlen("U4"));
	 HAL_UART_DeInit(huart2);
	 MX_USART2_UART_Init();
	 HAL_UART_Receive_IT(huart2,(uint8_t *)UART_RX_byte,1);

}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim2)
{

		wf._debug_count10++;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *INTSERIE)
{

	if(INTSERIE->Instance==USART1)
	 {
		EN_UART1_TMR=1;									//Habilito Timeout de software
		UART1_ticks=0;										//Reseteo Timer
		datarx_uart1[uart1pass]=datarx1[0];
		uart1pass++;
		if(uart1pass>=384) uart1pass=384;									//limito el vector para evitar que cuelgue el micro
		HAL_UART_Receive_IT(INTSERIE,(uint8_t *)datarx1,1);
	 }

	if(INTSERIE->Instance==USART2)
		 {
			UART_RX_vect[UART_RX_pos]=UART_RX_byte[0];
			UART_RX_pos++;
			if(UART_RX_pos>=384) UART_RX_pos=384;
			HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);//HAL_TIM_Base_Start_IT(&htim7);	//Habilito el timer
			TIM2->CNT=1;
			EN_UART2_TMR=1;	//Habilito Timeout de software
			HAL_UART_Receive_IT(INTSERIE,(uint8_t *)UART_RX_byte,1);
		 }

 }

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim2)
	{
		 HAL_TIM_OC_Stop_IT(htim2, TIM_CHANNEL_1); //Paro el timer
		 FLAG_UART2=1;
		 EN_UART2_TMR=0;
		 UART_RX_items=UART_RX_pos;
		 UART_RX_pos=0;
		 UART_RX_vect[384]='\0'; //Finalizo el vector a la fuerza ya que recibo hasta 124
		 CopiaVector(UART_RX_vect_hld,UART_RX_vect,UART_RX_items,1,CMP_VECT);
		 HAL_UART_Receive_IT(&huart2,(uint8_t *)UART_RX_byte,1); //Habilito le recepcón de puerto serie al terminar

		 if (ITM_AT_debug==1)
		 {
			 ITM0_Write((uint8_t *)UART_RX_vect_hld, UART_RX_items);
		 }
		 if (AT_PROC_ON==0)
		 {
		 HAL_UART_Transmit_IT(&huart1,(uint8_t *)UART_RX_vect_hld, UART_RX_items);
		 }
}


void ESP8266_HW_Reset(void)
{
	  ESP_REinit=0;
	  PIN_RESET_ON;
	  HAL_Delay(2000);											//Tiempo de reset del módulo
	  PIN_RESET_OFF;		//Habilito módulo
}

void ESP8266_Restart(void)
{
		PIN_RESET_ON;
		HAL_Delay(2000);//210419
		PIN_RESET_OFF;
		HAL_Delay(5000);//210419
		esp_restart=0;
}
void ITM0_Write(char *text, int length)
{
	for(int i=0; i<length; i++)
	{
		//ITM_Port32(_nport)=*text++;
		ITM_SendChar(*text++);
	}
}

uint8_t ESP8266_HW_Init(UART_HandleTypeDef *SerialPort) //Devuelve 1 si reinició OK, y 0 si no
{
		ITM0_Write(" ESP Restart\r", strlen(" ESP Restart\r"));
	  do{
		  HAL_UART_Transmit(SerialPort, "AT+RESTORE\r\n",strlen("AT+RESTORE\r\n"),100);
		  HAL_Delay(500);

		  wf._n_fcomp=strlen("ready");//wf._n_fcomp=strlen("Ai-Thinker Technology Co.,Ltd.");
		  wf._n_orig=UART_RX_items;

		  while(FT_String_ND(UART_RX_vect_hld,&wf._n_orig,"ready",&wf._n_fcomp,wf._uartRCVD_tok,&wf._n_tok,&ntestc,&wf._id_conn,FIND)!=1)//while(FT_String_ND(UART_RX_vect_hld,&wf._n_orig,"Ai-Thinker Technology Co.,Ltd.",&wf._n_fcomp,wf._uartRCVD_tok,&wf._n_tok,&ntestc,&wf._id_conn,FIND)!=1)
		  {
			  	  wf._n_orig=UART_RX_items;
			  	  if (ESP_ticks>=5000)
			  		 {
			  		 ESP_InitF=1;
			  		 break;
			  		 }

		  }

		  if(ESP_InitF==0)	//Si encontró la sentencia anterior analizo la siguiente
		  {
			  wf._n_fcomp=strlen("ready");
			  wf._n_orig=UART_RX_items;
			  while(FT_String_ND(UART_RX_vect_hld,&wf._n_orig,"ready",&wf._n_fcomp,wf._uartRCVD_tok,&wf._n_tok,&ntestc,&wf._id_conn,FIND)!=1)
			  {
				  wf._n_orig=UART_RX_items;
				  if (ESP_ticks>=5000)
					 {
					 break;
					 }
			  }
		  }

		  if (ESP_ticks<5000)
		  {
			  ESP_REinit=10;
			  ESP_ticks=0;
		  }
		  else
		  {
			  ESP_REinit++;
			  ITM0_Write(" ESP Reinit\r", strlen(" ESP Reinit\r"));
			  ESP_ticks=0;
		  }

	  } while (ESP_REinit<=5);

	  if(ESP_REinit==10)
	  {
		  return(1);
	  }
	  else
	  {
		  return(0);
	  }
}
void BorrarVect(void)
{
	wf._uartRCVD[0]='\0';
	wf._uartRCVD[1]='\0';
	wf._n_orig=2;
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
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
