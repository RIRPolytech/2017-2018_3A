/* Includes ------------------------------------------------------------------*/
#include "template.h"
#include "main.h"
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"

/*Defines---------------------------------------------------------------------*/
#define X_pos_1 10
#define Y_pos_1 200
#define W_1 50
#define H_1 50

#define X_pos_2 325
#define Y_pos_2 200
#define W_2 50
#define H_2 50


int id=1;

/* Private function prototypes -----------------------------------------------*/
void LCD_Config(void);
void SystemClock_Config(void);
void PID_X_Exec(void);
void CPU_CACHE_Enable(void);
void TS_Button_match(int buttonx, int buttony, int width, int hight);
void TS_Button_back(int buttonx, int buttony, int width, int hight);
void layer_choix (void);
void layer_match (void);


/* Private functions ---------------------------------------------------------*/
TS_StateTypeDef TS_State;
int Touchx ,Touchy;
int IsTouched;

int main(void)
{
	/* Enable the CPU Cache */
	CPU_CACHE_Enable();
	/* STM32F7xx HAL library initialization */
	HAL_Init();
	/* Configure the system clock to 216 MHz */
	SystemClock_Config();
	/* Configure TS : Initializes and configures the touch screen functionalities */
	BSP_TS_Init(100,100);
	/* Configure LCD : Only one layer is used */
	LCD_Config();
	/* Initializes the SDRAM device */
	BSP_SDRAM_Init();
	/* Initialize the Touch screen */
	BSP_TS_Init(480, 272);
	/* Enable the CRC Module */

	__CRC_CLK_ENABLE();
	/* Our main starts here */

	/* Pour afficher les différents layer utiliser la fonction switch avec 1 layer par case*/
	while(1)
	{
		switch(id)
		{
		case 1:
			layer_choix();
	  	break;

		case 2:
			layer_match();
		break;

		}

		PID_X_Exec();
	}
}

void LCD_Config(void)
{
  /* LCD Initialization */
  BSP_LCD_Init();

  /* LCD Initialization */
  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);

  /* Enable the LCD */
  BSP_LCD_DisplayOn();

  /* Select the LCD Background Layer  */
  BSP_LCD_SelectLayer(0);

  /* Clear the Background Layer */
  BSP_LCD_Clear(LCD_COLOR_WHITE);

  /* Configure the transparency for background */
  BSP_LCD_SetTransparency(0, 255);
}

void layer_choix (void)
{
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);

	BSP_LCD_DisplayStringAt(X_pos_1, Y_pos_1-30, (uint8_t*)"Match!", LEFT_MODE);
	BSP_LCD_DrawRect(X_pos_1,Y_pos_1,H_1,W_1);
	BSP_LCD_DisplayStringAt(0, 125, (uint8_t*)"Choice!", CENTER_MODE);
	TS_Button_match(X_pos_1,Y_pos_1,H_1,W_1);

}

void layer_match (void)
{


	BSP_LCD_DisplayStringAt(0, 0, (uint8_t*)"Match!", CENTER_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	BSP_LCD_SetBackColor(LCD_COLOR_GREEN);
	BSP_LCD_DrawRect(X_pos_2,Y_pos_2,H_2,W_2);
	TS_Button_back(X_pos_2,Y_pos_2,H_2,W_2);
}



void TS_Button_match(int buttonx, int buttony, int width, int hight)
{
	if(Touchx > buttonx && Touchx < buttonx+width)
	{
		if(Touchy > buttony && Touchy < buttony+hight && IsTouched == 1)
		{
			BSP_LCD_Clear(LCD_COLOR_GREEN); 		/* Nettoie la layer lorsqu'elle change*/
			id = 2;
		}
	}
}


void TS_Button_back(int buttonx, int buttony, int width, int hight)
{
	if(Touchx > buttonx && Touchx < buttonx+width)
	{
		if(Touchy > buttony && Touchy < buttony+hight && IsTouched == 1)
		{
			BSP_LCD_Clear(LCD_COLOR_ORANGE);/* Nettoie la layer lorsqu'elle change*/
			id = 1;
		}
	}
}


void PID_X_Exec(void)
{
  BSP_TS_GetState(&TS_State);
  if (TS_State.touchDetected)
  {
	IsTouched = 1;
	Touchx = (int)(TS_State.touchX[0]);
	Touchy = (int)(TS_State.touchY[0]);
  }

  else
  {
	if (IsTouched == 1)
	{
		IsTouched = 0;
	}
  }
}
