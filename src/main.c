/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "Odometry_RIR.h"

Coordinated Currentpos;
Counter codeurD;

int main(void)
{
while(1)
{
	codeurD.countX=30;
	codeurD.countY=31;

	Currentpos.X=250;
	Currentpos.Y=250;
	Currentpos.theta=25;

	Nposition(Currentpos,codeurD);
}
}
