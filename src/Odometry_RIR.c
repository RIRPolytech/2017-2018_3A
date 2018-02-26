/*
 * Odoetry_RIR.c
 *
 *  Created on: 23 janv. 2018
 *      Author: julien MERIMEE
 */
# include "Odometry_RIR.h"

// X= Droite Y= Gauche

Coordinated Nposition( Coordinated CurrentPos,Counter Count )
{
	double deltaLeft;
	double deltaRight;
	double deltaS;
	double deltaTheta;

	// old data

	Count.oldCountX = Count.countX;
	Count.oldCountY = Count.countY;
	CurrentPos.oldX = CurrentPos.X;
	CurrentPos.oldY = CurrentPos.Y;
	CurrentPos.oldTheta = CurrentPos.theta;

	// récupération des encodeurs

	Count.countX=50;
	Count.countY=49;

	// calcul de la position
	deltaLeft=(Count.countX -Count.oldCountX) / NbTicks * Pi * Diameter ;
	deltaRight = (Count.countY - Count.oldCountY) / NbTicks * Pi * Diameter;
	deltaTheta = (deltaRight - deltaLeft)/WheelDist;
	deltaS = (deltaLeft + deltaRight)/2;
	CurrentPos.X += deltaS*cos(CurrentPos.theta + deltaTheta/2);
	CurrentPos.Y += deltaS*sin(CurrentPos.theta + deltaTheta/2);
	CurrentPos.theta += deltaTheta;

	return CurrentPos;


}
