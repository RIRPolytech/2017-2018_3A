/*
 * PID_RIR.c
 *
 *  Created on: 24 janv. 2018
 *      Author: julien MERIMEE
 */

#include "PID_RIR.h"


Str_PID setPIDCoef(double kp_dist, double ki_dist, double kd_dist, double kp_angl, double ki_angl, double kd_angl,Str_PID PID)
{
	PID.Kp_dist = kp_dist;
	PID.Ki_dist = ki_dist;
	PID.Kd_dist = kd_dist;
	PID.Kp_angl = kp_angl;
	PID.Ki_angl = ki_angl;
	PID.Kd_angl = kd_angl;
	PID.X = 0;
	PID.Y = 0;
	PID.Theta = 0;
	PID.ErrorSumDist = 0;
	return PID ;

}

bool PID_calc(double x, double y, double theta, int config,Str_PID PID,Coordinated CurrentPos)
{
    int stateDist, stateAng;
    switch (config) {
        case 1:
        stateDist = 1;
        stateAng = 1;
        break;

        case 2:
        stateDist = -1;
        stateAng = 1;
        break;

        case 3:
        stateDist = 0;
        stateAng = 1;
        break;

        default:
        stateDist = 1;
        stateAng = 1;
        break;
    }

    // Reset sum error
    if(x != PID.X || y != PID.Y || theta != PID.Theta)
    {
        PID.ErrorSumDist = 0;
        PID.ErrorSumAngl = 0;
        PID.X0 = CurrentPos.X;
        PID.Y0 = CurrentPos.Y;
        PID.Theta0 = CurrentPos.theta;
    }
    PID.X = x;
    PID.Y = y;
    PID.Theta = theta;

    // Distance control
    double distSpeed = sqrt(pow(CurrentPos.X - CurrentPos.oldX, 2)+pow(CurrentPos.Y - CurrentPos.oldY, 2));
    double initDist = sqrt(pow(CurrentPos.X - PID.X0, 2)+pow(CurrentPos.Y - PID.Y0, 2));
    double distDiff = sqrt(pow(x - PID.X0, 2)+pow(y - PID.Y0, 2)) - initDist;
    //Serial.print(" initDist = ");
    //Serial.print(initDist);
    //Serial.print(" distDiff0 = ");
    //Serial.print(distDiff);
    if(initDist < 50)
        distDiff = initDist * 0.8 + 15;
    PID.ErrorSumDist += distDiff;
    if (PID.ErrorSumDist >= 5)
        PID.ErrorSumDist = 5;
    else if (PID.ErrorSumDist <= -5)
        PID.ErrorSumDist = -5;
    double distCommande = distDiff * PID.Kp_dist + PID.ErrorSumDist * PID.Ki_dist + distSpeed * PID.Kd_dist;

    if(distCommande > 70)
        distCommande = 70;
    else if(distCommande < -70)
        distCommande = -70;
    //Serial.print(" distDiff = ");
    //Serial.print(distDiff);
    //Serial.print(" mErrorSumDist = ");
    //Serial.print(mErrorSumDist);
    //Serial.print(" distSpeed = ");
    //Serial.print(distSpeed);

    // Angular control

    double anguSpeed = CurrentPos.theta - CurrentPos.oldTheta;
    double anguDiff = theta - CurrentPos.theta;
    PID.ErrorSumAngl += anguDiff;
    if (PID.ErrorSumAngl >= 5)
        PID.ErrorSumAngl = 5;
    else if (PID.ErrorSumAngl <= -5)
        PID.ErrorSumAngl = -5;
    double anguCommande = anguDiff * PID.Kp_angl + PID.ErrorSumAngl * PID.Ki_angl + anguSpeed * PID.Kd_angl;

    // Motor Control

    //Serial.print(" anguSpeed = ");
    //Serial.print(anguSpeed);
    //Serial.print(" anguDiff = ");
    //Serial.print(anguDiff);
    //Serial.print(" anguCommande = ");
    //Serial.print(anguCommande);
    PID.Dist = initDist;
    PID.Ang = anguDiff;
    PID.Right_order = stateDist * 1 * (distCommande + stateAng * anguCommande);
    PID.Left_order = stateDist * -1 * (distCommande - stateAng * anguCommande);


    //Serial.print(" pRight_order = ");
    //Serial.print(pRight_order);
    //Serial.print(" pLeft_order = ");
    //Serial.println(pLeft_order);

    if(distDiff < 5 && anguDiff < 0.18)
    {
        return true;
    }

    return false;

}
