//
//  PID_RIR.h
//  PID_RIR
//
//  Created by Adrien Picquart on 22/03/2017.
//  Modified by Julien Merimee on 24/01/2017.
//  Copyright © 2017 Adrien Picquart. All rights reserved.
//

#ifndef __PID_RIR__
#define __PID_RIR__
#include "Stdbool.h"
#include "Odometry_RIR.h"
#include <math.h>


/***** PID Structure *****/
typedef struct
{
    /* Motor instruction */
    int Right_order;
    int Left_order;
    double Dist;
    double Ang;
    /* PID Coefficient */
    double Kp_dist;
    double Ki_dist;
    double Kd_dist;
    double Kp_angl;
    double Ki_angl;
    double Kd_angl;
  /* PID Order */
    double X;
    double Y;
    double Theta;
    double X0;
    double Y0;
    double Theta0;
    double ErrorSumDist;
    double ErrorSumAngl;
    double PreDist_order;
    double PreAngl_order;
    double Angl_order;
    double DistSpeed_order;
    double AnglSpeed_order;



    /* Acceleration Coefficient */
    double Vmax;
    double Amax;
    double Freq;
}Str_PID;
;
Str_PID setPIDCoef(double kp_dist, double ki_dist, double kd_dist, double kp_angl, double ki_angl, double kd_angl,Str_PID PID);
bool PID_calc(double x, double y, double theta, int config,Str_PID PID,Coordinated CurrentPos);
#endif
