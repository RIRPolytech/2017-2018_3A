//
//  Odometry_RIR.h
//  Odometry_RIR
//
//  Created by Adrien Picquart on 22/03/2017.
//modified by Julien MERIMEE on 23/01/2018
//  Copyright © 2017 Adrien Picquart. All rights reserved.
//


#ifndef __ODOMETRY_RIR__
#define __ODOMETRY_RIR__
#include "math.h"

# define WheelDist 329
# define NbTicks 4096
# define Pi 3.1419
# define Diameter 101.6


// X = Droit
// Y = Gauche

/***** Odometry Structure *****/
typedef struct
{
    long countX ;
    long countY ;
    long oldCountX ;
    long oldCountY ;
} Counter;

typedef struct
{
    double X ;
    double Y ;
    double theta ;
    double oldX ;
    double oldY ;
    double oldTheta ;
} Coordinated;



#endif
