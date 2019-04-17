/*
***********************************************************************
* thrustallocation.h:
* function for control allocation based on Quadratic programming, using
* Mosek solver API
* This header file can be read by C++ compilers
*
* by Hu.ZH(Mr.SJTU)
***********************************************************************
*/

#ifndef _THRUSTALLOCATION_H_
#define _THRUSTALLOCATION_H_
#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include "constants.h"
#include "mosek.h" /* Include the MOSEK definition file. */
#include "realtimedata.h"
#include "timecounter.hpp"

#define QP_THREADS_USED_FIRST 1
#define QP_THREADS_USED_SECOND 1
#define QP_THREADS_USED_THIRD 1

#endif /* _THRUSTALLOCATION_H_*/