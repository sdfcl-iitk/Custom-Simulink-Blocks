
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#ifndef MATLAB_MEX_FILE
#include <MPU9250_driver.h>
MPU9250_driver mpu;
#endif
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void MPU9250_Start_wrapper(void)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Custom Start code goes here.
 */
    #ifndef MATLAB_MEX_FILE
    mpu.begin();
    #endif
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void MPU9250_Outputs_wrapper(real_T *acc,
			real_T *gyro,
			real_T *mag,
			real_T *temp)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/* This sample sets the output equal to the input
      y0[0] = u0[0]; 
 For complex signals use: y0[0].re = u0[0].re; 
      y0[0].im = u0[0].im;
      y1[0].re = u1[0].re;
      y1[0].im = u1[0].im;
 */
    #ifndef MATLAB_MEX_FILE
    mpu.update();

    acc[0] = mpu.getAcc(0);
    acc[1] = mpu.getAcc(1);
    acc[2] = mpu.getAcc(2);

    gyro[0] = mpu.getGyro(0);
    gyro[1] = mpu.getGyro(1);
    gyro[2] = mpu.getGyro(2);

    mag[0] = mpu.getMag(0);
    mag[1] = mpu.getMag(1);
    mag[2] = mpu.getMag(2);

    temp[0] = mpu.getTemperature();
    #else
    acc[0] = 0;
    acc[1] = 0;
    acc[2] = 0;

    gyro[0] = 0;
    gyro[1] = 0;
    gyro[2] = 0;

    mag[0] = 0;
    mag[1] = 0;
    mag[2] = 0;

    temp[0] = 0;
    #endif
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


