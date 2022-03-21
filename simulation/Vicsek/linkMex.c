/*==========================================================
 * arrayProduct.c - example in MATLAB External Interfaces
 *
 * Multiplies an input scalar (multiplier)
 * times a 1xN matrix (inMatrix)
 * and outputs a 1xN matrix (outMatrix)
 *
 * The calling syntax is:
 *
 *		outMatrix = arrayProduct(multiplier, inMatrix)
 *
 * This is a MEX-file for MATLAB.
 * Copyright 2007-2012 The MathWorks, Inc.
 *
 *========================================================*/

#include "mex.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
#define NAGENT 1141
#define MLEN 3306
//#define NAGENT 10621
//#define MLEN 31506
#define LOGDS 1

/* The computational routine */
void linkMex(double *m1, double *m2, double *posx, double *posy, double *ort, double eta_ts_rad, double *psi,unsigned int seed,long mLen,long iterLim, double rho, double k, double alpha)
{
    long i,idx1,idx2;
    double linkdistx;
    double linkdisty;
    double temp;
    double linkforcex;
    double linkforcey;
    double cosOrt, sinOrt, sumCos, sumSin;
    double rval;
    double fx[NAGENT], fy[NAGENT], velArr[NAGENT];
    long log_count=0,iter;
    double distLink[MLEN], distLink1[MLEN];
    double dist;
    
    double Fc_ts,posDot,velSetp;
    
    double ts = 0.1;
    double b = -250*1;
    double b_ts = b/ts;
    //double alpha = 0.001;
    double alpha_ts = alpha*ts;
    double vel = 0.002;
    double vel_ts = vel*ts;
    double beta_ts = 0.12*ts;
    
    double cos_ort_old[NAGENT], sin_ort_old[NAGENT],cos_ort[NAGENT], sin_ort[NAGENT];
    //double k = -5*1;
    
    b_ts = b_ts*rho;
    
    srand(seed);
    
    for (i=0; i<NAGENT; i++) {
        cos_ort[i] = 0.0;
        sin_ort[i] = 0.0;
    }
    
    // iter start
    for(iter=0;iter<iterLim;iter++) {
        
        
        for (i=0; i<NAGENT; i++) {
            cos_ort_old[i] = cos_ort[i];
            sin_ort_old[i] = sin_ort[i];
            cos_ort[i] = 0.0;
            sin_ort[i] = 0.0;
        }
        
        
        for (i=0; i<MLEN; i++) {
            idx1 = (long) m1[i];
            idx2 = (long) m2[i];
            
            cos_ort[idx1] = cos_ort[idx1] + cos_ort_old[idx2];
            cos_ort[idx2] = cos_ort[idx2] + cos_ort_old[idx1];
            
            sin_ort[idx1] = sin_ort[idx1] + sin_ort_old[idx2];
            sin_ort[idx2] = sin_ort[idx2] + sin_ort_old[idx1];
            
        }
        
        for (i=0;i<NAGENT;i++) {
            rval = (double) rand()/RAND_MAX*2.0-1.0;
            
            temp = atan2(sin_ort[i],cos_ort[i]) + rval*eta_ts_rad;
            cos_ort[i] = cos(temp);
            sin_ort[i] = sin(temp);
            
        }
        
        if(iter%LOGDS==0){
            sumCos = 0;
            sumSin = 0;
            for(i=0;i<NAGENT;i++) {
                sumCos = sumCos + cos_ort[i];
                sumSin = sumSin + sin_ort[i];
            }
            psi[log_count] = sqrt(sumCos*sumCos+sumSin*sumSin)/NAGENT;
            log_count+=1;
        }
    }
}

/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray *prhs[])
{
    double *m1;
    double *m2;
    //double *linkIL;
    long lpos;
    long lm;
    double *posx;
    double *posy;
    double *ort;
    double *psi;
    double eta_ts_rad;
    unsigned int seed;
    long mLen;
    long iterLim;
    double rho;
    double k;
    double alpha;
    
    m1 = mxGetPr(prhs[0]);
    m2 = mxGetPr(prhs[1]);
    //linkIL = mxGetPr(prhs[2]);
    posx = mxGetPr(prhs[2]);
    posy = mxGetPr(prhs[3]);
    ort = mxGetPr(prhs[4]);
    eta_ts_rad = mxGetScalar(prhs[5]);
    seed = (unsigned int)mxGetScalar(prhs[6]);
    mLen = (long)mxGetScalar(prhs[7]);
    iterLim = (long)mxGetScalar(prhs[8]);
    rho = mxGetScalar(prhs[9]);
    k = mxGetScalar(prhs[10]);
    alpha = mxGetScalar(prhs[11]);
    /* get dimensions of the input matrix */
    lm = mxGetN(prhs[0]);
    lpos = mxGetN(prhs[3]);
    
    /* create the output matrix */
    plhs[0] = mxCreateDoubleMatrix(iterLim/LOGDS,1,mxREAL);
    psi = mxGetPr(plhs[0]);
    
    /* call the computational routine */
    linkMex(m1,m2,posx,posy,ort,eta_ts_rad,psi,seed,mLen,iterLim,rho,k,alpha);
}
