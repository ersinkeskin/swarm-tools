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
    //double k = -5*1;
    
    b_ts = b_ts*rho;
//     double rho = 0;
    
    //b = b*rho;
    //k = k*(1-rho);
    
    srand(seed);
//     srand(6);
    //double temp2;
    
    //initialize link dists
    for (i=0; i<mLen; i++) {

            distLink[i] = 1.0;
            distLink1[i] = 1.0;
    }
    
    // init velArr
    for(i=0; i<NAGENT; i++) {
        velArr[i] = vel;
    }
    
    
    // iter start
    for(iter=0;iter<iterLim;iter++) {
        for (i=0; i<NAGENT; i++) {
            fx[i] = 0.0;
            fy[i] = 0.0;
        }
        
        //init end
        
        for (i=0; i<mLen; i++) {
            idx1 = (long) m1[i];
            idx2 = (long) m2[i];
            linkdistx = posx[idx1]-posx[idx2];
            linkdisty = posy[idx1]-posy[idx2];
            dist = sqrt(linkdistx*linkdistx+linkdisty*linkdisty);
            
//             temp = b_ts*(distLink[i]-distLink1[i])/distLink[i];
//             temp = b_ts*(dist-distLink1[i]);
//             distLink1[i] = dist;
            
            //temp = -5.0 - (-5.0)*linkIL[i]/sqrt(linkdistx*linkdistx+linkdisty*linkdisty);
            
//             linkforcex = temp*linkdistx;
//             linkforcey = temp*linkdisty;
            
            //temp = k*(1-1/distLink[i]); //-k(distlink-1)/distlink
//             temp = k*(distLink[i]-1.0)/distLink[i];
//             temp += k*(dist-1.0);
            
//             temp = temp/dist;
            
            temp = (b_ts*(dist-distLink1[i]) + k*(dist-1.0))/dist;
            distLink1[i] = dist;
            
            linkforcex = temp*linkdistx;
            linkforcey = temp*linkdisty;         
            
            fx[idx1] += linkforcex;
            fx[idx2] -= linkforcex;
            fy[idx1] += linkforcey;
            fy[idx2] -= linkforcey;
        }              
        
        sumCos = 0.0;
        sumSin = 0.0;
        for (i=0;i<NAGENT;i++) {
            cosOrt = cos(ort[i]);
            sinOrt = sin(ort[i]);
            
            // vel ctr
            
//             velSetp = (fx[i]*cosOrt + fy[i]*sinOrt)*alpha + vel;
//             Fc_ts = (velSetp - velArr[i])*ts;
//             velArr[i] = velArr[i] + Fc_ts;
//             posDot = velArr[i]*ts;
//             posx[i] += posDot*cosOrt;
//             posy[i] += posDot*sinOrt;
            
            
            // default
            
            temp = (fx[i]*cosOrt + fy[i]*sinOrt)*alpha_ts + vel_ts;
            posx[i] += temp*cosOrt;
            posy[i] += temp*sinOrt;
            
            
            rval = (double) rand()/RAND_MAX*2.0-1.0;
            ort[i] += beta_ts*(fy[i]*cosOrt-fx[i]*sinOrt) + rval*eta_ts_rad;
            
            //add if?
            sumCos += cosOrt;
            sumSin += sinOrt;
        }
        
        if(iter%LOGDS==0){
            psi[log_count] = sqrt(sumCos*sumCos+sumSin*sumSin)/NAGENT;
            log_count+=1;
        }
//         }
    }
//                 sumCos += cosOrt;
//             sumSin += sinOrt;
//             *psi = sqrt(sumCos*sumCos+sumSin*sumSin)/NAGENT;
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
