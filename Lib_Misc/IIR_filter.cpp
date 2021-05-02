#include "IIR_filter.h"
#include "mbed.h"
using namespace std;

/*
  IIR filter implemention for the following filter types:
  init for: first order differentiatior:   G(s) = s/(T*s + 1)
            first order lowpass with gain  G(s) = K/(T*s + 1)
            second order lowpass with gain G(s) = K*w0^2/(s^2 + 2*D*w0*s + w0*w0)        
            nth order, with arbitrary values
  the billinear transformation is used for s -> z
  reseting the filter only makes sence for static signals, whatch out if you're using the differnetiator
*/

// G(s) = s/(T*s + 1)
IIR_filter::IIR_filter(float T, float Ts){
           
    // filter orders
    nb = 1; // Filter Order
    na = 1; // Filter Order
    
    // filter coefficients
    B = (double*)malloc((nb+1)*sizeof(double));
    A = (double*)malloc(na*sizeof(double));    
    B[0] = 2.0/(2.0*(double)T + (double)Ts);
    B[1] = -B[0];
    A[0] = -(2.0*(double)T - (double)Ts)/(2.0*(double)T + (double)Ts);
    
    // signal arrays
    uk = (double*)malloc((nb+1)*sizeof(double));
    yk = (double*)malloc(na*sizeof(double));
    uk[0]= uk[1] = 0.0;
    yk[0] = 0.0;
    
    // dc-gain
    this->K = 0.0;
}

// G(s) = K/(T*s + 1)
IIR_filter::IIR_filter(float T, float Ts, float K){
    
    // filter orders
    nb = 1; // Filter Order
    na = 1; // Filter Order
    
    // filter coefficients
    B = (double*)malloc((nb+1)*sizeof(double));
    A = (double*)malloc(na*sizeof(double));      
    B[0] = (double)Ts/((double)Ts + 2.0*(double)T);
    B[1] = B[0];
    A[0] = ((double)Ts - 2.0*(double)T)/((double)Ts + 2.0*(double)T); 
    
    // signal arrays
    uk = (double*)malloc((nb+1)*sizeof(double));
    yk = (double*)malloc(na*sizeof(double));
    uk[0]= uk[1] = 0.0;
    yk[0] = 0.0;
    
    // dc-gain
    this->K = (double)K;
}

// G(s) = K*w0^2/(s^2 + 2*D*w0*s + w0^2) 
IIR_filter::IIR_filter(float w0, float D, float Ts, float K){
    
    // filter orders
    nb = 2; // Filter Order
    na = 2; // Filter Order
    
    // filter coefficients
    B = (double*)malloc((nb+1)*sizeof(double));
    A = (double*)malloc(na*sizeof(double));
    double k0 = (double)Ts*(double)Ts*(double)w0*(double)w0;
    double k1 = 4.0*(double)D*(double)Ts*(double)w0;
    double k2 = k0 + k1 + 4.0;    
    B[0] = (double)K*k0/k2;
    B[1] = 2.0*B[0];
    B[2] = B[0]; 
    A[0] = (2.0*k0 - 8.0)/k2;
    A[1] = (k0 - k1 + 4.0)/k2;
    
    // signal arrays
    uk = (double*)malloc((nb+1)*sizeof(double));
    yk = (double*)malloc(na*sizeof(double));
    uk[0]= uk[1] = uk[2] = 0.0;
    yk[0] = yk[1] = 0.0;
    
    // dc-gain
    this->K = (double)K;
}

IIR_filter::IIR_filter(float *b, float *a, int nb_, int na_){
    
    // filter orders
    this->nb = nb_-1;    // Filter Order
    this->na = na_;      // Filter Order
    
    // filter coefficients
    B = (double*)malloc((nb+1)*sizeof(double));
    A = (double*)malloc(na*sizeof(double));
    uk = (double*)malloc((nb+1)*sizeof(double));
    yk = (double*)malloc(na*sizeof(double));
    
    for(int k=0;k<=nb;k++){
        B[k]=b[k];
        uk[k]=0.0;
        }
    for(int k=0;k<na;k++){
        A[k] = a[k];
        yk[k] = 0.0;
        }
    
    // dc-gain
    this->K = 1.0;
}

    
IIR_filter::~IIR_filter() {} 
    
void IIR_filter::reset(float val) {
    for(int k=0;k < nb;k++)
        uk[k] = (double)val;
    for(int k=0;k < na;k++)
        yk[k] = (double)val*K;
        
}

/* 
    the filter is operating as follows: 
    (B[0] + B[1]*z^-1 + ... + B[nb]*z^-nb)*U(z) = (1 + A[0]*z^-1 + ... + A[na-1]*z^-na))*Y(z)
    y(n) =  B[0]*u(k)   + B[1]*u(k-1) + ... + B[nb]*u(k-nb) + ...
          - A[0]*y(k-1) - A[1]*y(k-2) - ... - A[na]*y(n-na)
*/
float IIR_filter::filter(double input){
    for(int k = nb;k > 0;k--)    // shift input values back
        uk[k] = uk[k-1];
    uk[0] = input;
    double ret = 0.0;
    for(int k = 0;k <= nb;k++)
        ret += B[k] * uk[k];
    for(int k = 0;k < na;k++)
        ret -= A[k] * yk[k];
    for(int k = na;k > 1;k--)
        yk[k-1] = yk[k-2];
    yk[0] = ret;
    return (float)ret;
}

