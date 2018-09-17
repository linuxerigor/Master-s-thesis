#include <xc.h>  
/****************************************
 * File:   lockin_dspic.h
 * Author: Marques-carneiro, Igor
 *
 * Created on 24 de Agosto de 2017
 ***************************************/
#include <math.h>
#include "main.h"
#include <stdio.h>

int simular_sinal(int  *, int ,int);
int gerar_sin_lut(int  *, int ,int);
int adc_shift(int *);
int adc_pll(int *, int *);
int lockin(int * , int * , int );
