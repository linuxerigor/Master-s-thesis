#include "xc.h"
/****************************************
 * File:   lockin_dspic.c
 * Author: Marques-carneiro, Igor
 *
 * Created on 24 de Agosto de 2017
 ***************************************/

#include "main.h"

int simular_sinal(int  *b[], int setf, int p)
{
    int cont;
            for ( cont = p; cont < SAMPLES; cont++)
                  b[cont] = (int) (500.0 * (1 + sin(2.0 * PI * (double)(setf * 1000 )  * (double)((double)cont / FA)))) ;
}

int gerar_sin_lut(int  *b[], int setf, int p)
{
    int cont;
    for ( cont = 0; cont < SAMPLES; cont++)
        b[p+cont] = (int) (10.0 * (sin(2.0 * PI * (double)setf   * (double)((double)cont / FA)))) ;
    
    if(p != 0 ){
        for(cont=p ; cont >= 0  ; cont--)
            b[p-cont] = (int) (10.0 * (   sin(   (2.0 * PI * (double)setf * (double)((double)cont / FA)) + PI )  ) ) ;
    }
}

int adc_shift(int  *buffer[])
{
    int i;
	int max, min;
    
    max = 0;
    min = 32767;
    for(i=1 ; i < SAMPLES ; i++){
		if( buffer[i] > max) 
			max = buffer[i];
        if( buffer[i] < min ) 
			min = buffer[i];
    }

	for(i=0 ; i < SAMPLES ; i++)
        buffer[i] -=  ( max + min) / 4 ;
}

int adc_pll(int  *buffer[] , int *V_ref[])
{
    int i;
    int count = 0;
    int start = 0;
	int end = -1;
    int temp, tempold = 0;
    char txt[100];
    int pontos;
 
    // encontrar o periodo
	for(i=1 ; i < SAMPLES ; i++){
        temp = buffer[i];
        
		if( ( temp & 0x8000 )  != 0x8000){ 
            if( ( tempold & 0x8000 )  == 0x8000){ 
    			if(!start){
                    start = i;
                }else{
                    end = i;
                    count++;
                }
            }
        }
         tempold = temp;
	}
      
	pontos = (int)( (double)(end - start) / (double)count ) ;
    gerar_sin_lut(V_ref, (int)( (  FA  /  (double)(  ((double)(end - start) / (double)count)   )   )    ), start);  
    return pontos;
}

int lockin(int *V_sig , int *V_ref , int pontos )
{
    int i;
    double temp_x = 0;
    double temp_y = 0;
    double temp_f = 0;
    char txt[100];
    double mag;
    double P;
    unsigned char *dado;
            
    // Lock-in Digital
	for(i=0 ; i < (SAMPLES - (pontos/4)) ; i++){
		temp_x = temp_x + (double)(V_sig[i] * V_ref[i]);
	    temp_y = temp_y + (double)(V_sig[i] * V_ref[i+(pontos/4)]);   // Sig * Verf (90)
	}

	temp_x /= (double)(SAMPLES);
	temp_y /= (double)(SAMPLES);
   	
    mag =( (2.0/10.0) * (sqrt((( temp_x * temp_x ) + (temp_y * temp_y) )  )));
    P = atan2(temp_x,temp_y); 
    
   // modo de envio bytes
   dado = (unsigned char *) &mag;
   txCHR(*dado++);
   txCHR(*dado++);
   txCHR(*dado++);
   txCHR(*dado);
   
   dado = (unsigned char *) &P;
   txCHR(*dado++);
   txCHR(*dado++);
   txCHR(*dado++);
   txCHR(*dado);
   
   dado = (unsigned char *) &temp_x;
   txCHR(*dado++);
   txCHR(*dado++);
   txCHR(*dado++);
   txCHR(*dado);
   
   dado = (unsigned char *) &temp_y;
   txCHR(*dado++);
   txCHR(*dado++);
   txCHR(*dado++);
   txCHR(*dado);
   
   temp_f = ( FA  / pontos);
   dado = (unsigned char *)&temp_f;
   txCHR(*dado++);
   txCHR(*dado++);
   txCHR(*dado++);
   txCHR(*dado);
   
}

