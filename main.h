#include <xc.h> 
#include "lockin_dspic.h"
/****************************************
 * File:   main.h
 * Author: Marques-carneiro, Igor
 *
 * Created on 24 de Agosto de 2017
 ***************************************/
// Frequencia final de operação do microcontrolador
#define FCY (unsigned long int) 40000000

/* Boud rate do UART */
#define UART2_BAUD  (long int) 57600     

/* Frequencia de amostragem do ADC */
#define FA          (long int) 250000   // Hz

/*   Informacoes do DDS */
#define CLOCK 10000000   // clock do DDS
#define DDS_q 4294967296 // 2^32   (Coeficiente frequencia DDS)  
#define F_DDS 1000000 // 1MHz  (frequencia do sinal de saida do DDS )

#define MILLISEC FCY/10000

/* Funcoes de delay */
#define __delay_ms(d) { __delay32( (unsigned long) (((unsigned long long) d)*(FCY)/1000ULL)); }
#define __delay_us(d) { __delay32( (unsigned long) (((unsigned long long) d)*(FCY)/1000000ULL)); }

/*	*	*	*	*	*	*	*	*	*	*	*	*
 *			Configuração de variáveis			*
 *	*	*	*	*	*	*	*	*	*	*	*	*/
#define SAMPLES 1024
#define PI 3.14159265358979323846

/*	*	*	*	*	*	*	*	*	*	*	*	*
 *				Configuração de I/Os			*
 *	*	*	*	*	*	*	*	*	*	*	*	*/
#define LED_TESTE PORTDbits.RD7

#define PORT_SPI_CS_DAC1 PORTDbits.RD0
#define PORT_SPI_CS_DAC2 PORTDbits.RD1
#define PORT_SPI_CS_DDS  PORTDbits.RD2

/*	*	*	*	*	*	*	*	*	*	*	*	*
 *		Definição de constantes/variáveis		*
 *	*	*	*	*	*	*	*	*	*	*	*	*/
#define  DMA_BUFF_SIZE	 		128
#define  NUM_CHS2SCAN			2 


 
