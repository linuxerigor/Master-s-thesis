#include <xc.h>
#include <math.h>
#include "main.h"
/****************************************
 * File:   main.c
 * Author: Marques-carneiro, Igor
 *
 * Created on 24 de Agosto de 2017
 ****************************************/
//1- Seleciona o oscilador
_FOSCSEL(FNOSC_PRIPLL & IESO_OFF);
//2- Configura a troca para o oscilador 
_FOSC( POSCMD_HS & OSCIOFNC_OFF &  FCKSM_CSECMD );
//Watchdog Timer Off
_FWDT(FWDTEN_OFF); 

int buffer_adc1[SAMPLES],i1=0;
int buffer_adc2[SAMPLES],i2=0;

/*SAMP_BUFF_SIZE*/
int  BufferA1[DMA_BUFF_SIZE] __attribute__((space(dma),aligned(DMA_BUFF_SIZE)));
int  BufferB1[DMA_BUFF_SIZE] __attribute__((space(dma),aligned(DMA_BUFF_SIZE)));
int  tempRX2;
unsigned char Gain_DAC_A  = 0x30;
unsigned char Gain_DAC_B  = 0xB0;

/*	*	*	*	*	*	*	*	*	*	*	*	*
 *			PLL		                    		*
 *	*	*	*	*	*	*	*	*	*	*	*	*/

void initPLL(void)
{
	// osc interno =  FIN =  7.37 MHz
	// osc cristal =  FIN =  20 MHz
	CLKDIVbits.DOZEN  = 0;           
	CLKDIVbits.PLLPRE = 0b000;       
	CLKDIVbits.PLLPOST = 0b000;       
	PLLFBDbits.PLLDIV = 14;             
	
	OSCCONbits.CLKLOCK = 0;       // Clock and PLL selections are not locked, configurations may be modified
	OSCCONbits.LPOSCEN = 0;       // Disable secondary oscillator
	OSCCONbits.NOSC    = 0b011;       // Primary oscillator (XT, HS, EC) with PLL
	OSCCONbits.OSWEN   = 1;       // Request oscillator switch to selection specified by NOSC<2:0> bits
    
    // espera a troca para o clock principal 0b011
    while (OSCCONbits.COSC != 0b011);

    // Espera o PLL ser ativado
    while(OSCCONbits.LOCK != 1) {};
}

/*	*	*	*	*	*	*	*	*	*	*	*	*
 *				ADC1			                *
 *	*	*	*	*	*	*	*	*	*	*	*	*/
// igor
void initAdc1(void)
{
	AD1CON1bits.FORM  = 0;		//0 = ADC insigned int  /  1 = int
	AD1CON1bits.SSRC  = 2;		//Fonte de clock para amostragem: Timer
	AD1CON1bits.ASAM  = 1;		//ADC Sample Control: Amostra inicia imediatamente
	AD1CON1bits.AD12B = 0;		//10-bit 
	AD1CON2bits.CSCNA = 0;		//Escolhe entradas para Scan
	AD1CON2bits.CHPS  = 1;		//Converte CH0 do ADC1
	AD1CON2bits.ALTS  = 0;		 // 1 = Enable Alternate Input Selection
	// This bit is only used when the SSRC<2:0> bits (ADxCON1<7:5>) = 111.
	AD1CON3bits.ADRC  = 0;		// 0 = Clock derivado do principal
	AD1CON3bits.SAMC  = 1;		// sempre 1 para simultaneo
	AD1CON3bits.ADCS  = 1;		//This bit is not used if ADxCON3<15> (ADRC) = 1
								//This bit only used if ADxCON1<7:5> (SSRC<2:0>) = 111
	AD1CON1bits.ADDMABM = 0; 	//If this bit is set, DMA buffers are written in the order of conversion.
	AD1CON2bits.SMPI    = (NUM_CHS2SCAN-1);    //2 entradas avaliadas
	AD1CON4bits.DMABL   = 7;	//Cada buffer com 128 palavras
	AD1CON1bits.SIMSAM 	= 1;
 //	AD1CON2bits.BUFM    = 1; //Always starts filling buffer at address 0x0
	AD1CON2bits.VCFG    = 1;//0;       // External VREF+ External VREF- 
    AD1CHS0bits.CH0SA = 4;      // Select AN4 for CH0 +ve input
    AD1CHS0bits.CH0NA = 0;      // Select VREF- for CH0 -ve input
    AD1CHS0bits.CH0SB = 4;      // Select AN4 for CH0 +ve input
    AD1CHS0bits.CH0NB = 0;      // Select VREF- for CH0 -ve input
   /* trecho onde selecionamos os canais e as entradas */
	AD1CHS123bits.CH123SA = 1 ;	// = CH1 positive input is AN3, CH2 positive input is AN4, CH3 positive input is AN5
	AD1CHS123bits.CH123NA = 0 ;	// 0x = CH1, CH2, CH3 negative input is VREFL
	AD1CHS123bits.CH123SB = 1 ;	// = CH1 positive input is AN3, CH2 positive input is AN4, CH3 positive input is AN5
	AD1CHS123bits.CH123NB = 0 ;	// 0x = CH1, CH2, CH3 negative input is VREFL
							// ou seja, CH1 para AN# e CH2 para AN4
							// CH1 -> AN3
							// CH2 -> AN4
    AD1CSSH = 0x0000; 
    AD1CSSL = 0x0000;
    AD1CSSLbits.CSS3   = 1;	    //Scan em AN3
    AD1CSSLbits.CSS4   = 1;     //Scan em AN4

 	AD1PCFGL=0xFFFF;
    AD1PCFGH=0xFFFF;
    AD1PCFGLbits.PCFG3 = 0;		//AN3 como entrada 
	AD1PCFGLbits.PCFG4 = 0;		// set AN4 as Analog Input

	IFS0bits.AD1IF   = 0;	    //Limpa flag
	IEC0bits.AD1IE   = 0;	    //Desliga interrupt
	AD1CON1bits.ADON = 1;	    //Liga CAD
}

/*	*	*	*	*	*	*	*	*	*	*	*	*
 *				 Timer3		                	*
 *	*	*	*	*	*	*	*	*	*	*	*	*/
void initTmr3(void) 
{
	IPC2bits.T3IP = 0x01;     // Set Timer3 Interrupt Priority Level
	IFS0bits.T3IF = 0;        // Clear Timer3 Interrupt Flag
	IEC0bits.T3IE = 0;        // 1 = Enable Timer3 interrupt

    PR3 = (int) ( FCY/ (FA) );

	TMR3HLD = 0x0000; 
	TMR3    = 0x0000;
	TMR2    = 0x0000;

	T2CONbits.TCS = 0;       // Internal clock (FOSC /2)
	T2CONbits.TCKPS = 0b00; 
	T3CONbits.TON = 1;
}
/*	*	*	*	*	*	*	*	*	*	*	*	*
 *				     DMA		            	*
 *	*	*	*	*	*	*	*	*	*	*	*	*/
void initDma0(void)
{
    DMA0CONbits.AMODE = 0;	//  = 0 Acesso indireto a DMA with Post-Increment mode
	DMA0CONbits.MODE  = 2;	//  = 2 Continuous, Modo Ping-Pong
	DMA0CONbits.HALF  = 0;  //
    DMA0CONbits.SIZE  = 0;  //  word    
	DMA0CONbits.DIR   = 0;  //  = 0Read from peripheral address, write to DMA RAM address

	DMA0PAD=(volatile unsigned int)&ADC1BUF0;
	DMA0CNT = (DMA_BUFF_SIZE - 1);           //(SAMP_BUFF_SIZE*NUM_CHS2SCAN)-1;					
	DMA0REQ = 13;		 	 		//Seleciona ADC1 como fonte para DMA

	DMA0STA = __builtin_dmaoffset(&BufferA1);		
	DMA0STB = __builtin_dmaoffset(&BufferB1);

	IFS0bits.DMA0IF = 0;	//Limpa flag
    IEC0bits.DMA0IE = 1;	//Habilita interrupt de DMA  m                                                             
	DMA0CONbits.CHEN= 1;	//Habilita DMA

	IFS0bits.DMA0IF = 0;
}


/*	*	*	*	*	*	*	*	*	*	*	*	*
 *	  	         IOs		                	*
 *	*	*	*	*	*	*	*	*	*	*	*	*/
void initIOs(void)
{
	TRISDbits.TRISD0=0;		//Configura I/Os para Chip Enable SPI DAC1
	TRISDbits.TRISD1=0;     //Configura I/Os para Chip Enable SPI DAC2
	TRISDbits.TRISD2=0;		//Configura I/Os para Chip Enable SPI DDS
	TRISDbits.TRISD4=0;		//Configura I/Os
	TRISDbits.TRISD5=0;
	TRISDbits.TRISD6=0;		//Configura I/Os
	TRISDbits.TRISD7=0;
	TRISGbits.TRISG0=0;		//Configura I/Os
	TRISGbits.TRISG1=0;
	TRISGbits.TRISG2=0;		//Configura I/Os
	TRISGbits.TRISG3=0;
	TRISGbits.TRISG6=0;
	TRISGbits.TRISG7=1;
	TRISGbits.TRISG8=0;
    TRISBbits.TRISB0 = 1;       // The port pins that are desired as analog inputs must have
	TRISBbits.TRISB1 = 1;		// their corresponding TRIS bit set (input).
}

unsigned char FLAG_DMA_WRITE = 0;
unsigned char FLAG_CPU_READ  = 0;

/*	*	*	*	*	*	*	*	*	*	*	*	*
 *				 DMA			            	*
 *	*	*	*	*	*	*	*	*	*	*	*	*/
void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
	static int DmaBuffer = 0;
	int i;
	
	if(DmaBuffer == 0)	
	{	
	    if (FLAG_CPU_READ == 0 )
		{
			FLAG_DMA_WRITE = 1;
	   		for ( i = 0; i < (DMA_BUFF_SIZE/NUM_CHS2SCAN) ; i++)
	   		{	
            	buffer_adc1[i1] = BufferA1[i*NUM_CHS2SCAN];
	        	buffer_adc2[i1] = BufferA1[i*NUM_CHS2SCAN+1];
	        	i1++;
	  	    	if( i1 >= SAMPLES)
				{
					i1  = 0;
					IEC0bits.DMA0IE = 0;
				}
	   		}	
		}
	}	
	else
	{
	    if (FLAG_CPU_READ == 0 )
		{
		   FLAG_DMA_WRITE = 1;
		   for ( i =0; i < (DMA_BUFF_SIZE/NUM_CHS2SCAN) ; i ++ )
		   {	
	   			buffer_adc1[i1] = BufferB1[i*NUM_CHS2SCAN];
	   	 		buffer_adc2[i1] = BufferB1[i*NUM_CHS2SCAN+1];
	   			i1++;
	   			if( i1 >= SAMPLES)
				{
	       			i1 = 0;
					IEC0bits.DMA0IE = 0;	
				}
	   		}	
		}
	}		
	DmaBuffer ^= 1;
	IFS0bits.DMA0IF = 0;	//Limpa flag de interrupt
    FLAG_DMA_WRITE  = 0;
}

/*	*	*	*	*	*	*	*	*	*	*	*	*
 *				 delay                          *
 *	*	*	*	*	*	*	*	*	*	*	*	*/
void Delay_ms(unsigned int N)
{
	unsigned int j,i;
	while(N--)
 			for(j=0;j < MILLISEC;j++);
}	
	 
void InitUART2(void)
{
    U2MODE = 0x0000;
    U2MODEbits.UARTEN = 0;
    //  U2BRG = (FCY/(16*UART2_BAUD))-1
	//  U2BRG = (37M/(16*9600))-1
	//  U2BRG = 240
    U2BRG 	= 42; // 42 = 57600    e 259 = 9600
    //U2MODEbits.RTSMD = 1;
    U2STA = 0x0000;
    IPC7 = 0x4400;
    IFS1bits.U2TXIF = 0;	// Clear the Transmit Interrupt Flag
	IEC1bits.U2TXIE = 0;//	// Enable Transmit Interrupts
	IFS1bits.U2RXIF = 0;	// Clear the Recieve Interrupt Flag
	IEC1bits.U2RXIE = 0;//	// Enable Recieve Interrupts
    U2MODEbits.UARTEN = 1;    
 	U2STAbits.UTXEN = 1;
    Delay_ms(10);
}

/*	*	*	*	*	*	*	*	*	*	*	*	*
 *	   		  envio de CHR	            		*
 *	*	*	*	*	*	*	*	*	*	*	*	*/
void txCHR(unsigned char dado)
{
    while (!U2STAbits.TRMT);
    U2TXREG=dado;
	while (!U2STAbits.TRMT);
    IFS1bits.U2TXIF = 0;
}

void lockin_pc()
{
    int  cont;
	char dado;
	for ( cont = 0; cont < SAMPLES; cont++)
	{
		  buffer_adc1[cont] = 0;
		  buffer_adc2[cont] = 0;
	}

    FLAG_DMA_WRITE = 0;
    FLAG_CPU_READ  = 0;
    LED_TESTE =0;
 
    while (1)               //Loop principal
    {
        while(!IFS1bits.U2RXIF);
            LED_TESTE = ~LED_TESTE;
             dado  = U2RXREG;
             IFS1bits.U2RXIF = 0;
             txCHR(dado);	 
             IEC0bits.DMA0IE = 0;	

             for(cont=0; cont < SAMPLES; cont++)
             {	
                dado = buffer_adc1[cont];
                txCHR(dado); 		 	

                dado = (buffer_adc1[cont]>>8);
                txCHR(dado); 		 	
             } 

             for(cont=0; cont < SAMPLES; cont++)
             {	
                dado = buffer_adc2[cont];
                txCHR(dado); 		 	

                dado = (buffer_adc2[cont]>>8);
                txCHR(dado); 		 	
             }	   		  

             txCHR('F');
             txCHR('I');	   		 
             txCHR('M');
             txCHR(13);	   		 
             txCHR(10);

             i1 = 0;	
             IFS0bits.DMA0IF  = 0;	//Limpa flag
             IEC0bits.DMA0IE  = 1;	//Habilita interrupt de DMA
             DMA0CONbits.CHEN = 1;	//Habilita DMA
             FLAG_CPU_READ    = 0;
    }
}


void lockin_igor()
{
    int  cont;
	char dado;
	int V_ref[SAMPLES];
	int pontos=0;
	char  txt[50];
    double  t[SAMPLES];
    int X = 0;
    int Y = 0;
    int mag = 0;
    int P = 0;
       
	for ( cont = 0; cont < SAMPLES; cont++)
	{
		  buffer_adc1[cont] = 0;
		  buffer_adc2[cont] = 0;
          V_ref[cont] = 0;
	}
    FLAG_DMA_WRITE = 0;
    FLAG_CPU_READ  = 0;
    LED_TESTE =0;
 
    while (1)               //Loop principal
    {
        while(!IFS1bits.U2RXIF);
            LED_TESTE = ~LED_TESTE;
             dado  = U2RXREG;
             IFS1bits.U2RXIF = 0;
             while(i1 != 0 );
             IEC0bits.DMA0IE = 0;	
            // simulando um sinal de entrada em kHz
            //simular_sinal(buffer_adc1, 20, 0);  // sinal medido
            //simular_sinal(buffer_adc2, 20, 0);  // referencia in
            
			adc_shift(buffer_adc1); 
			adc_shift(buffer_adc2);

			pontos = adc_pll(buffer_adc2, V_ref);
            
            lockin(buffer_adc1, V_ref, pontos);
            
             i1 = 0;	
             IFS0bits.DMA0IF  = 0;	//Limpa flag
             IEC0bits.DMA0IE  = 1;	//Habilita interrupt de DMA
             DMA0CONbits.CHEN = 1;	//Habilita DMA
             FLAG_CPU_READ    = 0;
    }
}
	 
/*	*	*	*	*	*	*	*	*	*	*	*	*
 *				 Principal			        	*
 *	*	*	*	*	*	*	*	*	*	*	*	*/
int main (void)
{
    int  cont;
	char dado;

	initPLL  ();			 //Inicializa PLL			
 	initAdc1 ();             //Inicializa ADC
	initDma0 ();			 //Inicializa DMA
    initTmr3 ();			 //Inializa Timer3
	initIOs  ();			 //Inicializa IOs

	InitUART2();			 //Inicializa UART
	Delay_ms(2);
    
     while (1)               //Loop principal
    {
        while(!IFS1bits.U2RXIF);   
             dado  = U2RXREG;
             IFS1bits.U2RXIF = 0;
                if(dado == 'I') 
                    lockin_igor();
                else if(dado == 'A')
                    lockin_pc();
            
     }
 
}
