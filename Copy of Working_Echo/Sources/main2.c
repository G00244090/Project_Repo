/*
 * main2.c
 *
 *  Created on: 29 Dec 2015
 *      Author: Aonghus
 */




/*
 *  Uses UART0 in interrupt mode to echo receive characters
 *
 *  Niall O'Keeffe
 *  V01
 *  11/11/15
 */

#include "board.h"
#include "fsl_clock_manager.h"
#include "fsl_debug_console.h"


#define RDRF_MASK 0x20	//Receive Data Register Full Flag Mask
#define RIE_MASK 0x20	//Receive Interrupt Enable Mask
#define TDRE_MASK 0x80u

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
char char_received();
void enable_UART0_receive_interrupt();
/*******************************************************************************
 * Code
 ******************************************************************************/

/*
 * UART0 Interrupt Handler
 * Echos received character
 */
void UART0_IRQHandler(void)
{
	PRINTF("Echo:");
    if(UART0_S1 & RDRF_MASK)
    	//PUTCHAR(UART0_D);
    	put_char(UART0_D);
}

void put_char(char c)
{
while((UART0_S1 & TDRE_MASK) == 0) //wait until tx buffer is empty
{}
UART0_D = c;
}

int main()
{
	hardware_init();
	enable_UART0_receive_interrupt();
	PRINTF("UART0 Test Code\n\r");
	PRINTF("Any entered character will be echoed\r\n\n");
	while(1)
	{
	}
}


char char_received()
{
	if(UART0_S1 & RDRF_MASK)
	{
		if((char *)UART0_D == 'A')
			PRINTF("hello Aonghus");
		return 1;
	}
	else
		return 0;
}


void enable_UART0_receive_interrupt()
{
	//Configure NVIC
	NVIC_ClearPendingIRQ(12);
	NVIC_EnableIRQ(12);
	UART0_C2 |= RIE_MASK;	//set RIE to enable receive interrupt
}




