#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/ssi.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//#include "printf.h"

/*GND - GND
  DT - PD2 - SSI1Rx
  SCK - PD0 - SSI1Clk
  Vcc - 3.3 V*/

void uart_Init(void) 
{
	//SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);


	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);                                    
  GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_2);   
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_2);  

  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

  IntMasterEnable();                                                             //enable processor interrupts
	
  IntEnable(INT_UART0);                                                          //enable the UART interrupt
	
  UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);                          //only enable RX and TX interrupts
}

uint32_t reset()
{
}

uint32_t ready()
{
}




// unsigned long hx_readMass()
uint32_t ReadCount(void)
{
	uint32_t Count = 0;
	uint32_t i;
	
	GPIO_PORTD_DATA_R |= 0x04; // ADDO = 1
	GPIO_PORTD_DATA_R &= ~0x01; // ADSK = 0
	
	Count = 0;
	
	// while (GPIO_PORTD_RIS_R&0x04); // while(ADDO); wait until Data Line goes LOW?
	while(GPIO_PORTD_DATA_R&0x04);
	
	for (i = 0; i < 24; i++)
	{
		SysCtlDelay(13333); //Delay for 1ms. delay = 1ms = .001s = x * 3 * 1/(40*(10^6))
		
		GPIO_PORTD_DATA_R |= 0x01; // ADSK = 1
		Count = Count << 1;
		
		GPIO_PORTD_DATA_R &= ~0x01; // ADSK = 0
		
		if (GPIO_PORTD_DATA_R&0x04)
		{
			Count++;
			// Count = Count << 1;
			// SysCtlDelay(13333);
		}
	}
	
	// pulse();
	
	GPIO_PORTD_DATA_R |= 0x01; // ADSK = 1
	
	Count = Count^0x800000;
	
	GPIO_PORTD_DATA_R &= ~0x01;
	return (Count);
	
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, 10000, 16); SSIEnable(SSI1_BASE);
	SSIEnable(SSI1_BASE);
	
	// sprintf(str, "%d", Count);
	// covert number to string. 	
}

int main(void)
{
	// initialize UART0
  uart_Init();
	
	ReadCount();
	
	//
	// Loop forever.
	//   
	
	while(1)
	{
	}
}

//#include "TM4C123GH6PM.h"

//void init_SSI1(void);
//// void LTC1661Write(int chan, short d);

//int main(void)
//{
//    int i;

//    init_SSI1();
//    
//    for(;;)
//    {
//        for (i = 0; i < 1023; i++)
//        {
//            // LTC1661Write(0, i); /* write a sawtooth to channel A */
//        }
//    }
//}

//void LTC1661Write(int chan, short data)
//{
//    GPIOF->DATA &= ~0x04;       /* assert SS low */
//    data = (data & 0x03FF) << 2; /* bit 1-0 unused */
//    if (chan == 0)              /* add control code with channel number */
//        data |= 0x9000;
//    else
//        data |= 0xA000;
//    while((SSI1->SR & 2) == 0); /* wait until FIFO not full */
//    SSI1->DR = data >> 8;       /* transmit high byte */
//    while((SSI1->SR & 2) == 0); /* wait until FIFO not full */
//    SSI1->DR = data & 0xFF;     /* transmit low byte */
//    while(SSI1->SR & 0x10);     /* wait until transmit complete */
//    GPIOF->DATA |= 0x04;        /* keep SS idle high */
//}

//void init_SSI1(void)
//{
//    SYSCTL->RCGCSSI |= 2;       /* enable clock to SSI1 */
//    SYSCTL->RCGCGPIO |= 8;      /* enable clock to GPIOD for SSI1 */
//    //SYSCTL->RCGCGPIO |= 0x20;   /* enable clock to GPIOF for slave select */

//    /* configure PORTD 2, 0 for SSI1 clock and Rx */
//    GPIOD->AMSEL &= ~0x05;      /* disable analog for these pins */
//    GPIOD->DEN |= 0x05;         /* and make them digital */
//    GPIOD->AFSEL |= 0x05;       /* enable alternate function */
//    GPIOD->PCTL &= ~0x0000F00F; /* assign pins to SSI1 */
//    GPIOD->PCTL |= 0x00002002;  /* assign pins to SSI1 */
//    	
//    /* configure PORTF 2 for slave select */
//    GPIOF->DEN |= 0x04;         /* make the pin digital */
//    GPIOF->DIR |= 0x04;         /* make the pin output */
//    GPIOF->DATA |= 0x04;        /* keep SS idle high */

//    /* SPI Master, POL = 0, PHA = 0, clock = 4 MHz, 16 bit data */
//    SSI1->CR1 = 0;          /* disable SSI and make it master */
//    SSI1->CC = 0;           /* use system clock */
//    SSI1->CPSR = 2;         /* prescaler divided by 2 */
//    SSI1->CR0 = 0x0007;     /* 8 MHz SSI clock, SPI mode, 8 bit data */
//    SSI1->CR1 |= 2;         /* enable SSI1 */
//}
