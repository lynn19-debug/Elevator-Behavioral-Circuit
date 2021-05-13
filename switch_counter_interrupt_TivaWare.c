#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/ssi.h"
#include "driverlib/uart.h"
#include "inc/tm4c123gh6pm.h"
#include "utils/uartstdio.h"

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
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_2);   
	// GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_2); ?

  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

  IntMasterEnable();                                                             //enable processor interrupts
	
  IntEnable(INT_UART0);                                                          //enable the UART interrupt
	
  UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);                          //only enable RX and TX interrupts
}

// unsigned long hx_readMass()
uint32_t ReadCount(void)
{
	uint32_t Count = 0;
	uint32_t i;
	
	GPIO_PORTD_DATA_R |= 0x04; // ADDO = 1
	GPIO_PORTD_DATA_R &= ~0x01; // ADSK = 0
	
	Count = 0;
	
	while (GPIO_PORTD_RIS_R&0x04); // while(ADDO); wait until Data Line goes LOW?
	// while(GPIO_PORTD_DATA_R&0x04);
	
	for (i = 0; i < 24; i++)
	{
		SysCtlDelay(13333); //Delay for 1ms. delay = 1ms = .001s = x * 3 * 1/(40*(10^6))
		
		GPIO_PORTD_DATA_R |= 0x01; // ADSK = 1
		Count = Count << 1;
		
		GPIO_PORTD_DATA_R &= ~0x01; // ADSK = 0
		
		if (GPIO_PORTD_RIS_R&0x04)
		{
			Count++;
			// Count = Count << 1;
			SysCtlDelay(13333);
		}
	}
	
	// pulse();
	
	GPIO_PORTD_DATA_R |= 0x01; // ADSK = 1
	
	Count = Count^0x800000;
	
	GPIO_PORTD_DATA_R &= ~0x01;
	return (Count);
	
	// sprintf(str, "%d", Count); ?	
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
