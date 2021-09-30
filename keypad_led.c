#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"   
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "TM4C123GH6PM.h"

/* Defines the size of rows and columns of keypad */
#define  RowsSize  4
#define  ColsSize  4

volatile unsigned long value;

#define KEYPAD_ROW GPIOE
#define KEYPAD_COL GPIOC

volatile unsigned long count = 0;

uint8_t current, previous, plus, minus;

uint32_t diff;

int row, col;

// Array of 4x4 to define characters which will be printe on specific key pressed 
int symbol[RowsSize][ColsSize] = {{1, 2, 3, 'A'},      
                                  {4, 5, 6, 'B'},      
                                  {7, 8, 9, 'C'},      
                                  {'*', 0,'#','D'}}; 

void delay_ms(int n); // mili second delay function 
void delay_us(int n); // micro second delay function 
																						 															 
void keypad_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	KEYPAD_ROW->DIR |= 0x0F;              // set row pins 3-0 as output 
	KEYPAD_ROW->DEN |= 0x0F;              // set row pins 3-0 as digital pins 
	KEYPAD_ROW->ODR |= 0x0F;              // set row pins 3-0 as open drain 
	 
	KEYPAD_COL->DIR &= ~0xF0;            // set column pin 7-4 as input 
	KEYPAD_COL->DEN |= 0xF0;             // set column pin 7-4 as digital pins 
	KEYPAD_COL->PUR |= 0xF0;             // enable pull-ups for pin 7-4 
  
	GPIOE->PDR |= 0x0F;                  // Enable pull down resistor on PORTE
	
//	IntEnable(INT_GPIOC);  							 // enable interrupt 30 in NVIC (GPIOC)
//	IntPrioritySet(INT_GPIOC, 0x00); 		 // configure GPIOC interrupt priority as 1-7.
//	GPIOIntEnable(GPIO_PORTC_BASE, GPIO_INT_PIN_4 | GPIO_INT_PIN_5 | GPIO_INT_PIN_6 | GPIO_INT_PIN_7);
//	GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_INT_PIN_4 | GPIO_INT_PIN_5 | GPIO_INT_PIN_6 | GPIO_INT_PIN_7, GPIO_FALLING_EDGE);
}																					 

// keypad_getkey() function returns the value of key pressed by the user by traversing columns and rows respectivley 
unsigned long keypad_getkey(void)
{
	int row, col;
	
	// check to see any key pressed first 
	KEYPAD_ROW->DATA = 0;              // enable all rows 
	col = KEYPAD_COL->DATA & 0xF0;     // read all columns 
  if (col == 0xF0); // return 0;     // no key pressed 

// If a key is pressed, it gets here to find out which key. 
// Although it is written as an infinite loop, it will take one of the breaks or return in one pass.
	while (1)
	{
		row = 0;
		KEYPAD_ROW->DATA = 0x0E; // enable row 0 
		delay_us(5);             // wait for signal to settle 
		col = KEYPAD_COL->DATA & 0xF0;
		if (col != 0xF0) break;
		 
		row = 1;
		KEYPAD_ROW->DATA = 0x0D; // enable row 1 
		delay_us(5);             // wait for signal to settle 
		col = KEYPAD_COL->DATA & 0xF0;
		if (col != 0xF0) break;
		 
		row = 2;
		KEYPAD_ROW->DATA = 0x0B; // enable row 2 
		delay_us(5);             // wait for signal to settle 
		col = KEYPAD_COL->DATA & 0xF0;
		if (col != 0xF0) break;
		 
		row = 3;
		KEYPAD_ROW->DATA = 0x07; // enable row 3 
		delay_us(5);             // wait for signal to settle 
		col = KEYPAD_COL->DATA & 0xF0;
		if (col != 0xF0) break;
		 
		// return 0; // if no key is pressed
	}
	 
	// gets here when one of the rows has key pressed 
	if (col == 0xE0) return symbol[row][0]; // key in column 0 
	if (col == 0xD0) return symbol[row][1]; // key in column 1 
	if (col == 0xB0) return symbol[row][2]; // key in column 2 
	if (col == 0x70) return symbol[row][3]; // key in column 3 
	// return 0; /* just to be safe */
}

/*Keypad
 * PC4 - C0
 * PC5 - C1
 * PC6 - C2
 * PC7 - C2
 * PE0 - R0
 * PE1 - R1
 * PE2 - R2
 * PE3 - R3
*/

//void get_key()
//{
//	
//}

//interrupt handler
//void GPIOPortC_Handler(void)
//{
//	if(GPIO_PORTC_RIS_R & 0x10) // C4, C0
//	{
//		GPIO_PORTC_ICR_R |= 0x10;

//		if((GPIO_PORTC_DATA_R & 0x10) == 0x00 && (GPIO_PORTE_DATA_R & 0x01) == 0x00) // R0
//		{
//			value = symbol[row][0];
//	  }
//		else if((GPIO_PORTC_DATA_R & 0x10) == 0x00 && (GPIO_PORTE_DATA_R & 0x02) == 0x00) // R1
//		{
//			value = symbol[row][0];
//	  }
//		else if((GPIO_PORTC_DATA_R & 0x10) == 0x00 && (GPIO_PORTE_DATA_R & 0x04) == 0x00) // R2
//		{
//			value = symbol[row][0];
//	  }
//		else if((GPIO_PORTC_DATA_R & 0x10) == 0x00 && (GPIO_PORTE_DATA_R & 0x08) == 0x00) // R3
//		{
//			value = symbol[row][0];
//	  }
//	}
//	
//	else if(GPIO_PORTC_RIS_R & 0x20) // C5, C1
//	{
//		GPIO_PORTC_ICR_R |= 0x20;

//		if((GPIO_PORTC_DATA_R & 0x20) == 0x00 && (GPIO_PORTE_DATA_R & 0x01) == 0x00) // R0
//		{
//			value = symbol[row][1];
//	  }
//		else if((GPIO_PORTC_DATA_R & 0x20) == 0x00 && (GPIO_PORTE_DATA_R & 0x02) == 0x00) // R1
//		{
//			value = symbol[row][1];
//	  }
//		else if((GPIO_PORTC_DATA_R & 0x20) == 0x00 && (GPIO_PORTE_DATA_R & 0x04) == 0x00) // R2
//		{
//			value = symbol[row][1];
//	  }
//		else if((GPIO_PORTC_DATA_R & 0x20) == 0x00 && (GPIO_PORTE_DATA_R & 0x08) == 0x00) // R3
//		{
//			value = symbol[row][1];
//	  }
//	}
//	
//	else if(GPIO_PORTC_RIS_R & 0x40) // C6, C2
//	{
//		GPIO_PORTC_ICR_R |= 0x40;

//		if((GPIO_PORTC_DATA_R & 0x40) == 0x00 && (GPIO_PORTE_DATA_R & 0x01) == 0x00) // R0
//		{
//			value = symbol[row][2];
//	  }
//		else if((GPIO_PORTC_DATA_R & 0x40) == 0x00 && (GPIO_PORTE_DATA_R & 0x02) == 0x00) // R1
//		{
//			value = symbol[row][2];
//	  }
//		else if((GPIO_PORTC_DATA_R & 0x40) == 0x00 && (GPIO_PORTE_DATA_R & 0x04) == 0x00) // R2
//		{
//			value = symbol[row][2];
//	  }
//		else if((GPIO_PORTC_DATA_R & 0x40) == 0x00 && (GPIO_PORTE_DATA_R & 0x08) == 0x00) // R3
//		{
//			value = symbol[row][2];
//	  }
//	}
//}

/*8x8 MATRIX
 * PA5 - DIN
 * PA3 - CS
 * PA2 - CLK*/

void Timer0A_Init(unsigned long period)
{   
	//
  // Enable Peripheral Clocks 
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);      // configure for 32-bit timer mode
  TimerLoadSet(TIMER0_BASE, TIMER_A, period -1);        // reload value
	IntPrioritySet(INT_TIMER0A, 0x00);  	                // configure Timer0A interrupt priority as 0
  IntEnable(INT_TIMER0A);    				                    // enable interrupt 19 in NVIC (Timer0A)
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);      // arm timeout interrupt
  TimerDisable(TIMER0_BASE, TIMER_A);                   // enable timer0A
}

void spi_write(uint16_t data)
{
    SSIDataPut(SSI0_BASE, data);
    while (SSIBusy(SSI0_BASE)){}
    return;
}

void display_led(unsigned long data)
{
	switch(data)
	{
		case 0: 
			spi_write(1<<8 | 0x3C); spi_write(2<<8 | 0x24);
			spi_write(3<<8 | 0x24); spi_write(4<<8 | 0x24);
			spi_write(5<<8 | 0x24); spi_write(6<<8 | 0x24);
			spi_write(7<<8 | 0x24); spi_write(8<<8 | 0x3C);
		break;
		case 1:
			spi_write(1<<8 | 0x08); spi_write(2<<8 | 0x18);
			spi_write(3<<8 | 0x28); spi_write(4<<8 | 0x08);
			spi_write(5<<8 | 0x08); spi_write(6<<8 | 0x08);
			spi_write(7<<8 | 0x08); spi_write(8<<8 | 0x3E);
		break;
		case 2:
			spi_write(1<<8 | 0x3C); spi_write(2<<8 | 0x04);
			spi_write(3<<8 | 0x04); spi_write(4<<8 | 0x04);
			spi_write(5<<8 | 0x3C); spi_write(6<<8 | 0x20);
			spi_write(7<<8 | 0x20); spi_write(8<<8 | 0x3C);
		break;
		case 3:
			spi_write(1<<8 | 0x3C); spi_write(2<<8 | 0x04);
			spi_write(3<<8 | 0x04); spi_write(4<<8 | 0x04);
			spi_write(5<<8 | 0x3C); spi_write(6<<8 | 0x04);
			spi_write(7<<8 | 0x04); spi_write(8<<8 | 0x3C);
		break;
		case 4:
			spi_write(1<<8 | 0x04); spi_write(2<<8 | 0x0C);
			spi_write(3<<8 | 0x14); spi_write(4<<8 | 0x24);
			spi_write(5<<8 | 0x24); spi_write(6<<8 | 0x3C);
			spi_write(7<<8 | 0x04); spi_write(8<<8 | 0x04);
    break;
		case 5:
			spi_write(1<<8 | 0x3C); spi_write(2<<8 | 0x20);
			spi_write(3<<8 | 0x20); spi_write(4<<8 | 0x20);
			spi_write(5<<8 | 0x3C); spi_write(6<<8 | 0x04);
			spi_write(7<<8 | 0x04); spi_write(8<<8 | 0x3C);
		break;
		case 6:
			spi_write(1<<8 | 0x3C); spi_write(2<<8 | 0x20);
			spi_write(3<<8 | 0x20); spi_write(4<<8 | 0x20);
			spi_write(5<<8 | 0x3C); spi_write(6<<8 | 0x24);
			spi_write(7<<8 | 0x24); spi_write(8<<8 | 0x3C);
    break;
		case 7:
			spi_write(1<<8 | 0x3C); spi_write(2<<8 | 0x04);
			spi_write(3<<8 | 0x04); spi_write(4<<8 | 0x08);
			spi_write(5<<8 | 0x10); spi_write(6<<8 | 0x20);
			spi_write(7<<8 | 0x20); spi_write(8<<8 | 0x20);
    break;
		case 8:
			spi_write(1<<8 | 0x18); spi_write(2<<8 | 0x24);
			spi_write(3<<8 | 0x24); spi_write(4<<8 | 0x24);
			spi_write(5<<8 | 0x18); spi_write(6<<8 | 0x24);
			spi_write(7<<8 | 0x24); spi_write(8<<8 | 0x18);
   break;
		case 9:
			spi_write(1<<8 | 0x3C); spi_write(2<<8 | 0x24);
			spi_write(3<<8 | 0x24); spi_write(4<<8 | 0x24);
			spi_write(5<<8 | 0x3C); spi_write(6<<8 | 0x04);
			spi_write(7<<8 | 0x04); spi_write(8<<8 | 0x3C);
		break;
	}
}

int main(void)
{
	uint32_t pui32DataRx;
	
	unsigned long period = 20000000; // 8000000 reload value to Timer0A to generate half second delay

	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);

	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_3 | GPIO_PIN_2);
	
	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,1000000, 16);
	
	SSIEnable(SSI0_BASE);

	while (SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx)){}

	spi_write(0x09 << 8 | 0x00);       // decoding :BCD
	spi_write(0x0a << 8 | 0x01);       // brightness
	spi_write(0x0b << 8 | 0x07);       // scanlimit;8 LEDs
	spi_write(0x0c << 8 | 0x01);       // power-down mode:0,normal mode:1
	spi_write(0x0f << 8 | 0x00);       // test display:1;EOT,display:0
		
  //Initialize keypad and TM4C123 GPIO pins
	keypad_Init(); 
	
	IntMasterEnable();
		
	while(1)
	{
		SysCtlDelay(SysCtlClockGet()/8);
		
	  value = keypad_getkey();    // get the key pressed value
		// get_key();
		
		diff = abs(value - count);
		
		if (diff != 0)
		{
			for (int i = 0; i < diff; i++)
			{
				if (count < value)
				{
					count ++; 
					display_led(count);
					//SysCtlDelay(13333333);
					SysCtlDelay(20000000); // delay 1.5s
				}
				else if (count > value)
				{
					count --;
					display_led(count);
					//SysCtlDelay(13333333);
					SysCtlDelay(20000000); // delay 1.5s
				}
			 }
		 }
		
	  else 
	  {
			display_led(count);
	  }
	 }
		delay_ms(1000);  // delay of one second
}

/* Mili seconds delay function */
void delay_ms(int n)
{
	int i,j;
	for(i=0;i<n;i++)
	for(j=0;j<3180;j++)
	{}
}

/* Micro seconds delay function */
void delay_us(int n)
{
	int i,j;
  for(i=0;i<n;i++)
  for(j=0;j<3;j++)
  {}
}

