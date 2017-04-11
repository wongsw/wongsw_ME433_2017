#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "math.h"

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins ??
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz (input clock is 8MHz crystal)
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

// PIC is the master, MCP4902 DAC is the slave
// SS1 -> CS (pin A0 -> pin 2)

// SDO4 -> SI (pin F5 -> pin 5)
// SDI4 -> SO (pin F4 -> pin 2)
// SCK4 -> SCK (pin B14 -> pin 6)

// Additional SRAM connections
// Vss (Pin 4) -> ground
// Vcc (Pin 8) -> 3.3 V

#define CS LATAbits.LATA0
//Initialize spi1
void initSPI1() {
  // set up the chip select pin as an output for MCP 4902 DAC
  // the chip select pin is used by the sram to indicate
  // when a command is beginning (clear CS to low) and when it
  // is ending (set CS high)
  TRISAbits.TRISA0 = 0; 
  TRISBbits.TRISB8 = 0;
  CS = 1;
  
  // Master - SPI1, pins are: SDI1(A1)--not used/mapped, SDO1(B8), SCK1(B14), SS1(A0). 
  RPB8Rbits.RPB8R = 0b0011; // assigns SDO1 to pin B8
  RPA0Rbits.RPA0R = 0b0011; // assigns SS1 to pin A0 
  
  // setup spi1
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 0x1;            // baud rate to 20 MHz [SPI1BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 1
  
  
}

// send a byte (8 bits) via spi and return the response
unsigned char spi1_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void setVoltage(char channel, char voltage) {
  CS = 0;                        // enable the dac by lowering the chip select line
  spi1_io(((channel << 7) | 0b01110000) | ((voltage >> 4) | 0b0000)); // bit 15 to 8
  spi1_io((voltage << 4) | 0b00000000); // bit 7 to 0
  CS = 1;                        // raise the chip select line, ending communication
}

int main(void) {
  int i = 0;
  int j = 0;
  char bufA[200];  // array for storing 100 values for sine wave on VoutA
  char bufB[200];  // array for storing 200 values for triangle wave on VoutB
  initSPI1(); 
  
  for (i=0; i<200; i++) {
	bufB[i] = i*255/199;
  }
 
  for (i=0; i<200; i++) {
	bufA[i] = 255*(sin(i*2*3.14/99))/2;
  }

  while (1) {
	for (j=0; j<5; j++){
		for (i=0; i<200; i++) {
			setVoltage(0, bufA[i]);
			setVoltage(1, bufB[i]);
			_CP0_SET_COUNT(0);
			while(_CP0_GET_COUNT() <= 2400){
					; //do nothing for 0.1ms
			}
		}
	}
  }
  return 0;
}
