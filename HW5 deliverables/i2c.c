#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"

#define SLAVE_ADDR 0b0100111 // 4 bits of manufacture-set (0100), 3 bits of A2/A1/A0

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

//initialize Expander
void init_Expander(){
    i2c_master_start(); // make the start bit
    i2c_master_send(SLAVE_ADDR<<1|0); // device opcode, left shift by 1, set last bit as '0' for writing
    i2c_master_send(0x00); // IODIR register 
    i2c_master_send(0b11110000); // config GP7-GP4 as input pins, GP3-GP0 as output pins
    i2c_master_stop(); //make the stop bit
    
    i2c_master_start(); // make the start bit
    i2c_master_send(SLAVE_ADDR<<1|0); // device opcode, left shift by 1, set last bit as '0' for writing
    i2c_master_send(0x06); // GPPU register 
    i2c_master_send(0b11110000); // set input pins to be default high
    i2c_master_stop(); //make the stop bit
    
}

//reading value from GPIO pins
char getExpander(void){ 
    i2c_master_start(); // make the start bit
    i2c_master_send(SLAVE_ADDR<<1|0); // device opcode, left shift by 1, set last bit as '0' for writing
    i2c_master_send(0x09); // GPIO register 
    i2c_master_restart(); // make restart bit
    i2c_master_send(SLAVE_ADDR<<1|1); // device opcode, left shift by 1, set last bit as '0' for reading
    unsigned char r = i2c_master_recv(); //save value returned
    i2c_master_ack(1); // make ACK so slave knows we received it
    i2c_master_stop(); //make the stop bit
    return r;
}

//setting pin value
void setExpander(char pin, char level){
    unsigned char manipulate_bits = (0x01 & level) << pin; // manipulate the correct level for the pin
    
    i2c_master_start(); // make the start bit
    i2c_master_send(SLAVE_ADDR<<1|0); // device opcode, left shift by 1, set last bit as '0' for writing
    i2c_master_send(0x09); // GPIO register 
    i2c_master_send(manipulate_bits); // config pin to level
    i2c_master_stop(); //make the stop bit
}


int main(void) {
    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISBbits.TRISB4 = 1; //sets RB4 (pin 11, Push Button) as input
    TRISAbits.TRISA4 = 0; //sets RA4 (pin 12, Green LED) as output
    LATAbits.LATA4 = 1; //sets Green LED to be high output 
    ANSELBbits.ANSB2 = 0; //turn off analog function on pin 6 of pic32
    ANSELBbits.ANSB3 = 0; //turn off analog function on pin 7 of pic32
    
    i2c_master_setup(); //turns on I2C peripheral
    init_Expander();
    
    __builtin_enable_interrupts();
  
    while(1) {
        unsigned char bp = getExpander();
        bp = bp >> 7;
        if(bp == 0){ //button pressed
            setExpander(0,1); // set GP0 to high output (1)
        }
        else{
            setExpander(0,0);
        }
    }   
    return 0;
}

