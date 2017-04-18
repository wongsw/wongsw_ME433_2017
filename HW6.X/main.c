#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
#include "ILI9163C.h"
#include <stdio.h>

#define BG 0xF800 //Background color = RED
#define TEXTCOLOR 0xFFFF //Text color = WHITE
#define BARCOLOR 0xFFFF //Bar color = WHITE

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

void display_char(unsigned char x, unsigned char y, unsigned short color1, char c) {
    int i = 0, j = 0;

    for (i=0; i<5; i++) { // i is the index of columns
        unsigned short k = ASCII[c-0x20][i]; // k is a byte in the ASCII table
        for (j=0; j<8; j++) { // j is the index of rows
            if(x > 128 || y > 128){ // error check that x and y are not out of bounds
                break;
            }
            if(k >> j & 1) {
                LCD_drawPixel(x+i, y+j, color1);
            }
            else {   
                LCD_drawPixel(x+i, y+j, BG);
            }
        }
    }
}

void display_string(char* msg1, unsigned char x, unsigned char y) {
    int i = 0;
    while(msg1[i]!=0){
        display_char(x+(i*6), y, TEXTCOLOR, msg1[i]); // Displace the spacing between letters
        i++;
    }
}

void draw_bar(unsigned char x, unsigned char y, unsigned char w, unsigned short color1, unsigned short color2, unsigned char len, unsigned char maxLength) {
    int i = 0, j = 0;
    for(j=0; j<len; j++){ //filled bar progress
        if(x>128||y >128){ // error checking 
            break;
        }
        for(i=0; i<w; i++){
            LCD_drawPixel(x, y+i, color1);
        }
        x++;
    }
    
    for(j=len; j<maxLength; j++){ //unfilled bar progress
        if(x>128||y >128){ // error checking 
            break;
        }
        for(i=0; i<w; i++){
            LCD_drawPixel(x, y+i, color2); //draws with alt color
        }
        x++;
    }
}

int main() {

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
    SPI1_init(); //initialize SPI1
    LCD_init(); //initialize LCD
    LCD_clearScreen(BG);
    
    __builtin_enable_interrupts();
    char msg[20], msg1[20];
    int count = 0;
    float fps = 0.0;
    while(1){
        for(count=0; count<100; count++){
            _CP0_SET_COUNT(0); // start core timer to count fps later
            sprintf(msg, "Hello World %d ", count);
            display_string(msg,28,32); // displays msg at (28,32)
                
            draw_bar(28,50,1,BARCOLOR,BG,count,100); // draws bar at every length
            fps = 24000000/_CP0_GET_COUNT(); // frequency taken to do operations above (1/time of operations)
            
            sprintf(msg1, "FPS = %.2f", fps);
            display_string(msg1,28,75); // displays msg1 at (28,75)
        
            while(_CP0_GET_COUNT() <= 4800000){
                ; //do nothing for 0.2s to achieve 0-100 in 20s
            }
            
        }
    }
    
    
}