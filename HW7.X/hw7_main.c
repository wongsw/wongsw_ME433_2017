#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"
#include <stdio.h>
#include "ILI9163C.h"

#define SLAVE_ADDR 0b1101011 // based on LSM6DS33 datasheet, SDO not connected (GND)
#define WHO_AM_I_ADDR 0b00001111 // based on Table 16 of LSM6DS33 datasheet, WHO_AM_I register address
#define CTRL1_XL_ADDR 0b00010000 // CTRL1_XL register address
#define CTRL2_G_ADDR 0b00010001 // CTRL2_G register address
#define CTRL3_C_ADDR 0b00010010 // CTRL3_C register address
#define OUT_TEMP_L_ADDR 0b00100000 // OUT_TEMP_L register address
#define BG 0xF800 //Background color = RED
#define TEXTCOLOR 0xFFFF //Text color = WHITE
#define BARCOLOR 0xFFFF //Bar color = WHITE
#define MAXBARLEN 50 //Maximum length for drawing bars

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

// No error checking for situations when bar is out of bounds

void display_char(unsigned char x, unsigned char y, unsigned short color1, char c) {
    int i = 0, j = 0;

    for (i = 0; i < 5; i++) { // i is the index of columns
        unsigned short k = ASCII[c - 0x20][i]; // k is a byte in the ASCII table
        for (j = 0; j < 8; j++) { // j is the index of rows
            if (x > 128 || y > 128) { // error check that x and y are not out of bounds
                break;
            }
            if (k >> j & 1) {
                LCD_drawPixel(x + i, y + j, color1);
            } else {
                LCD_drawPixel(x + i, y + j, BG);
            }
        }
    }
}

void display_string(char* msg1, unsigned char x, unsigned char y) {
    int i = 0;
    while (msg1[i] != 0) {
        display_char(x + (i * 6), y, TEXTCOLOR, msg1[i]); // Displace the spacing between letters
        i++;
    }
}

// Draw bar in horizontal direction
void draw_bar_X(unsigned char x, unsigned char y, unsigned char w, unsigned short color1, unsigned short color2, signed short len, unsigned char maxLength) {
    int i = 0, j = 0;
    if (len > 0 | len == 0) { // for positive values draw bar to right of start point (x,y)
        for (j = 0; j < len; j++) { //filled bar progress
            for (i = 0; i < w; i++) {
                LCD_drawPixel(x, y + i, color1); //draws the width of bar 
            }
            x++;
        }
        for (j = len; j < maxLength; j++) { //unfilled bar progress
            for (i = 0; i < w; i++) {
                LCD_drawPixel(x, y + i, color2); //draws the width of bar with alt color
            }
            x++;
        }
    } 
    else { //for negative values draw bar to left of start point (x,y))
        for (j = len; j < 0; j++) { //filled bar progress
            for (i = 0; i < w; i++) {
                LCD_drawPixel(x, y + i, color1); //draws the width of bar 
            }
            x--;
        }
        for (j = -maxLength; j < len; j++) { //unfilled bar progress
            for (i = 0; i < w; i++) {
                LCD_drawPixel(x, y + i, color2); //draws the width of bar with alt color
            }
            x--;
        }
    }
}

// Draw bar in vertical direction
void draw_bar_Y(unsigned char x, unsigned char y, unsigned char w, unsigned short color1, unsigned short color2, signed short len, unsigned char maxLength) {
    int i = 0, j = 0;
    if (len > 0 | len == 0) { // for positive values draw bar to right of start point (x,y)
        for (j = 0; j < len; j++) { //filled bar progress
            for (i = 0; i < w; i++) {
                LCD_drawPixel(x + i, y, color1); //draws the width of bar 
            }
            y++;
        }
        for (j = len; j < maxLength; j++) { //unfilled bar progress
            for (i = 0; i < w; i++) {
                LCD_drawPixel(x + i, y, color2); //draws the width of bar with alt color
            }
            y++;
        }
    } 
    else { //for negative values draw bar to left of start point (x,y))
        for (j = len; j < 0; j++) { //filled bar progress
            for (i = 0; i < w; i++) {
                LCD_drawPixel(x + i, y, color1); //draws the width of bar 
            }
            y--;
        }
        for (j = -maxLength; j < len; j++) { //unfilled bar progress
            for (i = 0; i < w; i++) {
                LCD_drawPixel(x + i, y, color2); //draws the width of bar with alt color
            }
            y--;
        }
    }
}

//function to check value of WHO_AM_I register, to ensure connections are correct

char getWHOAMI(void) {
    i2c_master_start(); //ST: start bit
    i2c_master_send(SLAVE_ADDR << 1 | 0); //SAD+W: slave address left shifted 1 and add a '0' bit for writing
    i2c_master_send(WHO_AM_I_ADDR); //SUB: send an 8-bit sub-address of the WHO_AM_I register used
    i2c_master_restart(); // SR: make restart bit
    i2c_master_send(SLAVE_ADDR << 1 | 1); // SAD+R: slave address left shifted 1 and add a '1' bit for reading
    unsigned char r = i2c_master_recv(); //save value returned
    i2c_master_ack(1); // NMAK
    i2c_master_stop(); //SP: stop bit
    return r;
}

//initialize IMU

void IMU_init() {
    // Turn on accelerometer (Register CTRL1_XL)
    i2c_master_start(); //ST: start bit
    i2c_master_send(SLAVE_ADDR << 1 | 0); //SAD+W: slave address left shifted 1 and add a '0' bit for writing
    i2c_master_send(CTRL1_XL_ADDR); //SUB: send an 8-bit sub-address of CTRL1_XL_ADDR register used
    i2c_master_send(0b10000010); // DATA: send 8-bit data to set sample rate to 1.66 kHz, with 2g sensitivity, and 100 Hz filter
    i2c_master_stop(); //SP: stop bit

    // Turn on gyroscope (Register CTRL2_G)
    i2c_master_start(); //ST: start bit
    i2c_master_send(SLAVE_ADDR << 1 | 0); //SAD+W: slave address left shifted 1 and add a '0' bit for writing
    i2c_master_send(CTRL2_G_ADDR); //SUB: send an 8-bit sub-address of CTRL2_G_ADDR register used
    i2c_master_send(0b10001000); // DATA: send 8-bit data to set sample rate to 1.66 kHz, 1000 dps
    i2c_master_stop(); //SP: stop bit

    // Change Register CTRL3_C 
    i2c_master_start(); //ST: start bit
    i2c_master_send(SLAVE_ADDR << 1 | 0); //SAD+W: slave address left shifted 1 and add a '0' bit for writing
    i2c_master_send(CTRL3_C_ADDR); //SUB: send an 8-bit sub-address of CTRL3_C_ADDR register used
    i2c_master_send(0b00000100); // DATA: send 8-bit data to set everything default, ensuring IF_INC bit is '1'
    i2c_master_stop(); //SP: stop bit
}

void I2C_read_multiple(unsigned char address, unsigned char register1, unsigned char * data, int length) {
    i2c_master_start(); //ST: start bit
    i2c_master_send(address << 1 | 0); //SAD+W: slave address left shifted 1 and add a '0' bit for writing
    i2c_master_send(register1); //SUB: send an 8-bit sub-address of register used
    i2c_master_restart(); // SR: make restart bit
    i2c_master_send(address << 1 | 1); // SAD+R: slave address left shifted 1 and add a '1' bit for reading
    int i = 0;
    for (i = 0; i < length - 1; i++) {
        data[i] = i2c_master_recv(); //DATA: save 13 8-bit bytes 
        i2c_master_ack(0); // MAK
    }
    data[length - 1] = i2c_master_recv(); //DATA: save the 14th 8-bit bytes
    i2c_master_ack(1); // NMAK
    i2c_master_stop(); //SP: stop bit
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
    IMU_init(); //initialize IMU
    SPI1_init(); //initialize SPI1
    LCD_init(); //initialize LCD
    LCD_clearScreen(BG);
    __builtin_enable_interrupts();

    char msg[100];
    unsigned char charArray[100]; // to store 8 bit bytes
    signed short shortArray[100]; // to store 16 bit shorts after recombination from charArray


    while (1) {
        //sprintf(msg, "Temp: %d", getWHOAMI());
        //display_string(msg,10,20);

        I2C_read_multiple(SLAVE_ADDR, OUT_TEMP_L_ADDR, charArray, 14); //reads 14 8-bit bytes of data, into the charArray
        int j = 0;
        for (j = 0; j < 7; j++) { //recombines charArray data into 7 16-bit shorts
            shortArray[j] = ((charArray[2 * j + 1] << 8) | (charArray[2 * j]));
        }
        /*
        sprintf(msg, "Temp: %hi", shortArray[0]);
        display_string(msg,10,20); 
        sprintf(msg, "gyroX: %hi", shortArray[1]);
        display_string(msg,10,30); 
        sprintf(msg, "gyroY: %hi", shortArray[2]);
        display_string(msg,10,40); 
        sprintf(msg, "gyroZ: %hi", shortArray[3]);
        display_string(msg,10,50);
         */
        sprintf(msg, "accelX: %hi", shortArray[4]);
        display_string(msg, 10, 10);
        sprintf(msg, "accelY: %hi", shortArray[5]);
        display_string(msg, 10, 20);
        signed short scaled_accelX = shortArray[4] / 500;
        signed short scaled_accelY = shortArray[5] / 500;

        draw_bar_X(64, 63, 2, BARCOLOR, BG, scaled_accelX, MAXBARLEN);
        draw_bar_Y(64, 63, 2, BARCOLOR, BG, scaled_accelY, MAXBARLEN);


        //sprintf(msg, "accelZ: %hi", shortArray[6]);
        //display_string(msg,10,80); 

        while (_CP0_GET_COUNT() <= 4800000) {
            ; //do nothing for 0.2s to achieve 5Hz
        }

    }

}

