
/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c
  Summary:
    This file contains the source code for the MPLAB Harmony application.
  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.
Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).
You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.
SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include <stdio.h>
#include <xc.h>
#include <sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"
#include "ILI9163C.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

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
#define MAX_MAF 5 
#define a 0.3
#define b 0.7

uint8_t APP_MAKE_BUFFER_DMA_READY dataOut[APP_READ_BUFFER_SIZE];
uint8_t APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];
int len, i = 0, j = 0, k = 1;
int startTime = 0;
int go = 0;
int sum_MAF = 0, ave_MAF = 0;
int ave_FIR = 0, ave_IIR = 0, prev_IIR = 0;
char msg[100];
unsigned char charArray[100]; // to store 8 bit bytes
signed short shortArray[100]; // to store 16 bit shorts after recombination from charArray
int MAFArray[MAX_MAF] = {0}; //MAF buffer array
float FIRWeight[MAX_MAF] = {0.0284, 0.2370, 0.4692, 0.2370, 0.0284};
// *****************************************************************************
/* Application Data
  Summary:
    Holds application data
  Description:
    This structure holds the application's data.
  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
 */

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

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


/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
        USB_DEVICE_CDC_INDEX index,
        USB_DEVICE_CDC_EVENT event,
        void * pData,
        uintptr_t userData
        ) {
    APP_DATA * appDataObject;
    appDataObject = (APP_DATA *) userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;

    switch (event) {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *) pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *) pData)->breakDuration;

            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->isReadComplete = true;
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            appDataObject->isWriteComplete = true;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/
void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch (event) {
        case USB_DEVICE_EVENT_SOF:

            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            break;

        case USB_DEVICE_EVENT_RESET:

            /* Update LED to show reset state */

            appData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuratio. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*) eventData;
            if (configuredEventData->configurationValue == 1) {
                /* Update LED to show configured state */

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t) & appData);

                /* Mark that the device is now configured */
                appData.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch LED to show suspended state */
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void) {
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if (appData.isConfigured == false) {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isReadComplete = true;
        appData.isWriteComplete = true;
        retVal = true;
    } else {
        retVal = false;
    }

    return (retVal);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )
  Remarks:
    See prototype in app.h.
 */

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    /* Initialize pins and IMU/I2C */
    TRISBbits.TRISB4 = 1; //sets RB4 (pin 11, Push Button) as input
    TRISAbits.TRISA4 = 0; //sets RA4 (pin 12, Green LED) as output
    LATAbits.LATA4 = 1; //sets Green LED to be high output 
    ANSELBbits.ANSB2 = 0; //turn off analog function on pin 6 of pic32
    ANSELBbits.ANSB3 = 0; //turn off analog function on pin 7 of pic32
    i2c_master_setup(); //turns on I2C peripheral
    IMU_init(); //initialize IMU
    
    /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;

    /* Device configured status */
    appData.isConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 9600;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bDataBits = 8;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read complete flag */
    appData.isReadComplete = true;

    /*Initialize the write complete flag*/
    appData.isWriteComplete = true;

    /* Reset other flags */
    appData.sofEventHasOccurred = false;
    //appData.isSwitchPressed = false;

    /* Set up the read buffer */
    appData.readBuffer = &readBuffer[0];

    startTime = _CP0_GET_COUNT();
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )
  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    /* Update the application state machine based
     * on the current state */

    switch (appData.state) {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE);

            if (appData.deviceHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if (appData.isConfigured) {
                /* If the device is configured then lets start reading */
                appData.state = APP_STATE_SCHEDULE_READ;
            }
            break;

        case APP_STATE_SCHEDULE_READ:

            if (APP_StateReset()) {
                break;
            }

            /* If a read is complete, then schedule a read
             * else wait for the current read to complete */

            appData.state = APP_STATE_WAIT_FOR_READ_COMPLETE;
            if (appData.isReadComplete == true) {
                appData.isReadComplete = false;
                appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

                USB_DEVICE_CDC_Read(USB_DEVICE_CDC_INDEX_0,
                        &appData.readTransferHandle, appData.readBuffer,
                        APP_READ_BUFFER_SIZE);

                if (appData.readTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID) {
                    appData.state = APP_STATE_ERROR;
                    break;
                }
            }

            break;

        case APP_STATE_WAIT_FOR_READ_COMPLETE:
        case APP_STATE_CHECK_TIMER:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was received or a switch was pressed.
             * The isReadComplete flag gets updated in the CDC event handler. */

            if (appData.isReadComplete || _CP0_GET_COUNT() - startTime > (48000000 / 2 / 100)) { // Change the frequency here. Currently set at 100 Hz
                appData.state = APP_STATE_SCHEDULE_WRITE;
            }

            break;


        case APP_STATE_SCHEDULE_WRITE:

            if (APP_StateReset()) {
                break;
            }

            /* Setup the write */

            appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            appData.isWriteComplete = false;
            appData.state = APP_STATE_WAIT_FOR_WRITE_COMPLETE;
            
            dataOut[0] = 0;

            if (appData.isReadComplete) {
                if (appData.readBuffer[0] == 'r') { // if key 'r' is inputted
                    go = 1;
                    USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0, // send required blank packet when read is not complete (i.e. dataOut[0] = 0, len = 1)
                            &appData.writeTransferHandle, dataOut, 1,
                            USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                } else {
                    len = sprintf(dataOut, "error\r\n"); // if any other key is pressed
                    USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                            &appData.writeTransferHandle,
                            dataOut, len,
                            USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                }
            } else {
                if (go == 1) { // 'r' is inputted
                    if (i < 100) { // for 100 values 
                        I2C_read_multiple(SLAVE_ADDR, OUT_TEMP_L_ADDR, charArray, 14); //reads 14 8-bit bytes of data, into the charArray
                        for (j = 0; j < 7; j++) { //recombines charArray data into 7 16-bit shorts
                            shortArray[j] = ((charArray[2 * j + 1] << 8) | (charArray[2 * j]));
                        }
                        MAFArray[k] = shortArray[6]; // input z-axis accelerometer data
                        for (j = 1; j < MAX_MAF+1; j++) {
                            sum_MAF = sum_MAF + MAFArray[j];
                            ave_FIR = ave_FIR + MAFArray[j]*FIRWeight[j];
                        }
                        ave_IIR = a*prev_IIR + b*MAFArray[k];
                        ave_MAF = sum_MAF / MAX_MAF; // average of the 5 values in MAFArray
                        len = sprintf(dataOut, "%d %d %d %d %d\r\n", i, MAFArray[k], ave_MAF, ave_IIR, ave_FIR); // prints index and z-axis accelerometer 
                        sum_MAF = 0;                 
                        ave_FIR = 0;
                        prev_IIR = ave_IIR;
                        i++;
                        if (k == MAX_MAF) { // if k index is more than 4, reset k 
                            k = 1;  
                        }
                        else {
                            k++; 
                        }
                        
                        USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                                &appData.writeTransferHandle,
                                dataOut, len,
                                USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                    } else { //reset it back to 0 once there are 100 values
                        i = 0;
                        go = 0;
                        USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0, // send required blank packet when read is not complete (i.e. dataOut[0] = 0, len = 1)
                                &appData.writeTransferHandle, dataOut, 1,
                                USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                    }
                } else { //print required blank packet when nothing is in buffer
                    USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                            &appData.writeTransferHandle, dataOut, 1,
                            USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                    startTime = _CP0_GET_COUNT();
                }
            }

            break;

        case APP_STATE_WAIT_FOR_WRITE_COMPLETE:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was sent. The isWriteComplete
             * flag gets updated in the CDC event handler */

            if (appData.isWriteComplete == true) {
                appData.state = APP_STATE_SCHEDULE_READ;
            }

            break;

        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}




/*******************************************************************************
 End of File
 */
