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
// *****************************************************************************
// *****************************************************************************

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

/* Mouse Report */
MOUSE_REPORT mouseReport APP_MAKE_BUFFER_DMA_READY;
MOUSE_REPORT mouseReportPrevious APP_MAKE_BUFFER_DMA_READY;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
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
// *****************************************************************************
// *****************************************************************************

void APP_USBDeviceHIDEventHandler(USB_DEVICE_HID_INDEX hidInstance,
        USB_DEVICE_HID_EVENT event, void * eventData, uintptr_t userData) {
    APP_DATA * appData = (APP_DATA *) userData;

    switch (event) {
        case USB_DEVICE_HID_EVENT_REPORT_SENT:

            /* This means the mouse report was sent.
             We are free to send another report */

            appData->isMouseReportSendBusy = false;
            break;

        case USB_DEVICE_HID_EVENT_REPORT_RECEIVED:

            /* Dont care for other event in this demo */
            break;

        case USB_DEVICE_HID_EVENT_SET_IDLE:

            /* Acknowledge the Control Write Transfer */
            USB_DEVICE_ControlStatus(appData->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            /* save Idle rate received from Host */
            appData->idleRate = ((USB_DEVICE_HID_EVENT_DATA_SET_IDLE*) eventData)->duration;
            break;

        case USB_DEVICE_HID_EVENT_GET_IDLE:

            /* Host is requesting for Idle rate. Now send the Idle rate */
            USB_DEVICE_ControlSend(appData->deviceHandle, &(appData->idleRate), 1);

            /* On successfully receiving Idle rate, the Host would acknowledge back with a
               Zero Length packet. The HID function driver returns an event
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the application upon
               receiving this Zero Length packet from Host.
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates this control transfer
               event is complete */

            break;

        case USB_DEVICE_HID_EVENT_SET_PROTOCOL:
            /* Host is trying set protocol. Now receive the protocol and save */
            appData->activeProtocol = *(USB_HID_PROTOCOL_CODE *) eventData;

            /* Acknowledge the Control Write Transfer */
            USB_DEVICE_ControlStatus(appData->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_HID_EVENT_GET_PROTOCOL:

            /* Host is requesting for Current Protocol. Now send the Idle rate */
            USB_DEVICE_ControlSend(appData->deviceHandle, &(appData->activeProtocol), 1);

            /* On successfully receiving Idle rate, the Host would acknowledge
              back with a Zero Length packet. The HID function driver returns
              an event USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the
              application upon receiving this Zero Length packet from Host.
              USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates
              this control transfer event is complete */
            break;

        case USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT:
            break;

        default:
            break;
    }
}

/*******************************************************************************
  Function:
    void APP_USBDeviceEventHandler (USB_DEVICE_EVENT event,
        USB_DEVICE_EVENT_DATA * eventData)
  Summary:
    Event callback generated by USB device layer.
  Description:
    This event handler will handle all device layer events.
  Parameters:
    None.
  Returns:
    None.
 */

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    USB_DEVICE_EVENT_DATA_CONFIGURED * configurationValue;
    switch (event) {
        case USB_DEVICE_EVENT_SOF:
            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            appData.setIdleTimer++;
            break;
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:

            /* Device got deconfigured */

            appData.isConfigured = false;
            appData.isMouseReportSendBusy = false;
            appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            //appData.emulateMouse = true;
            //BSP_LEDOn ( APP_USB_LED_1 );
            //BSP_LEDOn ( APP_USB_LED_2 );
            //BSP_LEDOff ( APP_USB_LED_3 );

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Device is configured */
            configurationValue = (USB_DEVICE_EVENT_DATA_CONFIGURED *) eventData;
            if (configurationValue->configurationValue == 1) {
                appData.isConfigured = true;

                //BSP_LEDOff ( APP_USB_LED_1 );
                //BSP_LEDOff ( APP_USB_LED_2 );
                //BSP_LEDOn ( APP_USB_LED_3 );

                /* Register the Application HID Event Handler. */

                USB_DEVICE_HID_EventHandlerSet(appData.hidInstance,
                        APP_USBDeviceHIDEventHandler, (uintptr_t) & appData);
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
            //BSP_LEDOff ( APP_USB_LED_1 );
            //BSP_LEDOn ( APP_USB_LED_2 );
            //BSP_LEDOn ( APP_USB_LED_3 );
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;

    }
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
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;
    appData.isConfigured = false;
    //appData.emulateMouse = true;
    appData.hidInstance = 0;
    appData.isMouseReportSendBusy = false;
    
    /* Initialize pins and IMU/I2C */
    TRISBbits.TRISB4 = 1; //sets RB4 (pin 11, Push Button) as input
    TRISAbits.TRISA4 = 0; //sets RA4 (pin 12, Green LED) as output
    LATAbits.LATA4 = 1; //sets Green LED to be high output 
    ANSELBbits.ANSB2 = 0; //turn off analog function on pin 6 of pic32
    ANSELBbits.ANSB3 = 0; //turn off analog function on pin 7 of pic32
    i2c_master_setup(); //turns on I2C peripheral
    IMU_init(); //initialize IMU
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )
  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    //static int8_t vector = 0;
    //static uint8_t movement_length = 0;
    //int8_t dir_table[] = {-4, -4, -4, 0, 4, 4, 4, 0};
    static uint8_t inc = 0;
    unsigned char charArray[100]; // to store 8 bit bytes
    signed short shortArray[100]; // to store 16 bit shorts after recombination from charArray
    int x_scaled = 0, y_scaled = 0;
    int j = 0;
    /* Check the application's current state. */
    switch (appData.state) {
            /* Application's initial state. */
        case APP_STATE_INIT:
        {
            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0,
                    DRV_IO_INTENT_READWRITE);

            if (appData.deviceHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle,
                        APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }
            break;
        }

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device is configured. The 
             * isConfigured flag is updated in the
             * Device Event Handler */

            if (appData.isConfigured) {
                appData.state = APP_STATE_MOUSE_EMULATE;
            }
            break;

        case APP_STATE_MOUSE_EMULATE: //Gets input from IMU and moves mouse
            I2C_read_multiple(SLAVE_ADDR, OUT_TEMP_L_ADDR, charArray, 14); //reads 14 8-bit bytes of data, into the charArray
            for (j = 0; j < 7; j++) { //recombines charArray data into 7 16-bit shorts
                shortArray[j] = ((charArray[2 * j + 1] << 8) | (charArray[2 * j]));
            }
            x_scaled = shortArray[4]/1000; //x-axis accelerometer value
            y_scaled = shortArray[5]/1000; //y-axis accelerometer value
            /* adds a delay for movement of mouse */
            if (inc == 10) { // every 10th loop 
                appData.mouseButton[0] = MOUSE_BUTTON_STATE_RELEASED;
                appData.mouseButton[1] = MOUSE_BUTTON_STATE_RELEASED;
                appData.xCoordinate = (int8_t) x_scaled;
                appData.yCoordinate = (int8_t) y_scaled;
                //vector++;
                //movement_length = 0;
                inc = 0; // reset the counter
            }
            else { 
                appData.mouseButton[0] = MOUSE_BUTTON_STATE_RELEASED;
                appData.mouseButton[1] = MOUSE_BUTTON_STATE_RELEASED;
                appData.xCoordinate = (int8_t) 0;
                appData.yCoordinate = (int8_t) 0;
                inc++;
            }

            if (!appData.isMouseReportSendBusy) {
                /* This means we can send the mouse report. The
                   isMouseReportBusy flag is updated in the HID Event Handler. */

                appData.isMouseReportSendBusy = true;

                /* Create the mouse report */

                MOUSE_ReportCreate(appData.xCoordinate, appData.yCoordinate,
                        appData.mouseButton, &mouseReport);

                if (memcmp((const void *) &mouseReportPrevious, (const void *) &mouseReport,
                        (size_t)sizeof (mouseReport)) == 0) {
                    /* Reports are same as previous report. However mouse reports
                     * can be same as previous report as the coordinate positions are relative.
                     * In that case it needs to be sent */
                    if ((appData.xCoordinate == 0) && (appData.yCoordinate == 0)) {
                        /* If the coordinate positions are 0, that means there
                         * is no relative change */
                        if (appData.idleRate == 0) {
                            appData.isMouseReportSendBusy = false;
                        } else {
                            /* Check the idle rate here. If idle rate time elapsed
                             * then the data will be sent. Idle rate resolution is
                             * 4 msec as per HID specification; possible range is
                             * between 4msec >= idlerate <= 1020 msec.
                             */
                            if (appData.setIdleTimer
                                    >= appData.idleRate * 4) {
                                /* Send REPORT as idle time has elapsed */
                                appData.isMouseReportSendBusy = true;
                            } else {
                                /* Do not send REPORT as idle time has not elapsed */
                                appData.isMouseReportSendBusy = false;
                            }
                        }
                    }

                }
                if (appData.isMouseReportSendBusy == true) {
                    /* Copy the report sent to previous */
                    memcpy((void *) &mouseReportPrevious, (const void *) &mouseReport,
                            (size_t)sizeof (mouseReport));

                    /* Send the mouse report. */
                    USB_DEVICE_HID_ReportSend(appData.hidInstance,
                            &appData.reportTransferHandle, (uint8_t*) & mouseReport,
                            sizeof (MOUSE_REPORT));
                    appData.setIdleTimer = 0;
                }
                //movement_length++;
            }

            break;

        case APP_STATE_ERROR:

            break;

            /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}


/*******************************************************************************
 End of File
 */