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
#include "ili9341.h"
#include "i2c_master_noint.h"
#include<stdio.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
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

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


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

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    // set input and output pins
    TRISBbits.TRISB4=1; // B4 i an input
    TRISAbits.TRISA4=0; //A4 is an output
    LATAbits.LATA4=1; //A4 initially set to 3.3V
    SPI1_init();
    initExp();
    LCD_init();    
    
    
   
    LCD_clearScreen(ILI9341_BLACK);

}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
            
        { 
        unsigned char message [20];
        unsigned char message1 [20]; 
        unsigned char message2 [20];
        unsigned char message3 [20];
        unsigned char message4 [20];
        unsigned char message5 [20];
    
        unsigned char bar_number_x [4]; //array that is going to contain the number indicating the acc
        signed short n = -10; //number that stores the progress of the progress bar
        unsigned char data [30]; //array that stores the output data from the LSM
         _CP0_SET_COUNT(0);        
        //make LED connected to A4 blink every half second
         while(_CP0_GET_COUNT()<24000000/50){ 
             LATAbits.LATA4=1;
         }
         _CP0_SET_COUNT(0);
                 
        while(_CP0_GET_COUNT()<24000000/50){
             LATAbits.LATA4=0;
        }
        _CP0_SET_COUNT(0);
        
        sprintf(message, "%d" , read_i2c(0b1101011,0x0F)); //read whoami   
        
        writeString(10, 20, message, ILI9341_WHITE, ILI9341_BLACK); //write whoami starting from pixel at x=28 y=32
        
        while(_CP0_GET_COUNT()<24000000/50){ //read at a 20 Hz frequency
         
        read_multiple_i2c(0b1101011,0x20,data,14); //read the seven consecutive registers indicating temperature, acceleration in xyz, and angular velocity in xyz
        //reconstruct the signed short by shifting the high byte and ORing it with the low byte 
        //signed short temp = data[1]<< 8 | data [0]; //temperature is given by two 8 bits number, store the second one, then shift on by 8 bits and store the first one
        signed short gyro_x = data[3]<< 8 | data [2];
        signed short gyro_y = data[5]<< 8 | data [4];
        signed short acc_x = data[9]<< 8 | data [8];
        signed short acc_y = data[11]<< 8 | data [10];
         
         //sprintf(message1, "%d" , temp);  
        
         //writeString(10, 10, message1, WHITE, BLACK); //write temp starting from pixel at x=28 y=32
         
        if(abs(acc_x)<1000){
         sprintf(message2, "Ax %d   " , acc_x);  
        }
        else{
            sprintf(message2, "Ax %d " , acc_x); 
        }
         writeString(10, 30, message2, ILI9341_WHITE, ILI9341_BLACK);  
         
         if(abs(acc_y)<1000){
         sprintf(message3, "Ay %d   " , acc_y);   
         }
         else{
          sprintf(message3, "Ay %d " , acc_y);    
         }
        
         writeString(10, 40, message3, ILI9341_WHITE, ILI9341_BLACK);
         
         sprintf(message4, "Gx %d    " , gyro_x);    
        
         writeString(10, 50, message4, ILI9341_WHITE, ILI9341_BLACK); 
         
         sprintf(message5, "Gy %d    " , gyro_y); 
        
         writeString(10, 60, message5, ILI9341_WHITE, ILI9341_BLACK); 
         
         drawBarV(64, 90, acc_y, ILI9341_YELLOW, ILI9341_BLUE, 5, 140, 228); //draw progress bar starting from pixel at x=13 y=80
         
         drawBarH(64, 90, acc_x, ILI9341_YELLOW, ILI9341_BLUE, 5, 108, 296); //draw progress bar starting from pixel at x=13 y=80
                
} 
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

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
