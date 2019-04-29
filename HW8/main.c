/*******************************************************************************
  MPLAB Harmony Project Main Source File

  Company:
    Microchip Technology Inc.
  
  File Name:
    main.c

  Summary:
    This file contains the "main" function for an MPLAB Harmony project.

  Description:
    This file contains the "main" function for an MPLAB Harmony project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state 
    machines of all MPLAB Harmony modules in the system and it calls the 
    "SYS_Tasks" function from within a system-wide "super" loop to maintain 
    their correct operation. These two functions are implemented in 
    configuration-specific files (usually "system_init.c" and "system_tasks.c")
    in a configuration-specific folder under the "src/system_config" folder 
    within this project's top-level folder.  An MPLAB Harmony project may have
    more than one configuration, each contained within it's own folder under
    the "system_config" folder.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

//Microchip licenses to you the right to use, modify, copy and distribute
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

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "system/common/sys_module.h"   // SYS function prototypes
#include "ili9341.h" //lcd library
#include "i2c_master_noint.h"
#include<stdio.h> //sprintf


// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************
void writeCharacter(unsigned short x, unsigned short y, char letter, unsigned short color_on, unsigned short color_off){
    
    unsigned short xi=0;   
    unsigned short yi=0;
    
    for (xi=0; xi<5; xi++) { //loop over the five pixels (in x) per character
        
        unsigned char pixels = ASCII[letter - 0x20][xi];
     
        for (yi=0; yi<8 ; yi++){ //loop over bits of each character (8 pixels in y)
            if ((pixels >> yi ) & 1) { // if the yith bit of the ascii (shift back by yi) is on, turn the pixel on
                LCD_drawPixel(x+xi, y+yi, color_on);
            }
            else{ // if the yith bit of the ascii (shift back by yi) is off, color the pixel of the off color
                LCD_drawPixel(x+xi, y+yi, color_off);
            }
        }
    }
    
    }

//function to write messages composed of multiple characters on the LCD
void writeString(unsigned short x, unsigned short y, unsigned char* words, unsigned short color_on, unsigned short color_off){
        
    unsigned int i=0;
    
    while (words[i]) { //run until there is a character 
    writeCharacter(x+5*i, y, words[i], color_on, color_off); //shift by 5 pixels per letter
    i++;
    }
    
    }
    
    
    //function to draw bars on the LCD
    void drawBarH(unsigned short x, unsigned short y, signed short n1o, unsigned short color_on, unsigned short color_off, unsigned int width, unsigned int length, signed short n1max){
       
        unsigned short ni1=0;
        unsigned short ni2=0;
        unsigned short yi=0;
        signed short length_max_v = length/2;
        signed short n1 = n1o/n1max;
        
        for(yi=0; yi<=width; yi++){ //loop over length of the bar
            
            if(n1>0){
                 for (ni2=0; ni2<=length_max_v ; ni2++){ //if acceleration is negative, turn off the positive side
                     LCD_drawPixel(x-ni2, y+yi, color_off);
                      }  
                if (n1<=length_max_v){
                    for (ni1=0; ni1<=n1 ; ni1++){ //color the bar up to the value of the acceleration
                     LCD_drawPixel(x+ni1, y+yi, color_on);
                    }
                     for (ni2=n1; ni2<=length_max_v ; ni2++){ //turn off the pixels that are greater than the acceleration
                     LCD_drawPixel(x+ni2, y+yi, color_off);
                      }     
                     }
                 
                else{
                     for (ni1=0; ni1<=length_max_v ; ni1++){ //turn on whole bar if acceleration is greater than the length
                     LCD_drawPixel(x+ni1, y+yi, color_on);
                    }
                    }
                
            }
            
           
            if(n1<0){
                for (ni2=0; ni2<=length_max_v ; ni2++){ //turn off the positive side of the bar 
                     LCD_drawPixel(x+ni2, y+yi, color_off);
                      } 
                if (-n1<=length_max_v){
                    for (ni1=0; ni1<=-n1 ; ni1++){ //turn on the part of the bar that matches the acceleration
                     LCD_drawPixel(x-ni1, y+yi, color_on);
                    }
                     for (ni2=-n1; ni2<=length_max_v ; ni2++){ //turn off the parts greater than the acceleration
                     LCD_drawPixel(x-ni2, y+yi, color_off);
                     }
                     }
                 
                else{
                     for (ni1=0; ni1<=length_max_v ; ni1++){ //loop over n1 values of n (color on of the bar)
                     LCD_drawPixel(x-ni1, y+yi, color_on);
                    }
                 }
            }
            
        
        
        }
    }
    
    void drawBarV(unsigned short x, unsigned short y, signed short n1o, unsigned short color_on, unsigned short color_off, unsigned int width, unsigned int length, signed short n1max){
       
        unsigned short ni1=0;
        unsigned short ni2=0;
        unsigned short xi=0;
        signed short length_max_v = length/2;
        signed short n1 = n1o/n1max;
        
        for(xi=0; xi<=width; xi++){ //loop over width of the bar
            
            if(n1>0){
                 for (ni2=0; ni2<=length_max_v ; ni2++){ //loop over n2 values of n (color off of the bar background)
                     LCD_drawPixel(x+xi, y-ni2, color_off);
                      }  
                if (n1<length_max_v){
                    for (ni1=0; ni1<=n1 ; ni1++){ //loop over n1 values of n (color on of the bar)
                     LCD_drawPixel(x+xi, y+ni1, color_on);
                    }
                     for (ni2=n1; ni2<=length_max_v ; ni2++){ //loop over n2 values of n (color off of the bar background)
                     LCD_drawPixel(x+xi, y+ni2, color_off);
                      }     
                     }
                 
                else{
                     for (ni1=0; ni1<=length_max_v ; ni1++){ //loop over n1 values of n (color on of the bar)
                     LCD_drawPixel(x+xi, y+ni1, color_on);
                    }
                    }
                
            }
            
           
            if(n1<0){
                for (ni2=0; ni2<=length_max_v ; ni2++){ //loop over n2 values of n (color off of the bar background)
                     LCD_drawPixel(x+xi, y+ni2, color_off);
                      } 
                if (-n1<=length_max_v){
                    for (ni1=0; ni1<=-n1 ; ni1++){ //loop over n1 values of n (color on of the bar)
                     LCD_drawPixel(x+xi, y-ni1, color_on);
                    }
                     for (ni2=-n1; ni2<=length_max_v ; ni2++){ //loop over n2 values of n (color off of the bar background)
                     LCD_drawPixel(x+xi, y-ni2, color_off);
                     }
                     }
                 
                else{
                     for (ni1=0; ni1<=length_max_v ; ni1++){ //loop over n1 values of n (color on of the bar)
                     LCD_drawPixel(x+xi, y-ni1, color_on);
                    }
                 }
            }
            
        
        
        }
    }
    

void write_i2c(unsigned char add, unsigned char reg, unsigned char val){
    i2c_master_start();                      //begin the start sequence
    i2c_master_send(add << 1|0);       // send the slave address of the register, left shifted by 1, clearing last bit and setting it to 0 indicating to write
    i2c_master_send(reg);                    //send the adress
    i2c_master_send(val);                    //send the bits
    i2c_master_stop();                       //stop sequence
    
}

unsigned char read_i2c(unsigned char add, unsigned char reg){
    i2c_master_start();                      //begin the start sequence
    i2c_master_send(add << 1|0);       // send the slave address, left shifted by 1, clearing last bit and setting it to 0 indicating to write
    i2c_master_send(reg);                   //send the address of the general purpose i/o register
    i2c_master_restart();                    //restart
    i2c_master_send(add << 1|1);       // send the slave address, left shifted by 1, clearing last bit and setting it to 1 indicating to write
    unsigned char r = i2c_master_recv();     //save the output of the slave
    i2c_master_ack(1);                       //done talking to the chip (0 if not sone but just recieved))
    i2c_master_stop();                      //stop sequence
    return r;                               //return the value read by the chip
}

unsigned char read_multiple_i2c(unsigned char add, unsigned char reg1, unsigned char* outs, unsigned int length){
    i2c_master_start();                      //begin the start sequence
    i2c_master_send(add << 1|0);       // send the slave address, left shifted by 1, clearing last bit and setting it to 0 indicating to write
    i2c_master_send(reg1);                   //send the address of the general purpose i/o register
    i2c_master_restart();                    //restart
    i2c_master_send(add << 1|1);       // send the slave address, left shifted by 1, clearing last bit and setting it to 1 indicating to write
    unsigned int i=0;
    for (i=0; i<=length; i++) {               //loop over all reading registers
    outs[i] = i2c_master_recv();             //save the output of the slave
    if(i==length){                           //when the loop reaches the last value
    i2c_master_ack(1);                       //not done talking to the chip 
    }
    else {                                   //for every other value read out
    i2c_master_ack(0);                       //done talking to the chip  
    }
    }
    i2c_master_stop();                      //stop sequence
}
    
//initialize LSM
void initExp (){
    ANSELBbits.ANSB2 = 0; //make B2 output digital
    ANSELBbits.ANSB3 = 0; //make B3 output digital
    
    i2c_master_setup();
    write_i2c(0b1101011,0x12,0b00000100); //communicate with crl3_c and set ifing (6th bit) to 1, which allows to read multiple registers in a row
    write_i2c(0b1101011,0x10,0b10000010); //communicate with ctrl1_xl and set the sample rate to 1.66 kHz (first four bits 1000), with 2g sensitivity (fifth and sixth bits to 00), and 100 Hz filter (last two bits to 10).
    write_i2c(0b1101011,0x11,0b10001100); //communicate with ctrl2_g and set the sample rate to 1.66 kHz (first four bits 1000), with with 1000 dps sensitivity (5th and 6th bits to 11)
    
}
int main ( void )
{
    /* Initialize all MPLAB Harmony modules, including application(s). */
    SYS_Initialize ( NULL );


    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );

    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

