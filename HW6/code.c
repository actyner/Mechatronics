#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "ili9341.h"
#include<stdio.h>
#include<stdlib.h>

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL =  ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 00000000 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module
#define CS LATAbits.LATA0       // chip select pin
#define DC LATBbits.LATB15


void LCD_drawchar(char c, int a, int b, unsigned short charcolor, unsigned short bgcolor)
{ 
    int i,j;
    int cbit=0;
    int casc=c;
    for (i=0;i<5;i++)
    {
    char chk=ASCII[casc-32][i];
        for (j=0;j<7;j++)
        {
            cbit=chk>>j&1;
            if(cbit==1)
                LCD_drawPixel(a+i,b+j,charcolor);
            if(cbit==0)
                LCD_drawPixel(a+i,b+j,bgcolor);
        }
    }
}

void LCD_drawstring(char*m,int x, int y,unsigned short charcolor, unsigned short bgcolor)
{
    int t=0;
    while(m[t])
    {
        LCD_drawchar(m[t],x+(t*5),y,charcolor,bgcolor);
        t++;
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

    //set input and output pins
    TRISBbits.TRISB4=1; //pin B4 is an input
    TRISAbits.TRISA4=0; //pin A4 is an output
    LATAbits.LATA4=1; //set the initial output of pin A4 to high (3.3V))
     
    SPI1_init();
    unsigned char message [20]; //array that is going to contain the  hello world message (remember to save one extra character for the zero in sprint f)
    unsigned char bar_number [4]; //array that is going to contain the number indicating the progress in the progress bar (remember to save one extra character for the zero in sprint f)
    unsigned short n = 0; //number that stores the progress of the progress bar
    
    
    
__builtin_enable_interrupts();

       LCD_init();
    _CP0_SET_COUNT(0);
    unsigned short bg=0x780F; // background color
    unsigned short charcolor=0x0000; //color of the text
    unsigned short offcolor=0xffff; // color for text background to distinguish b/w off and on pixels
    unsigned short pg_bar_p=0x0000; //progress indicator
    unsigned short pg_bar_n=0x07ff; //progress remaining indicator
        int i,j=0;
        //LCD_drawPixel(50,55,0X0000);
        //LCD_drawchar('S',56,50,0X0000,bg);
        while(1){
            while(_CP0_GET_COUNT()<120000000/5){ 
             LATAbits.LATA4=1;
         }
         _CP0_SET_COUNT(0);
                 
        while(_CP0_GET_COUNT()<120000000/5){
             LATAbits.LATA4=0;
        }
        _CP0_SET_COUNT(0);
        LCD_clearScreen(bg);
        char str[50]="ALEX TYNER";
        char str2[25]; 
        char str3[15];
        LCD_drawstring(str,56,50,charcolor,offcolor);
        int mid =1;
        while(mid<=100)
        {
            double clock_cur=_CP0_GET_COUNT();
        sprintf(str2,"Hello World %d",mid); 
        LCD_drawstring(str2,56,100,charcolor,offcolor);
        for (i=0;i<=mid;i++)
        {
                for(j=0;j<7;j++)
                {
                    LCD_drawPixel(5+i,150+j,pg_bar_p);
                }
        }
        for (i=mid+1;i<=100;i++)
        {
            for(j=0;j<7;j++)
                {
                LCD_drawPixel(5+i,150+j,pg_bar_n);
                }
        }   
        double del_clock=_CP0_GET_COUNT()-clock_cur;  
        if(del_clock>0)
        {int fps = 48000000/del_clock;
         sprintf(str3,"FPS %d",fps); 
        LCD_drawstring(str3,56,200,charcolor,offcolor);
        }
        double clock_1=_CP0_GET_COUNT();
         while(_CP0_GET_COUNT()-clock_1 < 2400000){ ; }
            mid++;
                }
        }}
