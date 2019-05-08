#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>
#include"i2c_master_utilities.h"
#include"ili9341.h"
#include<stdio.h>
#include<limits.h>
// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF// no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
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
#pragma config WDTPS = PS1 // use slowest wdt
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
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module
unsigned short SLAVE_ADDRESS = 0b1101011;
void LED_blink_init()
{
// setup initial pins configuration for LED
    TRISAbits.TRISA4 = 0; //A4 output
    TRISBbits.TRISB4 = 1; //B4 input
    LATAbits.LATA4 = 1; //A4
}
void I2C2_setIMU(unsigned char reg_address, unsigned char config_byte)
{
// set bits of IMU
I2C_master_start();                       // start bit
I2C_master_send(SLAVE_ADDRESS << 1 | 0);  // send control byte 0 for writing
I2C_master_send(reg_address);             // send register address
I2C_master_send(config_byte);             // send configuration bits
I2C_master_stop();                        // stop bit
}
void I2C2_getIMUdata(unsigned char reg_address, unsigned char *data, int length)
{
// get sequential data from IMU
int i;
I2C_master_start();                             //start bit
I2C_master_send(SLAVE_ADDRESS << 1 | 0);        // send control byte 0 for writing
I2C_master_send(reg_address);                   // register address
I2C_master_restart();
I2C_master_send(SLAVE_ADDRESS << 1 | 1);        // send control byte 1 for reading
for (i=0;i<length-1;i++)
    {
        data[i] = I2C_master_recv();
I2C_master_ack(0);
    }
    data[i+1] = I2C_master_recv();
I2C_master_ack(1);
I2C_master_stop();
}
void I2C2_initIMU()
{
// initialize I2C2 slave device
// CTRL1_XL register sample rate 1.66kHz 2g sensitivity 100Hz filter
//0b 1000 00 10
I2C2_setIMU(0x10, 0b10000010);
// CTRL2_G register sample rate 1.66kHz 1000 dps sensitivity
//0b 1000 10 0 0
I2C2_setIMU(0x11, 0b10001000);
// CTRL3_C enable IF_INC
//0b 0 0 0 0 0 1 0 0
I2C2_setIMU(0x12, 0b00000100);
}
unsigned char I2C2_getIMU_Address()
{
// get WHO_AM_I bits
I2C_master_start();                          // start bit
I2C_master_send(SLAVE_ADDRESS << 1 | 0);     // send control byte 0 for writing
I2C_master_send(0x0F);                       // GPIO register 0x09
I2C_master_restart();
I2C_master_send(SLAVE_ADDRESS << 1 | 1);     // send control byte 1 for reading
unsigned char recv_byte = I2C_master_recv(); // get GPIO byte
I2C_master_ack(1);                           // ack done
I2C_master_stop();                           // stop bit
return recv_byte;
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
LED_blink_init();
SPI1_init();
LCD_init();
LCD_clearScreen(ILI9341_PINK);
SPI2_init();
ScreenGUI_init();
__builtin_enable_interrupts();
// frequency config
int SysCLK_freq = 48e6; // system clock frequency
int LoopCLK_freq = SysCLK_freq/2; // main loop frequency
// LED
int LED_blink_freq = 2; // LED blink frequency
// LCD
int LCD_COM_freq = 5;
// periods
int T_LED_BLINK = LoopCLK_freq/LED_blink_freq/2;
int T_LCD_COM = LoopCLK_freq/LCD_COM_freq;
int T_TIMER_RESET;
// init counter
_CP0_SET_COUNT(0);
int TimerStart = _CP0_GET_COUNT();
int TimerNow = _CP0_GET_COUNT();
int Timer_LED = TimerStart;
int Timer_LCD = TimerStart;
int progress = 0;
int pos_x, pos_y, pos_idx;
int frame_timer;
double fps;
char s[10];
unsigned char data[14];
short temperature;
short gyroX, gyroY, gyroZ;
short accelX, accelY, accelZ;
double Xcomp, Ycomp;
short Xdelta, Ydelta;
unsigned short fullscale=ILI9341_TFTWIDTH-4;
unsigned short x_touch_raw, y_touch_raw, z_touch_raw;
unsigned short x_touch, y_touch;
unsigned short pressure_threshold=666;
int val_disp=0;
while(1) {
// use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
// remember the core timer runs at half the sysclk
/*-------------------------------------
         -------- LCD Functionalities ----------
         --------------------------------------*/
if (_CP0_GET_COUNT() - Timer_LCD > T_LCD_COM)
            {
                frame_timer = _CP0_GET_COUNT();
                pos_x=0;
                pos_y=0;

XPT2046_read(&x_touch_raw, &y_touch_raw, &z_touch_raw);
sprintf(s, "Xraw: %u     ", x_touch_raw);
LCD_drawString(s,pos_x,pos_y+24,ILI9341_ORANGE,ILI9341_BLACK);
sprintf(s, "Yraw: %u     ", y_touch_raw);
LCD_drawString(s,pos_x,pos_y+32,ILI9341_ORANGE,ILI9341_BLACK);
sprintf(s, "Zraw: %u     ", z_touch_raw);
LCD_drawString(s,pos_x,pos_y+40,ILI9341_ORANGE,ILI9341_BLACK);
XPT2046_raw2pixel(x_touch_raw, y_touch_raw, &x_touch, &y_touch);
sprintf(s, "X: %u     ", x_touch);
LCD_drawString(s,pos_x,pos_y+48,ILI9341_ORANGE,ILI9341_BLACK);
sprintf(s, "Y: %u     ", y_touch);
LCD_drawString(s,pos_x,pos_y+56,ILI9341_ORANGE,ILI9341_BLACK);
sprintf(s, "I= %d     ", val_disp);
LCD_drawString(s,pos_x+120,pos_y+165,ILI9341_ORANGE,ILI9341_BLACK);
// check callback of each buttons
if (IsInsideButton(button_plus, x_touch, y_touch) && (z_touch_raw > pressure_threshold))
                    {
//pressed
                        button_plus.state = PRESSED;
LCD_drawString(button_plus.s, button_plus.xc + button_plus.width/2, button_plus.yc + button_plus.height/2, ILI9341_RED, button_plus.bgcolor);
                    }
else
                    {
//released
if (button_plus.state == PRESSED)
                            val_disp+=1;
                        button_plus.state = RELEASED;
LCD_drawString(button_plus.s, button_plus.xc + button_plus.width/2, button_plus.yc + button_plus.height/2, button_plus.fgcolor, button_plus.bgcolor);
                    }
if (IsInsideButton(button_minus, x_touch, y_touch) && (z_touch_raw > pressure_threshold))
                    {
//pressed
                        button_minus.state = PRESSED;
LCD_drawString(button_minus.s, button_minus.xc + button_minus.width/2, button_minus.yc + button_minus.height/2, ILI9341_RED, button_minus.bgcolor);
                    }
else
                    {
//released
if (button_minus.state == PRESSED)
                            val_disp-=1;
                        button_minus.state = RELEASED;
LCD_drawString(button_minus.s, button_minus.xc + button_minus.width/2, button_minus.yc + button_minus.height/2, button_minus.fgcolor, button_minus.bgcolor);
                    }
                
                Timer_LCD = _CP0_GET_COUNT();
// show fps
//fps = LoopCLK_freq / (double)(Timer_LCD - frame_timer);
//sprintf(s, "fps: %.2f", fps);
//LCD_drawString(s,0,0,ILI9341_YELLOW, ILI9341_BLACK);
            }
        
/*-------------------------------------
         ------- LED Functionalities ----------
         --------------------------------------*/
if(_CP0_GET_COUNT() - Timer_LED > T_LED_BLINK)
            {
                LATAbits.LATA4 = !LATAbits.LATA4;
                Timer_LED = _CP0_GET_COUNT();
            }
while (!PORTBbits.RB4){LATAbits.LATA4 = 0;}
        

    }
}
