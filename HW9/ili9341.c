#include <xc.h>
#include "ili9341.h"

void LCD_init() {
    int time = 0;
    
    CS1 = 0; // CS
   
    LCD_command(ILI9341_SWRESET);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 7200000) {} // 300ms

    LCD_command(0xEF);
  	LCD_data(0x03);
	LCD_data(0x80);
	LCD_data(0x02);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xCF);
  	LCD_data(0x00);
	LCD_data(0xC1);
	LCD_data(0x30);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xED);
  	LCD_data(0x64);
	LCD_data(0x03);
	LCD_data(0x12);
    LCD_data(0x81);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xE8);
  	LCD_data(0x85);
	LCD_data(0x00);
	LCD_data(0x78);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xCB);
  	LCD_data(0x39);
	LCD_data(0x2C);
	LCD_data(0x00);
    LCD_data(0x34);
    LCD_data(0x02);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xF7);
  	LCD_data(0x20);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xEA);
  	LCD_data(0x00);
	LCD_data(0x00);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_PWCTR1);
  	LCD_data(0x23);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_PWCTR2);
  	LCD_data(0x10);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_VMCTR1 );
  	LCD_data(0x3e);
    LCD_data(0x28);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_VMCTR2);
  	LCD_data(0x86);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_MADCTL);
  	LCD_data(0x48);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
/*    
    LCD_command(ILI9341_VSCRSADD);
  	LCD_data(0x00);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
 */   
    LCD_command(ILI9341_PIXFMT);
  	LCD_data(0x55);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_FRMCTR1);
  	LCD_data(0x00);
    LCD_data(0x18);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command( ILI9341_DFUNCTR);
  	LCD_data(0x08);
    LCD_data(0x82);
    LCD_data(0x27);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xF2);
  	LCD_data(0); // 1
    LCD_data(0x00);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_GAMMASET);
  	LCD_data(0x01);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_GMCTRP1);
  	LCD_data(0x0F);
    LCD_data(0x31);
    LCD_data(0x2B);
    LCD_data(0x0C);
    LCD_data(0x0E);
    LCD_data(0x08);
    LCD_data(0x4E);
    LCD_data(0xF1);
    LCD_data(0x37);
    LCD_data(0x07);
    LCD_data(0x10);
    LCD_data(0x03);
    LCD_data(0x0E);
    LCD_data(0x09);
    LCD_data(0x00);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_GMCTRN1);
  	LCD_data(0x00);
    LCD_data(0x0E);
    LCD_data(0x14);
    LCD_data(0x03);
    LCD_data(0x11);
    LCD_data(0x07);
    LCD_data(0x31);
    LCD_data(0xC1);
    LCD_data(0x48);
    LCD_data(0x08);
    LCD_data(0x0F);
    LCD_data(0x0C);
    LCD_data(0x31);
    LCD_data(0x36);
    LCD_data(0x0F);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(0xB1);
  	LCD_data(0x00);
    LCD_data(0x10);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_SLPOUT);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    LCD_command(ILI9341_DISPON);
    
    CS1 = 1; // CS
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    
    CS1 = 0; // CS
    
    LCD_command(ILI9341_MADCTL);
    LCD_data(MADCTL_MX | MADCTL_BGR); // rotation
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 3600000) {} // 150ms
    
    CS1 = 1; // CS
}

void SPI1_init() {
  SDI1Rbits.SDI1R = 0b0100; // B8 is SDI1
  RPA1Rbits.RPA1R = 0b0011; // A1 is SDO1
  TRISBbits.TRISB7 = 0; // CS is B7
  CS1 = 1; // CS starts high

  // DC pin
  TRISBbits.TRISB9 = 0;
  DC1 = 1;
  
  SPI1CON = 0; // turn off the spi module and reset it
  SPI1BUF; // clear the rx buffer by reading from it
  SPI1BRG = 0; // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0; // clear the overflow bit
  SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1; // master operation
  SPI1CONbits.ON = 1; // turn on spi1
}

void SPI2_init() {
  SDI2Rbits.SDI2R = 0b0011; // B13 is SDI2
  RPB11Rbits.RPB11R = 0b0100; // B11 is SDO2
  TRISBbits.TRISB10 = 0; // CS is B10
  CS2 = 1; // CS starts high
  ANSELBbits.ANSB13 = 0;
  
  SPI2CON = 0; // turn off the spi2 module and reset it
  SPI2BUF; // clear the rx buffer by reading from it
  SPI2BRG = 3; // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
  SPI2STATbits.SPIROV = 0; // clear the overflow bit
  SPI2CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
  SPI2CONbits.MSTEN = 1; // master operation
  SPI2CONbits.ON = 1; // turn on spi2
}

unsigned char spi1_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

unsigned char spi2_io(unsigned char o) {
  SPI2BUF = o;
  while(!SPI2STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI2BUF;
}

void LCD_command(unsigned char com) {
    DC1 = 0; // DC
    spi1_io(com);
    DC1 = 1; // DC
}

void LCD_data(unsigned char dat) {
    spi1_io(dat);
}

void LCD_data16(unsigned short dat) {
    spi1_io(dat>>8);
    spi1_io(dat);
}

void LCD_setAddr(unsigned short x, unsigned short y, unsigned short w, unsigned short h) {
    LCD_command(ILI9341_CASET); // Column
    LCD_data16(x);
	LCD_data16(x+w-1);

	LCD_command(ILI9341_PASET); // Page
	LCD_data16(y);
	LCD_data16(y+h-1);

	LCD_command(ILI9341_RAMWR); // Into RAM
}

void LCD_drawPixel(unsigned short x, unsigned short y, unsigned short color) {
  
    
    CS1 = 0; // CS
    
    LCD_setAddr(x,y,1,1);
    LCD_data16(color);
    
    CS1 = 1; // CS
}

void LCD_clearScreen(unsigned short color) {
    int i;
    
    CS1 = 0; // CS
    
    LCD_setAddr(0,0,ILI9341_TFTWIDTH,ILI9341_TFTHEIGHT);
	for (i = 0;i < ILI9341_TFTWIDTH*ILI9341_TFTHEIGHT; i++){
		LCD_data16(color);
	}
    
    CS1 = 1; // CS
}

void writeCharacter(char letter, unsigned short x, unsigned short y, unsigned short color_on, unsigned short color_off){
    
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

int LCD_drawString(char* s, unsigned short x0, unsigned short y0, unsigned fgcolor, unsigned short bgcolor)
{
     unsigned int i=0;
    
    while (s[i]) { //run until there is a character 
    writeCharacter(s[i], x0+5*i, y0, fgcolor, bgcolor); //shift by 5 pixels per letter
    i++;
    }
}



void LCD_drawButton(BUTTON newbutton)
{
    // draw a botton on screen
    
    int i,j;
    for (i=0; i<newbutton.width; i++)
        for (j=0; j<newbutton.height; j++)
            LCD_drawPixel(newbutton.xc+i, newbutton.yc+j, newbutton.bgcolor);
    LCD_drawString(newbutton.s, newbutton.xc + newbutton.width/2, newbutton.yc + newbutton.height/2, newbutton.fgcolor, newbutton.bgcolor);
}

void ScreenGUI_init()
{
    // initialize GUI of LCD screen
    unsigned short x0=120;
    unsigned short y0=100;
    
    //BUTTON button_plus;
    button_plus.xc = x0;
    button_plus.yc = y0;
    button_plus.width = 40;
    button_plus.height = 40;
    button_plus.fgcolor = ILI9341_GREEN;
    button_plus.bgcolor = ILI9341_BLUE;
    button_plus.s[0] = '+';
    button_plus.state = RELEASED;
    
    LCD_drawButton(button_plus);
    
    x0=120;
    y0+=100;
    //BUTTON button_minus;
    button_minus.xc = x0;
    button_minus.yc = y0;
    button_minus.width = 40;
    button_minus.height = 40;
    button_minus.fgcolor = ILI9341_GREEN;
    button_minus.bgcolor = ILI9341_BLUE;
    button_minus.s[0] = '-';
    button_minus.state = RELEASED;
    
    LCD_drawButton(button_minus);
}

void XPT2046_read(unsigned short *x, unsigned short *y, unsigned short *z)
{
    // read data from touch screen controller
    unsigned char null_byte = 0x00;
    unsigned char command_byte;
    unsigned char send_byte, rec_byte;
    unsigned char bits_high, bits_low;
    unsigned short xtmp=0, ytmp=0, ztmp=0, z1tmp, z2tmp;
    
    // x
    CS2 = 0;
    // start bit(1) + channel(101) + 12-bits mode(0) + SER/DFR(0) + PD1/0 (11)
    command_byte = 0b11010011;
    // 1st
    spi2_io(command_byte);
    // 2st
    rec_byte = spi2_io(null_byte);
    bits_high = rec_byte;
    // 3rd
    rec_byte = spi2_io(null_byte);
    bits_low = rec_byte;
    CS2 = 1;
    // shift
    xtmp = (bits_low & 0x00ff) | (((short)bits_high)<<8);
    xtmp = xtmp >> 3; // remove trailing zeros
    *x = xtmp;
    
    // y
    CS2 = 0;
    // start bit(1) + channel(001) + 12-bits mode(0) + SER/DFR(0) + PD1/0 (11)
    command_byte = 0b10010011;
    // 1st
    spi2_io(command_byte);
    // 2st
    rec_byte = spi2_io(null_byte);
    bits_high = rec_byte;
    // 3rd
    rec_byte = spi2_io(null_byte);
    bits_low = rec_byte;
    CS2 = 1;
    // shift
    ytmp = (bits_low & 0x00ff) | (((short)bits_high)<<8);
    ytmp = ytmp >> 3; // remove trailing zeros
    *y = ytmp;
    
    // z1
    CS2 = 0;
    // start bit(1) + channel(011) + 12-bits mode(0) + SER/DFR(0) + PD1/0 (11)
    command_byte = 0b10110011;
    // 1st
    spi2_io(command_byte);
    // 2st
    rec_byte = spi2_io(null_byte);
    bits_high = rec_byte;
    // 3rd
    rec_byte = spi2_io(null_byte);
    bits_low = rec_byte;
    CS2 = 1;
    // shift
    z1tmp = (bits_low & 0x00ff) | (((short)bits_high)<<8);
    z1tmp = z1tmp >> 3; // remove trailing zeros
    
    // z2
    CS2 = 0;
    // start bit(1) + channel(100) + 12-bits mode(0) + SER/DFR(0) + PD1/0 (11)
    command_byte = 0b11000011;
    // 1st
    spi2_io(command_byte);
    // 2st
    rec_byte = spi2_io(null_byte);
    bits_high = rec_byte;
    // 3rd
    rec_byte = spi2_io(null_byte);
    bits_low = rec_byte;
    CS2 = 1;
    // shift
    z2tmp = (bits_low & 0x00ff) | (((short)bits_high)<<8);
    z2tmp = z2tmp >> 3; // remove trailing zeros
    
    // z
    *z = z1tmp - z2tmp + 4095;
}

void XPT2046_raw2pixel(unsigned short xraw, unsigned short yraw, unsigned short *xpixel, unsigned short *ypixel)
{
    // scale raw data from XPT2046 to LCD pixel units
    
    // calibration parameters
    unsigned short xbraw = 480;
    unsigned short ybraw = 480;
    unsigned short xeraw = 3930;
    unsigned short yeraw = 3930;
    
    *xpixel = (unsigned short) (((double)(xraw - xbraw) / (double)(xeraw - xbraw)) * ILI9341_TFTWIDTH );
    *ypixel = (unsigned short) ((1.0 - (double)(yraw - ybraw) / (double)(yeraw - ybraw)) * ILI9341_TFTHEIGHT );
}

int IsInsideButton(BUTTON button, unsigned short px, unsigned short py)
{
    int flag = 0;
    if ( ((px > button.xc) && (px < button.xc + button.width)) && ((py > button.yc) && (py < button.yc + button.height)) )
        flag = 1;
    return flag;
}
