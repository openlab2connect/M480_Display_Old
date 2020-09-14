/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Implement SPI Master loop back transfer. This sample code needs to
 *           connect MISO_0 pin and MOSI_0 pin together. It will compare the
 *           received data with transmitted data.
 *
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#include <string.h>
#include "font28x28.h"

#define TEST_COUNT  64
#define PLL_CLOCK   192000000

//#define SPI0_HALFDPX_MASTER

//RGB color definitions
#define Black           0x0000      /*   0,   0,   0 */
#define Navy            0x000F      /*   0,   0, 128 */
#define DarkGreen       0x03E0      /*   0, 128,   0 */
#define DarkCyan        0x03EF      /*   0, 128, 128 */
#define Maroon          0x7800      /* 128,   0,   0 */
#define Purple          0x780F      /* 128,   0, 128 */
#define Olive           0x7BE0      /* 128, 128,   0 */
#define LightGrey       0xC618      /* 192, 192, 192 */
#define DarkGrey        0x7BEF      /* 128, 128, 128 */
#define Blue            0x001F      /*   0,   0, 255 */
#define Green           0x07E0      /*   0, 255,   0 */
#define Cyan            0x07FF      /*   0, 255, 255 */
#define Red             0xF800      /* 255,   0,   0 */
#define Magenta         0xF81F      /* 255,   0, 255 */
#define Yellow          0xFFE0      /* 255, 255,   0 */
#define White           0xFFFF      /* 255, 255, 255 */
#define Orange          0xFD20      /* 255, 165,   0 */
#define GreenYellow     0xAFE5      /* 173, 255,  47 */

void SYS_Init(void)
{
    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Select PCLK0 as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Setup SPI0 multi-function pins */
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_SPI0_MOSI | SYS_GPA_MFPL_PA1MFP_SPI0_MISO | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA3MFP_SPI0_SS;
    
    #ifdef SPI0_HALFDPX_MASTER
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_SPI0_MOSI | SYS_GPA_MFPL_PA2MFP_SPI0_CLK  | SYS_GPA_MFPL_PA3MFP_SPI0_SS;
    #endif

    #ifdef SPI0_EXTRA_CONFIG
    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Enable SPI0 I/O high slew rate */
    GPIO_SetSlewCtl(PA, 0xF, GPIO_SLEWCTL_HIGH);
    #endif
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();
}

void delay_us(int usec)
{
    /*
     *  Configure Timer0, clock source from XTL_12M. Prescale 12
     */
    /* TIMER0 clock from HXT */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_CLKSEL1_TMR0SEL_HXT;
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;
    TIMER0->CTL = 0;        /* disable timer */
    TIMER0->INTSTS = (TIMER_INTSTS_TIF_Msk | TIMER_INTSTS_TWKF_Msk);   /* write 1 to clear for safety */
    TIMER0->CMP = usec;
    TIMER0->CTL = (11 << TIMER_CTL_PSC_Pos) | TIMER_ONESHOT_MODE | TIMER_CTL_CNTEN_Msk;

    while (!TIMER0->INTSTS);
}

void mdelay(int msec) {
    delay_us(msec *1000);
}

void set_reg_index(uint8_t reg) {
	uint16_t tx_buf = 0x7400 | reg;
	uint16_t rb;

	SPI_WRITE_TX(SPI0, (uint32_t)tx_buf);
	/* Check SPI0 busy status */
	while(SPI_IS_BUSY(SPI0));
	rb = SPI_READ_RX(SPI0);
	//printf("SPI:set %#x MISO:%#x\n", tx_buf, rb);
}

void reg_read(uint8_t reg) {
    uint16_t tx_buf;
    uint16_t startbyte = 0x7700;
    uint8_t rb;

    set_reg_index(reg);

    tx_buf = startbyte | reg;
    SPI_WRITE_TX(SPI0, (uint32_t)tx_buf);
	/* Check SPI0 busy status */
	while(SPI_IS_BUSY(SPI0));
	rb = SPI_READ_RX(SPI0);
	printf("SPI:read %#x MISO:%#x\n", tx_buf, rb);
}

void write_reg(uint8_t reg, uint8_t data) {
    uint16_t tx_buf;
    uint16_t startbyte = 0x7600;
    uint16_t rb;

    set_reg_index(reg);

    tx_buf = startbyte | data;
	SPI_WRITE_TX(SPI0, (uint32_t)tx_buf);
	/* Check SPI0 busy status */
	while(SPI_IS_BUSY(SPI0));
	rb = SPI_READ_RX(SPI0);
	printf("SPI:write %#x MISO:%#x\n", tx_buf, rb);
}

void window (unsigned int x, unsigned int y, unsigned int w, unsigned int h) {
    write_reg(0x03, x );
    write_reg(0x02, (x >> 8));
    write_reg(0x05, x+w-1 );
    write_reg(0x04, (x+w-1 >> 8));
    write_reg(0x07,  y );
    write_reg(0x06, ( y >> 8));
    write_reg(0x09, ( y+h-1 ));
    write_reg(0x08, ( y+h-1 >> 8));
}

// void set_orientation(int orientation) {
//     switch (orientation) {
//         case 0:
//             write_reg(0x16, 0x00);
//             break;
//         case 1:
//             write_reg(0x16, 0x60);
//             break;
//         case 2:
//             write_reg(0x16, 0xC0);
//             break;
//         case 3:
//             write_reg(0x16, 0xA0);
//             break;
//     }
// }

void set_addr_win(int xs, int ys, int xe, int ye) {
    write_reg(0x02, (xs >> 8) & 0xFF);
    write_reg(0x03, xs & 0xFF);
    write_reg(0x04, (xe >> 8) & 0xFF);
    write_reg(0x05, xe & 0xFF);
    write_reg(0x06, (ys >> 8) & 0xFF);
    write_reg(0x07, ys & 0xFF);
    write_reg(0x08, (ye >> 8) & 0xFF);
    write_reg(0x09, ye & 0xFF);
    set_reg_index(0x22);
}

void write_pixel(int count, uint16_t rgb565) {
	uint32_t tx_buf = 0;
    uint16_t startbyte = 0x7600;
	uint32_t rb;
    int i;

    for (i=1; i<=count; i++) {
	    tx_buf = (startbyte << 8) | rgb565;
	    SPI_WRITE_TX(SPI0, tx_buf);
	    /* Check SPI0 busy status */
	    while(SPI_IS_BUSY(SPI0));
	    rb = SPI_READ_RX(SPI0);
    }
}

static void reg_dump(uint16_t start, uint16_t end) {
    uint16_t reg;

    for (reg=start; reg <= end; reg++) {
        reg_read(reg);
    }
}

static int init_display() {
    //uint8_t par = 0x70;

    write_reg(0x60, 0x08);
    write_reg(0xDF, 0x52); //OTPS1B
    write_reg(0xE2, 0x02);
    write_reg(0xE8, 0x50); //50
    write_reg(0xEC, 0x3C); //3  // 3c

    /* Gamma settings  -----------------------------------------------------------*/
//    write_reg(0x40,0x00);   //
//    write_reg(0x41,0x00);   //
//    write_reg(0x42,0x01);   //
//    write_reg(0x43,0x13);   //
//    write_reg(0x44,0x10);   //
//    write_reg(0x45,0x26);   //
//    write_reg(0x46,0x08);   //
//    write_reg(0x47,0x51);   //
//    write_reg(0x48,0x02);   //
//    write_reg(0x49,0x12);   //
//    write_reg(0x4A,0x18);   //
//    write_reg(0x4B,0x19);   //
//    write_reg(0x4C,0x14);   //
//    write_reg(0x50,0x19);   //
//    write_reg(0x51,0x2F);   //
//    write_reg(0x52,0x2C);   //
//    write_reg(0x53,0x3E);   //
//    write_reg(0x54,0x3F);   //
//    write_reg(0x55,0x3F);   //
//    write_reg(0x56,0x2E);   //
//    write_reg(0x57,0x77);   //
//    write_reg(0x58,0x0B);   //
//    write_reg(0x59,0x06);   //
//    write_reg(0x5A,0x07);   //
//    write_reg(0x5B,0x0D);   //
//    write_reg(0x5C,0x1D);   //
//    write_reg(0x5D,0xCC);   //

    write_reg(0x40, 0x00); //VRP00~05
    write_reg(0x41, 0x06); //VRP10~15
    write_reg(0x42, 0x05); //VRP20~25
    write_reg(0x43, 0x1a); //VRP30~35
    write_reg(0x44, 0x20); //VRP40~45
    write_reg(0x45, 0x3F); //VRP50~55
    write_reg(0x46, 0x0B); //PRP00~06
    write_reg(0x47, 0x6C); //PRP10~16
    write_reg(0x48, 0x02); //PKP00~04
    write_reg(0x49, 0x0F); //PKP10~14
    write_reg(0x4A, 0x12); //PKP20~24
    write_reg(0x4B, 0x12); //PKP30~34
    write_reg(0x4C, 0x10); //PKP40~44
    write_reg(0x50, 0x00);
    write_reg(0x51, 0x1F);
    write_reg(0x52, 0x25);
    write_reg(0x53, 0x38);
    write_reg(0x54, 0x39);
    write_reg(0x55, 0x3F);
    write_reg(0x56, 0x03);
    write_reg(0x57, 0x64);
    write_reg(0x58, 0x0F);
    write_reg(0x59, 0x0d);
    write_reg(0x5A, 0x0d);
    write_reg(0x5B, 0x10);
    write_reg(0x5C, 0x1d);
    write_reg(0x5D, 0xCC);

    //Power Voltage Setting
    write_reg(0x1B, 0x1E);  //VRH 1A
    write_reg(0x25, 0x1E);  //NVRH 1B
    write_reg(0x1A, 0x05);  //BT 02
    write_reg(0x1E, 0x16);  //13 -> 16
    
    //VCOM
    write_reg(0x23, 0x70);  //for Flicker adjust //can reload from OTP 6E
    
    //Power on Setting
    write_reg(0x18, 0x34);  //I_RADJ,N_RADJ, Normal mode 60Hz
    write_reg(0x19, 0x01);  //OSC_EN='1',start Osc

    delay_us(1000);
    write_reg(0x1C, 0x03);
    delay_us(1000);
    write_reg(0x01, 0x02);  //RB7:DP_STB='0',out deep sleep
                            //RB1 INVON=1 invert bit
                            //RB0 PTLON patrial mode 
    write_reg(0x1F, 0xAC);
    delay_us(5000);
    write_reg(0x1F, 0xA4);
    delay_us(5000);
    write_reg(0x1F, 0xB4);
    delay_us(100000);
    write_reg(0x1F, 0xF4);
    delay_us(5000);
    write_reg(0x1F, 0xD4);

    //262k/65k color selection
    write_reg(0x17, 0x05);  //RGB 16bit/pixel
    
    //SET PANEL
    //write_reg(0x36, 0x09); //09 SS_P,GS_P,REV_P,BGR_P
    write_reg(0x36, 0x01);  //09 SS_P,GS_P,REV_P,BGR_P
    write_reg(0x2F, 0x22);  //NW

    //Display ON Setting 
    write_reg(0x28, 0x38);  //GON=1,DTE=1,D=10
    delay_us(40000);
    write_reg(0x28, 0x3C);  //GON=1, DTE=1, D=11

    //X-Y
    write_reg(0x16, 0x00);  //RB7 MY=0 MX=0 MV=1 ML=0 BGR=0
    //X-Y Exchange Y-Invert
    //write_reg(0x16, 0x60);  //MY=0 MX=1 MV=1 ML=0 BGR=0
    //X-Y Exchange
    //write_reg(0x16, 0x20);  //MY=0 MX=0 MV=1 ML=0 BGR=0

    //set_orientation(0);

    write_reg(0x02, 0x00);
    write_reg(0x03, 0x59);  //Column Start
    write_reg(0x04, 0x00);
    write_reg(0x05, 0x96);  //Column End
    write_reg(0x06, 0x00);
    write_reg(0x07, 0x00);  //Row Start
    write_reg(0x08, 0x01);
    write_reg(0x09, 0x3F);  //Row End

//    write_reg(0x02, 0x00);
//    write_reg(0x03, 0x00);  //Row Start
//    write_reg(0x04, 0x01);
//    write_reg(0x05, 0x3F);  //Row End
//    write_reg(0x06, 0x00);
//    write_reg(0x07, 0x59);  //Column Start
//    write_reg(0x08, 0x00);
//    write_reg(0x09, 0x96);  //Column End
    set_reg_index(0x22);

    return 0;
 }

void SPI_Read(char *byteArray, size_t byteSize) {
    uint32_t i;
    uint8_t rb;

    for (i=0; i < byteSize; i++) {
        SPI_WRITE_TX(SPI0, (uint32_t)byteArray[i]);
        /* Check SPI0 busy status */
        while(SPI_IS_BUSY(SPI0));

        #ifdef SPI0_HALFDPX_MASTER
        /* Wait for Rx FIFO not empty */
        while (SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0)) {}
        #endif

        rb = SPI_READ_RX(SPI0);
        printf("SPI:read %#x MISO:%#x\n", byteArray[i], rb);
    }
}

void SPI_Write(char * byteArray, size_t byteSize) {
    uint8_t i;
    uint8_t rb;

    for (i=0; i < byteSize; i++) {
        SPI_WRITE_TX(SPI0, (uint32_t)byteArray[i]);
        /* Check SPI0 busy status */
        while(SPI_IS_BUSY(SPI0));
        rb = SPI_READ_RX(SPI0);
        printf("SPI:write %#x MISO:0x%#x\n", byteArray[i], rb);
    }
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_3, 24, 48000000);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI0, SPI_SS, SPI_SS_ACTIVE_LOW);

    #ifdef SPI0_HALFDPX_MASTER

    /* Set master SPI0 to half-duplex mode */
    SPI0->CTL |= SPI_CTL_HALFDPX_Msk;
    /* Enable half-duplex will produce TXFBCLR (SPIx_FIFOCTL[9]) and RXFBCLR (SPIx_FIFOCTL[8])*/
    while (SPI0->STATUS & SPI_STATUS_TXRXRST_Msk) {}
    /* Set master SPI0 data direction to input */
    SPI0->CTL &= ~SPI_CTL_DATDIR_Msk;

    #endif
}

void cls(int xs, int ys, int xe, int ye) {
	int i;
	set_addr_win(xs, ys, xe, ye);
	for (i=0;  i<((xe-xs+1)*(ye-ys+1)); i++) {
		write_pixel(1, Black);
	}
}

int covertASCIIToFoneIndex(char * ascii, size_t len) {
	int c;
	for (c=0; c<len; c++) {
    	if (ascii[c] == 32) { //space
    		ascii[c] = 0;
    	} else if (ascii[c] >= 65 && ascii[c] < 97) {
    		ascii[c] -= 64; //A-Z
    	} else {
    		ascii[c] -= 70; //a-z
    	}
    }
}

void show_info(int start_x, int start_y, int h, int w) {
    int i;
	int j;
	int c;
	unsigned char pixel;
	char display[16] = {0};
	char info[] = "Fasten seat belt"; //in clude Null character

	memcpy(&display, info, sizeof(display));
	covertASCIIToFoneIndex(&display[0], sizeof(display));

    for (c=0; c<sizeof(display); c++) {
    	set_addr_win(start_x, start_y, (start_x+h-1), (start_y+w-1));
    	start_y += 18;
		for (i=0; i<112; i++) {
			pixel = Arial28x28[ display[c] ][i];
			for (j=0; j<8; j++) {
				if ((pixel >> j) & 0x1)
					write_pixel(1, White);
				else
					write_pixel(1, Black);
			}
		}
    }

}

void show_info2(int start_x, int start_y, int h, int w) {
    int i;
	int j;
	int c;
	unsigned char pixel;
	char display[10] = {0};
	char info[] = "No smoking";

	memcpy(&display, info, sizeof(display));
	covertASCIIToFoneIndex(&display[0], sizeof(display));

    for (c=0; c<sizeof(display); c++) {
    	set_addr_win(start_x, start_y, (start_x+h-1), (start_y+w-1));
    	start_y += 20;
		for (i=0; i<112; i++) {
			pixel = Arial28x28[ display[c] ][i];
			for (j=0; j<8; j++) {
				if ((pixel >> j) & 0x1)
					write_pixel(1, White);
				else
					write_pixel(1, Black);
			}
		}
    }

}

void show_info3(int start_x, int start_y, int h, int w) {
    int i;
	int j;
	int c;
	unsigned char pixel;
    char display[12] = {0};
    char display2[9] = {0};
	char info[] = "Come back to";
	char info2[] = "your seat";

	memcpy(&display, info, sizeof(display));
	memcpy(&display2, info2, sizeof(display2));
    
    covertASCIIToFoneIndex(&display[0], sizeof(display));
    covertASCIIToFoneIndex(&display2[0], sizeof(display2));

	for (c=0; c<sizeof(display); c++) {
		set_addr_win(start_x, start_y, (start_x+h-1), (start_y+w-1));
		start_y += 22;
		for (i=0; i<112; i++) {
			pixel = Arial28x28[ display[c] ][i];
			for (j=0; j<8; j++) {
				if ((pixel >> j) & 0x1)
					write_pixel(1, White);
				else
					write_pixel(1, Black);
			}
		}
	}

	start_x += 30;
	start_y = 5;
	for (c=0; c<sizeof(display2); c++) {
			set_addr_win(start_x, start_y, (start_x+h-1), (start_y+w-1));
			start_y += 20;
			for (i=0; i<112; i++) {
				pixel = Arial28x28[ display2[c] ][i];
				for (j=0; j<8; j++) {
					if ((pixel >> j) & 0x1)
						write_pixel(1, White);
					else
						write_pixel(1, Black);
				}
			}
   }
}

int main(void)
 {
	unsigned char pixel;
	int i;
	int j;
	int c;

	//int total_fone_size = 112;
	int byte_per_charX = 4;
	int h = byte_per_charX * 8;
	int w = 28;
	int start_x = 90;
    int start_y = 5;

    // space = 0,
    // A=1 a=26+1
    // B=2
    // C=3
    // D=4
    // E=5
    // F=6
    // G=7
    // H=8
    // I=9
    // J=10
    // K=11
    // L=12
    // M=13
    // N=14
    // O=15
    // P=16
    // Q=17
    // R=18
    // S=19
    // T=20
    // U=21
    // V=22
    // W=23
    // X=24
    // Y=25
    // Z=26 z=26+256

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("|                   M480 SPI0 Driver Test Code                      |\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("\n");

    init_display();
    
    //dump register
    //reg_dump(0xFF, 0xFF);
    cls(90, 5, 190, 320);

    show_info(start_x, start_y, h, w);
    delay_us(1000*1000*2); //delay 2sec
	cls(90, 5, 190, 320);

    show_info2(start_x, start_y, h, w);
    delay_us(1000*1000*2); //delay 2sec
    cls(90, 5, 190, 320);

    show_info3(start_x, start_y, h, w);

    /* Close SPI0 */
    SPI_Close(SPI0);

    while(1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
