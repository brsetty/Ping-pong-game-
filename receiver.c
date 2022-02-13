//--------------------------------------------------Receiver code----------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------
#include "TM4C123.h"
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/adc.h"


#include <stdio.h>
#include "ST7735.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"


#define X_MAX 124
#define Y_MAX 128
#define X_MIN 8
#define Y_MIN 10

#define INITX_MAX 80
#define INITX_MIN 64
#define MIN_PADDLE 4
#define MAX_PADDLE 110

#define dx -2
#define dyMax  2
#define dyMin -1

const uint16_t bCircle[]= {
                            0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF,
                            0XFFFF, 0X0000, 0X0000, 0X0000, 0X0000, 0XFFFF,
                            0XFFFF, 0X0000, 0X0000, 0X0000, 0X0000, 0XFFFF,
                            0XFFFF, 0X0000, 0X0000, 0X0000, 0X0000, 0XFFFF,
                            0XFFFF, 0X0000, 0X0000, 0X0000, 0X0000, 0XFFFF,
                            0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF,
};

const uint16_t wCircle[]= {
                            0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF,
                            0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF,
                            0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF,
                            0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF,
                            0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF,
                            0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF, 0XFFFF,
};

const uint16_t paddle_3[] = {
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
                             0X0000, 0X0000, 0X0000, 0X0000,0X0000,0XFFFF,0XFFFF, 0XFFFF,
};

 uint32_t buff[1]= {0};
uint32_t randVal[1]= {0};

 //functions definition
 void DelayWait10ms (uint32_t n);
 void display_init(void);
 void display(void);
 void circle(int x, int y);
 void paddle(int16_t x, int16_t y,uint16_t color);
uint8_t sampleADCData(uint32_t digital_val);


int main()
{



    uint32_t adcRanVal = 0;
    uint32_t adcResult = 0;

    int16_t xball = 0;
    int16_t yball = 0;
    int16_t dxBall = 0;
    int16_t dyBall = 0;

    int16_t xpaddle1, ypaddle1;
    int16_t xpaddle2, ypaddle2;
    int8_t paddleMove1;
    int8_t paddleMove2;

    bool start = true;


//*********************************UART configuration*************************************************************************
    //Configure UART
SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
GPIOPinConfigure(GPIO_PC4_U1RX);
GPIOPinConfigure(GPIO_PC5_U1TX);
GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
//***************************************************************************************************************************

    PLL_Init(Bus80MHz);
        // iniltiaze LCD
    ST7735_InitR(INITR_REDTAB);



    //start of [1], [3]
    //set system clock
    SysCtlClockSet (SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    //initialize ADCO for PD0 and PD1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); // ENABLE ADCO MODULE
    // SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1); // ENABLE ADC1 MODULE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    // setting GPIO for ADCO MODULE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    // ENABLE ANI6,7 OF ADCO MODULE
    GPIOPinTypeADC (GPIO_PORTD_BASE, 0x08);

    ADCSequenceDisable(ADC0_BASE, 3);
    // ADCO MODULE, TRIGGER IS PROCESSOR EVENT, SEQUENCER O IS CONFIGURED
    ADCSequenceConfigure (ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    // ADCO MODULE, SEQUENCER O , FOR 0 SAMPLING, INPUT IS FROM CHANNEL 7 PDB
//    ADCSequenceStepConfigure (ADC0_BASE, 3, 0, ADC_CTL_CH5);
    // ADCO MODULE, SEQUENCER 0 , FOR 1 SAMPLING, INPUT IS FROM CHANNEL 6 PD1
    ADCSequenceStepConfigure (ADC0_BASE, 3, 0, ADC_CTL_CH4 | ADC_CTL_IE | ADC_CTL_END);
    // ENABLE THE SEQUENCE 1 FOR ADCO
    ADCSequenceEnable(ADC0_BASE, 3);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1); // ENABLE ADC1 MODULE
    GPIOPinTypeADC (GPIO_PORTE_BASE, 0x08);
    ADCSequenceDisable(ADC1_BASE, 3);
    ADCReferenceSet(ADC1_BASE, ADC_REF_INT);
    ADCSequenceConfigure (ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure (ADC1_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC1_BASE, 3);


    ST7735_FillScreen(0xFFFF);
    //paddle();
while(1)
{
    if(start == true)
    {
    ADCIntClear(ADC1_BASE, 3);
    ADCProcessorTrigger(ADC1_BASE, 3);
    while (!ADCIntStatus(ADC1_BASE,3,0)){}
    ADCSequenceDataGet(ADC1_BASE, 3, randVal);

     adcRanVal = randVal[0] & 0xFFF;
     srand(adcRanVal);
     xball = (rand() % (INITX_MAX - INITX_MIN + 1)) + INITX_MIN;
     yball = (rand() % (Y_MAX - Y_MIN + 1)) + Y_MIN;

     ADCIntClear(ADC1_BASE, 3);
    ADCProcessorTrigger(ADC1_BASE, 3);
    while (!ADCIntStatus(ADC1_BASE,3,0)){}
    ADCSequenceDataGet(ADC1_BASE, 3, randVal);

   adcRanVal = randVal[0] & 0xFFF;
     srand(adcRanVal);
     dxBall = dx;
     dyBall = (rand()%(dyMax-dyMin +1))+dyMin;

     if(dyBall ==0)
         dyBall=1;

        xpaddle1 = 4;
        ypaddle1 = 54;
        paddle(xpaddle1, ypaddle1,ST7735_BLACK);

        xpaddle2 = 123;
        ypaddle2 = 54;
        paddle(xpaddle2, ypaddle2,ST7735_BLACK);
        //ST7735_FillRect(xpaddle2, ypaddle2, 4, 60,ST7735_BLACK);
//***************************************************************************************************************************
                   ST7735_DrawCharS(59, 4, '3', ST7735_Color565 (0, 0, 139), ST7735_Color565 (255, 255, 255), 2);
                   SysCtlDelay(10000000);
                   ST7735_DrawCharS(59, 4, '3', ST7735_Color565 (255, 255, 255), ST7735_Color565 (255, 255, 255), 2);
                   ST7735_DrawCharS(59, 4, '2', ST7735_Color565 (0, 0, 139), ST7735_Color565 (255, 255, 255), 2);
                   SysCtlDelay(10000000);
                   ST7735_DrawCharS(59, 4, '2', ST7735_Color565 (255, 255, 255), ST7735_Color565 (255, 255, 255), 2);
                   ST7735_DrawCharS(59, 4, '1', ST7735_Color565 (0, 0, 139), ST7735_Color565 (255, 255, 255), 2);
                   SysCtlDelay(10000000);
                   ST7735_DrawCharS(59, 4, '1', ST7735_Color565 (255, 255, 255), ST7735_Color565 (255, 255, 255), 2);
                   SysCtlDelay(5000000);
                   ST7735_DrawCharS(59, 4, 'G', ST7735_Color565 (0, 0, 139), ST7735_Color565 (255, 255, 255), 2);
                   ST7735_DrawCharS(70, 4, 'O', ST7735_Color565 (0, 0, 139), ST7735_Color565 (255, 255, 255), 2);

                   SysCtlDelay(5000000);
                   ST7735_DrawCharS(59, 4, 'G', ST7735_Color565 (255, 255, 255), ST7735_Color565 (255, 255, 255), 2);
                   ST7735_DrawCharS(70, 4, 'O', ST7735_Color565 (255, 255, 255), ST7735_Color565 (255, 255, 255), 2);


            start = false;
        }

        ball_white(xball,yball);
        paddle(xpaddle1, ypaddle1,ST7735_WHITE);
        paddle(xpaddle2, ypaddle2,ST7735_WHITE);
       // ST7735_FillRect(xpaddle2, ypaddle2, 4, 60,ST7735_WHITE);
        xball += dxBall;
        yball += dyBall;
//----------------------------x paddle 2------------------------------------------------------------------------------------------------------

        if( (xball <= xpaddle2) && (xball >= (xpaddle2 - 5)) )
        {
            if((yball >= ypaddle2) && (yball <= ypaddle2 + 20))
            {
                dxBall *= (-1);
                xball = xpaddle2 - 5;
            }
        }

//-----------------------------x paddle 1-----------------------------------------------------------------------------------------------------
        if( (xball >= xpaddle1) && (xball <= (xpaddle1 + 5)) )
        {
            if((yball >= ypaddle1) && (yball <= ypaddle1 + 20))
            {
                dxBall *= (-1);
                xball = xpaddle1 + 5;
            }
        }

//------------------------------------y max and min----------------------------------------------------------------------------------------------
        if((yball <= Y_MIN) || (yball >= Y_MAX))
        {
            dyBall *= -1;

            if(yball <= Y_MIN)
                yball = Y_MIN;

            if(yball >= Y_MAX)
                yball = Y_MAX;
        }
//----------------------------------------------------------------------------------------------------------------------------------
        if(xball < xpaddle1)
        {
            if( !((yball >= ypaddle1) && (yball <= ypaddle1 + 20)) )
            {
                xball = xpaddle1;

                //yball >= ypaddle2) && (yball <= ypaddle2 + 20)
                ball_black(xball,yball);
                paddle(xpaddle1, ypaddle1,ST7735_BLACK);
                //SysCtlDelay(500000);
                ball_white(xball,yball);
                paddle(xpaddle1, ypaddle1,ST7735_WHITE);

                start = true;

                continue;
            }
        }
//----------------------------------------------------------------------------------------------------------------------------------
                if(xball > xpaddle2)
                {
                    if( !((yball >= ypaddle2) && (yball <= ypaddle2 + 20)) )
                    {
                        xball = xpaddle2;
                        ball_black(xball,yball);
                        paddle(xpaddle2, ypaddle2,ST7735_BLACK);
                        //ST7735_FillRect(xpaddle2, ypaddle2, 4, 60,ST7735_BLACK);
                        //SysCtlDelay(500000);
                        ball_white(xball,yball);
                        paddle(xpaddle2, ypaddle2,ST7735_WHITE);
                        //ST7735_FillRect(xpaddle2, ypaddle2, 4, 60,ST7735_WHITE);

                        start = true;

                        continue;
                    }
                }


//-------------------------------------------------------------------------------
    ADCIntClear(ADC0_BASE, 3);
    ADCProcessorTrigger(ADC0_BASE, 3);
    while (!ADCIntStatus(ADC0_BASE,3,0)){}
    ADCSequenceDataGet(ADC0_BASE, 3, buff);
    adcRanVal = buff[0] & 0xFFF;

        paddleMove1 = sampleADCData(adcRanVal);

        if(ypaddle1 > MIN_PADDLE && ypaddle1 < MAX_PADDLE)
        {
            ypaddle1 += paddleMove1;

            if(ypaddle1 < MIN_PADDLE)
                ypaddle1 = MIN_PADDLE;

            if(ypaddle1 > MAX_PADDLE)
                ypaddle1 = MAX_PADDLE;
        }
        else if(ypaddle1 == MIN_PADDLE && paddleMove1 > 0)
        {
            ypaddle1 += paddleMove1;
        }
        else if(ypaddle1 == MAX_PADDLE && paddleMove1 < 0)
        {
            ypaddle1 += paddleMove1;
        }
//-------------------------------------------------------------------------------


                  paddleMove2 = UARTCharGet(UART1_BASE);

                if(ypaddle2 > MIN_PADDLE && ypaddle2 < MAX_PADDLE)
                {
                    ypaddle2 += paddleMove2;

                    if(ypaddle2 < MIN_PADDLE)
                        ypaddle2 = MIN_PADDLE;

                    if(ypaddle2 > MAX_PADDLE)
                        ypaddle2 = MAX_PADDLE;
                }
                else if(ypaddle2 == MIN_PADDLE && paddleMove2 > 0)
                {
                    ypaddle2 += paddleMove2;
                }
                else if(ypaddle2 == MAX_PADDLE && paddleMove2 < 0)
                {
                    ypaddle2 += paddleMove2;
                }
//-------------------------------------------------------------------------------
        ball_black(xball,yball);
        paddle(xpaddle1, ypaddle1,ST7735_BLACK);
        paddle(xpaddle2, ypaddle2,ST7735_BLACK);
        SysCtlDelay(250000);

       /* ST7735_DrawCharS(59, 4, 'E', ST7735_Color565 (0, 0, 139), ST7735_Color565 (255, 255, 255), 2);
        ST7735_DrawCharS(70, 4, 'N', ST7735_Color565 (0, 0, 139), ST7735_Color565 (255, 255, 255), 2);
        ST7735_DrawCharS(84, 4, 'D', ST7735_Color565 (0, 0, 139), ST7735_Color565 (255, 255, 255), 2);
        SysCtlDelay(1000000);
        ST7735_DrawCharS(59, 4, 'E', ST7735_Color565 (255, 255, 255), ST7735_Color565 (255, 255, 255), 2);
        ST7735_DrawCharS(70, 4, 'N', ST7735_Color565 (255, 255, 255), ST7735_Color565 (255, 255, 255), 2);
        ST7735_DrawCharS(84, 4, 'D', ST7735_Color565 (255, 255, 255), ST7735_Color565 (255, 255, 255), 2);
        SysCtlDelay(500000);*/

    }
    return 0;
  }
 //----------------------------------------------------------------------------
void ball_black(int x,int y){
    ST7735_DrawBitmap(x,y,bCircle,6,6);
}

void ball_white(int x,int y){
    ST7735_DrawBitmap(x,y,wCircle,6,6);
}
void paddle(int16_t x, int16_t y,uint16_t color)
{
    ST7735_FillRect(x, y, 4, 20,color);
}



uint8_t sampleADCData(uint32_t digital_val)
{
    uint8_t result = 0;

    if( (digital_val > 0) && (digital_val <= 409) )
        result = 10;
    else if( (digital_val >= 410) && (digital_val <= 818) )
        result = 8;
    else if( (digital_val >= 819) && (digital_val <= 1227) )
        result = 5;
    else if( (digital_val >= 1228) && (digital_val <= 1636) )
        result = 1;
    else if( (digital_val >= 1637) && (digital_val <= 2045) )
        result = 0;
    else if( (digital_val >= 2046) && (digital_val <= 2454) )
        result = -1;
    else if( (digital_val >= 2455) && (digital_val <= 2863) )
        result = -3;
    else if( (digital_val >= 2864) && (digital_val <= 3272) )
        result = -5;
    else if( (digital_val >= 3273) && (digital_val <= 3681) )
            result =-8;
    else if( (digital_val >= 3682) && (digital_val <= 4095) )
            result = -10;

    return result;
}
