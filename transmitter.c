/******************************************************************************************************************
*            Advanced embedded systems course - ECGR-5101
*			 Lab 9 - Design a Multiplayer ping pong game using TIVA C series board and BoosterPack MKII
*            lab partner : Revanth Badadha , Bharadwaaj Varadan,Karthik Madala,Dhanvin Athray
*            
*
* Reference://https://github.com/obergog/MSP432_Game_System/blob/master/game.c 
******************************************************************************************************************
											TRANSMITTER PROGRAM
/******************************************************************************************************************/
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

#define INITX_MAX 124
#define INITX_MIN 64
#define MIN_PADDLE 4
#define MAX_PADDLE 110


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


 void paddle(int16_t x, int16_t y,uint16_t color);
uint8_t sampleADCData(uint32_t digital_val);
//functions definition
void DelayWait10ms (uint32_t n);
void display_init(void);
void display(void);

int main()
{
//char buffer[50];

//***************************************************************************************************************************
    //Configure UART
SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
GPIOPinConfigure(GPIO_PC4_U1RX);
GPIOPinConfigure(GPIO_PC5_U1TX);
GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));


//***************************************************************************************************************************
    uint32_t adcRanVal = 0;
    uint32_t adcResult = 0;

    uint16_t xpaddle, ypaddle;
    int8_t paddleMove;

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
        xpaddle = 4;
        ypaddle = 54;
        paddle(xpaddle, ypaddle,ST7735_BLACK);

    while(1)
    {

            //uartTxBytes(UART0_BASE, buffer, strlen(buffer));

        //paddle(xpaddle, ypaddle,ST7735_WHITE)

        ADCIntClear(ADC0_BASE, 3);
           ADCProcessorTrigger(ADC0_BASE, 3);
           while (!ADCIntStatus(ADC0_BASE,3,0)){}
           ADCSequenceDataGet(ADC0_BASE, 3, buff);
           adcRanVal = buff[0] & 0xFFF;

               paddleMove = sampleADCData(adcRanVal);
               UARTCharPut(UART1_BASE,paddleMove);

               if(ypaddle > MIN_PADDLE && ypaddle < MAX_PADDLE)
               {
                   ypaddle += paddleMove;

                   if(ypaddle < MIN_PADDLE)
                       ypaddle = MIN_PADDLE;

                   if(ypaddle > MAX_PADDLE)
                       ypaddle = MAX_PADDLE;
               }
               else if(ypaddle == MIN_PADDLE && paddleMove > 0)
               {
                   ypaddle += paddleMove;
               }
               else if(ypaddle == MAX_PADDLE && paddleMove < 0)
               {
                   ypaddle += paddleMove;
               }

               paddle(xpaddle, ypaddle,ST7735_BLACK);
                      SysCtlDelay(250000);
               paddle(xpaddle, ypaddle,ST7735_WHITE);
    }
    return 0;
}

void paddle(int16_t x, int16_t y,uint16_t color)
{
    ST7735_FillRect(x, y, 4, 20,color);
}



uint8_t sampleADCData(uint32_t digital_val)
{
    int result = 0;

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
