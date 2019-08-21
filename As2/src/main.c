#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"
#include "lpc17xx.h"
#include "uart2.h"

#include "stdio.h"
#include "stdlib.h"
#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "temp.h"
#include "light.h"
#include "pca9532.h"
#include "math.h"
#include "regex.h"
#include "sys/types.h"

#include "rgb.h"
#include "led7seg.h"

//SW3 EINT0 interrupt definitions
#define PINSEL_EINT0    20
#define SBIT_EINT0      0
#define SBIT_EXTMODE0   0
#define SBIT_EXTPOLAR0  0

//Timer1 definitions
#define SBIT_TIMER1  2
#define SBIT_MR0I    0
#define SBIT_MR0R    1
#define PCLK_TIMER1  4
#define SBIT_CNTEN   0
#define mstous(x)  (x*1000)

//UA
#define UART_PORT   (LPC_UART_TypeDef *)LPC_UART3   // Select UART3

#define PI 3.1416
uint8_t rev_buf[255];                               // Reception buffer
uint32_t rev_cnt = 0;

int isReceived = 0;
int LIGHT_LOTHRESHOLD = 30;               //30
#define  BATTERY_THRESHOLD 12.5
int TILT_THRESHOLD = 30;
int flag_rda = 0;
volatile int force_sleep_flag = 0;
volatile uint8_t sw4 = 1;
int counter_for_send_data = 0;

uint8_t joystickStatus = 0;
static unsigned char TILT_THRESHOLD_TXT[] = "Poor Landing Attitude \r\n";
static unsigned char TILT_CLEAR_TXT[] = "Safe to Land! \r\n";

int password[40] = {'8','8','8','8'};
typedef enum{
	ANIM,
	START,
    ORBIT,
    ORBIT_TO_LANDING,
    LANDING,
    EXPLORING,
    SLEEPING
} mode_type;

volatile uint32_t msTicks;
volatile int flag_sw3 = 0;
volatile mode_type mode = ANIM;
volatile int Changemodeflag = 0;
int sendflag_orbit=1;
volatile int sendflag_landing=1;
volatile int sendflag_landing_success=1;
volatile int sendflag_exploring=1;
volatile int sendflag_sleeping=1;
volatile uint16_t battery_level = 0xFFFF;    // from 0 full power  - 16 no power
volatile int counter_for_battery_consumption = 0;           //a counter used in battery consumption
volatile int counter_for_oled_refreshment = 0;           //a counter used in battery consumption

volatile int cum_counter_battery = 0;         //0 - full power    - 16 no power
volatile int relative_battery_level =0;
volatile int isCharging = 0;
int32_t timer_current = -1;
int32_t timer_past = -1;
uint32_t period = 50;
static char* msg = NULL;

int radius = 2;
int counter_for_animation = 0;

static uint8_t TILT_FLAG = 1;
static uint8_t TILT_UART_FLAG = 0;

uint8_t joydisp_flag = 1;


int  test_counter = 0;
int is_sleeping_to_exploring_charging = 0;

const size_t nmatch = 1;
double initial_temperature;
int first_time_start = 1;
uint32_t my_temp;
int32_t x_off=0;
int32_t y_off=0;
int32_t z_off=0;
int8_t x_coord = 0;
int8_t y_coord = 0;
int8_t z_coord = 0;
int initial_x = 3;
int initial_y = 60;
int x0_t = 0;
int y0_t = 40;
int x1_t;
int y1_t;
double j = 8.0;
int increament = 5;
double frequency = 8.0;
unsigned int getPrescalarForUs(uint8_t timerPlckBit);


void SysTick_Handler (void) {
	msTicks++;
}

uint32_t getTicks (void){
	return msTicks;
}

void setMode(mode_type setmode){
	mode = setmode;
	Changemodeflag = 1;
}

void pinsel_uart3(void){
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 0;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 1;
    PINSEL_ConfigPin(&PinCfg);
}

void init_uart(void){
    UART_CFG_Type uartCfg;
    uartCfg.Baud_rate = 115200;
    uartCfg.Databits = UART_DATABIT_8;
    uartCfg.Parity = UART_PARITY_NONE;
    uartCfg.Stopbits = UART_STOPBIT_1;
    //pin select for uart3;
    pinsel_uart3();
    //supply power & setup working parameters for uart3
    UART_Init(LPC_UART3, &uartCfg);
    //enable transmit for uart3
    UART_TxCmd(LPC_UART3, ENABLE);

}

static void init_ssp(void) {
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;
	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);
}

static void init_i2c(void) {
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_gpio(void) {
	// SW4: P1.31
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1<<31, 0);

	// SW3: P2.10
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1<<10, 0);

	// Red LED: P2.0
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2,1,1);

	// Blue LED: P0.26
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 26;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(0,(1<<26),1);

	// Green LED: p2.1
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2,(1<<1),1);

	//joystick
 	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 15;
	PINSEL_ConfigPin(&PinCfg);     //construct

 	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 16;
	PINSEL_ConfigPin(&PinCfg);     //construct

 	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 17;
	PINSEL_ConfigPin(&PinCfg);     //construct

 	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 3;
	PINSEL_ConfigPin(&PinCfg);     //construct

 	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 4;
	PINSEL_ConfigPin(&PinCfg);     //construct

	//p2.6 sw4 interrupt
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 6;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1<<6, 0);



}

void init_Interrupt() {
	//SW3
    LPC_SC->EXTINT      = (1<<SBIT_EINT0)    ;	     /* Clear Pending interrupts */
    LPC_PINCON->PINSEL4=(1<<PINSEL_EINT0);           /*Configure P2_10 as EINT0*/
    LPC_SC->EXTMODE     = (1<<SBIT_EXTMODE0) ;       /* Configure EINT0 as Edge Triggered*/
    LPC_SC->EXTPOLAR    = (0<<SBIT_EXTPOLAR0);       /* Configure EINT0 as Falling Edge */


	//Timer 1
	LPC_SC->PCONP      |= (1<<SBIT_TIMER1); /* Power ON Timer1 */
    LPC_TIM1->MCR  = (1<<SBIT_MR0I) | (1<<SBIT_MR0R); // Clear TC on MR0 match and Generate Interrupt
    LPC_TIM1->PR   = getPrescalarForUs(PCLK_TIMER1); // Prescalar for 1us
    LPC_TIM1->MR0  = mstous(period); //        timer interrupt every 500ms
    LPC_TIM1->TCR  = (1<<SBIT_CNTEN); // Start timer by setting the Counter Enable

   // NVIC_ClearPendingIRQ(TIMER1_IRQn);



    LPC_GPIOINT->IO0IntEnF |= 1<<15;

    LPC_GPIOINT->IO0IntEnF |= 1<<16;

    LPC_GPIOINT->IO0IntEnF |= 1<<17;

    LPC_GPIOINT->IO2IntEnF |= 1<<3;

    LPC_GPIOINT->IO2IntEnF |= 1<<4;


    //sw4 interrupt as p2.6 using j5.21
    LPC_GPIOINT->IO2IntEnF |= 1<<6;

    NVIC_ClearPendingIRQ(TIMER1_IRQn);
    NVIC_ClearPendingIRQ(EINT3_IRQn);
    NVIC_ClearPendingIRQ(EINT0_IRQn);

    NVIC_EnableIRQ(EINT3_IRQn); // Enable EINT3 interrupt
    NVIC_EnableIRQ(EINT0_IRQn);
    NVIC_EnableIRQ(TIMER1_IRQn); // Enable Timer1 interrup

    NVIC_ClearPendingIRQ(TIMER1_IRQn);
    NVIC_ClearPendingIRQ(EINT3_IRQn);
    NVIC_ClearPendingIRQ(EINT0_IRQn);

    NVIC_SetPriorityGrouping(5);

    NVIC_SetPriority(EINT3_IRQn, 0x07);
    NVIC_SetPriority(TIMER1_IRQn, 0x08);
    NVIC_SetPriority(EINT0_IRQn, 0x01);
    LPC_GPIOINT->IO2IntClr = 1<<6;
}
void EINT3_IRQHandler(void)
{
	if(LPC_GPIOINT->IO2IntStatF>>6& 0x1){
		if(mode == EXPLORING || mode == SLEEPING){
			LPC_GPIOINT->IO2IntClr = 1<<6;
			sw4=0;
			battery_Recovery();
			sw4 = 1;
		}
	} else if(mode == ORBIT){
		if ((LPC_GPIOINT->IO0IntStatF>>15)& 0x1)

		{
			//DOWN
			joystickStatus = 1;
			LPC_GPIOINT->IO0IntClr = 1<<15;

		} else if ((LPC_GPIOINT->IO0IntStatF>>16)& 0x1)
		{
			//RIGHT
			joystickStatus = 2;
			LPC_GPIOINT->IO0IntClr = 1<<16;

		} else if ((LPC_GPIOINT->IO0IntStatF>>17)& 0x1)
		{   //CENTER
			joystickStatus = 3;
			LPC_GPIOINT->IO0IntClr = 1<<17;

		} else if ((LPC_GPIOINT->IO2IntStatF>>3)& 0x1)
		{
			joystickStatus = 4;       //UP
			LPC_GPIOINT->IO2IntClr = 1<<3;

		} else if ((LPC_GPIOINT->IO2IntStatF>>4)& 0x1)
		{
			joystickStatus = 5;       //LEFT
			LPC_GPIOINT->IO2IntClr = 1<<4;
		}
		joystick_select();
	}
}

void joystick_select(){
	switch (joystickStatus){
		case 2 :
			joydisp_flag=1;

			break;


		case 5:
			joydisp_flag=2;


			break;
		case 1: ///down
			if(joydisp_flag == 1){
				LIGHT_LOTHRESHOLD -= 10;
			} else if(joydisp_flag ==2){
				TILT_THRESHOLD -= 2;
			}
			break;
		case 4: //up
			if(joydisp_flag == 1){
				LIGHT_LOTHRESHOLD += 10;
			} else if(joydisp_flag ==2){
				TILT_THRESHOLD += 2;
			}
			break;
		default:
			break;

	}
}

void EINT0_IRQHandler(){
	//SW3 Interrupt
	LPC_SC -> EXTINT = (1<<SBIT_EINT0);               /* Clear Interrupt Flag */
	timer_current = getTicks();
	if(timer_current - timer_past <= 1000 && timer_past  > 0) {
		flag_sw3 = 1;
		//NVIC_ClearPendingIRQ(EINT3_IRQn);
		 NVIC_DisableIRQ(EINT3_IRQn);
	}
	timer_past = timer_current;
}

void TIMER1_IRQHandler(void){
	LPC_TIM1->IR = LPC_TIM1->IR; // Clear Timer1 interrupt
	uint32_t red = (GPIO_ReadValue(2) >> 0) & 0x01;     //read red
	uint32_t blue = (GPIO_ReadValue(0) >> 26) & 0x01;
	switch (mode) {
		case ANIM :
			start_animation();
			break;
		case ORBIT_TO_LANDING: // BLINK_BLUE

			if (blue) {
				GPIO_ClearValue(0, 1<<26);
			}
			else{
				GPIO_SetValue(0, 1<<26);
			}
		    break;
		case LANDING: // ALTERNATE_LED
			sent_data_to_uart();

			if (blue) {
				GPIO_ClearValue(0, 1<<26);
				GPIO_SetValue(2, 1<<0);
			}
			else {
				GPIO_SetValue(0, 1<<26);
				GPIO_ClearValue(2, 1<<0);
			}
		    break;
	    case EXPLORING:
	    	sent_data_to_uart();
	    	//battery check
			// 1) not charging
	    	if (!isCharging){
	    		battery_Consumption();
				GPIO_SetValue(0, (1<<26)); // BLUE
	    	}
	    	break;
	    case SLEEPING:
	    	break;
	}
}

void sent_data_to_uart(){
	int display_acc0[40];
	int display_acc1[40];
	if(counter_for_send_data == 9/*9*/) {                            //every 10 cycle,
		switch (mode) {
		case LANDING:
			//send x,y,z
			sprintf(display_acc0,"x coordinates: %.2fg, y coordinates: %.2f g, z coordinates: %.2f g\r\n", x_coord /64.0, y_coord/64.0, z_coord/64.0);
			UART_Send(LPC_UART3, (uint8_t *)display_acc0 , strlen(display_acc0), BLOCKING);
			sprintf(display_acc1,"light: %d lux\r\n", light_read());
			UART_Send(LPC_UART3, (uint8_t *)display_acc1 , strlen(display_acc1), BLOCKING);
			break;
		case EXPLORING:
			//send temperature
			sprintf(display_acc0,"temperature: %.2f deg\r\n", my_temp/10.0);
			UART_Send(LPC_UART3, (uint8_t *)display_acc0 , strlen(display_acc0), BLOCKING);
			break;
		}
		counter_for_send_data = 0;
	} else {
		counter_for_send_data ++;
	}
}



void battery_Consumption(){
	if(counter_for_battery_consumption == 9) {                            //every 10 cycle,
		counter_for_battery_consumption = 0;
		if(relative_battery_level> -14){
			relative_battery_level -= 1;                           //batter level decrease by 2
		} else {
			setMode(SLEEPING);
		}
	} else {
		counter_for_battery_consumption ++;
	}
}

void battery_Recovery(){

	if (sw4==0){
		if(is_sleeping_to_exploring_charging == 1){
			is_sleeping_to_exploring_charging = 0;
			return;
		} else if (mode == SLEEPING){
			is_sleeping_to_exploring_charging = 1;
		}
		isCharging = 1;
		if(relative_battery_level <0){                           //relative batt =  == full
				relative_battery_level += 1;					 //batter level increase by 1
				isCharging = 0;
				GPIO_ClearValue(1, 1<<31);
		}
	}else{
		isCharging = 0;
	}
}


void graphics_Display(int condition){
	int display_acc[40];
	int display_acc0[40];
	int display_acc1[40];
	int display_acc2[40];
	int display_acc3[40];
	int display_acc4[40];
	switch (mode) {
		case START:
			sprintf(display_acc0, "LanderNUS.");
			sprintf(display_acc1,"By Qin&YC.");
			oled_putString(0, 0, display_acc0, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
			oled_putString(0, 9, display_acc1, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
			break;
		case ORBIT:
			sprintf(display_acc0, "Orbiting Mode.");
			sprintf(display_acc1, "Press SW3 ");
			sprintf(display_acc2, "to Land.");
			oled_putString(0, 0, display_acc0, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
			oled_putString(0, 9, display_acc1, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
			oled_putString(0, 18, display_acc2, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
			led7seg_setChar(' ', FALSE);


			if(joydisp_flag==1){
				sprintf(display_acc3, "Light Th: %d\r\n",LIGHT_LOTHRESHOLD);
				oled_putString(0, 46, display_acc3, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
			}else if (joydisp_flag==2){
				sprintf(display_acc4, "Tilt Th: %d\r\n",TILT_THRESHOLD);
				oled_putString(0, 46, display_acc4, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
			}


			break;
		case ORBIT_TO_LANDING: // BLINK_BLUE
			sprintf(display_acc0, "ENTERING");
			sprintf(display_acc1, "LANDING MODE ");
			oled_putString(0, 0, display_acc0, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
			oled_putString(0, 9, display_acc1, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
		    break;
		case LANDING: // ALTERNATE_LED

			acc_read(&x_coord, &y_coord, &z_coord);
			x_coord+=x_off;
			y_coord+=y_off;
			z_coord+=z_off;

			if (condition == 1) {               //not safe landing angle

				UART_Send(LPC_UART3, (uint8_t *)TILT_THRESHOLD_TXT , strlen(TILT_THRESHOLD_TXT), BLOCKING);

				if (TILT_FLAG) {

					oled_clearScreen(OLED_COLOR_WHITE);
					TILT_FLAG = 0;

					//sendUARTMessage(5);

				}

				sprintf(display_acc0, "Poor");

				sprintf(display_acc1, "Landing");

				sprintf(display_acc2, "Altitude");

				oled_putString(0, 0, display_acc0, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
				oled_putString(0, 9, display_acc1, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
				oled_putString(0, 18, display_acc2, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

				TILT_UART_FLAG = 1;

			} else {                       //safe landing angle

				sprintf(display_acc0, "Acc-X: %2.2f g\r", x_coord/64.0);

				sprintf(display_acc1, "Acc-Y: %2.2f g\r", y_coord/64.0);

				sprintf(display_acc2, "Acc-Z: %2.2f g\r\n", z_coord/64.0);

				oled_putString(0, 0, display_acc0, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
				oled_putString(0, 9, display_acc1, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
				oled_putString(0, 18, display_acc2, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

				TILT_FLAG = 1;

			}
			graphic_simulate();
		break;
		case EXPLORING:
			draw_temperature();
			if (condition == 1){
				sprintf(display_acc3, "Already full power!");
				oled_putString(0, 27, display_acc3, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
			}
			sprintf(display_acc0, "EXPLORING");
			oled_putString(0, 0, display_acc0, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
			//temperature

			my_temp=temp_read();
			sprintf(display_acc, "Temp: %2.2f deg \n", my_temp/10.0);
			oled_putString(0, 9, display_acc, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
			//battery
			pca9532_setLeds(0xFFFF>>abs(relative_battery_level), 0xFFFF);
			float battery = 100;
			sprintf(display_acc1, "Battery: %.2f ", 100 - 6.25 *abs(relative_battery_level));
			oled_putString(0, 18, display_acc1, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
			break;
		case SLEEPING:
			sprintf(display_acc1, "SLEEPING");
			oled_putString(0, 0, display_acc1, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

			break;
	}
}

void mode_exploring(){

	if(force_sleep_flag == 1) {
		//force sleep
		setMode(SLEEPING);
		mode_sleeping();
	} else {
		if (relative_battery_level == (BATTERY_THRESHOLD-100)/12.5*2) {
				setMode (SLEEPING);
				//Send warning message to XBee
		}else {
			//Display a E
			led7seg_setChar('3', FALSE);
			//Display information
			graphics_Display(0);
			//UART
			if(sendflag_exploring==1){
				msg="EXPLORING Mode.\r\n";
				UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
				sendflag_exploring=0;
			}
		}
	}

}

void start_animation(){

	int display_acc0[40];
	sprintf(display_acc0, "LanderNUS.");
	if(counter_for_animation >= 10){
		period = 500;
	    LPC_TIM1->MR0  = mstous(period); //        timer interrupt every 500ms
	    LPC_TIM1->TCR  = (1<<SBIT_CNTEN); // Start timer by setting the Counter Enable

	    NVIC_ClearPendingIRQ(TIMER1_IRQn);
		setMode(START);
	}
	if(j == frequency){
		oled_circle(48,30,radius-increament,OLED_COLOR_WHITE);
		oled_circle(48,30,radius+1-increament,OLED_COLOR_WHITE);


		oled_circle(48,30,radius+increament-increament,OLED_COLOR_WHITE);
		oled_circle(48,30,radius+1+increament-increament,OLED_COLOR_WHITE);


		oled_circle(48,30,radius+increament*2-increament,OLED_COLOR_WHITE);
		oled_circle(48,30,radius+1+increament*2-increament,OLED_COLOR_WHITE);


		oled_circle(48,30,radius,OLED_COLOR_BLACK);
		oled_circle(48,30,radius+1,OLED_COLOR_BLACK);


		oled_circle(48,30,radius+increament,OLED_COLOR_BLACK);
		oled_circle(48,30,radius+1+increament,OLED_COLOR_BLACK);


		oled_circle(48,30,radius+increament*2,OLED_COLOR_BLACK);
		oled_circle(48,30,radius+1+increament*2,OLED_COLOR_BLACK);



		radius += increament;
		counter_for_animation++;
		if(frequency >0){
			frequency--;
		}
		if(frequency <= 3){

			oled_putString(24, 30, display_acc0, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

		}
		j =0;
	} else {
		j = j+ 0.5;
	}
}


void mode_sleeping(){
	GPIO_ClearValue(0, 1<<26);
	GPIO_ClearValue(2, 1<<0);
	led7seg_setChar(' ', FALSE);


	if(force_sleep_flag == 1) {
		graphics_Display(0);

		if(sendflag_sleeping==1){
			msg="Sleeping Mode. \r\n";
			UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
			sendflag_sleeping=0;
		}
	} else {
		if (relative_battery_level > (BATTERY_THRESHOLD-100)/12.5*2){
			sendflag_exploring = 1;
			sendflag_sleeping = 1;
			setMode (EXPLORING);
		} else {
			graphics_Display(0);
			if(sendflag_sleeping==1){
				msg="Sleeping Mode. \r\n";
				UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
				sendflag_sleeping=0;
			}
		}
	}
}


unsigned int getPrescalarForUs(uint8_t timerPclkBit) {
	unsigned int pclk,prescalarForUs;
    pclk = (LPC_SC->PCLKSEL0 >> timerPclkBit) & 0x03;  // get the pclk info for required timer

    switch (pclk) { // Decode the bits to determine the pclk
    case 0x00:
        pclk = SystemCoreClock/4;
        break;
    case 0x01:
        pclk = SystemCoreClock;
        break;
    case 0x02:
        pclk = SystemCoreClock/2;
        break;
    case 0x03:
        pclk = SystemCoreClock/8;
        break;
    default:
        pclk = SystemCoreClock/4;
        break;
    }
    return pclk/1000000 - 1; // Prescalar for 1us (1000000Counts/sec)
}

void init(){
	SysTick_Config(SystemCoreClock/1000);
	init_ssp();
	init_i2c();
	init_gpio();
	oled_init();
	led7seg_init();
	init_uart();
	pinsel_uart3();
	light_init();             //[fake] initialization
	init_Interrupt();
	joystick_init();
	oled_init();
	temp_init(getTicks);
	oled_clearScreen(OLED_COLOR_WHITE);
	acc_init();
	pca9532_init();
	//UART_Receive_Int_Init();
	uart3_interrupt_init();
	light_enable();           //[true] enable the sensor
	light_clearIrqStatus();
	light_setLoThreshold(LIGHT_LOTHRESHOLD);
	led7seg_setChar(' ', FALSE);
	GPIO_ClearValue(1,1<<31);

}

void countdown(void) {
	//Count down from 5 to 0 every 1 sec
	uint32_t countdown2=0;
	int count = 5;
	int Numbers[] = {0x24,0x7D,0xE0,0x70,0x39,0x32};
	uint32_t countdown1=getTicks();
	while (count>=0) {
		countdown2=getTicks();
		if (countdown2-countdown1>=1000){
			countdown1=countdown2;
			count--;
		}
		led7seg_setChar(Numbers[count], TRUE);
	}
	caliberate();
	setMode(LANDING);
	mode_landing();

}

void draw_temperature(){
	if(x0_t >= 92){
		x0_t = 0;
		oled_clearScreen(OLED_COLOR_WHITE);
	}

	y1_t = 40 - (int)(temp_read()/10.0-initial_temperature)/0.5  ;
	x1_t = x0_t + 3;
	if(y1_t >= 70){
		y1_t = 70;
		//UART


	}
	if(y1_t <= 30){
		y1_t = 30;
		//UART

	}
	oled_line(x0_t,y0_t,x1_t,y1_t,OLED_COLOR_BLACK);
	x0_t = x1_t;
	y0_t = y1_t;
}

void mode_orbit(){
	//Change to "orbiting to Land"
	if(flag_sw3 == 1) {
		setMode(ORBIT_TO_LANDING);
		flag_sw3 = 0;
		timer_past = -1;
	}
	else {
		graphics_Display(0);
		//UART   “Start: Orbiting, waiting for Landing”
		if(sendflag_orbit==1){
			sendflag_orbit=0;
			msg="Start: Orbiting, waiting for Landing.\r\n";
			UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
		}
	}
}

void mode_Toggle_orbit_to_landing(){
	//Display information on OLED
	graphics_Display(0);
	//RGB blink blue by TIMER1
	countdown();//count down from 5 to 0 every 1 second

}

void mode_landing(){
	int i = 0;
	if(is_safe_tilt_angle() ) {
		//not safe
		graphics_Display(1);

	} else {


		if (light_read() < LIGHT_LOTHRESHOLD) {

			if(sendflag_landing_success==1){                                      //send once to uart
				msg="LANDING SUCCESSFUL.\r\n";
				UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
				sendflag_landing_success=0;
			}
			initial_temperature = temp_read() /10.0;
		   NVIC_ClearPendingIRQ(EINT3_IRQn);
		   NVIC_EnableIRQ(EINT3_IRQn);
		   NVIC_ClearPendingIRQ(EINT3_IRQn);
		   LPC_GPIOINT->IO2IntClr = 1<<6;
		   NVIC_ClearPendingIRQ(UART3_IRQn);
			UART_IntConfig(LPC_UART3,UART_INTCFG_RBR,ENABLE);
		    NVIC_SetPriority(UART3_IRQn, 0x08);
		    NVIC_EnableIRQ(UART3_IRQn);
			setMode(EXPLORING);
			mode_exploring();

		} else {

			led7seg_setChar(' ', FALSE);
			//Display a L
			led7seg_setChar('7', FALSE);

			//ALTERNATE_LED by timer1

			//Display information
			graphics_Display(0);
			//UART
			if(sendflag_landing==1){
				msg="LANDING Mode.\r\n";
				UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
				sendflag_landing=0;
			}

		}
	}
}

void menu(){
	int k = 0;
	int data1 = 0;
	msg="Operations:\r\n";
	UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);

	msg="1. Start Moon Exploring System. \r\n";
	UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);

	msg="2. Change the authorized code. \r\n";
	UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
	do {
		UART_Receive (LPC_UART3, &data1, 1, BLOCKING);

		msg="\r\n";
		UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
		if(data1!= '\r' && data1!= '\n'){
			switch (data1){
				case '1':
					setMode(ORBIT);
					return;
				case '2':
					reset_password();
					break;
				default:
					msg="Invalid command! \r\n";
					UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
			}
		}
	} while (data1!='\r');


}



void UART3_IRQHandler(void){
	UART3_StdIntHandler();
}

void UART_IntReceive(void)

{

    /* Read the received data */

    if(UART_Receive(UART_PORT, &rev_buf[rev_cnt], 1, NONE_BLOCKING) == 1) {

        if(rev_buf[rev_cnt] == '\r'){

            isReceived = 1;

        }

        rev_cnt++;

        if(rev_cnt == 255) rev_cnt = 0;

    }
    if(rev_cnt == 1 && rev_buf[0]== 'S'&& force_sleep_flag == 0){
    	force_sleep_flag = 1;
    	setMode(SLEEPING);
    	msg="\r\n";
		UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
		msg="Forced Sleeping!\r\n";
		UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
		oled_clearScreen(OLED_COLOR_WHITE);
    	mode_sleeping();
    	rev_cnt = 0;
    } else if(rev_cnt == 1 && rev_buf[0]== 'W' && force_sleep_flag == 1){
    	force_sleep_flag = 0;

		oled_clearScreen(OLED_COLOR_WHITE);
		setMode(SLEEPING);
		mode_sleeping();

		msg="\r\n";
		UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
		msg="Quit Forced Sleeping\r\n";
		UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
		rev_cnt = 0;
    }
    rev_cnt = 0;
}

void uart3_interrupt_init(){
	UART_SetupCbs(LPC_UART3,0, (void*)UART_IntReceive);
}

void reset_password(){
	int k;
	int data2= 0;
	uint32_t len=0;
	uint8_t line[64];
	msg="Please enter 4-digit authorized code, end with ENTER\r\n";
	UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
	do {
		UART_Receive (LPC_UART3, &data2, 1, BLOCKING);
		if(data2!= '\r'){
			len++;
			password[len-1]=data2;
		}
	} while (len <63 &&(data2!='\r'));

	UART_Receive (LPC_UART3, &data2, 1, BLOCKING);
	msg="Code changed successfully \r\n";
	UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
	menu();

}

void password_confirmation(){
	int k;
	int data0 = 0;
	uint32_t len=0;
	uint8_t line[64];
	do {
		UART_Receive (LPC_UART3, &data0, 1, BLOCKING);
		if(data0!= '\r' && data0!= '\n'){
			len++;
			line[len-1]=data0;
		}
	} while (len <63 &&(data0!='\r'));

	for(k = 0; k <len; k++){
		if(line[k]!= password[k]){
			msg="Wrong password!\r\n";
			UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
			msg="Please enter again:\r\n";
			UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
			return;
		}
	}
	menu();
}

//return if tilt angle is safe,
//if safe, return 0
//if not safe, return 1
int is_safe_tilt_angle(){


	float x_angle,y_angle,z_angle = 0.0;
	x_angle=atan((float)x_coord/sqrt( (float) y_coord* (float)y_coord+ (float)z_coord* (float)z_coord));
	y_angle=atan((float)y_coord/sqrt((float)x_coord*(float)x_coord+(float)z_coord* (float)z_coord));
	z_angle=atan(sqrt((float)y_coord*(float)y_coord+(float)x_coord*(float)x_coord)/(float)z_coord);


	if(x_angle > TILT_THRESHOLD * PI /180.0  || x_angle < (-1)*TILT_THRESHOLD * PI /180.0) {
		return 1;
	}
	if(y_angle > TILT_THRESHOLD * PI /180.0  || y_angle < (-1)*TILT_THRESHOLD * PI /180.0) {
		return 1;
	}
	if(z_angle > TILT_THRESHOLD * PI /180.0  || z_angle < (-1)*TILT_THRESHOLD * PI /180.0) {
		return 1;
	}
	return 0;
}


void graphic_simulate(){


	oled_rect(24,30,70,60,OLED_COLOR_BLACK);

	oled_fillRect(25,31,69,59,OLED_COLOR_WHITE);
	oled_fillRect(42-x_coord/5.0,43+y_coord/6.3,53-x_coord/5.0,50+y_coord/6.3,OLED_COLOR_BLACK);

}

void caliberate(){
	int i = 0;
	acc_read(&x_coord,&y_coord,&z_coord);
	for(i= 0; i <= 1; i++){
		x_off+=(0-x_coord);
		y_off+=(0-y_coord);
		z_off+=(64-z_coord);
	}
	x_off /= 2;
	y_off /= 2;
	z_off /= 2;

}

void welcome_message(){

	msg="Welcome to use LanderNUS.\r\n";
	UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
	msg="Please enter the authorized code to operate.\r\n";
	UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
}

int main (void) {
	init();
	while (1){
		if (Changemodeflag == 1) {
			oled_clearScreen(OLED_COLOR_WHITE);
			Changemodeflag = 0;
		}
		if(mode == START){
			if(first_time_start = 1){
				welcome_message();
				graphics_Display(0);
				first_time_start = 0;
			}
			password_confirmation();
		} else if(mode == ORBIT) {
			mode_orbit();
		} else if(mode == ORBIT_TO_LANDING) {
			mode_Toggle_orbit_to_landing();
			//setMode(LANDING);

		} else if (mode == LANDING){
			mode_landing();
		} else if (mode ==EXPLORING){
			mode_exploring();


		} else if (mode==SLEEPING){
			mode_sleeping();
		}
	}
	return 0;
}
