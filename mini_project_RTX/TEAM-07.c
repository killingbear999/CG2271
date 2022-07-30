/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/

/* Team 7: 
Wang Zihao, 
Zhuang Mengjin, 
Wu Nan */
#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
#define PTB0_Pin 0	 // TPM1_CH0 // right-forward // purple
#define PTB1_Pin 1	 // TPM1_CM1 // right-reverse // blue
#define PTB2_Pin 2	 // TPM2_CH0 // left-forward // yellow
#define PTB3_Pin 3	 // TPM2_CH1 // left-reverse // green
#define PTE29_Pin 29 // audio
#define GREEN_LED1 0 // LED connect to PORTD Pin 0
#define GREEN_LED2 1 // LED connect to PORTD Pin 1
#define GREEN_LED3 2 // LED connect to PORTD Pin 2
#define GREEN_LED4 3 // LED connect to PORTD Pin 3
#define GREEN_LED5 4 // LED connect to PORTD Pin 4
#define GREEN_LED6 5 // LED connect to PORTD Pin 5
#define GREEN_LED7 6 // LED connect to PORTD Pin 6
#define GREEN_LED8 7 // LED connect to PORTD Pin 7
#define RED_LED1 8	 // LED connected to PORTB 8
#define RED_LED2 9	 // LED connected to PORTB 9
#define RED_LED3 10	 // LED connected to PORTB 10
#define RED_LED4 11	 // LED connected to PORTB 11
#define RED_LED5 2	 //LED connected to PORTE 2
#define RED_LED6 3	 //LED connected to PORTE 3
#define RED_LED7 4	 //LED connected to PORTE 4
#define RED_LED8 5	 //LED connected to PORTE 5
#define MASK(x) (1 << (x))
#define MUSICAL_NOTE_CNT1 117
#define MUSICAL_NOTE_CNT2 37
#define MUSICAL_NOTE_CNT3 43
#define MUSICAL_NOTE_CNT4 8
#define MUSICAL_NOTE_CNT5 8
#define FREQ_2_MOD(x) (18750 / x)
#define MSG_COUNT 1
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFTWARD 3
#define RIGHTWARD 4
#define COMPLETE 5
#define CONNECT 6
#define UART_TX_PORTE22 22
#define UART_INT_PRIO 128
#define BAUD_RATE 9600
#define UART_RX_PORTE23 23
#define MOVE_MASK(x) (x & 0x14)
#define BIT0_MASK(x) (x & 0x01)
#define STOP_FORWARD 0
#define BACKWARD_LEFTWARD 2
#define RIGHTWARD_COMPLETE 3

volatile int status = 0;
volatile int isConnected = 0;

int musical_notes1[MUSICAL_NOTE_CNT1] = {196, 262, 262, 330, 440, 330, 392, 392, 392, 440, 392, 330, 392, 330, 294, 294, 220, 294, 294, 349, 494, 494, 440, 392, 349, 349, 349, 330, 220, 247, 247, 262, 294, 294, 294, 294, 294,
										 196, 262, 262, 330, 440, 330, 392, 392, 392, 440, 392, 330, 392, 330, 294, 294, 220, 294, 294, 349, 494, 494, 440, 392, 349, 349, 349, 330, 247, 247, 294, 294, 262, 262, 262, 262, 262,
										 440, 440, 440, 392, 349, 392, 440, 392, 392, 294, 330, 370, 294, 392, 392, 392, 440, 440, 392, 392, 349, 349, 349, 294, 294, 494, 440, 392, 440, 392, 349, 349, 392, 440, 330, 330, 330, 294, 262, 262, 262, 262, 262};


osMessageQueueId_t redLEDMsg, greenLEDMsg, audioMsg, motorMsg, completeMsg, connectMsg, greenLEDFlashMsg;
osThreadId_t redLED_Id, greenLED_Id, audio_Id, motor_Id, control_Id, complete_Id, connect_Id, greenLEDFlash_Id;

typedef struct {
	uint8_t cmd;	  // 0x00 = disconnected; 0x01 = connected; 0x02 = complete; 0x03 = just connect
	uint8_t data;	  // 0x01 = moving; 0x02 = stop; 0x03 = flash
	uint8_t movement; // 0x01 = forward; 0x02 = backward; 0x03 = leftward; 0x04 = rightward; 0x05 = stop
} myDataPkt;

typedef enum motor_status {
	stop = STOP,
	forward = FORWARD,
	backward = BACKWARD,
	leftward = LEFTWARD,
	rightward = RIGHTWARD,
	complete = COMPLETE,
	connect = CONNECT,
} led_status_t;

void initGPIO(uint32_t baud_rate)
{
	// Enable Clock to PORTB and PORTE
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTE_MASK));
	SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK);

	// Red LED
	// Configure MUX settings
	PORTB->PCR[RED_LED1] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED1] |= PORT_PCR_MUX(1);
	PORTB->PCR[RED_LED2] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED2] |= PORT_PCR_MUX(1);
	PORTB->PCR[RED_LED3] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED3] |= PORT_PCR_MUX(1);
	PORTB->PCR[RED_LED4] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED4] |= PORT_PCR_MUX(1);
	PORTE->PCR[RED_LED5] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[RED_LED5] |= PORT_PCR_MUX(1);
	PORTE->PCR[RED_LED6] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[RED_LED6] |= PORT_PCR_MUX(1);
	PORTE->PCR[RED_LED7] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[RED_LED7] |= PORT_PCR_MUX(1);
	PORTE->PCR[RED_LED8] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[RED_LED8] |= PORT_PCR_MUX(1);

	// Green LED
	PORTD->PCR[GREEN_LED1] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[GREEN_LED1] |= PORT_PCR_MUX(1);
	PORTD->PCR[GREEN_LED2] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[GREEN_LED2] |= PORT_PCR_MUX(1);
	PORTD->PCR[GREEN_LED3] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[GREEN_LED3] |= PORT_PCR_MUX(1);
	PORTD->PCR[GREEN_LED4] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[GREEN_LED4] |= PORT_PCR_MUX(1);
	PORTD->PCR[GREEN_LED5] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[GREEN_LED5] |= PORT_PCR_MUX(1);
	PORTD->PCR[GREEN_LED6] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[GREEN_LED6] |= PORT_PCR_MUX(1);
	PORTD->PCR[GREEN_LED7] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[GREEN_LED7] |= PORT_PCR_MUX(1);
	PORTD->PCR[GREEN_LED8] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[GREEN_LED8] |= PORT_PCR_MUX(1);

	// Set DDR
	PTB->PDDR |= (MASK(RED_LED1) | MASK(RED_LED2) | MASK(RED_LED3) | MASK(RED_LED4));
	PTE->PDDR |= (MASK(RED_LED5) | MASK(RED_LED6) | MASK(RED_LED7) | MASK(RED_LED8));
	PTD->PDDR |= (MASK(GREEN_LED1) | MASK(GREEN_LED2) | MASK(GREEN_LED3) | MASK(GREEN_LED4) | MASK(GREEN_LED5) | MASK(GREEN_LED6) | MASK(GREEN_LED7) | MASK(GREEN_LED8));

	// Motor
	// Configure Mode 3 for the PWM pin operation
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);

	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);

	PORTB->PCR[PTB2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB2_Pin] |= PORT_PCR_MUX(3);

	PORTB->PCR[PTB3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB3_Pin] |= PORT_PCR_MUX(3);

	// Audio
	// Configure Mode 3 for the PWM pin operation
	PORTE->PCR[PTE29_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE29_Pin] |= PORT_PCR_MUX(3);

	// Enable Clock Gating for Timer 1 and Timer 2
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;

	// Select Clock for TPM module
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

	// Set Modulo Value
	TPM0->MOD = 3750;
	TPM1->MOD = 10000;
	TPM2->MOD = 10000;

	// Edge-aligned PWM
	// Update SnC register: CMOD = 01, PS = 111(128)
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);

	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);

	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK);

	//Enable PWM on TPM0 channel 2
	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	// Enable PWM on TPM1 channel 0
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	// Enable PWM on TPM1 channel 1
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	// Enable PWM on TPM2 channel 0
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	// Enable PWM on TPM2 channel 1
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	// UART2
	uint32_t divisor, bus_clock;

	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;

	PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);

	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);

	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

	bus_clock = (DEFAULT_SYSTEM_CLOCK) / 2;
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);

	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;

	UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

	NVIC_SetPriority(UART2_IRQn, 128);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);

	// enable transmit and receive interrupt
	UART2->C2 |= UART_C2_RIE_MASK | UART_C2_TE_MASK;
}

static void delay(volatile uint32_t nof)
{
	while (nof != 0)
	{
		__asm("NOP");
		nof--;
	}
}

void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	uint8_t rx_data = 0;

	if (UART2->S1 & UART_S1_RDRF_MASK) {
		rx_data = UART2->D;
		if (rx_data == 0b00000001) {
			status = forward;
		} else if (rx_data == 0b00000010) {
			status = backward;
		} else if (rx_data == 0b00000011) {
			status = leftward;
		} else if (rx_data == 0b00000100) {
			status = rightward;
		} else if (rx_data == 0b00000101) {
			status = complete;
		} else if (rx_data == 0b00000000) {
			status = stop;
		} else if (rx_data == 0b00000110) {
			status = connect;
		}
	}
}

void red_led_moving()
{
	PTB->PDOR |= MASK(RED_LED1);
	PTB->PDOR |= MASK(RED_LED2);
	PTB->PDOR |= MASK(RED_LED3);
	PTB->PDOR |= MASK(RED_LED4);
	PTE->PDOR |= MASK(RED_LED5);
	PTE->PDOR |= MASK(RED_LED6);
	PTE->PDOR |= MASK(RED_LED7);
	PTE->PDOR |= MASK(RED_LED8);
	if (status == 0)
		return;
	osDelay(500);
	PTB->PDOR &= ~MASK(RED_LED1);
	PTB->PDOR &= ~MASK(RED_LED2);
	PTB->PDOR &= ~MASK(RED_LED3);
	PTB->PDOR &= ~MASK(RED_LED4);
	PTE->PDOR &= ~MASK(RED_LED5);
	PTE->PDOR &= ~MASK(RED_LED6);
	PTE->PDOR &= ~MASK(RED_LED7);
	PTE->PDOR &= ~MASK(RED_LED8);
	if (status == 0)
		return;
	osDelay(500);
}

void red_led_stationary()
{
	PTB->PDOR |= MASK(RED_LED1);
	PTB->PDOR |= MASK(RED_LED2);
	PTB->PDOR |= MASK(RED_LED3);
	PTB->PDOR |= MASK(RED_LED4);
	PTE->PDOR |= MASK(RED_LED5);
	PTE->PDOR |= MASK(RED_LED6);
	PTE->PDOR |= MASK(RED_LED7);
	PTE->PDOR |= MASK(RED_LED8);
	osDelay(250);
	PTB->PDOR &= ~MASK(RED_LED1);
	PTB->PDOR &= ~MASK(RED_LED2);
	PTB->PDOR &= ~MASK(RED_LED3);
	PTB->PDOR &= ~MASK(RED_LED4);
	PTE->PDOR &= ~MASK(RED_LED5);
	PTE->PDOR &= ~MASK(RED_LED6);
	PTE->PDOR &= ~MASK(RED_LED7);
	PTE->PDOR &= ~MASK(RED_LED8);
	osDelay(250);
}

void red_led_thread() {
	myDataPkt myRxData;
	for (;;) {
		osMessageQueueGet(redLEDMsg, &myRxData, NULL, osWaitForever);
		if (myRxData.data == 0x01) {
			red_led_moving();
		} else if (myRxData.data == 0x02) {
			red_led_stationary();
		}
	}
}

void green_led_stationary()
{
	PTD->PDOR |= MASK(GREEN_LED1);
	PTD->PDOR |= MASK(GREEN_LED2);
	PTD->PDOR |= MASK(GREEN_LED3);
	PTD->PDOR |= MASK(GREEN_LED4);
	PTD->PDOR |= MASK(GREEN_LED5);
	PTD->PDOR |= MASK(GREEN_LED6);
	PTD->PDOR |= MASK(GREEN_LED7);
	PTD->PDOR |= MASK(GREEN_LED8);
}

void green_led_flashes()
{
	for (int i = 0; i < 2; i++)
	{	PTD->PDOR &= ~MASK(GREEN_LED1);
		PTD->PDOR &= ~MASK(GREEN_LED2);
		PTD->PDOR &= ~MASK(GREEN_LED3);
		PTD->PDOR &= ~MASK(GREEN_LED4);
		PTD->PDOR &= ~MASK(GREEN_LED5);
		PTD->PDOR &= ~MASK(GREEN_LED6);
		PTD->PDOR &= ~MASK(GREEN_LED7);
		PTD->PDOR &= ~MASK(GREEN_LED8);
		osDelay(600);
		PTD->PDOR |= MASK(GREEN_LED1);
		PTD->PDOR |= MASK(GREEN_LED2);
		PTD->PDOR |= MASK(GREEN_LED3);
		PTD->PDOR |= MASK(GREEN_LED4);
		PTD->PDOR |= MASK(GREEN_LED5);
		PTD->PDOR |= MASK(GREEN_LED6);
		PTD->PDOR |= MASK(GREEN_LED7);
		PTD->PDOR |= MASK(GREEN_LED8);
		osDelay(600);

	}
}

void green_led_moving()
{

	//GREEN LED 1 light up
	PTD->PDOR |= MASK(GREEN_LED1);
	PTD->PDOR &= ~MASK(GREEN_LED2);
	PTD->PDOR &= ~MASK(GREEN_LED3);
	PTD->PDOR &= ~MASK(GREEN_LED4);
	PTD->PDOR &= ~MASK(GREEN_LED5);
	PTD->PDOR &= ~MASK(GREEN_LED6);
	PTD->PDOR &= ~MASK(GREEN_LED7);
	PTD->PDOR &= ~MASK(GREEN_LED8);
	if (status == 0)
		return;
	osDelay(250);
	//GREEN LED 2 light up
	PTD->PDOR &= ~MASK(GREEN_LED1);
	PTD->PDOR |= MASK(GREEN_LED2);
	PTD->PDOR &= ~MASK(GREEN_LED3);
	PTD->PDOR &= ~MASK(GREEN_LED4);
	PTD->PDOR &= ~MASK(GREEN_LED5);
	PTD->PDOR &= ~MASK(GREEN_LED6);
	PTD->PDOR &= ~MASK(GREEN_LED7);
	PTD->PDOR &= ~MASK(GREEN_LED8);
	if (status == 0)
		return;
	osDelay(250);
	//GREEN LED 3 light up
	PTD->PDOR &= ~MASK(GREEN_LED1);
	PTD->PDOR &= ~MASK(GREEN_LED2);
	PTD->PDOR |= MASK(GREEN_LED3);
	PTD->PDOR &= ~MASK(GREEN_LED4);
	PTD->PDOR &= ~MASK(GREEN_LED5);
	PTD->PDOR &= ~MASK(GREEN_LED6);
	PTD->PDOR &= ~MASK(GREEN_LED7);
	PTD->PDOR &= ~MASK(GREEN_LED8);
	if (status == 0)
		return;
	osDelay(250);
	//GREEN LED 4 light up
	PTD->PDOR &= ~MASK(GREEN_LED1);
	PTD->PDOR &= ~MASK(GREEN_LED2);
	PTD->PDOR &= ~MASK(GREEN_LED3);
	PTD->PDOR |= MASK(GREEN_LED4);
	PTD->PDOR &= ~MASK(GREEN_LED5);
	PTD->PDOR &= ~MASK(GREEN_LED6);
	PTD->PDOR &= ~MASK(GREEN_LED7);
	PTD->PDOR &= ~MASK(GREEN_LED8);
	if (status == 0)
		return;
	osDelay(250);
	//GREEN LED 5 light up
	PTD->PDOR &= ~MASK(GREEN_LED1);
	PTD->PDOR &= ~MASK(GREEN_LED2);
	PTD->PDOR &= ~MASK(GREEN_LED3);
	PTD->PDOR &= ~MASK(GREEN_LED4);
	PTD->PDOR |= MASK(GREEN_LED5);
	PTD->PDOR &= ~MASK(GREEN_LED6);
	PTD->PDOR &= ~MASK(GREEN_LED7);
	PTD->PDOR &= ~MASK(GREEN_LED8);
	if (status == 0)
		return;
	osDelay(250);
	//GREEN LED 6 light up
	PTD->PDOR &= ~MASK(GREEN_LED1);
	PTD->PDOR &= ~MASK(GREEN_LED2);
	PTD->PDOR &= ~MASK(GREEN_LED3);
	PTD->PDOR &= ~MASK(GREEN_LED4);
	PTD->PDOR &= ~MASK(GREEN_LED5);
	PTD->PDOR |= MASK(GREEN_LED6);
	PTD->PDOR &= ~MASK(GREEN_LED7);
	PTD->PDOR &= ~MASK(GREEN_LED8);
	if (status == 0)
		return;
	osDelay(250);
	//GREEN LED 7 light up
	PTD->PDOR &= ~MASK(GREEN_LED1);
	PTD->PDOR &= ~MASK(GREEN_LED2);
	PTD->PDOR &= ~MASK(GREEN_LED3);
	PTD->PDOR &= ~MASK(GREEN_LED4);
	PTD->PDOR &= ~MASK(GREEN_LED5);
	PTD->PDOR &= ~MASK(GREEN_LED6);
	PTD->PDOR |= MASK(GREEN_LED7);
	PTD->PDOR &= ~MASK(GREEN_LED8);
	if (status == 0)
		return;
	osDelay(250);
	//GREEN LED 8 light up
	PTD->PDOR &= ~MASK(GREEN_LED1);
	PTD->PDOR &= ~MASK(GREEN_LED2);
	PTD->PDOR &= ~MASK(GREEN_LED3);
	PTD->PDOR &= ~MASK(GREEN_LED4);
	PTD->PDOR &= ~MASK(GREEN_LED5);
	PTD->PDOR &= ~MASK(GREEN_LED6);
	PTD->PDOR &= ~MASK(GREEN_LED7);
	PTD->PDOR |= MASK(GREEN_LED8);
	if (status == 0)
		return;
	osDelay(250);
}

void green_led_thread() {
	myDataPkt myRxData;
	for (;;) {
		osMessageQueueGet(greenLEDMsg, &myRxData, NULL, osWaitForever);
		if (myRxData.data == 0x01) {
			green_led_moving();
		} else if (myRxData.data == 0x02) {
			green_led_stationary();
		}
	}
}

void audio()
{
	for (int i = 0; i < MUSICAL_NOTE_CNT1; i++)
	{
		if (isConnected == 1) {
		  TPM0->MOD = FREQ_2_MOD(musical_notes1[i]);
		  TPM0_C2V = (FREQ_2_MOD(musical_notes1[i])) / 2;
		  osDelay(500);
		} else {
			TPM0->MOD = 0;
		}
	}
}

void audio_complete()
{
	int musical_notes4[MUSICAL_NOTE_CNT4] = {494, 440, 392, 349, 330, 294, 262, 0};
	for (int i = 0; i < MUSICAL_NOTE_CNT4; i++)
	{
		TPM0->MOD = FREQ_2_MOD(musical_notes4[i]);
		TPM0_C2V = (FREQ_2_MOD(musical_notes4[i])) / 2;
		osDelay(500);
	}
}

void autio_connect()
{
	int musical_notes5[MUSICAL_NOTE_CNT5] = {262, 294, 330, 349, 392, 440, 494, 0};
	for (int i = 0; i < MUSICAL_NOTE_CNT5; i++)
	{
		TPM0->MOD = FREQ_2_MOD(musical_notes5[i]);
		TPM0_C2V = (FREQ_2_MOD(musical_notes5[i])) / 2;
		osDelay(500);
	}
}

void motor_forward()
{
	TPM1_C0V = 8000;
	TPM1_C1V = 0;
	TPM2_C0V = 8000;
	TPM2_C1V = 0;
}

void motor_reverse()
{
	TPM1_C0V = 0;
	TPM1_C1V = 8000;
	TPM2_C0V = 0;
	TPM2_C1V = 8000
	;
}

void motor_leftward()
{
	TPM1_C0V = 8000;
	TPM1_C1V = 0;
	TPM2_C0V = 1000;
	TPM2_C1V = 0;
}

void motor_rightward()
{
	TPM1_C0V = 1000;
	TPM1_C1V = 0;
	TPM2_C0V = 8000;
	TPM2_C1V = 0;
}

void motor_stop()
{
	TPM1_C0V = 0;
	TPM1_C1V = 0;
	TPM2_C0V = 0;
	TPM2_C1V = 0;
}

void audio_thread(void *argument) {
	myDataPkt myRxData;
	for (;;) {
		osMessageQueueGet(audioMsg, &myRxData, NULL, osWaitForever);
		if (myRxData.cmd == 0x01 && isConnected == 1) {
			audio();
		}
	}
}

void motor_thread(void *argument) {
	myDataPkt myRxData;
	for (;;) {
		osMessageQueueGet(motorMsg, &myRxData, NULL, osWaitForever);
		if (myRxData.cmd == 0x01) {
			if (myRxData.movement == 0x01) {
				motor_forward();
			} else if (myRxData.movement == 0x02) {
				motor_reverse();
			} else if (myRxData.movement == 0x03) {
				motor_leftward();
			} else if (myRxData.movement == 0x04) {
				motor_rightward();
			} else if (myRxData.movement == 0x05) {
				motor_stop();
			}
		}
	}
}

void complete_thread(void *argument) {
	myDataPkt myRxData;
	osMessageQueueGet(completeMsg, &myRxData, NULL, osWaitForever);
	if (myRxData.cmd == 0x02) {
		isConnected = 0;
		audio_complete();
	}
}

void connect_thread(void *argument) {
	myDataPkt myRxData;
	osMessageQueueGet(connectMsg, &myRxData, NULL, osWaitForever);
	if (myRxData.cmd == 0x03) {
		autio_connect();
	}
	isConnected = 1;
	status = stop;
}

void greenLED_flash_thread(void *argument) {
	myDataPkt myRxData;
	osMessageQueueGet(greenLEDFlashMsg, &myRxData, NULL, osWaitForever);
	if (myRxData.data == 0x03)
	{
		green_led_flashes();
	}
}

void control_thread(void *argument) {
	myDataPkt myData;
	myData.cmd = 0x00;
	myData.data = 0x00;
	myData.movement = 0x00;

	for (;;) {
		if (status == forward){
			myData.cmd = 0x01;
			myData.data = 0x01;
			myData.movement = 0x01;
			osMessageQueuePut(redLEDMsg, &myData, NULL, 0);
			osMessageQueuePut(greenLEDMsg, &myData, NULL, 0);
			osMessageQueuePut(audioMsg, &myData, NULL, 0);
			osMessageQueuePut(motorMsg, &myData, NULL, 0);
		} else if (status == backward) {
			myData.cmd = 0x01;
			myData.data = 0x01;
			myData.movement = 0x02;
			osMessageQueuePut(redLEDMsg, &myData, NULL, 0);
			osMessageQueuePut(greenLEDMsg, &myData, NULL, 0);
			osMessageQueuePut(audioMsg, &myData, NULL, 0);
			osMessageQueuePut(motorMsg, &myData, NULL, 0);
		} else if (status == leftward) {
			myData.cmd = 0x01;
			myData.data = 0x01;
			myData.movement = 0x03;
			osMessageQueuePut(redLEDMsg, &myData, NULL, 0);
			osMessageQueuePut(greenLEDMsg, &myData, NULL, 0);
			osMessageQueuePut(audioMsg, &myData, NULL, 0);
			osMessageQueuePut(motorMsg, &myData, NULL, 0);
		} else if (status == rightward) {
			myData.cmd = 0x01;
			myData.data = 0x01;
			myData.movement = 0x04;
			osMessageQueuePut(redLEDMsg, &myData, NULL, 0);
			osMessageQueuePut(greenLEDMsg, &myData, NULL, 0);
			osMessageQueuePut(audioMsg, &myData, NULL, 0);
			osMessageQueuePut(motorMsg, &myData, NULL, 0);
		} else if (status == stop) {
			myData.cmd = 0x01;
			myData.data = 0x02;
			myData.movement = 0x05;
			osMessageQueuePut(redLEDMsg, &myData, NULL, 0);
			osMessageQueuePut(greenLEDMsg, &myData, NULL, 0);
			osMessageQueuePut(audioMsg, &myData, NULL, 0);
			osMessageQueuePut(motorMsg, &myData, NULL, 0);
		} else if (status == complete) {
			myData.cmd = 0x02;
			myData.data = 0x02;
			osMessageQueuePut(redLEDMsg, &myData, NULL, 0);
			osMessageQueuePut(greenLEDMsg, &myData, NULL, 0);
			osMessageQueuePut(completeMsg, &myData, NULL, 0);
		} else if (status == connect) {
			myData.cmd = 0x03;
			myData.data = 0x03;
			osMessageQueuePut(redLEDMsg, &myData, NULL, 0);
			osMessageQueuePut(greenLEDFlashMsg, &myData, NULL, 0);
			osMessageQueuePut(connectMsg, &myData, NULL, 0);
		}
	}
}

int main(void)
{
	// System Initialization
	SystemCoreClockUpdate();
	initGPIO(BAUD_RATE);

	osKernelInitialize(); // Initialize CMSIS-RTOS
	greenLED_Id = osThreadNew(green_led_thread, NULL, NULL);
	redLED_Id = osThreadNew(red_led_thread, NULL, NULL);
	motor_Id = osThreadNew(motor_thread, NULL, NULL);
	audio_Id = osThreadNew(audio_thread, NULL, NULL);
	control_Id = osThreadNew(control_thread, NULL, NULL);
	complete_Id = osThreadNew(complete_thread, NULL, NULL);
	connect_Id = osThreadNew(connect_thread, NULL, NULL);
	greenLEDFlash_Id = osThreadNew(greenLED_flash_thread, NULL, NULL);

	greenLEDMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	redLEDMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	audioMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	motorMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	completeMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	connectMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	greenLEDFlashMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);

	osKernelStart(); // Start thread execution
	for (;;)
	{
	}
}
