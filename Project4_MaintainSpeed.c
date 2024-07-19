/*
 * Copyright 2016-2024 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    Project4_MaintainSpeed.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"

/* --------------------------------------
 * Library (Move to own file)
 * --------------------------------------
 */
typedef uint8_t PIN;

typedef enum {
	FWD,
	REV,
	STOP,
	IDLE,
} Direction;

typedef enum {
	INPUT,
	OUTPUT,
} DataDirection;

typedef enum {
	PS1,
	PS2,
	PS4,
	PS8,
	PS16,
	PS32,
	PS64,
	PS128,
} TPM_PS;

typedef enum {
	ALT0,
	ALT1,
	ALT2,
	ALT3,
	ALT4,
	ALT5,
	ALT6,
	ALT7,
} PIN_MUX;

typedef struct {
	PIN pin;
	PORT_Type* port;
	PIN_MUX pinMux;
	DataDirection dataDirection;
	bool enableInterrupts;
	bool configured;
} PIN_Conn;

typedef struct {
	TPM_Type* tpm;
	TPM_PS prescaler;
	unsigned short mod;
	bool overflowHalt;
	bool configured;
} TPM_Module;

typedef struct {
	PIN_Conn* A;
	PIN_Conn* B;
	TPM_Module* tpmModule;
	int angular_res;
	int timeA;
	int timeB;
	int count;
	float speed; //in rotations per minute
	bool configured;
} Encoder;

typedef struct {
	PIN_Conn* pwmOutput;
	TPM_Module* tpmModule;
	uint8_t tpmChannel;
	PIN_Conn* in_1;
	PIN_Conn* in_2;
	float power; // for PWM of 1kHz a mod of 569 with 128PS is needed
	Direction direction;
	bool configured;
} Motor;

typedef struct {
	float desired;
	float k_prop;
	float k_int;
	float k_d;
	float prevError;
	float integral;
} PID_Controller;

int num=0;
float PIDTick(PID_Controller* pid, float measured, float delta){
	float error = pid->desired - measured;

	pid->integral += error * delta;
	float deltaE = error - (pid->prevError)/delta;
	pid->prevError = error;

	float p = pid->k_prop * error;
	float i = pid->k_int * pid->integral;
	float d = pid->k_d * deltaE;

//	if(num == 200){
//		PRINTF("p: %f, i: %f, d: %f\n", p , i , d);
//		num = 0;
//	}
//	num++;
	return p + i + d;
}

GPIO_Type* GPIOFromPort(PORT_Type* port){
	if (port == PORTA) {
		return GPIOA;
	} else if (port == PORTB) {
		return GPIOB;
	} else if (port == PORTC) {
		return GPIOC;
	} else if (port == PORTD) {
		return GPIOD;
	} else if (port == PORTE) {
		return GPIOE;
	}
	return NULL;
}

bool configureTPMModule(TPM_Module* tpmModule)
{
	TPM_Type* tpm = tpmModule->tpm;
	//enable tpm module clock gating
	uint8_t bit;
	if (tpm == TPM0) {
		bit = 24;
	} else if (tpm == TPM1) {
		bit = 25;
	} else if (tpm == TPM2) {
		bit = 26;
	} else {
		return false;
	}

	//enable clock gating for tpm
	SIM->SCGC6 |= (1 << bit);

	//TODO make these config items
	// set clock source to OSCERCLK
	SIM->SOPT2 |= (0x2 << 24);
	if(tpmModule->overflowHalt)
		tpm->CONF|= 0x1 << 17;
	else
		tpm->CONF &= ~(0x1 << 17); // don't Stop on Overflow

	//tpm->SC |= (0x1 << 5); //set to up counting

	tpm->SC = (0x1 << 7); // Reset Timer Overflow Flag

	// set prescaler
	tpm->SC |= tpmModule->prescaler;

	tpm->MOD = tpmModule->mod;

	return tpmModule->configured = true;
}

bool startTPM(TPM_Module* tpmModule){
	if (!tpmModule->configured)
		return false;

	tpmModule->tpm->SC |= 0x01 << 3; // Start the clock
	return true;
}

bool resetTPM(TPM_Module* tpmModule){
	if (!tpmModule->configured)
		return false;

	tpmModule->tpm->SC &= ~(0x3 << 3); //stop timer
	tpmModule->tpm->SC |= 0x1 << 7; // Reset Timer Overflow Flag
	tpmModule->tpm->CNT = 0x1;
	startTPM(tpmModule);
	return true;
}

bool configurePIN(PIN_Conn* pinConn){

	uint8_t bit;
	if (pinConn->port == PORTA) {
		bit = 9;
	} else if (pinConn->port == PORTB) {
		bit = 10;
	} else if (pinConn->port == PORTC) {
		bit = 11;
	} else if (pinConn->port == PORTD) {
		bit = 12;
	} else if (pinConn->port == PORTE) {
		bit = 13;
	} else {
		return false;
	}

	SIM->SCGC5 |= 1 << bit; //enable clock gating

	if(pinConn->enableInterrupts){
		if(pinConn->dataDirection == OUTPUT) //can't have interrupts on outputs
			return false;

		if(pinConn->port == PORTA)
			NVIC_EnableIRQ(30);
		else if(pinConn->port == PORTC || pinConn->port == PORTD)
			NVIC_EnableIRQ(31);
		else
			return false;
	}

	pinConn->port->PCR[pinConn->pin] &= ~0x700; // clears pin mux control of Pin Control Register
	pinConn->port->PCR[pinConn->pin] |= (pinConn->pinMux << 8); //sets mux

	GPIO_Type* gpio = GPIOFromPort(pinConn->port);
	gpio->PDDR &= ~(1 << pinConn->pin); // clear direction bit
	gpio->PDDR |= pinConn->dataDirection << pinConn->pin; // set direction bit

	pinConn->configured = true;
	return true;
}

bool updateMotor(Motor* motor){
	if(motor->power > 1.)
		motor->power = 1.;
	else if(motor->power < 0.)
		motor->power = 0.;

	unsigned short CnV = (unsigned short)((motor->power)*motor->tpmModule->mod);
	motor->tpmModule->tpm->CONTROLS[motor->tpmChannel].CnV = CnV;

	bool _in_1;
	bool _in_2;
	switch(motor->direction){
	case FWD:
		_in_1 = 1; _in_2 = 0;
		break;
	case REV:
		_in_1 = 0; _in_2 = 1;
		break;
	case STOP:
		_in_1 = 1; _in_2 = 1;
		break;
	case IDLE:
		_in_1 = 0; _in_2 = 0;
		break;

	}

	if(_in_1)
		GPIOFromPort(motor->in_1->port)->PSOR |= 1 << motor->in_1->pin;
	else
		GPIOFromPort(motor->in_1->port)->PCOR |= 1 << motor->in_1->pin;

	if(_in_2)
		GPIOFromPort(motor->in_2->port)->PSOR |= 1 << motor->in_2->pin;
	else
		GPIOFromPort(motor->in_2->port)->PCOR |= 1 << motor->in_2->pin;


	return true;
}

bool configureMotor(Motor* motor){
	if (!configureTPMModule(motor->tpmModule))
		return false;

	// configure tpm for PWM
	motor->tpmModule->tpm->CONTROLS[motor->tpmChannel].CnSC |= (0x2 << 2) | (0x2 << 4);

	if (!configurePIN(motor->in_1))
		return false;

	if (!configurePIN(motor->in_2))
		return false;

	if (!configurePIN(motor->pwmOutput))
		return false;

	motor->configured = true;

	if (!updateMotor(motor))
		return false;

	if(!startTPM(motor->tpmModule))
		return false;

	return true;
}

bool configureEncoder(Encoder* encoder){
	if(!(
			configurePIN(encoder->A) &&
			configurePIN(encoder->B) &&
			configureTPMModule(encoder->tpmModule)
	)) return false;

	// fail if pins are miscofigured
	if(
			encoder->A->dataDirection == OUTPUT ||
			encoder->B->dataDirection == OUTPUT ||
			encoder->A->enableInterrupts == false ||
			encoder->B->enableInterrupts == false
	) return false;

	//enable interrupts on rising edges PCR[pin] 16:19 = 1
	encoder->A->port->PCR[encoder->A->pin] &= ~(0xF0003);
	encoder->A->port->PCR[encoder->A->pin] |= 0x90002;

	encoder->B->port->PCR[encoder->B->pin] &= ~(0xF0003);
	encoder->B->port->PCR[encoder->B->pin] |= 0x90002;


	encoder->timeA = -1;
	encoder->timeB = -1;
	encoder->count = 0;
	encoder->speed = 0.;

	startTPM(encoder->tpmModule);

	return true;
}

bool handleEncoderInterrupt(Encoder* encoder, float* delta){
	if(encoder->A->port->PCR[encoder->A->pin] & 1 << 24){ // check if interrupt flag is set TODO: ahhh move to LIB
		encoder->A->port->PCR[encoder->A->pin] |= 1 << 24; // clear interrupt
		encoder->timeA = encoder->tpmModule->tpm->CNT;
	}
	if(encoder->B->port->PCR[encoder->B->pin] & 1 << 24){ // TODO
		encoder->B->port->PCR[encoder->B->pin] |= 1 << 24; // clear interrupt
		encoder->timeB = encoder->tpmModule->tpm->CNT;
	}

	if(encoder->timeA < 0 || encoder->timeB < 0) // both signals haven't been captured
		return false;

	resetTPM(encoder->tpmModule);

	int maxDur = encoder->timeA;
	if(encoder->timeB > maxDur)
		maxDur = encoder->timeB;

	// TODO: flipped for testing on arvin's board
	int direction = 1 - (2*(encoder->timeA < encoder->timeB));

	encoder->count += direction;

	// moved (1/angular_res)*360 degrees in (maxDur/65535)*(65535/(8000000/PS)) seconds
	// dps to rpm mult by (60 seconds / 360 degrees)
	// RPM = (1/angular_res)*60 / (maxDur/65535)*(65535/(8000000/PS))
	// remap to have max motor speed of 90PRM be 1 by dividing by 90
	int PS = (1 << encoder->tpmModule->prescaler);
	*delta = (maxDur/65535.)*(65535./(8000000./PS));
	encoder->speed = direction * ((((1./encoder->angular_res)*60.) / *delta)/90.);

	encoder->timeA = -1;
	encoder->timeB = -1;



	return true;

}

/* --------------------------------------
 * Peripheral Defines
 * --------------------------------------
 */
//					 Pin,    Port,  MUX,  Direction, Interrupts
PIN_Conn lMotorIn1 = { 1,   PORTB, ALT1,     OUTPUT, 	  false };
PIN_Conn lMotorIn2 = { 0,   PORTB, ALT1,     OUTPUT, 	  false };
PIN_Conn lMotorPWM = { 2,   PORTB, ALT3,	 OUTPUT, 	  false };
PIN_Conn lEncoderA = { 6,   PORTA, ALT1,	 INPUT , 	  true  };
PIN_Conn lEncoderB = { 7,   PORTA, ALT1,	 INPUT , 	  true  };

PIN_Conn rMotorIn1 = { 1,   PORTC, ALT1,	 OUTPUT, 	  false };
PIN_Conn rMotorIn2 = { 2,   PORTC, ALT1,	 OUTPUT, 	  false };
PIN_Conn rMotorPWM = { 3,   PORTB, ALT3,	 OUTPUT, 	  false };
PIN_Conn rEncoderA = { 15,  PORTA, ALT1, 	 INPUT , 	  true  };
PIN_Conn rEncoderB = { 14,  PORTA, ALT1,	 INPUT , 	  true  };


TPM_Module motorTPM    = { TPM2, PS1,   7999, false };
TPM_Module lEncoderTPM = { TPM0, PS8,   65535, false };
TPM_Module rEncoderTPM = { TPM1, PS8,   65535, false };

//PID_Controller lMotorPID = { 0., 0.1, 0.001, 0.003, 0., 0. };
//PID_Controller rMotorPID = { 0., 0.1, 0.001, 0.003, 0., 0. };
PID_Controller lMotorPID = { 0., 0.15, 0.15, 0.00005, 0., 0. };
PID_Controller rMotorPID = { 0., 0.15, 0.17, 0.00005, 0., 0. };


Motor lMotor;
Motor rMotor;
Encoder lEncoder;
Encoder rEncoder;

void PORTA_DriverIRQHandler(void){
	// update encoder values
	float delta;
	if(handleEncoderInterrupt(&lEncoder, &delta)){
		lMotor.power += PIDTick(&lMotorPID, lEncoder.speed, delta);
		updateMotor(&lMotor);
	}

	if(handleEncoderInterrupt(&rEncoder, &delta)){
		rMotor.power += PIDTick(&rMotorPID, rEncoder.speed, delta);
		updateMotor(&rMotor);
	}

}

void delay_ms(unsigned int n)
{
	unsigned int i = 0;
	unsigned int j;
	for(i=0; i<n*3500; i++)
	{
		j++;
	}
}

#define BTN_GPIO GPIOC
#define BTN2_GPIO GPIOC
#define BTN_PIN 3
#define BTN2_PIN 12
#define BTN_PORT PORTC
#define BTN2_PORT PORTC

int main(void) {

    /* Init board hardware. */
     BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    PRINTF("INIT\n");

    // Button Setup
	// Button is on PTC3

	SIM->SCGC5 |= 1<<11; // enable clock gating for PORTC

	BTN_PORT->PCR[BTN_PIN] &= ~0x703; // clear bit 8-10 of Pin 5 of PORTD's Pin Control Register
	BTN_PORT->PCR[BTN_PIN] |= 0x703 & (1 << 8 | 0x03); // set bit 8. This sets pin 5 of PORTD to GPIO
	BTN_GPIO->PDDR &= ~(1 << BTN_PIN); // clears bit of the port data direction register, 0 input 1 output

	BTN2_PORT->PCR[BTN2_PIN] &= ~0x703; // clear bit 8-10 of Pin 5 of PORTD's Pin Control Register
	BTN2_PORT->PCR[BTN2_PIN] |= 0x703 & (1 << 8 | 0x03); // set bit 8. This sets pin 5 of PORTD to GPIO
	BTN2_GPIO->PDDR &= ~(1 << BTN2_PIN); // clears bit of the port data direction register, 0 input 1 output

	lMotor = (Motor){ &lMotorPWM, &motorTPM, 0, &lMotorIn1, &lMotorIn2, 0.9, IDLE };
	rMotor = (Motor){ &rMotorPWM, &motorTPM, 1, &rMotorIn1, &rMotorIn2, 0.9, IDLE };

	lEncoder = (Encoder){ &lEncoderA, &lEncoderB, &lEncoderTPM, 135 };
	rEncoder = (Encoder){ &rEncoderA, &rEncoderB, &rEncoderTPM, 135 };


    if(
    		configureMotor(&rMotor) &&
			configureMotor(&lMotor) &&
			configureEncoder(&lEncoder) &&
			configureEncoder(&rEncoder)
	){
    	PRINTF("CONFIGURE SUCCESS\n");
    } else {
    	PRINTF("CONFIGURE FAIL\n");
    }

    delay_ms(500);

    while(true){
    	int btn_state = 0;
		int btn_2_state = 0;
		while(!btn_state && !btn_2_state)
		{
			btn_state = !((BTN_GPIO->PDIR & (1 << BTN_PIN)) >> BTN_PIN);
			btn_2_state = !((BTN2_GPIO->PDIR & (1 << BTN2_PIN)) >> BTN2_PIN);
		}
    	// check for button 1 press
    	if(btn_state){
    		delay_ms(2000);
    		lMotor.direction=FWD; lMotor.power = 0.7; lMotorPID.desired = 0.45;
    		rMotor.direction=FWD; rMotor.power = 0.7; rMotorPID.desired = 0.49;
    		updateMotor(&lMotor); updateMotor(&rMotor);
    		delay_ms(6000);
    		lMotor.direction=STOP; lMotorPID.desired = 0.;
			rMotor.direction=STOP; rMotorPID.desired = 0.;
			updateMotor(&lMotor); updateMotor(&rMotor);
			PRINTF("LCount: %i, RCount: %i\n", lEncoder.count, rEncoder.count);
			lEncoder.count = 0; rEncoder.count = 0;
    	}
    }

}
