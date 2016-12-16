/*
 * pumpif.c
 *
 * Created: 05.11.2016 08:34:29
 * Author : Rupert Dobrounig
 *
 * 
 */ 


//Our MCU Frequency (CKDIV8 set to save power)
#define F_CPU 1000000

//Inline Macros
#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC
#define adc_enable()  (ADCSRA |=  (1<<ADEN)) // re-enable ADC
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) //Clear Bit
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit)) //Set Bit
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/wdt.h>
#include "pumpif.h"


volatile struct avgdata myavgdata = {0,0,0,0};
volatile struct avgdata *p_myavgdata = &myavgdata;


//Global Variables
uint32_t _millis = 0;
uint32_t before_sleep = 0;
uint16_t _1000us = 0;
uint32_t old_millis = 0;
volatile uint8_t statusflags;
volatile uint8_t stateflags;
volatile uint16_t adcvalue;
volatile uint16_t adcvalue;
volatile uint8_t f_wdt = 1;

volatile struct chan1_buffer {
	uint16_t data[CHAN1_BUFFER_SIZE];
	uint8_t pWrite;
	uint8_t pRead;
} chan1_buffer = {{}, 0, 0}; //Init circular buffer struct to zero

volatile struct chan2_buffer {
	uint16_t data[CHAN2_BUFFER_SIZE];
	uint8_t pWrite;
	uint8_t pRead;
} chan2_buffer = {{}, 0, 0}; //Init circular buffer struct to zero

void state_pwroff(uint8_t * stateflags){
	if (!(*stateflags & 1<<STATEPOWEROFF)){ //Entry Event --> Pump was Power Switched off
		*stateflags = 1<<STATEPOWEROFF;
		OUTPORT &= ~((1<<OUT1)|(1<<OUT2));
	}
	if(p_myavgdata->chan1last2avg>PUMPPWRDOWN && p_myavgdata->chan2last2avg>ALARMPWRDOWN){ //Power on Event occurred
		statehandler = state_training; //Switch state to Training
	}
}
void state_training(uint8_t * stateflags){
	static uint32_t stateentrytime;
	if (!(*stateflags & 1<<STATETRAINING)){ //Entry Event --> Pump Power Switched on
		stateentrytime = millis();
		OUTPORT |= (1<<OUT1);				//Enable Pumping
		*stateflags = 1<<STATETRAINING;
	}
	if(Chan1BufferOut(0)<PUMPPWRDOWN && Chan2BufferOut(0)<ALARMPWRDOWN){
			statehandler = state_pwroff;
	}
	else if(((p_myavgdata->chan1last2avg>PUMPOFFVAL) && (p_myavgdata->chan2last2avg>ALARMOFFVAL))){ //Power on Event occurred
		OUTPORT &= ~(1<<OUT1);				//Disable Pumping
		statehandler = state_idle;			//Switch state to Idle
	}
	else if (p_myavgdata->chan1last2avg<=PUMPONVAL && p_myavgdata->chan2last2avg<=ALARMONVAL && (millis()-stateentrytime>TRAININGTOUT)){
		OUTPORT |= ((1<<OUT1)|(1<<OUT2));	//Enable Pumping and Alarm
		statehandler = state_alarm;			//Switch state to Alarm
	}
	else if (p_myavgdata->chan1last2avg<=PUMPONVAL && (millis()-stateentrytime>TRAININGTOUT)){
		statehandler = state_pumping;		//Switch state to Pumping
	}
}
void state_idle(uint8_t * stateflags){
	if (!(*stateflags & 1<<STATEIDLE)){ //Entry Event --> Pump transitioned to idle
		*stateflags = 1<<STATEIDLE;
	}
	if(Chan1BufferOut(0)<PUMPPWRDOWN && Chan2BufferOut(0)<ALARMPWRDOWN){
		statehandler = state_pwroff;
	}
	else if(p_myavgdata->chan2last2avg<p_myavgdata->chan2first4avg && p_myavgdata->chan2last2avg<=ALARMONVAL){ //Alarm Event occurred
		OUTPORT |= ((1<<OUT1)|(1<<OUT2));	//Enable Pumping and Alarm
		statehandler = state_alarm;			//Switch state to Alarm
	}
	else if(p_myavgdata->chan1last2avg<p_myavgdata->chan1first4avg && p_myavgdata->chan1last2avg<=PUMPONVAL){ //Pumping event occurred
		OUTPORT |= (1<<OUT1);				//Enable Pumping
		statehandler = state_pumping;		//Switch state to Pumping
	}
	
}

void state_pumping(uint8_t * stateflags){
	if (!(*stateflags & 1<<STATEPUMPING)){ //Entry Event --> Pump transitioned to Pumping
		*stateflags = 1<<STATEPUMPING;
	}
	if(Chan1BufferOut(0)<PUMPPWRDOWN && Chan2BufferOut(0)<ALARMPWRDOWN){
		statehandler = state_pwroff;
	}
	else if(p_myavgdata->chan2last2avg<p_myavgdata->chan2first4avg && p_myavgdata->chan2last2avg<=ALARMONVAL){ //Alarm Event occurred
		OUTPORT |= ((1<<OUT1)|(1<<OUT2));	//Enable Pumping and Alarm
		statehandler = state_alarm;			//Switch state to Alarm
	}
	else if(p_myavgdata->chan1last2avg>p_myavgdata->chan1first4avg && p_myavgdata->chan1last2avg>=PUMPOFFVAL){ //Idle event occurred
		OUTPORT &= ~(1<<OUT1);				//Disable Pumping
		statehandler = state_idle;			//Switch state to Idle
	}
	
}

void state_alarm(uint8_t * stateflags){
	if (!(*stateflags & 1<<STATEALARM)){ //Entry Event --> Pump transitioned to Alarm
		*stateflags = 1<<STATEALARM;
	}
	if(Chan1BufferOut(0)<PUMPPWRDOWN && Chan2BufferOut(0)<ALARMPWRDOWN){
		statehandler = state_pwroff;
	}
	else if(p_myavgdata->chan2last2avg>p_myavgdata->chan2first4avg && p_myavgdata->chan2last2avg>=ALARMOFFVAL){ //Alarm Cleared
		OUTPORT &= ~(1<<OUT2);				//Disable Alarm
		statehandler = state_pumping;		//Switch state to Alarm
	}
}



uint8_t Chan1BufferIn(uint16_t data)
{
	uint8_t next = ((chan1_buffer.pWrite + 1) & CHAN1_BUFFER_MASK);
	chan1_buffer.data[chan1_buffer.pWrite] = data;
	chan1_buffer.pRead = chan1_buffer.pWrite; //We want to know were to get the most recent value
	chan1_buffer.pWrite = next; //Where to write next time
	return BUFFER_SUCCESS;
}

uint16_t Chan1BufferOut(int8_t position){ //0 to -7
	if ((int8_t)chan1_buffer.pRead + position >= 0){
		return chan1_buffer.data[(int8_t)chan1_buffer.pRead + position];
	}
	else{
		return chan1_buffer.data[((int8_t)chan1_buffer.pRead + position)+8];
	}
}

uint8_t Chan2BufferIn(uint16_t data)
{
	uint8_t next = ((chan2_buffer.pWrite + 1) & CHAN2_BUFFER_MASK);
	chan2_buffer.data[chan2_buffer.pWrite] = data;
	chan2_buffer.pRead = chan2_buffer.pWrite; //We want to know were to get the most recent value
	chan2_buffer.pWrite = next; //Where to write next time
	return BUFFER_SUCCESS;
}

uint16_t Chan2BufferOut(int8_t position){ //0 to -7
	if ((int8_t)chan2_buffer.pRead + position >= 0){
		return chan2_buffer.data[(int8_t)chan2_buffer.pRead + position];
	}
	else{
		return chan2_buffer.data[((int8_t)chan2_buffer.pRead + position)+8];
	}
}

void ProcessData(avgdata * p_myavgdata){
	p_myavgdata->chan1last2avg = ((Chan1BufferOut(0) + Chan1BufferOut(-1)) / 2);
	p_myavgdata->chan2last2avg = ((Chan2BufferOut(0) + Chan2BufferOut(-1)) / 2);
	p_myavgdata->chan1first4avg = ((Chan1BufferOut(-7) + Chan1BufferOut(-6) + Chan1BufferOut(-5) + Chan1BufferOut(-4)) / 4);
	p_myavgdata->chan2first4avg = ((Chan2BufferOut(-7) + Chan2BufferOut(-6) + Chan2BufferOut(-5) + Chan2BufferOut(-4)) / 4);

}

uint8_t MuxChannel(uint8_t channel){
	
	if (channel==ANALOG1){
		//MUX set to PB3
		ADMUX |= ((1<<MUX1)|(1<<MUX0)); 
		ADMUX &= ~((1<<MUX3)|(1<<MUX2));
		return ANALOG1;
	}
	else if (channel==ANALOG2){
		//MUX set to PB4
		ADMUX |= ((1<<MUX1)); 
		ADMUX &= ~((1<<MUX3)|(1<<MUX2)|(1<<MUX0));
		return ANALOG2;
	}
	else if (channel==BANDGAP){
		//MUX set to internal Bandgap reference (should read 1.1V)
		ADMUX |= ((1<<MUX3)|(1<<MUX2));
		ADMUX &= ~((1<<MUX1)|(1<<MUX0)); 
		return BANDGAP;
	}
	else if (channel==QUERY){ //Used to query the current setting
		if ((ADMUX & 0x0f) == ((1<<MUX1)|(1<<MUX0))){
			return ANALOG1;
		}
		else if ((ADMUX & 0x0f) == (1<<MUX1)) {
			return ANALOG2;
		}
		else if ((ADMUX & 0x0f) == ((1<<MUX3)|(1<<MUX2))) {
			return BANDGAP;
		}
		else {
			return 0xff;
		}
		
	}
	else {
		return 0xff;
	}
}

//Atomic access to the millis timer
uint32_t millis() {
  uint32_t m;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
	  m = _millis;	  
  }
  return m;
}

//ISR for ADC conversion complete
ISR(ADC_vect){
	uint8_t lowbyte=ADCL;
	if(MuxChannel(QUERY)==ANALOG1){ //if we are currently measuring the first channel
		Chan1BufferIn(ADCH<<8 | lowbyte);
		MuxChannel(ANALOG2); //Switch to Second Channel
		ADCSRA |= (1 << ADSC); //Start next ADC Measurement
	}
	else if(MuxChannel(QUERY)==ANALOG2){ //we are currently measuring the second channel
		Chan2BufferIn(ADCH<<8 | lowbyte);
		adc_disable();
	}
	statusflags |= (1<<STATUSADCVALUEAVAILABLE);
}

//ISR for timer 0 Overflow (our millis counter)
ISR(TIM0_OVF_vect) {
  _1000us += 256;
  while (_1000us > 1000) {
    _millis++;
    _1000us -= 1000;
  }
}


int main(void)
{
	//Setup Watchdog for safety reset
	wdt_enable(WDTO_500MS);
	
	//Setup Digital outputs
	DDRB |= ((1<<OUT1)|(1<<OUT2)|(1<<LEDON));	//Set Pins To Output
	PORTB &= ~((1<<OUT1)|(1<<OUT2)|(1<<LEDON));  //Set all outputs to LOW
	#ifdef DEBUGLED
	PORTB |= (1<<LEDON); //Enable Debugging LEDs
	#endif
	
	//Init Analog inputs + ADC
	DDRB &= ~((1<<ANALOG1)|(1<<ANALOG2)); //Set Pins To Input
	PORTB &= ~((1<<ANALOG1)|(1<<ANALOG2)); //Set Pullup Disabled
	ADMUX |= ((1<<REFS2)|(1<<REFS1)); //Set analog reference (Vref) to Internal 2.56V reference without external bypass capacitor
	MuxChannel(BANDGAP);
	ADCSRA |= ((1<<ADIE)|(1<<ADPS1)|(1<<ADPS0)); //Enable ADC Interrupt and set Timing Prescaler to 1/8 fCPU (125 kHz) 

	//Init Timer
	// timer0 running at 1/1 the clock rate
	// overflow timer0 every 0.256 ms
	TCCR0B |= (1<<CS00);
	// enable timer overflow interrupt
	TIMSK  |= (1<<TOIE0);

	// Enable global interrupts
	sei();

	// Preload now value before entering main loop
	uint32_t hundrettwetyfifemstimer = millis();
	uint8_t everysecondtime = 0;	

    //We start in the Power off State
	statehandler = state_pwroff;
    
	/* The Main Loop */
	while (1) 
    {
 		if (millis()-hundrettwetyfifemstimer>125){
			hundrettwetyfifemstimer = millis(); //set time immediately after loop call, therefore function is called every 125 ms
			if (everysecondtime > 0){
				//process the data odd times
				ProcessData(p_myavgdata);
				everysecondtime=0;
			}
			else{
				//handle states at even time
				statehandler(&stateflags);
				everysecondtime++;
			}
			adc_enable();
			MuxChannel(ANALOG1);
			ADCSRA |= (1 << ADSC); //Start a ADC Measurement
		} 
		//Reset Watchdog every loop
		wdt_reset();
    }
}

