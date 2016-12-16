/**
 * @file
 * @Author Rupert Dobrounig (rupert.dobrounig@gmail.com)
 * @date   September, 2016
 * @brief  Header File for the MicroBlue Pump KNX Interface controller
 * @version 1.0
 * @mainpage
 * @section intro_sec Introduction
 * Blue Diamond MicroBlue Condensate Pump KNX Interface
 * implemented as finite state machine
 * @section descript_sec Detailed description
 * This hardware / software allows to directly integrate a Blue Diamond MicroBlue (Other Blue Diamond pumps might work as well, but untested)
 * \n into a KNX bus for monitoring, alarming and power saving purposes!
 * \n There are 2 independent relais outputs, one of them is internally fused (to be used for the pump), the second one can be used to turn off the AC unit in case of a pump issue.
 * \n The system is galvanically isolated between the Pump and the KNX bus using a RSO0505-D DCDC converter and optocouplers
 * \n Due to the design the system requires draws ~18mA from the Bus with both Relais switched on
 * \n This does not comply with the KNX max allowed consumption of 12mA, therefore be warned if your KNX Power supply is already close to its limits.
 * @section credit_sec Credits
 * The System fits into a 2TE REG housing and is mostly built on a modified version of the Selfbus 2TE Controller with LPC922 and the "Kombi Aktor 2In 2Out fuer 2TE" 
 * \n as LPC922 firmware Version 1.0b can be used
 * @see http://selfbus.myxwiki.org/xwiki/bin/view/Technik/Controller_922_2TE
 * @see http://selfbus.myxwiki.org/xwiki/bin/view/Ger%C3%A4te/Ausg%C3%A4nge/2in2out_2TE
 * @see https://github.com/selfbus/software/raw/master/2in2out/Releases/2in2out_1.0b.hex
 * @section download_sec Download
 * @see https://drive.google.com/drive/folders/0B65eQ9bjsU6ZTXUzSnhiRGNyeTA?usp=sharing
 * @see https://drive.google.com/drive/folders/0B65eQ9bjsU6ZTXUzSnhiRGNyeTA?usp=sharing
 * @copyright (C) 2016 Rupert Dobrounig, Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
 * 
 */
#ifndef PUMPIF_H_
#define PUMPIF_H_


/** @name PIN Definitions
 @{**/
#define OUTPORT PORTB	//The Port everything is connected to
#define OUT1 PB1		//Output Channel 1 - Blue LED - Pump ON
#define OUT2 PB0		//Output Channel 2 - Red LED - Alarm
#define LEDON PB2		//LED Enable
#define ANALOG1 PB3		//Analog Voltage Input 1 - Normal (Pump ON) Water Level
#define ANALOG2	PB4		//Analog Voltage Input 2 - Alarm Water Level
#define BANDGAP 253		//Used to switch to Bandgap ADC in
#define QUERY 255		//Used to query the current Muxer settings
/** @}*/

/** @name ADC Buffer value Definitions
 @{**/
#define CHAN1_BUFFER_SIZE 8 // Channel 1 buffer size, 2^n values
#define CHAN1_BUFFER_MASK (CHAN1_BUFFER_SIZE - 1) // buffer mask, used for modulo arithmetic
#define CHAN2_BUFFER_SIZE 8 // Channel 2 buffer size, 2^n values
#define CHAN2_BUFFER_MASK (CHAN2_BUFFER_SIZE - 1) // buffer mask, used for modulo arithmetic
#define BUFFER_FAIL     0
#define BUFFER_SUCCESS  1
/** @}*/

/** @name ADC Measurement value Definitions
 @{**/
#define PUMPONVAL		575		//Pump switch on hysteresis value
#define PUMPOFFVAL		610		//Pump switch off hysteresis value
#define PUMPPWRDOWN		200		//Pump power switched off low value
#define ALARMONVAL		585		//Alarm switch on hysteresis value
#define ALARMOFFVAL		620		//Alarm switch off hysteresis value
#define ALARMPWRDOWN	200		//Alarm power switched off low value
/** @}*/

/** @name Status Flag Definitions
 @{**/
#define STATUSADCVALUEAVAILABLE 0
/** @}*/

/** @name State Flag Definitions
 @{**/
#define STATEPOWEROFF 0
#define STATETRAINING 1
#define STATEIDLE 2
#define STATEPUMPING 3
#define STATEALARM 4
/** @}*/

/** @name Other Defines
 @{**/
#define TRAININGTOUT 16000	//Training Time Out in ms
#define DEBUGLED			//Set this define to enable the debug LEDs, needs more power
/** @}*/

/** @brief average Data memory
 *
 *  structure to hold the average values calculated
 **/
typedef volatile struct avgdata avgdata;
struct avgdata {
	uint16_t chan1last2avg;
	uint16_t chan2last2avg;
	uint16_t chan1first4avg;
	uint16_t chan2first4avg;
};
/**
* @brief    get milliseconds since system started, will overflow every ~49 Days
* @return	Milliseconds.
*/
uint32_t millis();
/**
 @brief			Function to write into the overwriting Channel1 ringbuffer 
 @param[in]		data	the new value to store in the ringbuffer
 @return		whether buffer write was successful or not
 @retval		BUFFER_SUCCESS
 @retval		BUFFER_FAIL
*/
uint8_t Chan1BufferIn(uint16_t data);
/**
 @brief			Function to read data from Channel1 ringbuffer 
 @param[in]		buffer index to return, -7 to 0, more negative means older data
 @return		the requested data
*/
uint16_t Chan1BufferOut(int8_t position);
/**
 @brief			Function to write into the overwriting Channel2 ringbuffer 
 @param[in]		data	the new value to store in the ringbuffer
 @return		whether buffer write was successful or not
 @retval		BUFFER_SUCCESS
 @retval		BUFFER_FAIL
*/
uint8_t Chan2BufferIn(uint16_t data);
/**
 @brief			Function to read data from Channel2 ringbuffer 
 @param[in]		buffer index to return, -7 to 0, more negative means older data
 @return		the requested data
*/
uint16_t Chan2BufferOut(int8_t position);
/**
 @brief			Function to set or get the current ADC Mux Settings 
 @param[in]		channel	the channel to switch the Muxer to or QUERY to return the current setting
 @arg			ANALOG1
 @arg			ANALOG2
 @arg			BANDGAP
 @arg			QUERY
 @return		The Current Muxer channel
 @retval		ANALOG1
 @retval		ANALOG2
 @retval		BANDGAP
*/
uint8_t MuxChannel(uint8_t channel);
/**
 @brief			Function to calculate average values from the Buffer
 @param[in]		p_mydata Where to save the calculated average values
 @return		none
*/
void ProcessData(avgdata * p_mydata);
/**
 @brief			Callback function pointer to FSM state functions
 @param[in]		stateflags the stateflags
 @return		none
*/
void (*statehandler) (uint8_t * stateflags);
/**
 @brief			FSM state functions for state: Poweroff
 @param[in]		stateflags the stateflags
 @return		none
*/
void state_pwroff(uint8_t * stateflags);
/**
 @brief			FSM state functions for state: Training
 @param[in]		stateflags the stateflags
 @return		none
*/
void state_training(uint8_t * stateflags);
/**
 @brief			FSM state functions for state: Idle
 @param[in]		stateflags the stateflags
 @return		none
*/
void state_idle(uint8_t * stateflags);
/**
 @brief			FSM state functions for state: Pumping
 @param[in]		stateflags the stateflags
 @return		none
*/
void state_pumping(uint8_t * stateflags);
/**
 @brief			FSM state functions for state: Alarm
 @param[in]		stateflags the stateflags
 @return		none
*/
void state_alarm(uint8_t * stateflags);

#endif /* PUMPIF_H_ */