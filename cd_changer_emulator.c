/*
 * blink.c:
 *	Standard "blink" program in wiringPi. Blinks an LED connected
 *	to the first GPIO pin.
 *
 * Copyright (c) 2012-2013 Gordon Henderson. <projects@drogon.net>
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

 #include <stdio.h>
#include <stdint.h>
 #include <wiringPi.h>

// // LED Pin - wiringPi pin 0 is BCM_GPIO 17.


// #define	LED	17

// int main (void)
// {

//   wiringPiSetupGpio () ;
//   pinMode (LED, OUTPUT) ;

//   for (;;)
//   {
//     digitalWrite (LED, HIGH) ;	// On
//     delay (1000) ;		// mS
//     digitalWrite (LED, LOW) ;	// Off
//     delay (1000) ;
//   }
//   return 0 ;
// }

//************************************************************************************************************************

/* Melbus CDCHGR Emulator
 * Program that emulates the MELBUS communication from a CD-changer (CD-CHGR) in a Volvo V70 (HU-xxxx) to enable AUX-input through the 8-pin DIN-contact. 
 * Tis setup is using an Arduino Nano 5v clone
 * 
 * The HU enables the CD-CHGR in its source-menue after a successful initialization procedure is accomplished. 
 * The HU will remove the CD-CHGR everytime the car starts if it wont get an response from CD-CHGR (second init-procedure).
 * 
 * Karl Hagström 2015-11-04
 * 
 * This project went realy smooth thanks to these sources: 
 * http://volvo.wot.lv/wiki/doku.php?id=melbus
 * https://github.com/festlv/screen-control/blob/master/Screen_control/melbus.cpp
 * http://forums.swedespeed.com/showthread.php?50450-VW-Phatbox-to-Volvo-Transplant-(How-To)&highlight=phatbox
 */

const uint8 MELBUS_CLOCKBIT_INT = 1; //interrupt numer (INT1) on DDR3
const uint8 MELBUS_CLOCKBIT = 3; //Pin D3 - CLK
const uint8 MELBUS_DATA = 4; //Pin D4  - Data
const uint8 MELBUS_BUSY = 5; //Pin D5  - Busy

volatile uint8_t melbus_ReceivedByte = 0;
volatile uint8_t melbus_LastReadByte[8] = {0, 0, 0, 0 ,0, 0, 0, 0};
volatile uint8_t melbus_Bitposition = 7;
uint8_t ByteToSend = 0;
volatile long Counter = 0;

volatile bool InitialSequence = false;
volatile bool ByteIsRead = false;
volatile bool sending_byte = false;
volatile bool melbus_MasterRequested = false;
volatile bool melbus_MasterRequestAccepted = false;

volatile bool Connected = false;
volatile bool testbool = false;
volatile bool AllowInterruptRead = true;

//Startup seequence
void setup() {

	wiringPiSetupGpio();

	//Data is deafult input high
	pinMode(MELBUS_DATA, INPUT_PULLUP);

	//Activate interrupt on clock pin (INT1, D3)
	attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
	//Set Clockpin-interrupt to input
	pinMode(MELBUS_CLOCKBIT, INPUT_PULLUP);

	//Initiate serial communication to debug via serial-usb (arduino)
	Serial.begin(115200);
	Serial.println("Initiating contact with Volvo-Melbus:");

	//Call function that tells HU that we want to register a new device
	melbus_Init_CDCHRG();
}

//Main loop
void loop() {
	//Waiting for the clock interrupt to trigger 8 times to read one byte before evaluating the data
	if (ByteIsRead) {
		//Reset bool to enable reading of next byte
		ByteIsRead=false;

		Counter++;
		//If we failed to connect, reset and retry the init procedure
		if(Counter>100 && Connected == false){
			Serial.print("\nRetrying to connect...\n");
			Counter=0;
			melbus_MasterRequested = false;
			melbus_MasterRequestAccepted = false;
			melbus_Init_CDCHRG();
		}
		if(Counter>100)
			Counter = 0;

		//Check if this is the very first initial sequence (07 1A EE ....)
		if(melbus_LastReadByte[2] == 0x07 && melbus_LastReadByte[1] == 0x1A && melbus_LastReadByte[0] == 0xEE)
		{
			InitialSequence = true;
			Serial.println("Initiating CD-CHGR...");
		}
		//Now check if this is the car ignition startup sequence.
		//The HU performes a check everytime the car starts, to evaluate if the initiated applications are still present: (00 00 1C ED ....)
		//This function is not necessary since the Arduino reconnects if not connected by calling the first init-procedure!
		else if(melbus_LastReadByte[3] == 0x00 && melbus_LastReadByte[2] == 0x00 && melbus_LastReadByte[1] == 0x1C && melbus_LastReadByte[0] == 0xED){
			Serial.println("Initiating ignition CD-CHGR...");
			InitialSequence = true;
		}

		//If this is the initial sequense and the HU is now asking if the CD-CHGR (0xE8) is prsent?
		if(melbus_LastReadByte[0] == 0xE8 && InitialSequence == true){
			InitialSequence = false;

			//Returning the expected byte to the HU, to confirm that the CD-CHGR is present (0xEE)! see "ID Response"-table here http://volvo.wot.lv/wiki/doku.php?id=melbus
			SendByteToMelbus(0xEE);
			Serial.println("\nConnected!");
			//Make sure the bit-counter is reset before we go back to reading bytes...
			melbus_Bitposition=7;
		}

		//Check if the HU is asking for current track information (E9 1B E0 01 08 .......) - about once a second
		//The HU is writing out CD ERROR if it wont get a response on this... the AUX works anyway!
		else if(melbus_LastReadByte[4] == 0xE9 && melbus_LastReadByte[3] == 0x1B && melbus_LastReadByte[2] == 0xE0  && melbus_LastReadByte[1] == 0x01 && melbus_LastReadByte[0] == 0x08 ){
			Connected = true;
			Serial.println("\nTrack info requested!");
			/* This is where you could request master mode and send the HU your cartridge and track info to display instead of "CD ERROR":
			 * 1. Wait for Busy-pin to go high again (HU not currently using melbus)
			 * 2. Pull datapin low for 2ms
			 * 3. listen for the "Master request broadcast" and then respond to your address followed by 8 bits (track info)
			 *  see http://volvo.wot.lv/wiki/doku.php?id=melbus for further info
			 *
			 *  For some reson I didn't get the HU to send out the Master request broadcast...
			 *  I gave up since I don't care that the display sais CD Error (legit message since my smartphone lacks a CD!)
			 */
		}
	}

	//If BUSYPIN is HIGH => HU is in between transmissions
	if (digitalRead(MELBUS_BUSY)==HIGH)
	{
		//Make sure we are in sync when reading the bits by resetting the clock reader
		melbus_Bitposition = 7;

	}
}

//Notify HU that we want to trigger the first initiate procedure to add a new device (CD-CHGR) by pulling BUSY line low for 1s
void melbus_Init_CDCHRG() {
	//Disabel interrupt on INT1 quicker then: detachInterrupt(MELBUS_CLOCKBIT_INT);
	EIMSK &= ~(1<<INT1);

	// Wait untill Busy-line goes high (not busy) before we pull BUSY low to request init
	while(digitalRead(MELBUS_BUSY)==LOW){}
	delayMicroseconds(10);

	pinMode(MELBUS_BUSY, OUTPUT);
	digitalWrite(MELBUS_BUSY, LOW);
	delay(1200);
	digitalWrite(MELBUS_BUSY, HIGH);
	pinMode(MELBUS_BUSY, INPUT_PULLUP);

	//Enable interrupt on INT1, quicker then: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
	EIMSK |= (1<<INT1);
}

//This is a function that sends a byte to the HU - (not using interrupts)
void SendByteToMelbus(uint8_t byteToSend){
	//Disabel interrupt on INT1 quicker then: detachInterrupt(MELBUS_CLOCKBIT_INT);
	EIMSK &= ~(1<<INT1);

	//Convert datapin to output
	//pinMode(MELBUS_DATA, OUTPUT); //To slow, use DDRD instead:
	DDRD |= (1<<MELBUS_DATA);

	//For each bit in the byte
	for(int i=7;i>=0;i--)
	{
		//If bit [i] is "1" - make datapin high
		if(byteToSend & (1<<i)){
			//digitalWrite(, HIGH); //To slow, use AVR-style instead:
			PORTD |= (1<<MELBUS_DATA);
		}
		//if bit [i] is "0" - make databpin low
		else{
			PORTD &= ~(1<<MELBUS_DATA);
		}
		//dummyloop until clk goes low and then high again (I think HU is recording the value on datapin when clk goes back high again)
		while(PIND & (1<<MELBUS_CLOCKBIT)){}
		while(!(PIND & (1<<MELBUS_CLOCKBIT))){}
		//while(digitalRead(MELBUS_CLOCKBIT)==HIGH){}
		//while(digitalRead(MELBUS_CLOCKBIT)==LOW){}
	}

	//Reset datapin to high and return it to an input
	pinMode(MELBUS_DATA, INPUT_PULLUP);

	//Enable interrupt on INT1, quicker then: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
	EIMSK |= (1<<INT1);
}

//Global external interrupt that triggers when clock pin goes high after it has been low for a short time => time to read datapin
void MELBUS_CLOCK_INTERRUPT() {
	//Read status of Datapin and set status of current bit in recv_byte
	if (digitalRead(MELBUS_DATA)==HIGH){
		melbus_ReceivedByte |= (1<<melbus_Bitposition); //set bit nr [melbus_Bitposition] to "1"
	}
	else {
		melbus_ReceivedByte &=~(1<<melbus_Bitposition); //set bit nr [melbus_Bitposition] to "0"
	}

	//if all the bits in the byte are read:
	if (melbus_Bitposition==0) {
		//Move every lastreadbyte one step down the array to keep track of former bytes
		for(int i=7;i>0;i--){
			melbus_LastReadByte[i] = melbus_LastReadByte[i-1];
		}

		//Insert the newly read byte into first position of array
		melbus_LastReadByte[0] = melbus_ReceivedByte;

		//set bool to true to evaluate the bytes in main loop
		ByteIsRead = true;

		//Reset bitcount to first bit in byte
		melbus_Bitposition=7;
	}
	else {
		//set bitnumber to address of next bit in byte
		melbus_Bitposition--;
	}

}
