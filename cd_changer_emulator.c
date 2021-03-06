#include <stdio.h>
#include <stdint.h>
#include <wiringPi.h>

typedef int bool;

/************************* OLD **************************/
//const uint8_t MELBUS_CLOCKBIT_INT = 1; //interrupt numer (INT1) on DDR3
//const uint8_t MELBUS_CLOCKBIT = 3; //Pin D3 - CLK
//const uint8_t MELBUS_DATA = 4; //Pin D4  - Data
//const uint8_t MELBUS_BUSY = 5; //Pin D5  - Busy
/************************* OLD **************************/

const uint8_t MELBUS_CLOCKBIT_INT = 10; // GPIO 14 TXD
const uint8_t MELBUS_CLOCKBIT = 7; // GPIO 7 - CLK
const uint8_t MELBUS_DATA = 2; // GPIO 2  - Data
const uint8_t MELBUS_BUSY = 4; // GPIO 4  - Busy

volatile uint8_t melbus_ReceivedByte = 0;
volatile uint8_t melbus_LastReadByte[8] = {0, 0, 0, 0 ,0, 0, 0, 0};
uint8_t melbus_Bitposition = 7;
volatile long Counter = 0;
uint8_t ByteToSend = 0;

volatile bool InitialSequence = FALSE;
volatile bool ByteIsRead = FALSE;
volatile bool sending_byte = FALSE;
volatile bool melbus_MasterRequested = FALSE;
volatile bool melbus_MasterRequestAccepted = FALSE;
volatile bool Connected = FALSE;
volatile bool testbool = FALSE;
volatile bool AllowInterruptRead = TRUE;

// Declaration of functions
void setup(void);
int main(void);
void melbus_Init_CDCHRG(void);
void SendByteToMelbus(uint8_t byteToSend);
void MELBUS_CLOCK_INTERRUPT(void);

/**
 * Startup sequence
 */
void setup() {
	// Setup wiringPi
	wiringPiSetupGpio () ;

	// Data is deafult input high
	pinMode(MELBUS_DATA, INPUT);
	pullUpDnControl (MELBUS_DATA, PUD_DOWN);


	// Activate interrupt on clock pin (
//	wiringPiISR (MELBUS_CLOCKBIT_INT, INT_EDGE_RISING, MELBUS_CLOCK_INTERRUPT);

	// Set Clockpin-interrupt to input
	pinMode(MELBUS_CLOCKBIT_INT, INPUT);
	pullUpDnControl (MELBUS_CLOCKBIT_INT, PUD_DOWN);

	printf("Requesting Volvo-Melbus:\n");
	// Call function that tells HU that we want to register a new device
	melbus_Init_CDCHRG();
}

int main(void) {
	// Setup functions
	setup();
while(TRUE){
	while(!ByteIsRead){}
	//printf("While true\n");
		// Waiting for the clock interrupt to trigger 8 times to read one byte before evaluating the data
		if(ByteIsRead)
		{
			printf("Byte is read\n");
			// Reset bool to enable reading of next byte
			ByteIsRead = FALSE;
			Counter++;
			// If we failed to connect, reset and retry the init procedure
			if((Counter > 100) && (Connected == FALSE))
			{
				printf("Trying to reconnect...\n");
				Counter = 0;
				melbus_MasterRequested = FALSE;
				melbus_MasterRequestAccepted = FALSE;
				melbus_Init_CDCHRG();
			}
			if(Counter>100)
			{
				Counter = 0;
			}

			// Check if this is the very first initial sequence (07 1A EE ....)
			if((melbus_LastReadByte[2] == 0x07) && (melbus_LastReadByte[1] == 0x1A)
					&& (melbus_LastReadByte[0] == 0xEE))
			{
				InitialSequence = TRUE;
				printf("Initiating CD-CHGR...\n");
			}
			// Now check if this is the car ignition startup sequence.
			// The HU performes a check everytime the car starts, to evaluate if the initiated
			// applications still are present: (00 00 1C ED ....)
			// This function is not necessary since the Arduino
			// reconnects if not connected by calling the first init-procedure!
			else if((melbus_LastReadByte[3] == 0x00) && (melbus_LastReadByte[2] == 0x00)
					&& (melbus_LastReadByte[1] == 0x1C) && (melbus_LastReadByte[0] == 0xED))
			{
				printf("Initiating ignition CD-CHGR...\n");
				InitialSequence = TRUE;
			}

			// If this is the initial sequence and the HU is now asking if the CD-CHGR (0xE8) is present?
			if((melbus_LastReadByte[0] == 0x8/*0xE8*/) && (InitialSequence == TRUE))
			{
				InitialSequence = FALSE;

				// Returning the expected byte to the HU, to confirm that the CD-CHGR is present
				// (0xEE)! see "ID Response" - table here http://volvo.wot.lv/wiki/doku.php?id=melbus
				SendByteToMelbus(0xEE);
				printf("Connected!\n");
				// Make sure the bit-counter is reset before we go back to reading bytes...
				melbus_Bitposition = 7;
			}

			//Check if the HU is asking for current track information (E9 1B E0 01 08 .......) - about once a second
			//The HU is writing out CD ERROR if it wont get a response on this... the AUX works anyway!
			else if((melbus_LastReadByte[4] == 0xE9) && (melbus_LastReadByte[3] == 0x1B)
					&& (melbus_LastReadByte[2] == 0xE0)  && (melbus_LastReadByte[1] == 0x01)
					&& (melbus_LastReadByte[0] == 0x08))
			{
				Connected = TRUE;
				printf("Track info requested!\n");
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
		if (digitalRead(MELBUS_BUSY) == HIGH)
		{
			//Make sure we are in sync when reading the bits by resetting the clock reader
			melbus_Bitposition = 7;

		}
	}
}

/**
 * Notify HU that we want to trigger the first initiate procedure to add a new device (CD-CHGR) by pulling BUSY line low for 1s
 */
void melbus_Init_CDCHRG() {
	// Disable interrupt on INT1 quicker then: detachInterrupt(MELBUS_CLOCKBIT_INT);
	// EIMSK &= ~(1<<INT1);
//	pinMode(MELBUS_CLOCKBIT_INT, OUTPUT);

	pinMode(MELBUS_BUSY, INPUT);

	// Wait until Busy-line goes high (not busy) before we pull BUSY low to request init
	while(digitalRead(MELBUS_BUSY)== LOW){
		// Busy-wait
	}
	delay(10);

	pinMode(MELBUS_BUSY, OUTPUT);
	digitalWrite(MELBUS_BUSY, LOW);
	delay(1600);
	digitalWrite(MELBUS_BUSY, HIGH);
	pinMode(MELBUS_BUSY, PUD_UP);

	// Enable interrupt on INT1, quicker then: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
	//EIMSK |= (1<<INT1);
	wiringPiISR (MELBUS_CLOCKBIT_INT, INT_EDGE_RISING, &MELBUS_CLOCK_INTERRUPT);

	printf("Volvo-Melbus notified about requested initiation procedure\n");
}

/**
 * This is a function that sends a byte to the HU - (not using interrupts)
 */
void SendByteToMelbus(uint8_t byteToSend){
	// Disable interrupt on INT1 quicker then: detachInterrupt(MELBUS_CLOCKBIT_INT);
	//EIMSK &= ~(1<<INT1);
	pinMode(MELBUS_CLOCKBIT_INT,OUTPUT);

	// Convert datapin to output
	pinMode(MELBUS_DATA, OUTPUT); //To slow, use DDRD instead:
	// DDRD |= (1<<MELBUS_DATA);

	int i = 7;
	//For each bit in the byte
	for(i = 7; i >= 0; i--)
	{
		// If bit [i] is "1" - make datapin high
		if(byteToSend & (1<<i))
		{
			digitalWrite(MELBUS_DATA, HIGH); //To slow, use AVR-style instead:
			//PORTD |= (1<<MELBUS_DATA);
		}
		// If bit [i] is "0" - make databpin low
		else
		{
			digitalWrite(MELBUS_DATA, LOW);
			//PORTD &= ~(1<<MELBUS_DATA);
		}
		//dummyloop until clk goes low and then high again (I think HU is recording the value on datapin when clk goes back high again)
		//while(PIND & (1<<MELBUS_CLOCKBIT)){}
		//while(!(PIND & (1<<MELBUS_CLOCKBIT))){}
		//while(digitalRead(MELBUS_CLOCKBIT)==HIGH){}
		//while(digitalRead(MELBUS_CLOCKBIT)==LOW){}
	}

	//Reset datapin to high and return it to an input
	pinMode(MELBUS_DATA, PUD_UP);
	pinMode(MELBUS_DATA, INPUT); //????

	//Enable interrupt on INT1, quicker then: attachInterrupt(MELBUS_CLOCKBIT_INT, MELBUS_CLOCK_INTERRUPT, RISING);
	//EIMSK |= (1<<INT1);
	wiringPiISR(MELBUS_CLOCKBIT_INT, INT_EDGE_RISING, MELBUS_CLOCK_INTERRUPT);
}

/**
 * Global external interrupt that triggers when clock pin goes high after it has been low for a short time => time to read datapin
 */
void MELBUS_CLOCK_INTERRUPT() {
	// Read status of Datapin and set status of current bit in recv_byte
	printf("Interrupt\n");
	if (digitalRead(MELBUS_DATA)==HIGH)
	{
		printf("High\n");
		melbus_ReceivedByte |= (1<<melbus_Bitposition); // Set bit nr [melbus_Bitposition] to "1"
	}
	else
	{
		printf("Low\n");
		melbus_ReceivedByte &=~(1<<melbus_Bitposition); // Set bit nr [melbus_Bitposition] to "0"
	}

	// If all the bits in the byte are read:
	if (melbus_Bitposition == 0)
	{
		printf("all read\n");
		int i = 7;
		// Move every lastreadbyte one step down the array to keep track of former bytes
		for(i = 7; i > 0; i--)
		{
			melbus_LastReadByte[i] = melbus_LastReadByte[i-1];
		}

		// Insert the newly read byte into first position of array
		melbus_LastReadByte[0] = melbus_ReceivedByte;
		printf("Byte %d\n", melbus_ReceivedByte);

		// Set bool to TRUE to evaluate the bytes in main loop
		ByteIsRead = TRUE;

		// Reset bitcount to first bit in byte
		melbus_Bitposition=7;
	}
	else
	{
		// Set bitnumber to address of next bit in byte
		melbus_Bitposition--;
		printf("Counter %d\n", melbus_Bitposition);
	}
}
