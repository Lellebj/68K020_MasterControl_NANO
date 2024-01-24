#include <Arduino.h>

#include <68K020_masterctrl.h>

#include <SPI.h>
#include <gpio_MCP23S17.h>   // import library

// 68020 pin A2	 (orange)	//PB1//	Bus request - (Set LOW to request DMA!)
#define P_BR 1
// 68010 pin B2 (red)		//PB0//	Bus Grant
#define P_BG 0

#define BURNTIME 2
#define INHIBITTIME 10
#define ERASEBYTE 0xAA
#define N_BYTES 256


#define NUM_SEGMENTS 20
#define USING_READBYTES	1	// max segment size 46!	

// undef FTDI_SERIAL1  //if no FTDI device is used
// #define FTDI_SERIAL1		// use FTDI device at Serial 1
// connection to FTDI 232	DTX none
	// connect to TX1/RX1		RX	ARD19 (Orange)
//							TX	ARD18 (Red)
//							VCC	none
//							TSC	none
//							GND		(Yellow)

#ifndef FTDI_SERIAL1
#undef DEBUG
#endif

#undef DEBUG 
// #define DEBUG 




HardwareSerial *mySerial;		// used as resulting Serial 0 or 1.
void setup_wait_DMA_signals();
void reset_wait_DMA_signals(byte rst);
void establish_contact();
// int ex_SectorEraseFLASHSST39();
// int ex_ChipEraseFLASHSST39();
// void ex_resetSRAMCommand();
// void setAddress(unsigned long address);
int readLong(unsigned long *lg);
int readWord(unsigned int *dbdata);
// unsigned int readMEM(unsigned long address);
// void writeSRAM(unsigned long address, unsigned int indata);
// void writeFLASHSST39(unsigned long address, unsigned int indata);
// void pulse_SST39(unsigned long address, unsigned int indata, unsigned int p_width = 1);

bool toggle = false;
long count = 0;
unsigned int ch_av = 0;
bool pr_acc = false;

char buf[120];
char bin_buf[256];

String message = "";		 // a String to hold incoming data
bool stringComplete = false; // whether the string is complete
bool establishedContact = false;
char val; // Data received from the Serial1 port

unsigned long targetAddress, startaddress, seg_pos, seg_pos_ptr;

unsigned int startaddress_l, body_size, bytecount;
uint8_t		n_segments, segment_size, last_segment_size;
uint8_t Adr_Bytes;
String loopStr;
unsigned int seg_start[NUM_SEGMENTS], seg_size[NUM_SEGMENTS];
String arg1, arg2;

// Bus request / Granted
bool BusGranted_is_true = false;

unsigned int start, size;
unsigned int byteValue;
unsigned int num = 0;
String parts[10];
String *pek, *pp;
String inString;
bool is_EEPROM = false;

// Pin testing variables
unsigned int pin_val, act_pin, signalAD;
unsigned long t_adr;     //testaddress
bool pinTest = false;

// time measureing variables
unsigned long time = 0, init_time = 0, oldtime = 0;
unsigned long diff_time = 0;
float time_fact = 500000.0f; //= 256.0f*8192.0f;



//define CS pin and using HAEN or not
//to use HAEN, address should be 0x20 to 0x27
gpio_MCP23S17 AdrBus(10,0x24);	//Adress bus instance [24])
gpio_MCP23S17 DataBus(15,0x27);	//Adress bus instance [27])



void	*act_memChip;

MemChip  *actl_s,*actl_d ;
MemChip  *AS6C4008_L =	new MemChip(SRAM,SRAM1_CE_PIN);
MemChip  *SF040_L = 	new MemChip(FLASH, FLASH1_CE_PIN);
MemChip  *AS6C4008_H = 	new MemChip(SRAM,SRAM2_CE_PIN);
MemChip  *SF040_H = 	new MemChip(FLASH, FLASH2_CE_PIN);

void setup()
{	// Set all gpio pins as inpts 
	AdrBus.begin();//x.begin(1) will override automatic SPI initialization
	AdrBus.gpioPinMode(INPUT);
	DataBus.begin();//x.begin(1) will override automatic SPI initialization
	DataBus.gpioPinMode(INPUT);

	DDRD = 0x00;		// All ports D  are set input

	pinMode(RST_PIN, OUTPUT);
	pinMode(CPU_RST,INPUT_PULLUP);
	digitalWrite(RST_PIN,HIGH);	// set reset state of 23S17 chips (active low)

	pinMode(BG_PIN, INPUT_PULLUP);
	pinMode(BR_PIN, OUTPUT);
	digitalWrite(BR_PIN,HIGH);

	actl_s = actl_d = SF040_L;		// begin to point to  low flash chip	

#ifdef FTDI_SERIAL1		// Use the hardserial Serial1 (pin 18/19)
	mySerial = &Serial1;
	Serial.begin(115200); 		// Start Serial1 communication at 115200 bps
	while (!Serial) 	{ 	; 	// wait for Serial1 port to connect. Needed for native USB port only 	
	}
	mySerial->begin(115200); 	// Start Serial1 communication at 115200 bps
	// FTDI232 device connected to pin 18/19
	while (!Serial1)	{ 	; 	// wait for Serial1 port to connect. Needed for native USB port only 	
	}

#else
	mySerial = &Serial;
	Serial.begin(115200); 		// Start Serial1 communication at 115200 bps
	while (!Serial)	{	; 	// wait for Serial1 port to connect. Needed for native USB port only 	
	}
#endif

// 	DataBus.gpioPinMode(0x00FF);  //GPB output GPA input
// 	AdrBus.gpioPinMode(OUTPUT);
// 	startaddress=0x53579;
// 	// digitalWrite(RST_PIN,HIGH);

// 	AdrBus.gpioRegisterWriteWord(MCP23S17_GPIO, startaddress & 0xFFFF);
// 	//set address lines A16 -  A18 out

// 	DataBus.gpioRegisterWriteByte(MCP23S17_GPIO , (byte)(startaddress>>16));
// 	// PORTD &= ~(1 << actl->CS | 1 << FRD_PIN);		// lower cs pin and OE pin
// 	startaddress +=1;
// 	delay(100);
// while (1){};


	// d_t = DataBus.gpioRegisterReadByte(MCP23S17_GPIO|1);

	// Hardware debugging...
	// DataBus.gpioPinMode(OUTPUT);	// all signals output
	// AdrBus.gpioPinMode(OUTPUT);	// all signals output
	// unsigned long sh =3;
	// while (1) 
	// {
	// // PORTD |= (1 << FWR_PIN );		//*WE high (init)
	// // PORTD &= ~(1 << SRAM1_CE_PIN);	// CE LOW (init)

	// //set address pins
	// // setAddress(address);

	// // AdrBus.gpioRegisterWriteByte(MCP23S17_GPIO, sh);
	// AdrBus.gpioRegisterWriteByte(MCP23S17_GPIO|1, sh);
	// // DataBus.gpioRegisterWriteByte(MCP23S17_GPIO|1, sh);
	// DataBus.gpioRegisterWriteByte(MCP23S17_GPIO, sh);

	// delay(120);
	// sh *=2; 
	// if (sh > 100000) sh=3;
	// };

	// establish_contact();
	// reset_wait_DMA_signals(0);


#ifdef DEBUG
		Serial.println("DEBUG-SETUP()");
#endif

}

void establish_contact()
{
	while (mySerial->available() <= 0)
	{
		mySerial->write("M68K-.....\n");
		delay(200);
	};
	inString = mySerial->readStringUntil('\n');
	mySerial->println(inString);
	// mySerial->write("M68K-Connected\n");


	mySerial->flush();
}

void loop()
{

	//unsigned int buf_pos=0; // position in Ard EEPROM

	// if we get a valid byte, read analog ins:
	while (!mySerial->available())
	{
	}

	// get incoming byte:
	inString = mySerial->readStringUntil('\n');

#ifdef DEBUG
		Serial.println("--->" + inString);
#endif

	// delay(100);
	if (inString == "Q")
		exit(0);

	if (inString == "REQ_PYTHON") //
	{
		mySerial->write("NANO_ACK!\n");
	}


	// Program breadboard FLASH/ SRAM from PC by SRecords
	// if (inString == "GetSRecord") //
	// {

	// 	actl_s->ex_getSRecordsCommand();
	// }

	if (inString == "List")
	{

		actl_s->ex_listCommand();
	}

	if (inString == "Byte")
	{
		// Get byte from apparent address (prog counter)
		actl_s->ex_byteCommand();
	}

	if (inString == "LongWord")
	{
		// Get 4 bytes byte from apparent address (prog counter)
		actl_s->ex_longWordCommand();
	}

	if (inString == "Poke")
	{
		// store one byte value in adress
		actl_d->ex_pokeCommand();
	}

	if (inString == "SectorErase")
	{
		//Clear actl. sector in Flash mem
		actl_d->ex_SectorEraseFLASHSST39();
	}

	if (inString == "ChipErase")
	{
		//Clear actl. sector in Flash mem
		actl_d->ex_ChipEraseFLASHSST39();
	}


	if (inString == "CopyToMem")
	{
		//Copy from memory location to EEPROM $4000,n blocks
		actl_s->ex_CopyMemToMem();
	}




	if (inString == "DebugMSG")
	{
		//Clear actl. sector in Flash mem
		actl_s->ex_reqDebugMSG();
	}


	if (inString == "ClrMEM")
	{
		//Clear n <words> in SRAM starting with PC.
		actl_d->ex_resetSRAMCommand();
	}

	if (inString == "LeaveBUSRQ")
	{
		//reset BR status to normal
#ifdef DEBUG
			Serial.println("LeaveBUSRQ");
#endif
		reset_wait_DMA_signals(0);
	}


	if (inString == "LeaveBUSRQRST")
	{
		//reset BR status to normal and execute reset
		reset_wait_DMA_signals(1);
	}


	if (inString == "EnterBUSRQ")
	{
		//reset BR status to normal
#ifdef DEBUG
			Serial.println("EnterBUSRQ");
#endif
		setup_wait_DMA_signals();
	}


//
// 	if (inString == "TestBlink")
// 	{
// 		//reset BR status to normal
//
// 		setup_wait_DMA_signals();
// #ifdef DEBUG
// 			Serial.println("TestBlink(start)");
// #endif
// 		// start puls by interrupt on Arduino pin 3
// 		pinTest = true;
//
// 		signalAD = mySerial->read();
// 		pin_val = mySerial->read();
//
//
// 		// if (signalAD == 'A')
// 		// 	act_pin = ADDR[pin_val];
// 		// if (signalAD == 'D')
// 		// 	act_pin = DATA[pin_val];
// 		// if (signalAD == 'C') //control lines
// 		// {
// 		// 	if ((pin_val <= 53 && pin_val >= 50) || (pin_val == 11) || (pin_val == 12) || (pin_val == 38))
// 		// 		act_pin = pin_val;
// 		// }
//
// 		t_adr = 1<<pin_val;
//
// 		sprintf(buf, "Blink at pin activated:%d,   Adress:%c%d\n", t_adr, signalAD, pin_val);
// 		mySerial->write(buf);
//
//
// 		for (int debcount=0; debcount<40; debcount++)
// 		{
// 			actl_s->setAddress(t_adr);
// 			delay(300);
// 			actl_s->setAddress(0);
// 			delay(300);
//
// 		}
// 		// if (signalAD == 'A' || signalAD == 'D' || signalAD == 'C')
// 		// {
// 		// 	attachInterrupt(digitalPinToInterrupt(DEB_PULSE), onDebPulse, CHANGE);
// 		// 	pinMode(act_pin, OUTPUT);
// 		// 	delay(50);
// 		// 	sprintf(buf, "Blink at pin activated:%u,   Adress:%c%u\n", act_pin, signalAD, pin_val);
// 		// 	mySerial->write(buf);
// 		// }
//
// 		// if (signalAD == '0')
// 		// {
// 		// 	// end testblink
// 		// 	detachInterrupt(digitalPinToInterrupt(DEB_PULSE));
// 		// 	pinTest = false;
// 		// 	delay(50);
// 		// 	pinMode(act_pin, INPUT);
// 		// 	sprintf(buf, "Blink at pin Finished\n");
// 		// 	mySerial->write(buf);
// 		// }
//
// #ifdef DEBUG
// 			Serial.println("TestBlinkStart(finish)");
// #endif
// 	}

	if (inString == "BinToMem")
	{
		//Clear n <words> in SRAM starting with PC.
		actl_d->ex_binaryToMEMcommand();
	}

/*
// 	if (inString == "TestBlink")
// 	{
// 		//reset BR status to normal

// 		setup_wait_DMA_signals();
// #ifdef DEBUG
// 			Serial.println("TestBlink(start)");
// #endif
// 		// start puls by interrupt on Arduino pin 3
// 		pinTest = true;

// 		signalAD = mySerial->read();
// 		pin_val = mySerial->read();

// 		if (signalAD == 'A')
// 			act_pin = ADDR[pin_val];
// 		if (signalAD == 'D')
// 			act_pin = DATA[pin_val];
// 		if (signalAD == 'C') //control lines
// 		{
// 			if ((pin_val <= 53 && pin_val >= 50) || (pin_val == 11) || (pin_val == 12) || (pin_val == 38))
// 				act_pin = pin_val;
// 		}
// 		if (signalAD == 'A' || signalAD == 'D' || signalAD == 'C')
// 		{
// 			attachInterrupt(digitalPinToInterrupt(DEB_PULSE), onDebPulse, CHANGE);
// 			pinMode(act_pin, OUTPUT);
// 			delay(50);
// 			sprintf(buf, "Blink at pin activated:%u,   Adress:%c%u\n", act_pin, signalAD, pin_val);
// 			mySerial->write(buf);
// 		}

// 		if (signalAD == '0')
// 		{
// 			// end testblink
// 			detachInterrupt(digitalPinToInterrupt(DEB_PULSE));
// 			pinTest = false;
// 			delay(50);
// 			pinMode(act_pin, INPUT);
// 			sprintf(buf, "Blink at pin Finished\n");
// 			mySerial->write(buf);
// 		}

// #ifdef DEBUG
// 			Serial.println("TestBlinkStart(finish)");
// #endif
// 	}*/


	inString = "";
#ifdef DEBUG
		Serial.println("<-:->");
#endif
}

void  setActlSMemChip(unsigned long adr)
	//  set source chip...
{
	actl_s = 0;
	if (adr<0x200000 && adr>=0x180000)  actl_s = AS6C4008_H;
	if (adr<0x180000 && adr>=0x100000)  actl_s = AS6C4008_L;
	if (adr<0x100000 && adr>=0x080000)  actl_s = SF040_H;
	if (adr<0x080000 && adr>=0x0000)  	actl_s = SF040_L;


}

void  setActlDMemChip(unsigned long adr)
	//  set destination chip...
{
	actl_d = 0;
	if (adr<0x200000 && adr>=0x180000)  actl_d = AS6C4008_H;
	if (adr<0x180000 && adr>=0x100000)  actl_d = AS6C4008_L;
	if (adr<0x100000 && adr>=0x080000)  actl_d = SF040_H;
	if (adr<0x080000 && adr>=0x0000)  	actl_d = SF040_L;


}


int MemChip::ex_byteCommand()
{
	// Get byte from apparent address (prog counter)
	unsigned int result;
#ifdef DEBUG
		Serial.println("Byte begin");
#endif
	setup_wait_DMA_signals();

	//read act value ProgCounter
	readLong(&startaddress);
	setActlSMemChip(startaddress);     // source chip
	result = actl_s->readMEM(startaddress);

	mySerial->write((uint8_t)(result & 0XFF));

#ifdef DEBUG
		Serial.println(startaddress, HEX);
		Serial.println(result, HEX);
		Serial.println("Byte end");
#endif
	//reset_wait_DMA_signals();
	return 0;
}

int MemChip::ex_longWordCommand()
{
	// Get 4 bytes from apparent address (prog counter)
	unsigned int result=0;
	setup_wait_DMA_signals();

	//read act value ProgCounter
	readLong(&startaddress);

	setActlSMemChip(startaddress);     // source chip
	for (uint8_t  b=0; b<4; b++)
	{
		
		result = actl_s->readMEM(startaddress+b);
		mySerial->write((uint8_t)(result));
	}

	return 0;
}

int MemChip::ex_pokeCommand()
{
	uint8_t	len;
	char bytes[20];
	setup_wait_DMA_signals();
	//print mem contents

#ifdef DEBUG
		Serial.println("Poke begin");
#endif
	readLong(&startaddress);
	mySerial->readBytes(&len,1);
	mySerial->readBytes(bytes, len);

	setActlDMemChip(startaddress);		// destination chip

	for (uint8_t  pokeloop=0; pokeloop<len; pokeloop++)
		actl_d->write(startaddress+pokeloop, bytes[len-1-pokeloop]);
	 
	//# if Arduino needs to print back message....
	// sprintf(buf, "actl.CS %d store %d values %s at address:%06lX\n", actl_d->CS, len, bytes, startaddress);
	// mySerial->write(buf);


#ifdef DEBUG
		Serial.println("Poke EEPROM/FLASH");
		Serial.println(startaddress, HEX);
		Serial.println(byteValue, HEX);
		Serial.println(s_Val, HEX);
		Serial.println("Poke end");
#endif
	//reset_wait_DMA_signals();
	return 0;
}



int MemChip::ex_binaryToMEMcommand()
{
	uint8_t loop_size;

	// read startaddress and size
	if (inString  =="BinToMem") // send testmessage
	{
	setup_wait_DMA_signals();

	readLong(&startaddress);
	readWord(&bytecount);					// amount of bytes
	mySerial->readBytes((char*)&n_segments,1);		//n_segments from PC
	mySerial->readBytes((char*)&segment_size,1);		//n_segments from PC
	mySerial->readBytes((char*)&last_segment_size,1);				//n_segments from PC

	// sprintf(buf, "(Ard)%lX : %d bytes; n_segments:%d\n",startaddress,(int) bytecount,(int) n_segments);
	// Serial.write(buf);

	setActlDMemChip(startaddress);		// destination chip

	for (uint8_t seg_i=0; seg_i<=n_segments; seg_i++) 		// loop the segments
	{
		loop_size = segment_size;	
		if (seg_i==n_segments)
		{
			loop_size=last_segment_size;
		}

		mySerial->readBytes((char *)bin_buf, loop_size);

		for (uint8_t aCB_= 0; aCB_<loop_size; aCB_++)
		{
			actl_d->write(startaddress+aCB_, bin_buf[aCB_]);
		}

		sprintf(buf, "%lX: segment:%d : %d Bytes\n",startaddress, (int)seg_i, (int)loop_size);
		Serial.write(buf);
		startaddress +=loop_size;

	}	

	sprintf(buf, "(Ard): Finished....%d bytes; %d segments\n", (int)bytecount, (int)n_segments+1);
	Serial.write(buf);
    } 
	return 0;  
}	  





int MemChip::ex_getSRecordsCommand()
{

// 	//size_t a_length;
// 	uint8_t offset;
// 	unsigned char byte_buf[256];
// 	// unsigned int wordValue;
// 	uint8_t		byteValue;
// 	unsigned char srec_checkSum;
// 	unsigned char CharChecksum =0;
// 	unsigned long mainChecksum = 0;
// 	unsigned int tempChecksum=0;

// #ifdef DEBUG
//  	Serial.println("GetSRecord (1)");
// #endif
// 	setup_wait_DMA_signals();
// 	loopStr = mySerial->readStringUntil('\n');

// #ifdef DEBUG
//  	Serial.println(loopStr);
// #endif

// 	while (loopStr[0] == 'S')
// 	{
// 		delay(10);

// 		if (loopStr == "S0")
// 		{
// 		}

// 		if (loopStr == "S1" || loopStr == "S2" || loopStr == "S3")
// 		{

// 			memset(byte_buf, 0, 255);					// clear the buffer
// 			readLong(&startaddress);					// the address from SRecord
// 			Adr_Bytes = mySerial->read();					// no of bytes in SRecord Address (2-4)...
// 			bytecount = mySerial->read();					// amount of bytes in SRecord...
// 			srec_checkSum = mySerial->read();				// checksum from Srecord
// #ifdef USING_READBYTES
// 			mySerial->readBytes(byte_buf, (size_t)(bytecount +1)); //read SRecord to byte_buf.
// #else
// 			for (uint8_t t=0; t<=bytecount; t++)
// 			{
// 				while (!mySerial->available()) {}
// 				byte_buf[t]=mySerial->read();
// 			}
// #endif

// 			setActlDMemChip(startaddress);		// select the correct chip (destination)

// 			// Serial.print(startaddress,HEX );Serial.print( ":_");
// 			// Serial.print(bytecount,HEX );Serial.print( ":_");
// 			// for (uint8_t t=0; t<=bytecount; t++){ Serial.print(byte_buf[t],HEX); Serial.print( " ");}
// 			// Serial.println(" !");


// 			offset = 2 + Adr_Bytes; // No of bytes to skip (S.Nr & Addres bytes)
// #ifdef DEBUG
// 			{ Serial.print("Adr: "); Serial.println(startaddress, HEX);}
// 			{ Serial.print("Adr.Bytes: "); Serial.println(Adr_Bytes, HEX);}
// 			{ Serial.print("bytecount: "); Serial.println(bytecount, HEX);}
// 			{ Serial.print("a_length: "); Serial.println(a_length, HEX);}
// #endif			
// 			// start the checksum with address value...
// 			for (uint8_t u= 1 ; u<(2+Adr_Bytes); u++)
// 			{
// 				mainChecksum += byte_buf[u];
// #ifdef DEBUG
// 			 {Serial.print(byte_buf[u],HEX); Serial.print("_");}
// #endif
// 			}

// 			seg_pos = 0;
// 			while (seg_pos <= (bytecount - offset)) // skip the S.Nr, Addr bytes and checksum byte
// 			{
// 				seg_pos_ptr = startaddress + seg_pos;
// 				// Serial.print(byte_buf[seg_pos+offset], HEX); Serial.print(" : ");
// 				// Serial.println(byte_buf[seg_pos+offset+1], HEX);
// 				// wordValue = (byte_buf[seg_pos + offset] << 8 | byte_buf[seg_pos + 1 + offset]);
// 				byteValue = (byte_buf[seg_pos + offset] );

// 					// Write to SRAM/FLASH with choosed chip 
// 				actl_d->write(seg_pos_ptr, byteValue);
				

// 				tempChecksum = actl_d->readMEM(seg_pos_ptr);
// 				mainChecksum += tempChecksum & 0xFF;

// 				seg_pos += 1;
// 			}



// 			CharChecksum = (char)(~(mainChecksum & 0x000000FF));
// #ifdef DEBUG
// 			 Serial.print("Wordval:");Serial.print(wordValue,HEX); Serial.print("  Checksum:");Serial.println(CharChecksum);
// #endif			 

// 			mainChecksum=0;	

// 			mySerial->flush();
// 			if (srec_checkSum == CharChecksum)
// 				mySerial->write('X');
// 			else 
// 				mySerial->write('E');			// report checksumm error

// 		}

// 		if (loopStr == "S9" || loopStr == "S7" || loopStr == "S8") {}
// 			//break;


// 		loopStr = mySerial->readStringUntil('\n');
// #ifdef DEBUG
//  		Serial.println("ReadSUntil Next loopStr   " +loopStr);
// #endif

// 	}
// #ifdef DEBUG
// 	  Serial.println("Last loopStr  " + loopStr);
// #endif

// 	// sprintf(buf, "(A5): Finished Record/s:   \n");
// 	// mySerial->write(buf);
// #ifdef DEBUG
// 		Serial.println("GetSRecord End (5): Checksum : " + mainChecksum);
// #endif

	return 0;
}


void MemChip::ex_resetSRAMCommand()
{
	unsigned long seg_pos, seg_pos_ptr;

	//clear mem contents
	setup_wait_DMA_signals();

	readLong(&startaddress);
	readWord(&body_size);

	setActlDMemChip(startaddress);		// select the correct chip (destination)

	sprintf(buf, "Clear memory section from:%4lX,   Size:%4X\n", startaddress, body_size);
	mySerial->write(buf);

	seg_pos = 0;
	while (seg_pos < body_size)
	{
		seg_pos_ptr = startaddress + seg_pos;

		if (actl_d->type == SRAM)
		{
			write(seg_pos_ptr, 0x00);
		}

		seg_pos++;
	}
	mySerial->write("MEM Reset\n");
}

int MemChip::ex_SectorEraseFLASHSST39()
{

	//Clear actl. sector in Flash mem
	setup_wait_DMA_signals();
#ifdef DEBUG
		Serial.println("Sector Erase begin");
#endif
	readLong(&startaddress);

	sprintf(buf, "Clear Flash SST39SF040 4k (A0-A11) sector incl:%6lX\n", startaddress);
	mySerial->write(buf);
	setActlDMemChip(startaddress);		// select the correct chip (destination)

	DataBus.gpioPinMode(OUTPUT);	// all signals output

	// prog cycle for SST39SF040 programming WE# controlled timing
	//set address pins
	PORTD &= ~(1 << actl_d->CS); //CS low and
	PORTD |= (1 << FRD_PIN);						   // OE high

	this->pulse_SST39(0x5555, 0xAA);
	this->pulse_SST39(0x2AAA, 0x55);
	this->pulse_SST39(0x5555, 0x80);
	this->pulse_SST39(0x5555, 0xAA);
	this->pulse_SST39(0x2AAA, 0x55);
	this->pulse_SST39(startaddress, 0x30, 50000); // delay >25 msec

	PORTD |= (1 << actl_d->CS); //CS high and
	PORTD |= (1 << FRD_PIN);						  // OE high
	mySerial->write("SST39SF020/40 Erase\n");

#ifdef DEBUG
		Serial.println("Sector Erase end");
#endif		
	return (0);
}


int MemChip::ex_ChipEraseFLASHSST39()
{

	//Clear actl. sector in Flash mem
	setup_wait_DMA_signals();
#ifdef DEBUG
		Serial.println("Sector Erase begin");
#endif
	readLong(&startaddress);
	setActlDMemChip(startaddress);		// select the correct chip (destination)

	sprintf(buf, "Chip erase SST39SF040 \n");
	mySerial->write(buf);

	DataBus.gpioPinMode(0x00FF);		// datalines (GPB) = output

	// prog cycle for SST39SF040 programming WE# controlled timing
	//set address pins
	PORTD &= ~(1 << FLASH1_CE_PIN); //CS low and
	PORTD |= (1 << FRD_PIN);						   // OE high

	pulse_SST39(0x5555, 0xAA);
	pulse_SST39(0x2AAA, 0x55);
	pulse_SST39(0x5555, 0x80);
	pulse_SST39(0x5555, 0xAA);
	pulse_SST39(0x2AAA, 0x55);
	pulse_SST39(0x5555, 0x10, 10000);
	delay(500);
	// pulse_SST39(startaddress, 0x3030, 50000); // delay >25 msec

	PORTB |= (1 << FLASH1_CE_PIN); //CS high and
	PORTB |= (1 << FRD_PIN);						  // OE high
	mySerial->write("SST39SF020/40 Erase\n");

#ifdef DEBUG
		Serial.println("Sector Erase end");
#endif		
	return (0);
}


int MemChip::ex_CopyMemToMem()
{
	// Copy from memory location to EEPROM $4000

	setup_wait_DMA_signals();

	readLong(&targetAddress);	// Target memory pos
	readLong(&startaddress);	// Source pos in memory
	readWord(&bytecount);		// number of bytes

	setActlSMemChip(startaddress);
	setActlDMemChip(targetAddress);		// select the correct chip (destination)


	unsigned int bc=bytecount; 	// block count
	unsigned long dst_adr,src_adr;
	uint8_t act_byte;

	DataBus.gpioPinMode(OUTPUT);	// all signals output

	// if target type memory = FLASH, have to erase correct block
	if (actl_d->type == FLASH)
	{

		for (unsigned int nb=0; nb <= bytecount>>0xC; nb++)
		{
			// erase as many blocks as ....	

			// prog cycle for SST39SF040 programming WE# controlled timing
			//set address pins
			PORTD &= ~(1 << actl_d->CS); //CS low and
			PORTD |= (1 << FRD_PIN);						   // OE high

			bc=nb*0x1000 + startaddress;			// One address in each block to delete

			actl_d->pulse_SST39(0x5555, 0xAA);
			actl_d->pulse_SST39(0x2AAA, 0x55);
			actl_d->pulse_SST39(0x5555, 0x80);
			actl_d->pulse_SST39(0x5555, 0xAA);
			actl_d->pulse_SST39(0x2AAA, 0x55);
			actl_d->pulse_SST39(bc, 0x30, 50000); // , start with block #4, and then nb #blocks, delay >25 msec

			delay(100);					// extra delay

			PORTD |= (1 << actl_d->CS); //CS high and
			PORTD |= (1 << FRD_PIN);						  // OE high

		}	
	}



	// if target > source and target-source<bytecount  ==> negative step
	int diff = targetAddress-startaddress;
	if (diff >0 && diff<bytecount)
	{
		// use negative step
		src_adr= startaddress+bytecount;
		dst_adr= targetAddress+bytecount;
		for (unsigned int nb=0; nb < bytecount; nb++)			// loop through all bytes
		{
			src_adr-=1;
			act_byte = actl_s->readMEM(src_adr);			// source
			dst_adr-=1;  
			actl_d->write(dst_adr,  act_byte );	// destination
		}

	}
	else 
	{
		//use positive step
		src_adr= startaddress;
		dst_adr= targetAddress;
		for (unsigned int nb=0; nb < bytecount; nb++)			// loop through all bytes
		{
			act_byte = actl_s->readMEM(src_adr);			// source
			src_adr+=1;
			actl_d->write(dst_adr,  act_byte );	// destination
			dst_adr+=1;  
		}
	}


	DataBus.gpioPinMode(INPUT);	// all signals output

	sprintf(buf, "Copy from %6lX to %6lX: ..%6X bytes \n", startaddress, targetAddress,  (int)bytecount);
	// sprintf(buf, "Copy Mem to SST39SF040 ($4000) \n");
	mySerial->write(buf);


}


int MemChip::ex_reqDebugMSG()
{

	//Clear actl. sector in Flash mem
	setup_wait_DMA_signals();
#ifdef DEBUG
		Serial.println("Sector Erase begin");
#endif
	readLong(&startaddress);


	sprintf(buf, "Debug: %d___%d____%d____%d__: %d!\n", SF040_L->CS,SF040_H->CS,AS6C4008_L->CS, AS6C4008_H->CS, actl_s->CS );
	mySerial->write(buf);

	mySerial->write("DBG MSG Finished\n");

#ifdef DEBUG
		Serial.println("Sector Erase end");
#endif		
	return (0);
}


void setup_wait_DMA_signals()
{
	// set BR low and wait for BG
	// Buses go to tristate cond
	// #ifdef DEBUG Serial.println("setup_wait_DMA_signals");

	digitalWrite(BR_PIN, LOW);
	while (digitalRead(BG_PIN)) {}; //Bus Grant should go LOW (PB6 Low)

	BusGranted_is_true = true;
	digitalWrite(RST_PIN,HIGH);	// activate (remove reset) 23S17 chips (active low)



	DDRD = 0xFF;		// All ports D  are set output
	PORTD = 0xFF; 		// all signals high

	AdrBus.gpioPinMode(OUTPUT);
	DataBus.gpioPinMode(0x00FF); 	// data lines (GPB) = input, adresslines (GPA)

#ifdef DEBUG
	 Serial.println("setup_wait_DMA_signals_ FINISHED");
#endif
}


void reset_wait_DMA_signals(byte rst)
{
	// set BR high and return buses
	// #ifdef DEBUG Serial.println("reset_wait_DMA_signals");


	DDRD = 0x00;		// All ports D  are set input

	AdrBus.gpioPinMode(INPUT);
	DataBus.gpioPinMode(INPUT); 	// data lines (GPB) = input, adresslines (GPA)

	digitalWrite(RST_PIN,LOW);	// set reset state of 23S17 chips (active low)

	pinMode(BG_PIN, INPUT_PULLUP);
	pinMode(BR_PIN, OUTPUT);

	digitalWrite(BR_PIN, HIGH); // Bus Request  HIGH);
	while (!digitalRead(BG_PIN)) 	{}; // Bus Grant should go HIGH (PB6 High)
	BusGranted_is_true = false;
	if (rst==1)
	{
		// Execute cpu reset
		pinMode(CPU_RST,OUTPUT);
		digitalWrite(CPU_RST,LOW);
		_delay_ms(500);
		digitalWrite(CPU_RST,HIGH);
		pinMode(CPU_RST,INPUT_PULLUP);
	}
}

/*
	Read long integer (address) to variable
	*/
int readLong(unsigned long *lg)
{
	return mySerial->readBytes((char *)lg, 4);
}

/*
	Read word integer (double byte, 16b data) to variable
	*/
int readWord(unsigned int *dbdata)
{
	return mySerial->readBytes((char *)dbdata, 2);
}

/*
		Output the address bits and outputEnable signal using shift registers.
	*/
void MemChip::setAddress(unsigned long address)
{

	//set address lines A0 -  A15 out
	AdrBus.gpioRegisterWriteWord(MCP23S17_GPIO, address & 0xFFFF);
	//set address lines A16 -  A18 out
	adrHigh = (byte)(address>>16);
	DataBus.gpioRegisterWriteByte(MCP23S17_GPIO , adrHigh);

	//pulse the output register pin in 595
	// address output ready
}

unsigned int MemChip::readMEM(unsigned long address)
{
	unsigned int data = 0;

	DataBus.gpioPinMode(0x00FF);		// data lines input

		// set adress pins...

	this->setAddress(address); // send address to 23S17 chips
	PORTD &= ~(1 << actl_s->CS | 1 << FRD_PIN);		// Read: lower cs pin and OE pin

	delayMicroseconds(4);

	data = DataBus.gpioRegisterReadByte(MCP23S17_GPIO|1);		// read data...(8 bits)
	PORTD |= (1 << actl_s->CS | 1 << FRD_PIN);		// set CS high again
	delayMicroseconds(4);

		//#ifdef DEBUG Serial.println("Read SRAM Section");

// #ifdef DEBUG
// 		 {Serial.println("READ data =");Serial.println(data,HEX);}
// #endif

	return data;
}

/*
		Write a byte to the SRAM at the specified address.
		Adress > $8000
	*/

void MemChip::write(unsigned long address, unsigned int indata)
{
	DataBus.gpioPinMode(OUTPUT);	// all signals output

	PORTD |= (1 << FWR_PIN );	  //*WE high (init)
	PORTD &= ~(1 << actl_d->CS);						  // CE LOW (init)  destination chip

	//set address pins
	setAddress(address);


	switch (actl_d->type)
	{
		case SRAM:
		{
			DataBus.gpioRegisterWriteByte(MCP23S17_GPIO|1, indata);
			delayMicroseconds(6);
			//write pulse Tas
			PORTD &= ~(1 << FWR_PIN); //WE# (pin 29 low)
			delayMicroseconds(6);
			break;
		}	
		case FLASH:
			// prog cycle for SST39SF040 programming WE# controlled timing 
		{

			PORTD |= (1 << FRD_PIN); 						// OE HIGH (init)

			pulse_SST39(0x5555, 0xAA);
			pulse_SST39(0x2AAA, 0x55);
			pulse_SST39(0x5555, 0xA0);
			pulse_SST39(address, indata, 30); // delay 20 µsec
			break;
		}
	}

	PORTD |= (1 << FWR_PIN  | 1 << actl_d->CS );	  //WE# (pin 29 high)	  //CS high  destination chip
}


void MemChip::pulse_SST39(unsigned long address, unsigned int indata, unsigned int p_width)
{
	//set address pins
	setAddress(address);
	
	DataBus.gpioRegisterWriteByte(MCP23S17_GPIO|1,indata);

	delayMicroseconds(4);
	//write pulse Tas
	PORTD &= ~(1 << FWR_PIN); //WE# (pin 27/31 low)
	delayMicroseconds(4);
	PORTD |= (1 << FWR_PIN); //WE# (pin 27/31 high)
	delayMicroseconds(p_width);
}

/*
		Read the contents of the MEM and print them to the Serial1 monitor.
	*/
int MemChip::ex_listCommand()
{
	// char buf1[250];
	// char buf[120];
	char d_h;
	uint32_t j_addr;
	uint16_t j_bsize;
	byte ascii_t;
	char data[20];
	byte char_data[20];
	unsigned char CharChecksum = 0;
	unsigned long mainChecksum = 0;

#ifdef DEBUG
		Serial.println("List (1)");
#endif

	setup_wait_DMA_signals();
	//print mem contents
	readLong(&startaddress);
	readWord(&body_size);

	// sprintf(buf, "Memorydump startaddress:,   Size:\n");
	sprintf(buf, "Dump memory from address: %6lX,Size :%6X\n", startaddress, (int)body_size);
	mySerial->write(buf);

	setActlSMemChip(startaddress);

	j_addr = startaddress & 0xFFFFF0;
	j_bsize = body_size | 0x000F;
	CharChecksum = 0;
	mainChecksum = 0;

	for (unsigned int base = 0; base < j_bsize; base += 16)
	{
		for (int offset = 0; offset <= 15; offset ++)
		{
			data[offset] = actl_s->readMEM(j_addr + base + offset);
			ascii_t = data[offset] & 0XFF;
		
			if (ascii_t > 32 and ascii_t < 126)
				d_h = ascii_t;
			else d_h= '.';	
			
			// if (ascii_t == 0) d_h = 0x5F;

			char_data[offset] = d_h;
			
			mainChecksum += (data[offset] & 0xFF);
		}
		data[16]='\n'; 			// set newline
		char_data[16]='\n';		// set newline
		// memset(buf, 0, 120);
		// memset(buf1, 0, 250);

		// sprintf(buf, "%06lX:  %02X%02X %02X%02X %02X%02X %02X%02X     ",
		// 		base + j_addr, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);

		// strcat(buf1, buf);
		// sprintf(buf, "%02X%02X %02X%02X %02X%02X %02X%02X   ",
		// 		data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);
		// strcat(buf1, buf);


		// sprintf(buf, "| %c%c %c%c %c%c %c%c   %c%c %c%c %c%c %c%c |\n",
		// 		char_data[0], char_data[1], char_data[2], char_data[3], char_data[4], char_data[5], char_data[6], char_data[7],
		// 		char_data[8], char_data[9], char_data[10], char_data[11], char_data[12], char_data[13], char_data[14], char_data[15]);

		// strcat(buf1, buf);
		mySerial->write(data,20);
		memset(data, 0, 20);
		mySerial->write(char_data,20);
		memset(char_data, 0, 20);

	}

	CharChecksum = (char)(~(mainChecksum & 0x000000FF));
	sprintf(buf, " CHK: %4X\n",CharChecksum);
	mySerial->write(buf,20);

	mySerial->write("EOL\n",20);
	return 0;
}
