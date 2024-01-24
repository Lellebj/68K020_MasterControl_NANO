#include <Arduino.h>


#include <SPI.h>
#include <gpio_MCP23S17.h>   // import library


#define FRD_PIN 	6	//D6=PD6	all chips *OE (chip pin 24)
#define FWR_PIN 	7 	//D7=PD7	SRAM *WE (pin29), FLASH PGM(pin 31)
#define FLASH1_CE_PIN 2 //D2=PD2	FLASH1 *CE (pin 22)
#define FLASH2_CE_PIN 3 //D3=PD3	FLASH2 *CE (pin 22)
#define SRAM1_CE_PIN  4	//D4=PD4	SRAM  *CE (pin 22)
#define SRAM2_CE_PIN  5	//D4=PD5	SRAM  *CE (pin 22)
#define BR_PIN 		9	//PB1		68020 pinA2 (orange) Bus request - (Set LOW to request DMA!)
#define BG_PIN 		PORTB0	//PB0		68020 pinB2 (red)  Bus Acknowledge

#define RST_PIN		14	//PD0		Reset of 23S17 chips (pin18)  Chip reset (put in High imp.)
#define CPU_RST		16	//PD0		Executes CPU Reset for 68EC020 (put in High imp. when not in use)



enum memType {SRAM, FLASH};

//****************************************************

char *ptr = NULL;
char output[15];


class MemChip
{
private:
	/* data */
	byte 	adrHigh;	

public:
	memType	type;
    byte    A_Range;   // high or low chip  (A19 high or low)
    byte    CS;
	byte  	aCB_value;
	
	MemChip(memType tp, byte pin);
	~MemChip();
	void setAddress(unsigned long adr);
	unsigned int readMEM(unsigned long address);
	int ex_listCommand();
	int ex_byteCommand();
	int ex_longWordCommand();
	int ex_pokeCommand();
	int ex_getSRecordsCommand();
	int ex_reqDebugMSG();
	int ex_binaryToMEMcommand();


	void pulse_SST39(unsigned long address, unsigned int indata, unsigned int p_width=1);
	void ex_resetSRAMCommand();
	void write(unsigned long address, unsigned int indata);
	int ex_SectorEraseFLASHSST39();
	int ex_ChipEraseFLASHSST39();
	int ex_CopyMemToMem();
};

	MemChip::MemChip(memType tp, byte pin)
	{
		// HL is high or low chip A19 high or low  
		CS = pin;   
		type = tp;

	};

	MemChip::~MemChip()
	{};



