/*
 * This program mimics MCP23008 8-bit IO Expander with I2C interface.
 * Only a subset of MCP23008 registers are emulated. Interrupts are not,
 * neither was it possible to map pins 1:1 - this is handled by pins table
 * mapping.
 * Tested against personal use scenario with 8-module relay switch and
 * reed switch.
 * I2C operation may differ.
 * I2C: A4 (SDA) and A5 (SCL).
 */

#include "Arduino.h"

#include <Wire.h>

/* #define DEBUG_ON */

#define MCP23008_BASE_ADDR		0x20

/*
  MCP23008 registers in detail
  http://ww1.microchip.com/downloads/en/DeviceDoc/21919e.pdf
*/
#define _IODIR     0x00
#define _IPOL      0x01
#define _GPINTEN   0x02
#define _DEFVAL    0x03
#define _INTCON    0x04
#define _IOCON     0x05
#define _GPPU      0x06
#define _INTF      0x07
#define _INTCAP    0x08
#define _GPIO      0x09
#define _OLAT      0x0A

/* Buffers MCP23008 registers */
uint8_t mcp23008_regs[_OLAT+1] = { 0 };

/*
  Basic version of this program will be compatible with 8 channel expander.
  These numbers correspond to D2..D9 pins of arduino nano
*/
uint8_t data_pins[8] = { 2, 3, 4, 5, 6, 7, 8, 9 };
/*
  Pins for setting i2c address on bus
  with addr 0  1  0 ' 0  A2, A1, A0
                          ^   ^  ^
*/
uint8_t addr_pins[8] = { 10, 11, 12 };
uint8_t mcp23008_addr = MCP23008_BASE_ADDR;

void init_regs(void);
void write_reg(uint8_t reg, uint8_t val);
uint8_t read_reg(uint8_t reg);
uint8_t decode_addr(void);

#ifdef DEBUG_ON
#define DEBUG(txt) Serial.println(txt)
#else
#define DEBUG(txt)
#endif

int cmd_addr = 0;
int led = LED_BUILTIN;

void init_regs(void)
{
	int reg;
	/* set all pins' direction to input state */
	mcp23008_regs[_IODIR] = 0xFF;
	/* relay switch is controled with state LOW so
	   make sure all pins are set to HIGH */
	mcp23008_regs[_GPIO] = 0xFF;
	/* it is required to update MCP23008 registers to their defaults */
	for (reg = _IODIR; reg <= _OLAT; reg++)
		write_reg(reg, mcp23008_regs[reg]);

}

/* Keep value for given register in */
void write_reg(uint8_t reg, uint8_t val)
{
	uint8_t i;

	mcp23008_regs[reg] = val;

	switch (reg) {
	case _IODIR:
		DEBUG("==== IODIR");
		for (i = 0; i < 8; i++) {
			pinMode(data_pins[i], (val & (1<<i)) ? INPUT : OUTPUT);
			DEBUG(i);
			DEBUG((val & (1<<i)) ? "INPUT" : "OUTPUT");
		}
		break;

	case _GPPU:
		DEBUG("==== GPPU");
		for (i = 0; i < 8; i++) {
			/* if IODIR is as input then if requested set pull-up*/
			pinMode(data_pins[i], (val & (1<<i)) ? INPUT_PULLUP : INPUT);
			DEBUG(i);
			DEBUG((val & (1<<i)) ? "INPUT" : "INPUT_PULLUP");
		}
		break;

	case _IPOL:
		/* don't break, need to reset ping according to new polarity */
	case _GPIO:
		DEBUG("==== GPIO");
		for (i = 0; i < 8; i++) {
			/* if IPOL bit set for given pin use inverted logic */
			if (mcp23008_regs[_IPOL] & (1<<i)) {
				digitalWrite(data_pins[i], (val & (1<<i)) ? LOW : HIGH);
				DEBUG(i);
				DEBUG((val & (1<<i)) ? "LOW" : "HIGH");
			} else {
				digitalWrite(data_pins[i], (val & (1<<i)) ? HIGH : LOW);
				DEBUG(i);
				DEBUG((val & (1<<i)) ? "HIGH" : "LOW");

			}
		}
		break;
	}
}

uint8_t read_reg(uint8_t reg)
{
	uint8_t val;
	uint8_t i;

	/*
	  for all other registers we hold state in mcp23008_regs[], but for
	  gpio this must be actually read (only if INPUT?)
	*/
	if (reg == _GPIO || reg == _OLAT) {
		for (i = 0; i < 8; i++) {
			val = digitalRead(data_pins[i]);
			if (val)
				mcp23008_regs[reg] |= (1<<i);
			else
				mcp23008_regs[reg] &= ~(1<<i);
		}
	}
	DEBUG("Reading regs");
	DEBUG(mcp23008_regs[reg]);
	return mcp23008_regs[reg];
}


void receiveEvent(int num)
{
	int val;
	cmd_addr = Wire.read();
	/*
	  I don't know how to validate R/W bit so I assume that is we
	  have more than 1 param  then it is write - this works for basic
	  get/set I2C operation
	*/
	if (num > 1) {
		val = Wire.read();
		write_reg(cmd_addr, val);
	}
}

void requestEvent()
{
	Wire.write(read_reg(cmd_addr));
}

uint8_t decode_addr(void)
{
	pinMode(addr_pins[0], INPUT);
	pinMode(addr_pins[1], INPUT);
	pinMode(addr_pins[2], INPUT);
	return MCP23008_BASE_ADDR
		| digitalRead(addr_pins[0]) << 2  /* A2 */
		| digitalRead(addr_pins[1]) << 1  /* A1 */
		| digitalRead(addr_pins[2]) << 0; /* A0 */
}

void setup()
{
	uint8_t i2c_addr = decode_addr();
	Serial.begin(9600);
	DEBUG("In setup");

	pinMode(led, OUTPUT);
	init_regs();
	mcp23008_regs[_GPIO] = 0xFF;
	write_reg(_GPIO, mcp23008_regs[_GPIO]);

	Wire.begin(i2c_addr);
	Wire.onRequest(requestEvent);
	Wire.onReceive(receiveEvent);
}

void loop()
{
	delay(1000);

}
