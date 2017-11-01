# ard-mcp
Arduino with MCP23008 interface

This program mimics MCP23008 8-bit IO Expander with I2C interface.
Only a subset of MCP23008 registers are emulated. Interrupts are not,
neither was it possible to map pins 1:1 - this is handled by pins table
mapping.
Tested against personal use scenario with 8-module relay switch and
reed switch.
