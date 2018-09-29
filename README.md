# thermometer-LM35-TM1637
Digital LED thermometer on ATTINY13 + LM35 and TM1637 4-digit LED Arduino module. 
Uses analog temperature sensor LM35 for 0-100C degrees measurement range.

How to connect

LM35 Vout to ATTINY13 PB4 pin
TM1637 DIO_PIN to ATTINY13 PB0 pin
TM1637 CLK_PIN to ATTINY13 PB1 pin

VCC (5V from USB or LM7805 stabilizer) and GND must be connected to all : LM35, ATTINY13 and TM1637 module

5V from VCC is used as a reference for LM35/TM1637 measurement so must be stable for good results.
