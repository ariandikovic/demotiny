/*-------------------------------
This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>
--------------------------------
	arian@kset.org

WARNING !! ->> MISO and MOSI on TINY series of microcontrolers is not the same as DO and DI !!!
//It is reversed

//note to myself : think about pin direction !
--------------------------------*/


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


unsigned char scratch [15];	//scratchpad!

static inline void spiInit (void) //initialise hardware SPI interface
{
PORTB = PORTB | (1<< PB4);//set SS high, do not touch other bits
return;
}

static inline void attinyinit (void) //microcontroller peripherial setup
{
DDRB = 0b11010000;			//PB7-SCK, PB6-D0,PB4-SS output, PB5-DI input, PB3-PB0 inputs
PORTB = PORTB | 0b00001111; //activate pull-up on PB3-PB0
//use logical OR to activate pull up where needed, leave the rest alone

DDRD = 0b00011000; //PD7-non existent,PD6 and PD5 inputs,PD4 and PD3 outputs,PD2-PD0 inputs
PORTD = PORTD | 0b01100011; //pull up on PD6,PD5,PD1,PD0

PORTD = PORTD | (1<< PD3);				//set PD3 high, LED RED off
PORTD = PORTD | (1<< PD4);				//set PD4 high, LED GREEN off

MCUCR = 0x00; 	//low level INT0 generates interrupt
GIMSK = 0b01000000; //allow interrupts from INT0
return;
}

void spiWrite (unsigned char addr,unsigned char data) //writes data to address addr to SPI
{
PORTB = PORTB & (~(1<< PB4));	//SS low, leave others

USIDR = (addr | (1<<7));		//SPI address for write + MSB=1 for write
USISR = (1<<USIOIF); 
   do { 
      USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC); 
   } while ((USISR & (1<<USIOIF)) == 0);

USIDR = data;	//load data to USIDR
USISR = (1<<USIOIF); 	//write to USIOIF to clear it and reset the counter
do { 
      USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC); 
   } while ((USISR & (1<<USIOIF)) == 0);

PORTB = PORTB | (1<< PB4);		//set SS high
return;
}

void spiWriteBurst (char addr,unsigned char *buf,unsigned char broj) //writes to SPI bus in BURST mode
{	//needs start address and pointer to data, number of bytes in data field
unsigned char i;				//for loop
PORTB = PORTB & (~(1<< PB4));	//SS low, leave others
USIDR = (addr | (1<<7));		//SPI address for write + MSB=1 for write

USISR = (1<<USIOIF); //wait for byte to SPI transfer--atmel datasheet!!
   do { 
      USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC); 
   } while ((USISR & (1<<USIOIF)) == 0);

for(i=0 ; i<broj; i++){ 		//loop to write all bytes starting from addr
USIDR=buf[i]; 					//write dat from scratch to USIDR
USISR = (1<<USIOIF); //wait for byte to SPI transfer--atmel datasheet!!
   do { 
      USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC); 
   } while ((USISR & (1<<USIOIF)) == 0);
}
PORTB = PORTB | (1<< PB4);		//set SS high
return;
}


unsigned char spiRead (unsigned char addr) //read from address
{
PORTB = PORTB & (~(1<< PB4));	//SS low, leave others

USIDR = (addr & 0b01111111);		//start SPI adress for burst read + MSB=0 for read
USISR = (1<<USIOIF); 	//write to USIOIF to clear it and reset the counter
do { 
      USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC); 
   } while ((USISR & (1<<USIOIF)) == 0);

USIDR = 0; //just some data, will ignore it
USISR = (1<<USIOIF); 	//write to USIOIF to clear it and reset the counter
do { 
      USICR = (1<<USIWM0)|(1<<USICS1)|(1<<USICLK)|(1<<USITC); 
   } while ((USISR & (1<<USIOIF)) == 0);

PORTB = PORTB | (1<< PB4);		//set SS high
return USIDR;					//return data from address addr
}

static inline void RFM22init (void) //init RFM22 module
{
_delay_ms(150);			//wait 150 ms, init...
spiWrite(0x07, 0x80);		//software reset RFM22
_delay_ms(150);			//wait another 150 ms to be sure

spiWrite(0x05, 0x00);	//disable all interrupts on RFM
spiWrite(0x06, 0x00);	//disable all interrupts on RFM

spiRead (0x03);//read interrupt status 1 registar, to clear interrupt flag if set
spiRead (0x04);//read interrupt status 2 registar, to clear interrupt flag if set

spiWrite(0x07, 0x01);	//to READY mode

//Si4432 V2 silicon specific -> silicon labs AN415 (rev 0.6) str. 24
spiWrite(0x5A, 0x7F); //write 0x7F to the VCO Current Trimming register
spiWrite(0x58, 0x80); //write 0xD7 to the ChargepumpCurrentTrimmingOverride register
spiWrite(0x59, 0x40); //write 0x40 to the Divider Current Trimming register
//best receiver performances setup
spiWrite(0x6A, 0x0B); //write 0x0B to the AGC Override 2 register,RSSI readout correction
spiWrite(0x68, 0x04); //write 0x04 to the Deltasigma ADC Tuning 2 register
spiWrite(0x1F, 0x03); //write 0x03 to the Clock Recovery Gearshift Override register

spiWrite(0x09, 0x7B);////XTAL correction for TINY P1
spiWrite(0x0A, 0x00);//GPIO clock izlaz 30 Mhz, no clock tail, no Low freq clock
spiWrite(0x0B, 0xD2);//GPIO 0 - strong drive (HH), no pullup, TX state
spiWrite(0x0C, 0xD5);//GPIO 1 - strong drive (HH), no pullup, RX state
spiWrite(0x0D, 0x00);//GPIO 2 - strong drive (HH), no pullup, CLK output

spiWrite(0x0F, 0x70);//ADC input ->GND
spiWrite(0x10, 0x00);//ADC offset ->0
spiWrite(0x12, 0x00);//temp. sensor calibration off
spiWrite(0x13, 0x00);//temp. sensor offset ->0

spiWrite(0x1C, 0x04);//IF filter bandwith -> RFM datasheet str. 44
spiWrite(0x1D, 0x40);//AFC enable
spiWrite(0x1E, 0x05);//AFC timing -> ?

spiWrite(0x20, 0xC8);//clock recovery oversampling
spiWrite(0x21, 0x00);//clock recovery offset 2
spiWrite(0x22, 0xA3);//clock recovery offset 1
spiWrite(0x23, 0xD7);//clock recovery offset 0
spiWrite(0x24, 0x00);//clock recovery timing loop 1
spiWrite(0x25, 0xA6);//clock recovery timing loop 0

spiWrite(0x30, 0x8E);//CRC-16 on,TX packet handling on,CRC over entire packet,RX packet handling on
spiWrite(0x32, 0x00);//no header check
spiWrite(0x33, 0x02);//NO header, sync word 3 and 2 ON -> 2D, D4, variable packet lenght on
spiWrite(0x34, 0x10);//preamble 16 nibbles ->64 bits
spiWrite(0x35, 0x30);//preamble detection 4 nibbles -> 24 bits
spiWrite(0x36, 0x2D);//sync word 3
spiWrite(0x37, 0xD4);//sync word 2
spiWrite(0x69, 0x20);//AGC enable
//-----------------------------------------------------
spiWrite(0x3E, 0x05);//packet lenght 5 bytes (payload)
spiWrite(0x6D, 0x00);//output power 8 dBm

spiWrite(0x70, 0x21);//whitening ON, DATA RATE under 30 kbps!!!- > bit 5 SET!!!!!
spiWrite(0x71, 0x23);//GFSK, FIFO mode

spiWrite(0x72, 0x40);//freq. dev. 40 khz

spiWrite(0x6E, 0xA3);//TX data rate  1-> 20 kbps
spiWrite(0x6F, 0xD7);//TX data rate  0-> 20 kbps

spiWrite(0x73, 0x00);//no frequency offset
spiWrite(0x74, 0x00);//no frequency offset
spiWrite(0x79, 0x00);//no frequency hopping
spiWrite(0x7A, 0x00);//no frequency hopping

spiWrite (0x75,0x53);//freq. band select 430-440 MHz

spiWrite (0x76,0x62);//carrier 433.92 MHz
spiWrite (0x77,0x00);//carrier 433.92 MHz

spiWrite(0x05, 0x04);//interrupt enable 1 register-packet sent

spiWrite (0x08,0x01);//clear TX FIFO
spiWrite (0x08,0x00);//clear TX FIFO

return;
}

unsigned char getDIPvalue (void)
{
unsigned char a,value = 0;
a=PIND;	//read PORTD
a=a<<1,		//shift left a
a=a & 0b11000000;//leave 2 highest bits (PD6,PD7)
value=value | a; //paste it to value

a=PIND;	//read PORTD
a=a<<4,		//shift 4 places left
a=a & 0b00110000;//leave 2 bits
value=value | a; //paste to value

a=PINB;//read PORTB
a=a & 0b00001111;//leave lowest 4 bita (PB0-PB3)
value=value | a; //paste to  value

value = value ^ 0xFF; //invert bits -> when switch ON pin is read as 0!!!


if (value==0) value++;// if no switch ON default is 1 sec
return value;
}


volatile unsigned char flag; //so ISR can modify flag

ISR(INT0_vect) 
{ 
spiWrite (0x07,0x41);//enable READY mode and battery readout
spiRead (0x03);//read interrupt/status register 1
spiRead (0x04);//read interrupt/status register 2
flag=1;
}

int main (void)
{
	unsigned char i;
	attinyinit ();	//init microcontroller peripherials
	spiInit ();		// init SPI
	RFM22init ();	//init RFM 
	
	sei ();	//global interrupt enable
	spiWrite (0x07,0x41);//enable READY mode and battery readout

while (1)
{
	flag=0;
	scratch [0]=2;		//write node ID = 2
	i=spiRead (0x1B);	//read battery status
	scratch [1]=i;		//write batt. status u scratch
	scratch[2]=0xBB;	//write DAT1=0xBB
	i=getDIPvalue();	//read DIP switcha
	scratch[3]=i;		//write DAT2=dip switch
	scratch[4]=0xBB;	//write DAT3=0xBB
	
	spiWriteBurst(0x7F,scratch,5);//to address 7F goes scratch, writing 5 bytes
	
	spiWrite (0x07,0x49);//start TX with battery readout enabled
	
	while (!flag);
	
	spiWrite (0x08,0x01);//clear TX FIFO
	spiWrite (0x08,0x00);//clear TX FIFO

	PORTD = PORTD & (~(1<< PD4));//pin PD4 low, green ON
	_delay_ms(300);				//wait 0.3 sek
	PORTD = PORTD | (1<< PD4);	//pin PD4 high, green OFF
	i= getDIPvalue ();		//read DIP switch value
	for(; i>0; i--)  _delay_ms(1000);
			
}
	return 1;

		}

