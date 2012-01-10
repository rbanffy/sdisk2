/*------------------------------------------------------

	DISK II Emulator Farmware (1 of 2) for ATMEGA328P
	
	original:	2009.11.16 by Koichi Nishida
	version 1.3	2011. 1.28 by Koichi Nishida

------------------------------------------------------*/

/*
if the crystal on your SDISK II is 25 MHz, 
I recommend you to replace it with 27 MHz,
or ask Nishida Radio.
see also sub.S
*/

/*
hardware information:

use ATMEGA328P AVR.
connect 27MHz (overclock...) crystal to the AVR.
supply 3.3V power.

fuse setting : LOW 11011110
connection:

	D0: DO (SD card)
	D1: CS (SD card)
 	D2: WRITE REQUEST (APPLE II disk IF, pull up with 10K ohm)
	D3: EJECT SWITCH (LOW if SD card is inserted)
	D4: DI (SD card)
	D5: CLK (SD card)
	D6-D7: NC
	B0: PHASE-0 (APPLE II disk IF)
	B1: PHASE-1 (APPLE II disk IF)
	B2: PHASE-2 (APPLE II disk IF)
	B3: PHASE-3 (APPLE II disk IF)
	B4: LED (through 330 ohm)
	B5: NC
	B6-B7: connect to the crystal
	C0: DRIVE ENABLE (APPLE II disk IF)
	C1: READ PULSE (APPLE II disk IF through 74HC125 3state)
	C2: WRITE (APPLE II disk IF)
	C3: WRITE PROTECT (APPLE II disk IF through 74HC125 3state)
	C4-C6: NC
	
	Note that the enable input of the 3state buffer 74HC125,
	should be connected with DRIVE ENABLE.
*/

/*
This is a part of the firmware for DISK II emulator by Nishida Radio.

Copyright (C) 2011 Koichi NISHIDA
email to Koichi NISHIDA: tulip-house@msf.biglobe.ne.jp

This program is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define WAIT 1
#define BUF_NUM 5
#define FAT_DSK_ELEMS 18
#define FAT_NIC_ELEMS 35
#define nop() __asm__ __volatile__ ("nop")

// C prototypes

// cancel read
void cancelRead(void);
// write a byte data to the SD card
void writeByteSlow(unsigned char c);
void writeByteFast(unsigned char c);
// read data from the SD card
unsigned char readByteSlow(void);
unsigned char readByteFast(void);
// wait until finish a command
void waitFinish(void);
// issue SD card command slowly without getting response
void cmd_(unsigned char cmd, unsigned long adr);
// issue SD card command fast and wait normal response
void cmdFast(unsigned char cmd, unsigned long adr);
// get command response slowly from the SD card
unsigned char getRespSlow(void);
// get command response fast from the SD card
unsigned char getRespFast(void);
// issue command 17 and get ready for reading
void cmd17Fast(unsigned long adr);
// find a file extension
int findExt(char *str, unsigned char *protect, unsigned char *name);
// prepare the FAT table on memory
void prepareFat(int i, unsigned short *fat, unsigned short len,
	unsigned char fatNum, unsigned char fatElemNum);
// memory copy	
void memcp(unsigned char *dst, unsigned char *src, const unsigned short len);
// duplicate FAT for FAT16
void duplicateFat(void);
// write to the SD cart one by one
void writeSD(unsigned long adr, unsigned char *data, unsigned short len);
// create a NIC image file
int createNic(unsigned char *name);
// translate a NIC image into a DSK image
void nic2Dsk(void);
// translate a DSK image into a NIC image
void dsk2Nic(void);
// initialization called from check_eject
void init(void);
// called when the SD card is inserted or removed
void check_eject(void);
// write data back to a NIC image 
void writeBack(void);
void writeBackSub(void);
void writeBackSub2(unsigned char bn, unsigned char sc, unsigned char track);
// buffer clear
void buffClear(void);

// assembler functions
void wait5(unsigned short time);

// SD card information
unsigned long bpbAddr, rootAddr;
unsigned long fatAddr;					// the beginning of FAT
unsigned short fileFatTop;
unsigned char sectorsPerCluster, sectorsPerCluster2;	// sectors per cluster
unsigned short sectorsPerFat;	
unsigned long userAddr;					// the beginning of user data
// unsigned short fatDsk[FAT_DSK_ELEMS];// use writeData instead
unsigned short fatNic[FAT_NIC_ELEMS];
unsigned char prevFatNumDsk, prevFatNumNic;
unsigned short nicDir, dskDir;

// DISK II status
unsigned char ph_track;					// 0 - 139
unsigned char sector;					// 0 - 15
unsigned short bitbyte;					// 0 - (8*512-1)
unsigned char prepare;
unsigned char readPulse;
unsigned char inited;
unsigned char magState;
unsigned char protect;
unsigned char formatting;
const unsigned char volume = 0xfe;

// write data buffer
unsigned char writeData[BUF_NUM][350];
unsigned char sectors[BUF_NUM], tracks[BUF_NUM];
unsigned char buffNum;
unsigned char *writePtr;

// a table for head stepper moter movement 
PROGMEM prog_uchar stepper_table[4] = {0x0f,0xed,0x03,0x21};

// encode / decode table for a nib image
PROGMEM prog_uchar encTable[] = {
	0x96,0x97,0x9A,0x9B,0x9D,0x9E,0x9F,0xA6,
	0xA7,0xAB,0xAC,0xAD,0xAE,0xAF,0xB2,0xB3,
	0xB4,0xB5,0xB6,0xB7,0xB9,0xBA,0xBB,0xBC,
	0xBD,0xBE,0xBF,0xCB,0xCD,0xCE,0xCF,0xD3,
	0xD6,0xD7,0xD9,0xDA,0xDB,0xDC,0xDD,0xDE,
	0xDF,0xE5,0xE6,0xE7,0xE9,0xEA,0xEB,0xEC,
	0xED,0xEE,0xEF,0xF2,0xF3,0xF4,0xF5,0xF6,
	0xF7,0xF9,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF
};
PROGMEM prog_uchar decTable[] = {
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x02,0x03,0x00,0x04,0x05,0x06,
	0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x08,0x00,0x00,0x00,0x09,0x0a,0x0b,0x0c,0x0d,
	0x00,0x00,0x0e,0x0f,0x10,0x11,0x12,0x13,0x00,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1b,0x00,0x1c,0x1d,0x1e,
	0x00,0x00,0x00,0x1f,0x00,0x00,0x20,0x21,0x00,0x22,0x23,0x24,0x25,0x26,0x27,0x28,
	0x00,0x00,0x00,0x00,0x00,0x29,0x2a,0x2b,0x00,0x2c,0x2d,0x2e,0x2f,0x30,0x31,0x32,
	0x00,0x00,0x33,0x34,0x35,0x36,0x37,0x38,0x00,0x39,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f
};

// a table for translating logical sectors into physical sectors
PROGMEM prog_uchar physicalSector[] = {
		0,13,11,9,7,5,3,1,14,12,10,8,6,4,2,15};

// for bit flip
PROGMEM prog_uchar FlipBit[] = { 0,  2,  1,  3  };
PROGMEM prog_uchar FlipBit1[] = { 0, 2,  1,  3  };
PROGMEM prog_uchar FlipBit2[] = { 0, 8,  4,  12 };
PROGMEM prog_uchar FlipBit3[] = { 0, 32, 16, 48 };

// buffer clear
void buffClear(void)
{
	unsigned char i;
	unsigned short j;
	
	for (i=0; i<BUF_NUM; i++)
		for (j=0; j<350; j++)
			writeData[i][j]=0;
	for (i=0; i<BUF_NUM; i++)
		sectors[i]=tracks[i]=0xff;
}

// cancel read from the SD card
void cancelRead(void)
{
	unsigned short i;
	if (bitbyte<(402*8)) {
		PORTD = 0b00010000;
		for (i=bitbyte; i<(514*8); i++) {
			if (bit_is_set(PIND,3)) return;
			PORTD = 0b00110000;
			PORTD = 0b00010000;
		}
		bitbyte = 402*8;
	}
}

// write a byte data to the SD card
void writeByteSlow(unsigned char c)
{
	unsigned char d;
	for (d = 0b10000000; d; d >>= 1) {
		if (c&d) {
			PORTD = 0b00010000;
			wait5(WAIT);
			PORTD = 0b00110000;
		} else {
			PORTD = 0b00000000;
			wait5(WAIT);
			PORTD = 0b00100000;
		}
		wait5(WAIT);
	}
	PORTD = 0b00000000;
}
void writeByteFast(unsigned char c)
{
	unsigned char d;
	for (d = 0b10000000; d; d >>= 1) {
		if (c&d) {
			PORTD = 0b00010000;
			PORTD = 0b00110000;
		} else {
			PORTD = 0b00000000;
			PORTD = 0b00100000;
		}
	}
	PORTD = 0b00000000;
}

// read data from the SD card
unsigned char readByteSlow(void)
{
	unsigned char c = 0;
	volatile unsigned char i;

	PORTD = 0b00010000;
	wait5(WAIT);
	for (i = 0; i != 8; i++) {
		PORTD = 0b00110000;
		wait5(WAIT);
		c = ((c<<1) | (PIND&1));
		PORTD = 0b00010000;
		wait5(WAIT);	
	}
	return c;
}

unsigned char readByteFast(void)
{
	unsigned char c = 0;
	volatile unsigned char i;

	PORTD = 0b00010000;	
	for (i = 0; i != 8; i++) {
		PORTD = 0b00110000;
		c = ((c<<1) | (PIND&1));
		PORTD = 0b00010000;	
	}
	return c;
}
// wait until data is written to the SD card
void waitFinish(void)
{
	unsigned char ch;
	do {
		ch = readByteFast();
		if (bit_is_set(PIND,3)) return;
	} while (ch != 0xff);
}

// issue a SD card command slowly without getting response
void cmd_(unsigned char cmd, unsigned long adr)
{
	writeByteSlow(0xff);
	writeByteSlow(0x40+cmd);
	writeByteSlow(adr>>24);
	writeByteSlow((adr>>16)&0xff);
	writeByteSlow((adr>>8)&0xff);
	writeByteSlow(adr&0xff);
	writeByteSlow(0x95);
	writeByteSlow(0xff);
}

// issue a SD card command and wait normal response
void cmdFast(unsigned char cmd, unsigned long adr)
{
	unsigned char res;
	do {
		writeByteFast(0xff);
		writeByteFast(0x40+cmd);
		writeByteFast(adr>>24);
		writeByteFast((adr>>16)&0xff);
		writeByteFast((adr>>8)&0xff);
		writeByteFast(adr&0xff);
		writeByteFast(0x95);
		writeByteFast(0xff);
	} while (((res=getRespFast())!=0) && (res!=0xff));
}

// get a command response slowly from the SD card
unsigned char getRespSlow(void)
{
	unsigned char ch;
	do {
		ch = readByteSlow();
		if (bit_is_set(PIND,3)) return 0xff;
	} while ((ch&0x80) != 0);
	return ch;
}

// get a command response fast from the SD card
unsigned char getRespFast(void)
{
	unsigned char ch;
	do {
		ch = readByteFast();
		if (bit_is_set(PIND,3)) return 0xff;
	} while ((ch&0x80) != 0);
	return ch;
}

// issue command 17 and get ready for reading
void cmd17Fast(unsigned long adr)
{
	unsigned char ch;

	cmdFast(17, adr);
	do {	
		ch = readByteFast();
		if (bit_is_set(PIND,3)) return;
	} while (ch != 0xfe);
}

// find a file extension
int findExt(char *str, unsigned char *protect, unsigned char *name)
{
	short i;
	unsigned max_file = 512;
	unsigned short max_time = 0, max_date = 0;

	// find NIC extension
	for (i=0; i!=512; i++) {
		unsigned char ext[3], d;
		unsigned char time[2], date[2];
		
		if (bit_is_set(PIND,3)) return 512;
		// check first char
		cmdFast(16, 1);
		cmd17Fast(rootAddr+i*32);
		d = readByteFast();
		readByteFast(); readByteFast(); // discard CRC bytes
		if ((d==0x00)||(d==0x05)||(d==0x2e)||(d==0xe5)) continue;
		if (!(((d>='A')&&(d<='Z'))||((d>='0')&&(d<='9')))) continue;
		cmd17Fast(rootAddr+i*32+11);
		d = readByteFast();
		readByteFast(); readByteFast(); // discard CRC bytes
		if (d&0x1e) continue;
		if (d==0xf) continue;
		// check extension
		cmdFast(16, 4);
		cmd17Fast(rootAddr+i*32+8);
		ext[0] = readByteFast(); ext[1] = readByteFast(); ext[2] = readByteFast();
		if (protect) *protect = ((readByteFast()&1)<<3); else readByteFast();
		readByteFast(); readByteFast(); // discard CRC bytes
		
		
		// check time stamp
		cmdFast(16, 4);
		cmd17Fast(rootAddr+i*32+22);
		time[0] = readByteFast(); time[1] = readByteFast();
		date[0] = readByteFast(); date[1] = readByteFast();
		readByteFast(); readByteFast(); // discard CRC bytes
		if ((ext[0]==str[0])&&(ext[1]==str[1])&&(ext[2]==str[2])) {
			unsigned short tm = *(unsigned short *)time;
			unsigned short dt = *(unsigned short *)date;

			if ((dt>max_date)||((dt==max_date)&&(tm>=max_time))) {
				max_time = tm;
				max_date = dt;
				max_file = i;
			}
		}
	}
	if ((max_file != 512) && (name != 0)) {
		unsigned char j;
		cmdFast(16, 8);
		cmd17Fast(rootAddr+max_file*32);
		for (j=0; j<8; j++) name[j] = readByteFast();
		readByteFast(); readByteFast();
	}
	return max_file;
	// if 512 then not found...
}

// prepare a FAT table on memory
void prepareFat(int i, unsigned short *fat, unsigned short len,
	unsigned char fatNum, unsigned char fatElemNum)
{
	unsigned short ft;
	unsigned char fn;

	if (bit_is_set(PIND,3)) return;
	cmdFast(16, (unsigned long)2);
	cmd17Fast(rootAddr+i*32+26);
	ft = readByteFast();
	ft += (unsigned short)readByteFast()*0x100;
	readByteFast(); readByteFast(); // discard CRC bytes
	if (0==fatNum) fat[0] = ft;
	for (i=0; i<len; i++) {
		fn = (i+1)/fatElemNum;
		cmd17Fast((unsigned long)fatAddr+(unsigned long)ft*2);
		ft = readByteFast();
		ft += (unsigned short)readByteFast()*0x100;
		readByteFast(); readByteFast(); // discard CRC bytes
		if (fn==fatNum) fat[(i+1)%fatElemNum] = ft;
		if ((ft>0xfff6)||(fn>fatNum)) break;
	}
	cmdFast(16, (unsigned long)512);	
}

// memory copy
void memcp(unsigned char *dst, unsigned char *src, unsigned short len)
{
	unsigned short i;
	
	for (i=0; i<len; i++) dst[i]=src[i];
}

void writeSD(unsigned long adr, unsigned char *data, unsigned short len)
{
	unsigned int i;
	unsigned char *buf = &writeData[0][0];

	if (bit_is_set(PIND,3)) return;

	cmdFast(16, 512);
	cmd17Fast(adr&0xfffffe00);
	for (i=0; i<512; i++) buf[i] = readByteFast();
	readByteFast(); readByteFast(); // discard CRC bytes
	memcp(&(buf[adr&0x1ff]), data, len);
	
	PORTD = 0b00000010;
	PORTD = 0b00000000;	
				
	cmdFast(24,adr&0xfffffe00);		
	writeByteFast(0xff);
	writeByteFast(0xfe);
	for (i=0; i<512; i++) writeByteFast(buf[i]);
	writeByteFast(0xff);
	writeByteFast(0xff);
	readByteFast();
	waitFinish();
	
	PORTD = 0b00000010;
	PORTD = 0b00000000;	
}

void duplicateFat(void)
{
	unsigned short i, j;
	unsigned long adr = fatAddr;
	unsigned char *buf = &writeData[0][0];

	if (bit_is_set(PIND,3)) return;

	cmdFast(16, 512);
	for (j=0; j<sectorsPerFat; j++) {
		cmd17Fast(adr);
		for (i=0; i<512; i++) buf[i] = readByteFast();
		readByteFast(); readByteFast(); // discard CRC bytes

		PORTD = 0b00000010;
		PORTD = 0b00000000;	
		
		cmdFast(24,adr+(unsigned long)sectorsPerFat*512);		
		writeByteFast(0xff);
		writeByteFast(0xfe);
		for (i=0; i<512; i++) writeByteFast(buf[i]);
		writeByteFast(0xff);
		writeByteFast(0xff);
		readByteFast();
		waitFinish();
		adr += 512;
		
		PORTD = 0b00000010;
		PORTD = 0b00000000;	
	}
}

// create a NIC image file
int createNic(unsigned char *name)
{
	unsigned short re, clusterNum;
	unsigned long ft, adr;
	unsigned short d, i;
	unsigned char c, dirEntry[32], at;
	static unsigned char last[2] = {0xff, 0xff};

	if (bit_is_set(PIND,3)) return 0;
	
	for (i=0; i<32; i++) dirEntry[i]=0;
	memcp(dirEntry, name, 8);
	memcp(dirEntry+8, (unsigned char *)"NIC", 3);
	*(unsigned long *)(dirEntry+28) = (unsigned long)286720;
	
	// search a root directory entry
	for (re=0; re<512; re++) {
		cmdFast(16, 1);
		cmd17Fast(rootAddr+re*32+0);
		c = readByteFast();
		readByteFast(); readByteFast(); // discard CRC bytes
		cmd17Fast(rootAddr+re*32+11);
		at = readByteFast();
		readByteFast(); readByteFast(); // discard CRC bytes
		if (((c==0xe5)||(c==0x00))&&(at!=0xf)) break;  // find a RDE!
	}	
	if (re==512) return 0;
	// write a directory entry
	writeSD(rootAddr+re*32, dirEntry, 32);	
	// search the first fat entry
	adr = (rootAddr+re*32+26);
	clusterNum = 0;
	for (ft=2;
		(clusterNum<((560+sectorsPerCluster-1)>>sectorsPerCluster2)); ft++) {
		cmdFast(16, 2);
		cmd17Fast(fatAddr+ft*2);
		d = readByteFast();
		d += (unsigned short)readByteFast()*0x100;
		readByteFast(); readByteFast(); // discard CRC bytes
		if (d==0) {
			clusterNum++;
			writeSD(adr, (unsigned char *)&ft, 2);
			adr = fatAddr+ft*2;
		}
	}
	writeSD(adr, last, 2);
	duplicateFat();
	return 1;
}

// translate a DSK image into a NIC image
void dsk2Nic(void)
{
	unsigned char trk, logic_sector;

	unsigned short i;
	unsigned char *dst = (&writeData[0][0]+512);
	unsigned short *fatDsk = (unsigned short *)(&writeData[0][0]+1024);

	PORTB |= 0b00010000;

	prevFatNumNic = prevFatNumDsk = 0xff;

	for (i=0; i<0x16; i++) dst[i]=0xff;

	// sync header
	dst[0x16]=0x03;
	dst[0x17]=0xfc;
	dst[0x18]=0xff;
	dst[0x19]=0x3f;
	dst[0x1a]=0xcf;
	dst[0x1b]=0xf3;
	dst[0x1c]=0xfc;
	dst[0x1d]=0xff;
	dst[0x1e]=0x3f;
	dst[0x1f]=0xcf;
	dst[0x20]=0xf3;
	dst[0x21]=0xfc;	
	
	// address header
	dst[0x22]=0xd5;
	dst[0x23]=0xaa;
	dst[0x24]=0x96;
	dst[0x2d]=0xde;
	dst[0x2e]=0xaa;
	dst[0x2f]=0xeb;
	
	// sync header
	for (i=0x30; i<0x35; i++) dst[i]=0xff;
	
	// data
	dst[0x35]=0xd5;
	dst[0x36]=0xaa;
	dst[0x37]=0xad;
	dst[0x18f]=	0xde;
	dst[0x190]=0xaa;
	dst[0x191]=0xeb;
	for (i=0x192; i<0x1a0; i++) dst[i]=0xff;
	for (i=0x1a0; i<0x200; i++) dst[i]=0x00;	

	cmdFast(16, (unsigned long)512);	
	for (trk = 0; trk < 35; trk++) {
		PORTB ^= 0b00010000;
		for (logic_sector = 0; logic_sector < 16; logic_sector++) {
			unsigned char *src;
			unsigned short ph_sector = (unsigned short)pgm_read_byte_near(physicalSector+logic_sector);

			if (bit_is_set(PIND,3)) return;

			if ((logic_sector&1)==0) {
				unsigned short long_sector = (unsigned short)trk*8+(logic_sector/2);
				unsigned short long_cluster = long_sector>>sectorsPerCluster2;
				unsigned char fatNum = long_cluster/FAT_DSK_ELEMS;
				unsigned short ft;

				if (fatNum != prevFatNumDsk) {
					prevFatNumDsk = fatNum;						
					prepareFat(dskDir, fatDsk,
						(280+sectorsPerCluster-1)>>sectorsPerCluster2, fatNum, FAT_DSK_ELEMS);	
				}
				ft = fatDsk[long_cluster%FAT_DSK_ELEMS];
				cmd17Fast((unsigned long)userAddr+(((unsigned long)(ft-2)<<sectorsPerCluster2)
					+ (long_sector&(sectorsPerCluster-1)))*(unsigned long)512);
				for (i=0; i<512; i++) {
					if (bit_is_set(PIND,3)) return;
					*(&writeData[0][0]+i)=readByteFast();
				}
				readByteFast(); readByteFast(); // discard CRC bytes				
				src = &writeData[0][0];
			} else {
				src = (&writeData[0][0]+256);
			}
			{
				unsigned char c, x, ox = 0;

				dst[0x25]=((volume>>1)|0xaa);
				dst[0x26]=(volume|0xaa);
				dst[0x27]=((trk>>1)|0xaa);
				dst[0x28]=(trk|0xaa);
				dst[0x29]=((ph_sector>>1)|0xaa);
				dst[0x2a]=(ph_sector|0xaa);
				c = (volume^trk^ph_sector);
				dst[0x2b]=((c>>1)|0xaa);
				dst[0x2c]=(c|0xaa);
				for (i = 0; i < 86; i++) {
					x = (pgm_read_byte_near(FlipBit1+(src[i]&3)) |
						pgm_read_byte_near(FlipBit2+(src[i+86]&3)) |
						((i<=83)?pgm_read_byte_near(FlipBit3+(src[i+172]&3)):0));
					dst[i+0x38] = pgm_read_byte_near(encTable+(x^ox));
					ox = x;
				}
				for (i = 0; i < 256; i++) {
					x = (src[i] >> 2);
					dst[i+0x8e] = pgm_read_byte_near(encTable+(x^ox));
					ox = x;
				}
				dst[0x18e]=pgm_read_byte_near(encTable+ox);
			}
			{
				unsigned char c, d;
				unsigned short long_sector = (unsigned short)trk*16+ph_sector;
				unsigned short long_cluster = long_sector>>sectorsPerCluster2;
				unsigned char fatNum = long_cluster/FAT_NIC_ELEMS;
				unsigned short ft;
			
				if (fatNum != prevFatNumNic) {
					prevFatNumNic = fatNum;
					prepareFat(nicDir, fatNic,
						(560+sectorsPerCluster-1)>>sectorsPerCluster2, fatNum, FAT_NIC_ELEMS);
				}
				ft = fatNic[long_cluster%FAT_NIC_ELEMS];

				PORTD = 0b00000010;
				PORTD = 0b00000000;

				cmdFast(24, userAddr+(((unsigned long)(ft-2)<<sectorsPerCluster2)
					+ (long_sector&(sectorsPerCluster-1)))*(unsigned long)512);
				writeByteFast(0xff);
				writeByteFast(0xfe);
				for (i = 0; i < 512; i++) {
					register unsigned char D1=0b00010000, D2=0b00110000, D3=0b00000000, D4=0b00100000;
					if (bit_is_set(PIND,3)) return;
					c = dst[i];
					for (d = 0b10000000; d; d >>= 1) {
						if (c&d) {
							PORTD = D1;
							PORTD = D2;
						} else {
							PORTD = D3;
							PORTD = D4;
						}
					}
				}
				PORTD = 0b00000000;
				writeByteFast(0xff);
				writeByteFast(0xff);
				readByteFast();
				waitFinish();
				
				PORTD = 0b00000010;
				PORTD = 0b00000000;	
			}
		}
	}
	buffClear();
	PORTB &= 0b11101111;
}

// initialization called from check_eject
void init(void)
{
	unsigned char ch;
	unsigned short i;
	char str[5];
	unsigned char filebase[8];

	inited = 0;
	PORTB = 0b00110000;	// LED on

	// initialize the SD card
	PORTD = 0b00000010;	
	for (i = 0; i != 200; i++) {
		PORTD = 0b00110010; 
		wait5(WAIT);
		PORTD = 0b00010010;
		wait5(WAIT);
	 }	// input 200 clock
 	PORTD = 0b000000000;
	
	cmd_(0, 0);	// command 0
 	do {	
		if (bit_is_set(PIND,3)) return;	
		ch = readByteSlow();
	} while (ch != 0x01);

	PORTD = 0b00000010;
	while (1) {
		if (bit_is_set(PIND,3)) return;
		PORTD = 0b00000000;
		cmd_(55, 0);	// command 55
		ch = getRespSlow();
		if (ch == 0xff) return;
		if (ch & 0xfe) continue;
		// if (ch == 0x00) break;
		PORTD = 0b00000010;
		PORTD = 0b00000000;
		cmd_(41, 0);	// command 41	
		if (!(ch=getRespSlow())) break;
		if (ch == 0xff) return;
		PORTD = 0b00000010;
	}

	// BPB address
	cmdFast(16,5);
	cmd17Fast(54);
	for (i=0; i<5; i++) str[i] = readByteFast();
	readByteFast(); readByteFast(); // discard CRC
	if ((str[0]=='F')&&(str[1]=='A')&&(str[2]=='T')&&
		(str[3]=='1')&&(str[4]=='6')) {
		bpbAddr = 0;
	} else {
		cmdFast(16, 4);
		cmd17Fast((unsigned long)0x1c6);
		bpbAddr = readByteFast();
		bpbAddr += (unsigned long)readByteFast()*0x100;
		bpbAddr += (unsigned long)readByteFast()*0x10000;
		bpbAddr += (unsigned long)readByteFast()*0x1000000;
		bpbAddr *= 512;
		readByteFast(); readByteFast(); // discard CRC bytes
	}
	if (bit_is_set(PIND,3)) return;

	// sectorsPerCluster and reservedSectors
	{
		unsigned short reservedSectors;
		volatile unsigned char k;
		cmdFast(16, 3);
		cmd17Fast(bpbAddr+0xd);
		sectorsPerCluster = k = readByteFast();
		sectorsPerCluster2 = 0;
			while (k != 1) {
			sectorsPerCluster2++;
			k >>= 1;
		}
		reservedSectors = readByteFast();
		reservedSectors += (unsigned short)readByteFast()*0x100;
		readByteFast(); readByteFast(); // discard CRC bytes	
		// sectorsPerCluster = 0x40 at 2GB, 0x10 at 512MB
		// reservedSectors = 2 at 2GB
		fatAddr = bpbAddr + (unsigned long)512*reservedSectors;
	}
	if (bit_is_set(PIND,3)) return;

	{
		// sectorsPerFat and rootAddr
		cmdFast(16, 2);
		cmd17Fast(bpbAddr+0x16);
		sectorsPerFat = readByteFast();
		sectorsPerFat += (unsigned short)readByteFast()*0x100;
		readByteFast(); readByteFast(); // discard CRC bytes		
		// sectorsPerFat =  at 512MB,  0xEF at 2GB
		rootAddr = fatAddr + ((unsigned long)sectorsPerFat*2*512);
		userAddr = rootAddr+(unsigned long)512*32;
	}
	if (bit_is_set(PIND,3)) return;

	// find "NIC" extension
	nicDir = findExt("NIC", &protect, (unsigned char *)0);
	if (nicDir == 512) { // create NIC file if not exists
		// find "DSK" extension
		dskDir = findExt("DSK", (unsigned char *)0, filebase);
		if (dskDir == 512) return;
		if (!createNic(filebase)) return;
		nicDir = findExt("NIC", &protect, (unsigned char *)0);
		if (nicDir == 512) return;
		// convert DSK image to NIC image
		dsk2Nic();
	}
	if (bit_is_set(PIND,3)) return;
	
	prevFatNumNic = 0xff;
	prevFatNumDsk = 0xff;
	bitbyte = 0;
	readPulse = 0;
	magState = 0;
	prepare = 1;
	ph_track = 0;
	sector = 0;
	buffNum = 0;
	formatting = 0;
	writePtr = &(writeData[buffNum][0]);
	cmdFast(16, (unsigned long)512);
	buffClear();
	inited = 1;
}

// called when the card is inserted or removed
void check_eject(void)
{
	unsigned long i;

	if (bit_is_set(PIND,3)) {
		// added 2010/5/28
		for (i=0; i!=0x50000;i++)
			if (bit_is_clear(PIND,3)) return;	
		TIMSK0 &= ~(1<<TOIE0);
		EIMSK &= ~(1<<INT0);
		inited = 0;
		prepare = 0;
	} else if (!inited) {
		for (i=0; i!=0x50000;i++)
			if (bit_is_set(PIND,3)) return;
		cli();
		init();
		if (inited) {
			TIMSK0 |= (1<<TOIE0);
			EIMSK |= (1<<INT0);
		}
		sei();
	}
}

int main(void)
{
	static unsigned char stp, oldStp = 0;
	
	DDRB = 0b00010000;	
	DDRC = 0b00001010;
	DDRD = 0b00110010;

	PORTB = 0b00110000;
	PORTC = 0b00000010;
	PORTD = 0b00000000;

	sector = 0;
	ph_track = 0;
	prepare = 0;
	inited = 0;
	readPulse = 0;
	magState = 0;
	protect = 0;

	bitbyte = 0;
	magState = 0;
	prepare = 1;
	ph_track = 0;
	buffNum = 0;
	formatting = 0;
	writePtr = &(writeData[buffNum][0]);

	// timer interrupt
	OCR0A = 0;
	TCCR0A = 0;
	TCCR0B = 1;

	// int0 interrupt
	MCUCR = 0b00000010;
	EICRA = 0b00000010;

	while (1) {
		check_eject();
		if (bit_is_set(PINC, 0)) { // disable drive
			PORTB = 0b00100000;	 // LED off	
		} else { // enable drive                                                                                                                                                                   
			PORTB = 0b00110000;
			// protect = ((PIND&0b10000000)>>4);
			stp = (PINB & 0b00001111);
			if (stp != oldStp) {
				oldStp = stp;
				unsigned char ofs =
					((stp==0b00001000)?2:
					((stp==0b00000100)?4:
					((stp==0b00000010)?6:
					((stp==0b00000001)?0:0xff))));
				if (ofs != 0xff) {
					ofs = ((ofs+ph_track)&7);
					unsigned char bt = pgm_read_byte_near(stepper_table + (ofs>>1));
					oldStp = stp;
					if (ofs&1) bt &= 0x0f; else bt >>= 4;
					ph_track += ((bt & 0x08) ? (0xf8 | bt) : bt);
					if (ph_track > 196) ph_track = 0;	
					if (ph_track > 139) ph_track = 139;
				}
			}			
			if (inited && prepare) {
				cli();
				sector = ((sector+1)&0xf);
				{
					unsigned char trk = (ph_track>>2);
					unsigned short long_sector = (unsigned short)trk*16+sector;
					unsigned short long_cluster = long_sector>>sectorsPerCluster2;
					unsigned char fatNum = long_cluster/FAT_NIC_ELEMS;
					unsigned short ft;

					if (fatNum != prevFatNumNic) {
						prevFatNumNic = fatNum;
						prepareFat(nicDir, fatNic, (560+sectorsPerCluster-1)>>sectorsPerCluster2, fatNum, FAT_NIC_ELEMS);
					}
					ft = fatNic[long_cluster%FAT_NIC_ELEMS];

					if (((sectors[0]==sector)&&(tracks[0]==trk)) ||
						((sectors[1]==sector)&&(tracks[1]==trk)) ||
						((sectors[2]==sector)&&(tracks[2]==trk)) ||
						((sectors[3]==sector)&&(tracks[3]==trk)) ||
						((sectors[4]==sector)&&(tracks[4]==trk)))		
						writeBackSub();	
					cmd17Fast(userAddr+(((unsigned long)(ft-2)<<sectorsPerCluster2)
						+ (long_sector&(sectorsPerCluster-1)))*512);
					bitbyte = 0;
					prepare = 0;	
				}	
				sei();
			}
		}
	}
}

void writeBackSub2(unsigned char bn, unsigned char sc, unsigned char track)
{
	unsigned char c,d;
	unsigned short i;
	unsigned short long_sector = (unsigned short)track*16+sc;
	unsigned short long_cluster = long_sector>>sectorsPerCluster2;
	unsigned char fatNum = long_cluster/FAT_NIC_ELEMS;
	unsigned short ft;

	if (bit_is_set(PIND,3)) return;

	if (fatNum != prevFatNumNic) {
		prevFatNumNic = fatNum;
		prepareFat(nicDir, fatNic,
			(560+sectorsPerCluster-1)>>sectorsPerCluster2, fatNum, FAT_NIC_ELEMS);
	}
	ft = fatNic[long_cluster%FAT_NIC_ELEMS];
	
	PORTD = 0b00000010;
	PORTD = 0b00000000;	

	cmdFast(24, (unsigned long)userAddr+(((unsigned long)(ft-2)<<sectorsPerCluster2)
		+ (unsigned long)(long_sector&(sectorsPerCluster-1)))*512);

	writeByteFast(0xff);
	writeByteFast(0xfe);
	// 22 ffs
	for (i = 0; i < 22*8; i++) {
		PORTD = 0b00010000;
		PORTD = 0b00110000;
	}
	PORTD = 0b00000000;

	// sync header
	writeByteFast(0x03);
	writeByteFast(0xfc);
	writeByteFast(0xff);
	writeByteFast(0x3f);
	writeByteFast(0xcf);
	writeByteFast(0xf3);
	writeByteFast(0xfc);
	writeByteFast(0xff);
	writeByteFast(0x3f);
	writeByteFast(0xcf);
	writeByteFast(0xf3);
	writeByteFast(0xfc);

	// address header
	writeByteFast(0xd5);
	writeByteFast(0xAA);
	writeByteFast(0x96);
	writeByteFast((volume>>1)|0xaa);
	writeByteFast(volume|0xaa);
	writeByteFast((track>>1)|0xaa);
	writeByteFast(track|0xaa);
	writeByteFast((sc>>1)|0xaa);
	writeByteFast(sc|0xaa);
	c = (volume^track^sc);
	writeByteFast((c>>1)|0xaa);
	writeByteFast(c|0xaa);
	writeByteFast(0xde);
	writeByteFast(0xAA);
	writeByteFast(0xeb);

	// sync header
	writeByteFast(0xff);	
	writeByteFast(0xff);
	writeByteFast(0xff);
	writeByteFast(0xff);
	writeByteFast(0xff);

	// data
	for (i = 0; i < 349; i++) {
		c = writeData[bn][i];
		for (d = 0b10000000; d; d >>= 1) {
			if (c&d) {
				PORTD = 0b00010000;
				PORTD = 0b00110000;
			} else {
				PORTD = 0b00000000;
				PORTD = 0b00100000;
			}
		}
	}
	PORTD = 0b00000000;
	for (i = 0; i < 14*8; i++) {
		PORTD = 0b00010000;
		PORTD = 0b00110000;
	}
	PORTD = 0b00000000;
	for (i = 0; i < 96*8; i++) {
		PORTD = 0b00000000;
		PORTD = 0b00100000;
	}
	PORTD = 0b00000000;	
	writeByteFast(0xff);
	writeByteFast(0xff);
	readByteFast();
	waitFinish();
	
	PORTD = 0b00000010;
	PORTD = 0b00000000;		
}

void writeBackSub(void)
{
	unsigned char i, j;

	if (bit_is_set(PIND,3)) return;
	for (j=0; j<BUF_NUM; j++) {
		if (sectors[j]!=0xff) {
			for (i=0; i<BUF_NUM; i++) {
				if (sectors[i] != 0xff)
					writeBackSub2(i, sectors[i], tracks[i]);
				sectors[i] = 0xff;
				tracks[i] = 0xff;
				writeData[i][2]=0;
			}
			buffNum = 0;
			writePtr = &(writeData[buffNum][0]);
			break;
		}
	}
}

// write back writeData into the SD card
void writeBack(void)
{
	static unsigned char sec;
	
	if (bit_is_set(PIND,3)) return;
	if (writeData[buffNum][2]==0xAD) {
		if (!formatting) {
			sectors[buffNum]=sector;
			tracks[buffNum]=(ph_track>>2);
			sector=((((sector==0xf)||(sector==0xd))?(sector+2):(sector+1))&0xf);
			if (buffNum == (BUF_NUM-1)) {
				// cancel reading
				cancelRead();
				writeBackSub();
				prepare = 1;
			} else {
				buffNum++;
				writePtr = &(writeData[buffNum][0]);
			}
		} else {
			sector = sec;
			formatting = 0;
			if (sec == 0xf) {
				// cancel reading
				cancelRead();
				prepare = 1;
			}
		}
	} if (writeData[buffNum][2]==0x96) {
		sec = (((writeData[buffNum][7]&0x55)<<1) | (writeData[buffNum][8]&0x55));
		formatting = 1;
	}
}