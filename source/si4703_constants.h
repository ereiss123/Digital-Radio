#ifndef si4703_constants
#define si4703_constants

	uint16_t si4703_registers[16]; //There are 16 registers, each 16 bits large
	static const uint16_t  FAIL = 0;
	static const uint16_t  SUCCESS = 1;

	static const int  SI4703 = 0x10; //0b._001.0000 = I2C address of Si4703 - note that the Wire function assumes non-left-shifted I2C address, not 0b.0010.000W
	static const uint16_t  I2C_FAIL_MAX = 10; //This is the number of attempts we will try to contact the device before erroring out
	static const uint16_t  SEEK_DOWN = 0; //Direction used for seeking. Default is down
	static const uint16_t  SEEK_UP = 1;

	//Define the register names
	static const uint16_t  DEVICEID = 0x00;
	static const uint16_t  CHIPID = 0x01;
	static const uint16_t  POWERCFG = 0x02;
	static const uint16_t  CHANNEL = 0x03;
	static const uint16_t  SYSCONFIG1 = 0x04;
	static const uint16_t  SYSCONFIG2 = 0x05;
	static const uint16_t  STATUSRSSI = 0x0A;
	static const uint16_t  READCHAN = 0x0B;
	static const uint16_t  RDSA = 0x0C;
	static const uint16_t  RDSB = 0x0D;
	static const uint16_t  RDSC = 0x0E;
	static const uint16_t  RDSD = 0x0F;

	//Register 0x02 - POWERCFG
	static const uint16_t  SMUTE = 15;
	static const uint16_t  DMUTE = 14;
	static const uint16_t  SKMODE = 10;
	static const uint16_t  SEEKUP = 9;
	static const uint16_t  SEEK = 8;

	//Register 0x03 - CHANNEL
	static const uint16_t  TUNE = 15;

	//Register 0x04 - SYSCONFIG1
	static const uint16_t  RDS = 12;
	static const uint16_t  DE = 11;

	//Register 0x05 - SYSCONFIG2
	static const uint16_t  SPACE1 = 5;
	static const uint16_t  SPACE0 = 4;

	//Register 0x0A - STATUSRSSI
	static const uint16_t  RDSR = 15;
	static const uint16_t  STC = 14;
	static const uint16_t  SFBL = 13;
	static const uint16_t  AFCRL = 12;
	static const uint16_t  RDSS = 11;
	static const uint16_t  STEREO = 8;

#endif 