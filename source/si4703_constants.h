#ifndef si4703_constants
#define si4703_constants

	//extern unsigned short int si4703_registers[16]; //There are 16 registers, each 16 bits large
	#define FAIL 0
	#define SUCCESS 1
	
	#define SI4703 0x10<<1 //0b._001.0000 = I2C address of Si4703 - note that the Wire function assumes non-left-shifted I2C address, not 0b.0010.000W
	#define I2C_FAIL_MAX 10 //This is the number of attempts we will try to contact the device before erroring out
	#define SEEK_DOWN 0 //Direction used for seeking. Default is down
	#define SEEK_UP 1
	
	//define register names
	#define DEVICEID 0x00
	#define CHIPID 0x01
	#define POWERCFG 0x02
	#define CHANNEL 0x03
	#define SYSCONFIG1 0x04
	#define SYSCONFIG2 0x05
	#define STATUSRSSI 0x0A
	#define READCHAN 0x0B
	#define RDSA 0x0C
	#define RDSB 0x0D
	#define RDSC 0x0E
	#define RDSD 0x0F

	//Define the read register names
	#define DEVICEID_R 0x06 //0x00
	#define CHIPID_R 0x07 //0x01
	#define POWERCFG_R 0x08 //0x02
	#define CHANNEL_R 0x09 //0x03
	#define SYSCONFIG1_R 0x0A //0x04
	#define SYSCONFIG2_R 0x0B //0x05
	#define STATUSRSSI_R 0x00 //0x0A
	#define READCHAN_R 0x01 //0x0B
	#define RDSA_R 0x02 //0x0C
	#define RDSB_R 0x03 //0x0D
	#define RDSC_R 0x04 //0x0E
	#define RDSD_R 0x05 //0x0F
	
	//Define the write register names
	#define DEVICEID_W 0x0A //0x00
	#define CHIPID_W 0x0B //0x01
	#define POWERCFG_W 0x00 //0x02
	#define CHANNEL_W 0x01 //0x03
	#define SYSCONFIG1_W 0x02 //0x04
	#define SYSCONFIG2_W 0x03 //0x05
	#define STATUSRSSI_W 0x04 //0x0A
	#define READCHAN_W 0x05 //0x0B
	#define RDSA_W 0x06 //0x0C
	#define RDSB_W 0x07 //0x0D
	#define RDSC_W 0x08 //0x0E
	#define RDSD_W 0x09 //0x0F

	//Register 0x02 - POWERCFG
	#define SMUTE 15
	#define DMUTE 14
	#define SKMODE 10
	#define SEEKUP 9
	#define SEEK 8

	//Register 0x03 - CHANNEL
	#define TUNE 15

	//Register 0x04 - SYSCONFIG1
	#define RDS 12
	#define DE 11

	//Register 0x05 - SYSCONFIG2
	#define SPACE1 5
	#define SPACE0 4

	//Register 0x0A - STATUSRSSI
	#define RDSR 15
	#define STC 14
	#define SFBL 13
	#define AFCRL 12
	#define RDSS 11
	#define STEREO 8
#endif 
