/******************************************************************************
lsm9ds0 Beaglebone Library
Alex Fuhr, The Ohio State University
https://github.com/projectzen/Beaglebone-LSM9DS0
Implements the LSM9DS0 functions on Beaglebone Black
******************************************************************************/

#include "lsm9ds0.h"
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stropts.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#define MAX_BUS 64

// Put in main code
//LSM9DS0(int bus, unsigned char gAddr, unsigned char xmAddr)
//{
	//// interfaceMode will keep track of whether we're using SPI or I2C:
	//I2CBus = bus;
	
	//// xmAddress and gAddress will store the 7-bit I2C address, if using I2C.
	//// If we're using SPI, these variables store the chip-select pins.
	//xmAddress = xmAddr;
	//gAddress = gAddr;
//}

unsigned short initIMU(enum gyro_scale gScl, enum accel_scale aScl, enum mag_scale mScl, 
						enum gyro_odr gODR, enum accel_odr aODR, enum mag_odr mODR, struct IMU* reads) {
	// Store the given scales in class variables. These scale variables
	// are used throughout to calculate the actual g's, DPS,and Gs's.
	printf("initialising IMU\n");
	reads -> gScale = gScl;
	reads -> aScale = aScl;
	reads -> mScale = mScl;
	
	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcgRes(reads); // Calculate DPS / ADC tick, stored in gRes variable
	calcmRes(reads); // Calculate Gs / ADC tick, stored in mRes variable
	calcaRes(reads); // Calculate g / ADC tick, stored in aRes variable
	
	// Now, initialize our hardware interface.
	reads -> file = initI2C(reads -> I2CBus, reads -> file);					// Initialize I2C
	printf("File #%i\n",reads -> file);
	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	unsigned char gTest = gReadByte(WHO_AM_I_G, reads -> gAddress, reads -> file);		// Read the gyro WHO_AM_I
	if (gTest < 1) printf("Address 0x%02X register 0x%02X failed to setup\n",WHO_AM_I_G, reads -> gAddress);
	else printf("gtest passed, %02x\n",gTest);
	unsigned char xmTest = xmReadByte(WHO_AM_I_XM, reads -> xmAddress, reads -> file);	// Read the accel/mag WHO_AM_I
	if (xmTest < 1) printf("Address 0x%02X register 0x%02X failed to setup\n",WHO_AM_I_G, reads -> gAddress);
	else printf("xmtest passed, %02x\n",xmTest);
	// Gyro initialization stuff:
	initGyro(reads);	// This will "turn on" the gyro. Setting up interrupts, etc.
	setGyroODR(gODR, reads); // Set the gyro output data rate and bandwidth.
	setGyroScale(reads); // Set the gyro range
	// Accelerometer initialization stuff:
	initAccel(reads); // "Turn on" all axes of the accel. Set up interrupts, etc.
	setAccelODR(aODR, reads); // Set the accel data rate.
	setAccelScale(reads); // Set the accel range.
	// Magnetometer initialization stuff:
	initMag(reads); // "Turn on" all axes of the mag. Set up interrupts, etc.
	setMagODR(mODR, reads); // Set the magnetometer output data rate.
	setMagScale(reads); // Set the magnetometer's range.
	// Once everything is initialized, return the WHO_AM_I registers we read:
	printf("Return 0x%08X%08X\n", xmTest, gTest);
	getchar();
	return (xmTest << 8) | gTest;
}

void initGyro(struct IMU* reads) {
	printf("initialising Gyro\n");
	/* CTRL_REG1_G sets output data rate, bandwidth, power-down and enables
	Bits[7:0]: DR1 DR0 BW1 BW0 PD Zen Xen Yen
	DR[1:0] - Output data rate selection
		00=95Hz, 01=190Hz, 10=380Hz, 11=760Hz
	BW[1:0] - Bandwidth selection (sets cutoff frequency)
		 Value depends on ODR. See datasheet table 21.
	PD - Power down enable (0=power down mode, 1=normal or sleep mode)
	Zen, Xen, Yen - Axis enable (o=disabled, 1=enabled)	*/
	gWriteByte(CTRL_REG1_G, 0x0F, reads -> gAddress, reads -> file); // Normal mode, enable all axes
	printf("CTRL_REG1_G (0x0F) 0x%2X\n", gReadByte(CTRL_REG1_G, reads -> gAddress, reads -> file)); 
	
	/* CTRL_REG2_G sets up the HPF
	Bits[7:0]: 0 0 HPM1 HPM0 HPCF3 HPCF2 HPCF1 HPCF0
	HPM[1:0] - High pass filter mode selection
		00=normal (reset reading HP_RESET_FILTER, 01=ref signal for filtering,
		10=normal, 11=autoreset on interrupt
	HPCF[3:0] - High pass filter cutoff frequency
		Value depends on data rate. See datasheet table 26.
	*/
	gWriteByte(CTRL_REG2_G, 0x00, reads -> gAddress, reads -> file); // Normal mode, high cutoff frequency
	printf("CTRL_REG2_G (0x00) 0x%2X\n", gReadByte(CTRL_REG2_G, reads -> gAddress, reads -> file)); 
	
	/* CTRL_REG3_G sets up interrupt and DRDY_G pins
	Bits[7:0]: I1_IINT1 I1_BOOT H_LACTIVE PP_OD I2_DRDY I2_WTM I2_ORUN I2_EMPTY
	I1_INT1 - Interrupt enable on INT_G pin (0=disable, 1=enable)
	I1_BOOT - Boot status available on INT_G (0=disable, 1=enable)
	H_LACTIVE - Interrupt active configuration on INT_G (0:high, 1:low)
	PP_OD - Push-pull/open-drain (0=push-pull, 1=open-drain)
	I2_DRDY - Data ready on DRDY_G (0=disable, 1=enable)
	I2_WTM - FIFO watermark interrupt on DRDY_G (0=disable 1=enable)
	I2_ORUN - FIFO overrun interrupt on DRDY_G (0=disable 1=enable)
	I2_EMPTY - FIFO empty interrupt on DRDY_G (0=disable 1=enable) */
	// Int1 enabled (pp, active low), data read on DRDY_G:
	gWriteByte(CTRL_REG3_G, 0x88, reads -> gAddress, reads -> file); 
	printf("CTRL_REG3_G (0x88) 0x%2X\n", gReadByte(CTRL_REG3_G, reads -> gAddress, reads -> file)); 
	
	/* CTRL_REG4_G sets the scale, update mode
	Bits[7:0] - BDU BLE FS1 FS0 - ST1 ST0 SIM
	BDU - Block data update (0=continuous, 1=output not updated until read
	BLE - Big/little endian (0=data LSB @ lower address, 1=LSB @ higher add)
	FS[1:0] - Full-scale selection
		00=245dps, 01=500dps, 10=2000dps, 11=2000dps
	ST[1:0] - Self-test enable
		00=disabled, 01=st 0 (x+, y-, z-), 10=undefined, 11=st 1 (x-, y+, z+)
	SIM - SPI serial interface mode select
		0=4 wire, 1=3 wire */
	gWriteByte(CTRL_REG4_G, 0x00, reads -> gAddress, reads -> file); // Set scale to 245 dps
	printf("CTRL_REG4_G (0x00) 0x%2X\n", gReadByte(CTRL_REG4_G, reads -> gAddress, reads -> file)); 
	
	/* CTRL_REG5_G sets up the FIFO, HPF, and INT1
	Bits[7:0] - BOOT FIFO_EN - HPen INT1_Sel1 INT1_Sel0 Out_Sel1 Out_Sel0
	BOOT - Reboot memory content (0=normal, 1=reboot)
	FIFO_EN - FIFO enable (0=disable, 1=enable)
	HPen - HPF enable (0=disable, 1=enable)
	INT1_Sel[1:0] - Int 1 selection configuration
	Out_Sel[1:0] - Out selection configuration */
	gWriteByte(CTRL_REG5_G, 0x00, reads -> gAddress, reads -> file);
	printf("CTRL_REG5_G (0x00) 0x%2X\n", gReadByte(CTRL_REG5_G, reads -> gAddress, reads -> file)); 
	
	// Temporary !!! For testing !!! Remove !!! Or make useful !!!
	// configGyroInt(0x2A, 0, 0, 0, 0); // Trigger interrupt when above 0 DPS...
}

void initAccel(struct IMU* reads) {
	printf("initialising Accelerometer\n");
	/* CTRL_REG0_XM (0x1F) (Default value: 0x00)
	Bits (7-0): BOOT FIFO_EN WTM_EN 0 0 HP_CLICK HPIS1 HPIS2
	BOOT - Reboot memory content (0: normal, 1: reboot)
	FIFO_EN - Fifo enable (0: disable, 1: enable)
	WTM_EN - FIFO watermark enable (0: disable, 1: enable)
	HP_CLICK - HPF enabled for click (0: filter bypassed, 1: enabled)
	HPIS1 - HPF enabled for interrupt generator 1 (0: bypassed, 1: enabled)
	HPIS2 - HPF enabled for interrupt generator 2 (0: bypassed, 1 enabled)   */
	if (xmWriteByte(CTRL_REG0_XM, 0x00, reads -> xmAddress, reads -> file) > 0)
		printf("CTRL_REG0_XM (0x00) 0x%2X\n", xmReadByte(CTRL_REG0_XM, reads -> xmAddress, reads -> file)); 
	
	/* CTRL_REG1_XM (0x20) (Default value: 0x07)
	Bits (7-0): AODR3 AODR2 AODR1 AODR0 BDU AZEN AYEN AXEN
	AODR[3:0] - select the acceleration data rate:
		0000=power down, 0001=3.125Hz, 0010=6.25Hz, 0011=12.5Hz, 
		0100=25Hz, 0101=50Hz, 0110=100Hz, 0111=200Hz, 1000=400Hz,
		1001=800Hz, 1010=1600Hz, (remaining combinations undefined).
	BDU - block data update for accel AND mag
		0: Continuous update
		1: Output registers aren't updated until MSB and LSB have been read.
	AZEN, AYEN, and AXEN - Acceleration x/y/z-axis enabled.
		0: Axis disabled, 1: Axis enabled									 */	
	xmWriteByte(CTRL_REG1_XM, 0x67, reads -> xmAddress, reads -> file); // 100Hz data rate, x/y/z all (enabled x57)
	printf("CTRL_REG1_XM (0x67) 0x%2X\n", xmReadByte(CTRL_REG1_XM, reads -> xmAddress, reads -> file));
	//Serial.println(xmReadByte(CTRL_REG1_XM));
	/* CTRL_REG2_XM (0x21) (Default value: 0x00)
	Bits (7-0): ABW1 ABW0 AFS2 AFS1 AFS0 AST1 AST0 SIM
	ABW[1:0] - Accelerometer anti-alias filter bandwidth
		00=773Hz, 01=194Hz, 10=362Hz, 11=50Hz
	AFS[2:0] - Accel full-scale selection
		000=+/-2g, 001=+/-4g, 010=+/-6g, 011=+/-8g, 100=+/-16g
	AST[1:0] - Accel self-test enable
		00=normal (no self-test), 01=positive st, 10=negative st, 11=not allowed
	SIM - SPI mode selection
		0=4-wire, 1=3-wire													 */
	xmWriteByte(CTRL_REG2_XM, 0x00, reads -> xmAddress, reads -> file); // Set scale to 2g
	printf("CTRL_REG2_XM (0x00) 0x%2X\n", xmReadByte(CTRL_REG2_XM, reads -> xmAddress, reads -> file));
	/* CTRL_REG3_XM is used to set interrupt generators on INT1_XM
	Bits (7-0): P1_BOOT P1_TAP P1_INT1 P1_INT2 P1_INTM P1_DRDYA P1_DRDYM P1_EMPTY
	*/
	// Accelerometer data ready on INT1_XM (0x04)
	xmWriteByte(CTRL_REG3_XM, 0x04, reads -> xmAddress, reads -> file); 
	printf("CTRL_REG3_XM (0x04) 0x%2X\n", xmReadByte(CTRL_REG3_XM, reads -> xmAddress, reads -> file));
}

void initMag(struct IMU* reads) {	
	printf("initialising Magnomenter\n");
	/* CTRL_REG5_XM enables temp sensor, sets mag resolution and data rate
	Bits (7-0): TEMP_EN M_RES1 M_RES0 M_ODR2 M_ODR1 M_ODR0 LIR2 LIR1
	TEMP_EN - Enable temperature sensor (0=disabled, 1=enabled)
	M_RES[1:0] - Magnetometer resolution select (0=low, 3=high)
	M_ODR[2:0] - Magnetometer data rate select
		000=3.125Hz, 001=6.25Hz, 010=12.5Hz, 011=25Hz, 100=50Hz, 101=100Hz
	LIR2 - Latch interrupt request on INT2_SRC (cleared by reading INT2_SRC)
		0=interrupt request not latched, 1=interrupt request latched
	LIR1 - Latch interrupt request on INT1_SRC (cleared by readging INT1_SRC)
		0=irq not latched, 1=irq latched 									 */
	xmWriteByte(CTRL_REG5_XM, 0x94, reads -> xmAddress, reads -> file); // Mag data rate - 100 Hz, enable temperature sensor
	printf("CTRL_REG5_XM (0x94) 0x%2X\n", xmReadByte(CTRL_REG5_XM, reads -> xmAddress, reads -> file));
	
	/* CTRL_REG6_XM sets the magnetometer full-scale
	Bits (7-0): 0 MFS1 MFS0 0 0 0 0 0
	MFS[1:0] - Magnetic full-scale selection
	00:+/-2Gauss, 01:+/-4Gs, 10:+/-8Gs, 11:+/-12Gs							 */
	xmWriteByte(CTRL_REG6_XM, 0x00, reads -> xmAddress, reads -> file); // Mag scale to +/- 2GS
	printf("CTRL_REG6_XM (0x00) 0x%2X\n", xmReadByte(CTRL_REG6_XM, reads -> xmAddress, reads -> file));
	
	/* CTRL_REG7_XM sets magnetic sensor mode, low power mode, and filters
	AHPM1 AHPM0 AFDS 0 0 MLP MD1 MD0
	AHPM[1:0] - HPF mode selection
		00=normal (resets reference registers), 01=reference signal for filtering, 
		10=normal, 11=autoreset on interrupt event
	AFDS - Filtered acceleration data selection
		0=internal filter bypassed, 1=data from internal filter sent to FIFO
	MLP - Magnetic data low-power mode
		0=data rate is set by M_ODR bits in CTRL_REG5
		1=data rate is set to 3.125Hz
	MD[1:0] - Magnetic sensor mode selection (default 10)
		00=continuous-conversion, 01=single-conversion, 10 and 11=power-down */
	xmWriteByte(CTRL_REG7_XM, 0x00, reads -> xmAddress, reads -> file); // Continuous conversion mode
	printf("CTRL_REG7_XM (0x00) 0x%2X\n", xmReadByte(CTRL_REG7_XM, reads -> xmAddress, reads -> file));
	
	/* CTRL_REG4_XM is used to set interrupt generators on INT2_XM
	Bits (7-0): P2_TAP P2_INT1 P2_INT2 P2_INTM P2_DRDYA P2_DRDYM P2_Overrun P2_WTM
	*/
	xmWriteByte(CTRL_REG4_XM, 0x04, reads -> xmAddress, reads -> file); // Magnetometer data ready on INT2_XM (0x08)
	printf("CTRL_REG4_XM (0x04) 0x%X\n", xmReadByte(CTRL_REG4_XM, reads -> xmAddress, reads -> file));
	
	/* INT_CTRL_REG_M to set push-pull/open drain, and active-low/high
	Bits[7:0] - XMIEN YMIEN ZMIEN PP_OD IEA IEL 4D MIEN
	XMIEN, YMIEN, ZMIEN - Enable interrupt recognition on axis for mag data
	PP_OD - Push-pull/open-drain interrupt configuration (0=push-pull, 1=od)
	IEA - Interrupt polarity for accel and magneto
		0=active-low, 1=active-high
	IEL - Latch interrupt request for accel and magneto
		0=irq not latched, 1=irq latched
	4D - 4D enable. 4D detection is enabled when 6D bit in INT_GEN1_REG is set
	MIEN - Enable interrupt generation for magnetic data
		0=disable, 1=enable) */
	xmWriteByte(INT_CTRL_REG_M, 0x09, reads -> xmAddress, reads -> file); // Enable interrupts for mag, active-low, push-pull
	printf("INT_CTRL_REG_M (0x09) 0x%X\n", xmReadByte(INT_CTRL_REG_M, reads -> xmAddress, reads -> file));
}

// This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
// them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
// for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
// the data as there are in the ADXL345, a precursor to the LSM9DS0, or the MPU-9150, so we have to
// subtract the biases ourselves. This results in a more accurate measurement in general and can
// remove errors due to imprecise or varying initial placement. Calibration of sensor data in this manner
// is good practice.
void calIMU(float * gbias, float * abias, struct IMU* reads) {  
  unsigned char data[6] = {0, 0, 0, 0, 0, 0};
  short gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  int samples, i;
  
  // First get gyro bias
  char c = gReadByte(CTRL_REG5_G, reads -> gAddress, reads -> file);
  gWriteByte(CTRL_REG5_G, c | 0x40, reads -> gAddress, reads -> file);         // Enable gyro FIFO  
  usleep(20000);                                 // Wait for change to take effect
  gWriteByte(FIFO_CTRL_REG_G, 0x20 | 0x1F, reads -> gAddress, reads -> file);  // Enable gyro FIFO stream mode and set watermark at 32 samples
  sleep(1);  // delay 1000 milliseconds to collect FIFO samples
  
  samples = (gReadByte(FIFO_SRC_REG_G, reads -> gAddress, reads -> file) & 0x1F); // Read number of stored samples

  for(i = 0; i < samples ; i++) {            // Read the gyro data stored in the FIFO
    gReadBytes(OUT_X_L_G,  &data[0], 6, reads -> gAddress, reads -> file);
    gyro_bias[0] += (((short)data[1] << 8) | data[0]);
    gyro_bias[1] += (((short)data[3] << 8) | data[2]);
    gyro_bias[2] += (((short)data[5] << 8) | data[4]);
  }  

  gyro_bias[0] /= samples; // average the data
  gyro_bias[1] /= samples; 
  gyro_bias[2] /= samples; 
  
  gbias[0] = (float)gyro_bias[0] * reads -> gRes;  // Properly scale the data to get deg/s
  gbias[1] = (float)gyro_bias[1] * reads -> gRes;
  gbias[2] = (float)gyro_bias[2] * reads -> gRes;
  
  c = gReadByte(CTRL_REG5_G, reads -> gAddress, reads -> file);
  gWriteByte(CTRL_REG5_G, c & ~0x40, reads -> gAddress, reads -> file);  // Disable gyro FIFO  
  usleep(20000);
  gWriteByte(FIFO_CTRL_REG_G, 0x00, reads -> gAddress, reads -> file);   // Enable gyro bypass mode
  

  //  Now get the accelerometer biases
  c = xmReadByte(CTRL_REG0_XM, reads -> xmAddress, reads -> file);
  xmWriteByte(CTRL_REG0_XM, c | 0x40, reads -> xmAddress, reads -> file);      // Enable accelerometer FIFO  
  usleep(20000);                                // Wait for change to take effect
  xmWriteByte(FIFO_CTRL_REG, 0x20 | 0x1F, reads -> xmAddress, reads -> file);  // Enable accelerometer FIFO stream mode and set watermark at 32 samples
  sleep(1);  // delay 1000 milliseconds to collect FIFO samples

  samples = (xmReadByte(FIFO_SRC_REG, reads -> xmAddress, reads -> file) & 0x1F); // Read number of stored accelerometer samples

   for(i = 0; i < samples ; i++) {          // Read the accelerometer data stored in the FIFO
    xmReadBytes(OUT_X_L_A, &data[0], 6, reads -> xmAddress, reads -> file);
    accel_bias[0] += (((short)data[1] << 8) | data[0]);
    accel_bias[1] += (((short)data[3] << 8) | data[2]);
    accel_bias[2] += (((short)data[5] << 8) | data[4]) - (short)(1./ reads -> aRes); // Assumes sensor facing up!
  }  

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples; 
  accel_bias[2] /= samples; 
  
  abias[0] = (float)accel_bias[0] * reads -> aRes; // Properly scale data to get gs
  abias[1] = (float)accel_bias[1] * reads -> aRes;
  abias[2] = (float)accel_bias[2] * reads -> aRes;

  c = xmReadByte(CTRL_REG0_XM, reads -> xmAddress, reads -> file);
  xmWriteByte(CTRL_REG0_XM, c & ~0x40, reads -> xmAddress, reads -> file);    // Disable accelerometer FIFO  
  usleep(20000);
  xmWriteByte(FIFO_CTRL_REG, 0x00, reads -> xmAddress, reads -> file);       // Enable accelerometer bypass mode
}

void readAccel(struct IMU* reads) {
	//int i;
	unsigned char temp[6]; // We'll read six chars from the accelerometer into temp	
	xmReadBytes(OUT_X_L_A, temp, 6, reads -> xmAddress, reads -> file); // Read 6 chars, beginning at OUT_X_L_A
	//for (i = 0; i < 6; i++) printf("Accel read %i = %02x\t",i,temp[i]);
	//printf("\n");
	reads -> ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
	reads -> ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
	reads -> az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
}

void readMag(struct IMU* reads) {
	//int i;
	unsigned char temp[6]; // We'll read six chars from the mag into temp	
	xmReadBytes(OUT_X_L_M, temp, 6, reads -> xmAddress, reads -> file); // Read 6 chars, beginning at OUT_X_L_M
	//for (i = 0; i < 6; i++) printf("Mag read %i = %02x\t",i,temp[i]);
	//printf("\n");
	reads -> mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
	reads -> my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
	reads -> mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
}

void readTemp(struct IMU* reads) {
	//int i;
	unsigned char temp[2]; // We'll read two chars from the temperature sensor into temp	
	xmReadBytes(OUT_TEMP_L_XM, temp, 2, reads -> xmAddress, reads -> file); // Read 2 chars, beginning at OUT_TEMP_L_M
	//for (i = 0; i < 2; i++) printf("Temp read %i = %02x\t",i,temp[i]);
	//printf("\n");
	reads -> temperature = (((short) temp[1] << 12) | temp[0] << 4 ) >> 4; // Temperature is a 12-bit signed integer
}

void readGyro(struct IMU* reads) {
	unsigned char temp[6]; // We'll read six chars from the gyro into temp
	//int i;
	gReadBytes(OUT_X_L_G, temp, 6, reads -> gAddress, reads -> file); // Read 6 chars, beginning at OUT_X_L_G
	//for (i = 0; i < 6; i++) printf("Gyro read %i = %02x\t",i,temp[i]);
	//printf("\n");
	reads -> gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
	reads -> gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
	reads -> gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
}

float calcGyro(short gyro, float gRes) {
	// Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
	return gRes * gyro; 
}

float calcAccel(short accel, float aRes) {
	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
	return aRes * accel;
}

float calcMag(short mag, float mRes) {
	// Return the mag raw reading times our pre-calculated Gs / (ADC tick):
	return mRes * mag;
}

void setGyroScale(struct IMU* reads) {
	// We need to preserve the other chars in CTRL_REG4_G. So, first read it:
	unsigned char temp = gReadByte(CTRL_REG4_G, reads -> gAddress, reads -> file);
	// Then mask out the gyro scale bits:
	temp &= 0xFF^(0x3 << 4);
	// Then shift in our new scale bits:
	temp |= reads -> gScale << 4;
	// And write the new register value back into CTRL_REG4_G:
	gWriteByte(CTRL_REG4_G, temp, reads -> gAddress, reads -> file);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update gScale:
	// reads -> gScale = gScl;
	// Then calculate a new gRes, which relies on gScale being set correctly:
	calcgRes(reads);
}

void setAccelScale(struct IMU* reads) {
	// We need to preserve the other chars in CTRL_REG2_XM. So, first read it:
	unsigned char temp = xmReadByte(CTRL_REG2_XM, reads -> xmAddress, reads -> file);
	// Then mask out the accel scale bits:
	temp &= 0xFF^(0x3 << 3);
	// Then shift in our new scale bits:
	temp |= reads -> aScale << 3;
	// And write the new register value back into CTRL_REG2_XM:
	xmWriteByte(CTRL_REG2_XM, temp, reads -> xmAddress, reads -> file);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update aScale:
	// reads -> aScale = aScl;
	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes(reads);
}

void setMagScale(struct IMU* reads) {
	// We need to preserve the other chars in CTRL_REG6_XM. So, first read it:
	unsigned char temp = xmReadByte(CTRL_REG6_XM, reads -> xmAddress, reads -> file);
	// Then mask out the mag scale bits:
	temp &= 0xFF^(0x3 << 5);
	// Then shift in our new scale bits:
	temp |= reads -> mScale << 5;
	// And write the new register value back into CTRL_REG6_XM:
	xmWriteByte(CTRL_REG6_XM, temp, reads -> xmAddress, reads -> file);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update mScale:
	// reads -> mScale = mScl;
	// Then calculate a new mRes, which relies on mScale being set correctly:
	calcmRes(reads);
}

void setGyroODR(enum gyro_odr gRate, struct IMU* reads) {
	// We need to preserve the other chars in CTRL_REG1_G. So, first read it:
	unsigned char temp = gReadByte(CTRL_REG1_G, reads -> gAddress, reads -> file);
	// Then mask out the gyro ODR bits:
	temp &= 0xFF^(0xF << 4);
	// Then shift in our new ODR bits:
	temp |= (gRate << 4);
	// And write the new register value back into CTRL_REG1_G:
	gWriteByte(CTRL_REG1_G, temp, reads -> gAddress, reads -> file);
}
void setAccelODR(enum accel_odr aRate, struct IMU* reads) {
	// We need to preserve the other chars in CTRL_REG1_XM. So, first read it:
	unsigned char temp = xmReadByte(CTRL_REG1_XM, reads -> xmAddress, reads -> file);
	// Then mask out the accel ODR bits:
	temp &= 0xFF^(0xF << 4);
	// Then shift in our new ODR bits:
	temp |= (aRate << 4);
	// And write the new register value back into CTRL_REG1_XM:
	xmWriteByte(CTRL_REG1_XM, temp, reads -> xmAddress, reads -> file);
}
void setAccelABW(enum accel_abw abwRate, struct IMU* reads) { 
	// We need to preserve the other chars in CTRL_REG2_XM. So, first read it:
	unsigned char temp = xmReadByte(CTRL_REG2_XM, reads -> xmAddress, reads -> file);
	// Then mask out the accel ABW bits:
	temp &= 0xFF^(0x3 << 7);
	// Then shift in our new ODR bits:
	temp |= (abwRate << 7);
	// And write the new register value back into CTRL_REG2_XM:
	xmWriteByte(CTRL_REG2_XM, temp, reads -> xmAddress, reads -> file);
}
void setMagODR(enum mag_odr mRate, struct IMU* reads) {
	// We need to preserve the other chars in CTRL_REG5_XM. So, first read it:
	unsigned char temp = xmReadByte(CTRL_REG5_XM, reads -> xmAddress, reads -> file);
	// Then mask out the mag ODR bits:
	temp &= 0xFF^(0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= (mRate << 2);
	// And write the new register value back into CTRL_REG5_XM:
	xmWriteByte(CTRL_REG5_XM, temp, reads -> xmAddress, reads -> file);
}

void configGyroInt(unsigned char int1Cfg, unsigned short int1ThsX, unsigned short int1ThsY, 
	unsigned short int1ThsZ, unsigned char duration, struct IMU* reads) {
	gWriteByte(INT1_CFG_G, int1Cfg, reads -> gAddress, reads -> file);
	gWriteByte(INT1_THS_XH_G, (int1ThsX & 0xFF00) >> 8, reads -> gAddress, reads -> file);
	gWriteByte(INT1_THS_XL_G, (int1ThsX & 0xFF), reads -> gAddress, reads -> file);
	gWriteByte(INT1_THS_YH_G, (int1ThsY & 0xFF00) >> 8, reads -> gAddress, reads -> file);
	gWriteByte(INT1_THS_YL_G, (int1ThsY & 0xFF), reads -> gAddress, reads -> file);
	gWriteByte(INT1_THS_ZH_G, (int1ThsZ & 0xFF00) >> 8, reads -> gAddress, reads -> file);
	gWriteByte(INT1_THS_ZL_G, (int1ThsZ & 0xFF), reads -> gAddress, reads -> file);
	if (duration)
		gWriteByte(INT1_DURATION_G, 0x80 | duration, reads -> gAddress, reads -> file);
	else
		gWriteByte(INT1_DURATION_G, 0x00, reads -> gAddress, reads -> file);
}

void calcgRes(struct IMU* reads) {
	// Possible gyro scales (and their register bit settings) are:
	// 245 DPS (00), 500 DPS (01), 2000 DPS (10). Here's a bit of an algorithm
	// to calculate DPS/(ADC tick) based on that 2-bit value:
	switch (reads -> gScale)
	{
	case G_SCALE_245DPS:
		reads -> gRes = 245.0 / 32768.0;
		break;
	case G_SCALE_500DPS:
		reads -> gRes = 500.0 / 32768.0;
		break;
	case G_SCALE_2000DPS:
		reads -> gRes = 2000.0 / 32768.0;
		break;
	}
}

void calcaRes(struct IMU* reads) {
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100). Here's a bit of an 
	// algorithm to calculate g/(ADC tick) based on that 3-bit value:
	reads -> aRes = reads -> aScale == A_SCALE_16G ? 16.0 / 32768.0 : 
		   (((float) reads -> aScale + 1.0) * 2.0) / 32768.0;
}

void calcmRes(struct IMU* reads) {
	// Possible magnetometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10) 12 Gs (11). Here's a bit of an algorithm
	// to calculate Gs/(ADC tick) based on that 2-bit value:
	reads -> mRes = reads -> mScale == M_SCALE_2GS ? 2.0 / 32768.0 : 
	       (float) (reads -> mScale << 2) / 32768.0;
}
	
void gWriteByte(unsigned char subAddress, unsigned char data, unsigned char gAddress, int file) {
	// Whether we're using I2C or SPI, write a char using the
	// gyro-specific I2C address or SPI CS pin.
	I2CwriteByte(gAddress, subAddress, data, file);
}

unsigned char xmWriteByte(unsigned char subAddress, unsigned char data, unsigned char xmAddress, int file) {
	// Whether we're using I2C or SPI, write a char using the
	// accelerometer-specific I2C address or SPI CS pin.
	return I2CwriteByte(xmAddress, subAddress, data, file);
}

unsigned char gReadByte(unsigned char subAddress, unsigned char gAddress, int file) {
	return I2CreadByte(gAddress, subAddress, file);
}

int gReadBytes(unsigned char subAddress, unsigned char * dest, 
		unsigned char count, unsigned char gAddress, int file) {
	// Whether we're using I2C or SPI, read multiple chars using the
	// gyro-specific I2C address or SPI CS pin.
	return I2CreadBytes(gAddress, subAddress, dest, count, file);
}

unsigned char xmReadByte(unsigned char subAddress, unsigned char xmAddress, int file) {
	// Whether we're using I2C or SPI, read a char using the
	// accelerometer-specific I2C address or SPI CS pin.
	return I2CreadByte(xmAddress, subAddress, file);
}

void xmReadBytes(unsigned char subAddress, unsigned char * dest, unsigned char count, unsigned char xmAddress, int file) {
	// Whether we're using I2C or SPI, read multiple chars using the
	// accelerometer-specific I2C address or SPI CS pin.
	I2CreadBytes(xmAddress, subAddress, dest, count, file);
}

int initI2C(int I2CBus, int file) {
	printf("initialising I2C\n");
	//Open I2C file as bidirectional
	char namebuf[MAX_BUS];
	snprintf(namebuf, sizeof(namebuf), "/dev/i2c-%d", I2CBus);
	if ((file=open(namebuf, O_RDWR)) < 0) {
		printf("Failed to open sensor on %s I2C Bus\n",namebuf);
	} else printf("Sensor is openned\n"); 
	return file;
}

int I2CwriteByte(unsigned char address, unsigned char subAddress, unsigned char data, int file) {
	// void *ptrData = &subAddress;
	unsigned char buf[2];
	buf[0] = subAddress;
	buf[1] = data;
	int status = ioctl(file, I2C_SLAVE, address);
	if (status < 0) {
		printf("I2C_SLAVE address 0x%02X failed...(#%i)\n", address, status);
		getchar();
	} else {
		status = write(file, buf, 2);
		if (status != 2) 
			printf("I2C_SLAVE register 0x%02X location not found..\n", subAddress);
		//else {
			//ptrData = &data;	
			//status = write(file, ptrData, 1);
			//if (status != 1) 
				//printf("I2C_SLAVE register 0x%02X location not found..\n", subAddress);
			//else {
				//for (i = 0; i < 100000; i++);
				//ptrData = &check;
				//if (read(file, ptrData, 1) < 1) 
					//printf("I2C_SLAVE register 0x%02X read failed..\n", subAddress);
				//else
					//printf("%ud bytes written. Register reads 0x%2X\n", status, check);
			//}
	}
	return status;
}

unsigned char I2CreadByte(unsigned char address, unsigned char subAddress, int file) {
	unsigned char buf;
	void *ptrData = &subAddress;
	int status = ioctl(file, I2C_SLAVE, address);
	if (status < 0) {
		printf("I2C_SLAVE address 0x%02X failed...(#%i)\n", address, status);
		getchar();
	} else { 
		if (write(file, ptrData, 1) < 1) 
			printf("I2C_SLAVE register 0x%02X location not found..\n", subAddress);
		else {
			ptrData = &buf;
			if (read(file, ptrData, 1) < 1) 
				printf("I2C_SLAVE register 0x%02X read failed..\n", subAddress);
			write(file, ptrData,0);
			return buf; // 
			// i2c_smbus_read_byte_data(file, subAddress);
		}
	}
	return -1;
}

int I2CreadBytes(unsigned char address, unsigned char subAddress, unsigned char * dest, unsigned char count, int file) {
	unsigned char sAddr = subAddress | 0x80; // or with 0x80 to indicate multi-read
	void *ptrData = &sAddr;
	int status = ioctl(file, I2C_SLAVE, address);
	if (status < 0) {
		printf("I2C_SLAVE address 0x%02X failed...(#%i)\n", address, status);
		getchar();
	} else {
		// i2c_smbus_read_i2c_block_data(file, sAddr, count, dest);
		status = write(file, ptrData, 1);
		if (status != 1)  
			printf("I2C_SLAVE register 0x%02X location not found..\n", subAddress);
		else {
			status = read(file, dest, count);
			if (status != count)
				printf("I2C_SLAVE register 0x%02X read failed..\n", subAddress);
			else write(file, ptrData,0);
		}
	}
	return status;

}
