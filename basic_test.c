#include "lsm9ds0.h"
#include <unistd.h>
#include <stdio.h>

#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW

int main() {
		struct IMU dof;
		dof.I2CBus = 2;
		dof.gAddress = LSM9DS0_G;
		dof.xmAddress = LSM9DS0_XM;
		enum gyro_scale gScl = G_SCALE_245DPS;
		enum accel_scale aScl = A_SCALE_2G;
		enum mag_scale mScl = M_SCALE_2GS;
		enum gyro_odr gODR = G_ODR_95_BW_125;
		enum accel_odr aODR = A_ODR_50; 
		enum mag_odr mODR = M_ODR_50;
		int status = initIMU(gScl, aScl, mScl, gODR, aODR, mODR, &dof);
		printf("Status: %i\n", status);
		int count = 0;
		while (count < 10) {
			count++;
			printf("count %i\n", count);
			// To read from the accelerometer, you must first call the
			// readAccel() function. When this exits, it'll update the
			// ax, ay, and az variables with the most current data.
			readAccel(&dof);
	
			// Now we can use the ax, ay, and az variables as we please.
			// Either print them as raw ADC values, or calculated in g's.
			// If you want to print calculated values, you can use the
			// calcAccel helper function to convert a raw ADC value to
			// g's. Give the function the value that you want to convert.
			printf( "A: %f, %f, %f\n",calcAccel(dof.ax,dof.aRes),
				calcAccel(dof.ay,dof.aRes),calcAccel(dof.az,dof.aRes));

			readGyro(&dof);
	
			// Now we can use the gx, gy, and gz variables as we please.
			// Either print them as raw ADC values, or calculated in DPS.
			printf( "G: %f, %f, %f\n",calcGyro(dof.gx,dof.gRes),
				calcGyro(dof.gy,dof.gRes),calcGyro(dof.gz,dof.gRes));

			readMag(&dof);
			
			printf( "M: %f, %f, %f\n",calcMag(dof.mx,dof.mRes),
				calcMag(dof.my,dof.mRes),calcMag(dof.mz,dof.mRes));
			readTemp(&dof);
			printf("T: %f\n", (21.0 + (float)dof.temperature/8.));
			sleep(1);
		}
	return 0;
}
	
