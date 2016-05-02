#ifndef AHRS_H_
#define AHRS_H_

#include <cmath>
#include "gmtl/gmtl.h"
#include "SensorData.h"

using namespace gmtl;

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2
#define ROLL 0
#define PITCH 1
#define YAW 2
#define ACCEL_CUTOFF 0.25
#define KP_MAG 5
#define KI_MAG 0
#define KP_ACC 1
#define KI_ACC 0

class AHRS
{
private:
	float exAcc, eyAcc, ezAcc; // accel error
	float exAccInt, eyAccInt, ezAccInt; // accel integral error

	float exMag, eyMag, ezMag; // mag error
	float exMagInt, eyMagInt, ezMagInt; // mag integral error

	float kpAcc, kiAcc;

	// auxiliary variables to reduce number of repeated operations
	float q0, q1, q2, q3;
	float q0q0, q0q1, q0q2, q0q3;
	float q1q1, q1q2, q1q3;
	float q2q2, q2q3;
	float q3q3;

	Vec3d eAcc;
	Vec3d eAccInt;
	Vec3d eMag;
	Vec3d eMagInt;
	Quatd q;

	float halfT;

	bool initialized;

public:
	double attitude[3];

	Vec3d e_accel;

	AHRS() {
		initialized = false;
		q0 = 1.0f;	q1 = 0.0f;	q2 = 0.0f;	q3 = 0.0f;
		exAcc    = 0.0f;    eyAcc = 0.0f;    ezAcc = 0.0f;
		exAccInt = 0.0f; eyAccInt = 0.0f; ezAccInt = 0.0f;
		exMag    = 0.0f; eyMag    = 0.0f; ezMag    = 0.0f;
		exMagInt = 0.0f; eyMagInt = 0.0f; ezMagInt = 0.0f;
	}

	void initialize(Sensors s)
	{
	    float initialRoll, initialPitch;
	    float cosRoll, sinRoll, cosPitch, sinPitch;
	    float magX, magY;
	    float initialHdg, cosHeading, sinHeading;

	    float ax, ay, az, mx, my, mz;
	    ax = s.accel[XAXIS];
	    ay = s.accel[YAXIS];
	    az = s.accel[ZAXIS];
	    mx = s.mag[XAXIS];
	    my = s.mag[YAXIS];
	    mz = s.mag[ZAXIS];

	    initialRoll  = atan2(-ay, -az);
	    initialPitch = atan2( ax, -az);

	    cosRoll  = cosf(initialRoll);
	    sinRoll  = sinf(initialRoll);
	    cosPitch = cosf(initialPitch);
	    sinPitch = sinf(initialPitch);

	    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

	    magY = my * cosRoll - mz * sinRoll;

	    initialHdg = atan2f(-magY, magX);

	    cosRoll = cosf(initialRoll * 0.5f);
	    sinRoll = sinf(initialRoll * 0.5f);

	    cosPitch = cosf(initialPitch * 0.5f);
	    sinPitch = sinf(initialPitch * 0.5f);

	    cosHeading = cosf(initialHdg * 0.5f);
	    sinHeading = sinf(initialHdg * 0.5f);

	    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
	    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
	    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
	    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

	    // auxillary variables to reduce number of repeated operations, for 1st pass
	    q0q0 = q0 * q0;
	    q0q1 = q0 * q1;
	    q0q2 = q0 * q2;
	    q0q3 = q0 * q3;
	    q1q1 = q1 * q1;
	    q1q2 = q1 * q2;
	    q1q3 = q1 * q3;
	    q2q2 = q2 * q2;
	    q2q3 = q2 * q3;
	    q3q3 = q3 * q3;
	}

	void update(Sensors s, float dt, bool magDataUpdate) {
	    float norm, normR;
	    float hx, hy, hz, bx, bz;
	    float vx, vy, vz, wx, wy, wz;
	    float q0i, q1i, q2i, q3i;

	    float gx, gy, gz, ax, ay, az, mx, my, mz;
	    gx = s.gyro[XAXIS];
	    gy = s.gyro[YAXIS];
	    gz = s.gyro[ZAXIS];
	    ax = s.accel[XAXIS];
	    ay = s.accel[YAXIS];
	    az = s.accel[ZAXIS];
	    mx = s.mag[XAXIS];
	    my = s.mag[YAXIS];
	    mz = s.mag[ZAXIS];

	    if ((initialized == false) && (magDataUpdate == true)) {
	        initialize(s);
	        initialized = true;
	    }
	    if (initialized == true) {
	        halfT = dt * 0.5f;

	        norm = sqrt(ax*ax + ay*ay + az*az);//accel

	        if (norm != 0.0f) {
	            kpAcc = KP_ACC; //KpAcc = 1.0f; always 1
	            kiAcc = KI_ACC; //KiAcc = 0.0f; always 0

	            normR = 1.0f / norm;
	            ax *= normR;
	            ay *= normR;
	            az *= normR;

	            // estimated direction of gravity (v)
	            vx = 2.0f * (q1q3 - q0q2);
	            vy = 2.0f * (q0q1 + q2q3);
	            vz = q0q0 - q1q1 - q2q2 + q3q3;

	            // error is sum of cross product between reference direction
			    // of fields and direction measured by sensors
			    exAcc = vy * az - vz * ay;
	            eyAcc = vz * ax - vx * az;
	            ezAcc = vx * ay - vy * ax;

	            gx += exAcc * kpAcc;
	            gy += eyAcc * kpAcc;
	            gz += ezAcc * kpAcc;

	            if (kiAcc > 0.0f) {
			    	exAccInt += exAcc * kiAcc;
	                eyAccInt += eyAcc * kiAcc;
	                ezAccInt += ezAcc * kiAcc;

	                gx += exAccInt;
	                gy += eyAccInt;
	                gz += ezAccInt;
			    }
		    }

	        //-------------------------------------------mag

	        norm = sqrt(mx*mx + my*my + mz*mz);

	        if ((magDataUpdate == true) && (norm != 0.0f)) {
	            normR = 1.0f / norm;
	            mx *= normR;
	            my *= normR;
	            mz *= normR;

	            // compute reference direction of flux
	            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
	            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
	            hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
	            bx = sqrt((hx * hx) + (hy * hy));
	            bz = hz;

	            // estimated direction of flux (w)
	            wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
	            wy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
	            wz = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));

	            exMag = my * wz - mz * wy;
	            eyMag = mz * wx - mx * wz;
	            ezMag = mx * wy - my * wx;

				// use un-extrapolated old values between magnetometer updates
				// dubious as dT does not apply to the magnetometer calculation so
				// time scaling is embedded in KpMag and KiMag
				gx += exMag * KP_MAG;
				gy += eyMag * KP_MAG;
				gz += ezMag * KP_MAG;

				if (KI_MAG > 0.0f) {
					exMagInt += exMag * KI_MAG;
					eyMagInt += eyMag * KI_MAG;
					ezMagInt += ezMag * KI_MAG;

					gx += exMagInt;
					gy += eyMagInt;
					gz += ezMagInt;
				}
	        }

	        //-------------------------------------------

	        // integrate quaternion rate
	        q0i = (-q1 * gx - q2 * gy - q3 * gz) * halfT; //-x-y-z
	        q1i = ( q0 * gx + q2 * gz - q3 * gy) * halfT; //x-yz
	        q2i = ( q0 * gy - q1 * gz + q3 * gx) * halfT; //xy-z
	        q3i = ( q0 * gz + q1 * gy - q2 * gx) * halfT; //-xyz
	        q0 += q0i;
	        q1 += q1i;
	        q2 += q2i;
	        q3 += q3i;

	        // normalise quaternion
	        normR = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	        q0 *= normR;
	        q1 *= normR;
	        q2 *= normR;
	        q3 *= normR;

	        // auxiliary variables to reduce number of repeated operations
	        q0q0 = q0 * q0;
	        q0q1 = q0 * q1;
	        q0q2 = q0 * q2;
	        q0q3 = q0 * q3;
	        q1q1 = q1 * q1;
	        q1q2 = q1 * q2;
	        q1q3 = q1 * q3;
	        q2q2 = q2 * q2;
	        q2q3 = q2 * q3;
	        q3q3 = q3 * q3;

	        attitude[ROLL ] = atan2f( 2.0f * (q0q1 + q2q3), q0q0 - q1q1 - q2q2 + q3q3 );
			attitude[PITCH] = -asinf( 2.0f * (q1q3 - q0q2) );
			attitude[YAW  ] = atan2f( 2.0f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3 );

		    rotationMatrix.mData[0] = q0q0 + q1q1 -q2q2 - q3q3;
		    rotationMatrix.mData[1] = 2.0f * (q1q2 - q0q3);
		    rotationMatrix.mData[2] = 2.0f * (q0q2 + q1q3);
		    rotationMatrix.mData[3] = 2.0f * (q1q2 + q0q3);
		    rotationMatrix.mData[4] = q0q0 - q1q1 + q2q2 - q3q3;
		    rotationMatrix.mData[5] = 2.0f * (q2q3 - q0q1);
		    rotationMatrix.mData[6] = 2.0f * (q1q3 - q0q2);
		    rotationMatrix.mData[7] = 2.0f * (q0q1 + q2q3);
		    rotationMatrix.mData[8] = q0q0 - q1q1 - q2q2 + q3q3;
	    }
	}

	Matrix33d rotationMatrix;

	void computeEarthAccel(Sensors s)
	{
	    e_accel = rotationMatrix*Vec3d(s.accel[XAXIS], s.accel[YAXIS], s.accel[ZAXIS]) + Vec3d(0, 0, 9.80655);


	    // Do some filtering on the result
	    //earthAxisAccels[XAXIS] = firstOrderFilter(earthAxisAccels[XAXIS], &firstOrderFilters[EARTH_AXIS_ACCEL_X_HIGHPASS]);
	    //earthAxisAccels[YAXIS] = firstOrderFilter(earthAxisAccels[YAXIS], &firstOrderFilters[EARTH_AXIS_ACCEL_Y_HIGHPASS]);
	    //earthAxisAccels[ZAXIS] = firstOrderFilter(earthAxisAccels[ZAXIS], &firstOrderFilters[EARTH_AXIS_ACCEL_Z_HIGHPASS]);
	}
};

#endif
