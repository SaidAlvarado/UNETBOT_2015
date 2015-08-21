#include <Wire.h>
#include "ahrs.h"
#include "HMC5883L.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "BMP085.h"

#define OUTPUT_EULER 0 //heading, pitch, roll
#define OUTPUT_MATRIX 1 //a north-east-down rotation matrix
#define OUTPUT_QUATERNION 2 //a quaternion!

BMP085 bmp;
HMC5883L hmc;
ADXL345 adxl(false);
ITG3200 itg(false);

int output_mode = OUTPUT_EULER;
bool auto_output = true;

unsigned long delta_t = 0;

template <class T> class BrownLinearExpo
{
	public:
		BrownLinearExpo() { a = 0.5; }
		BrownLinearExpo(float f, volatile T init_est) { a = f; estimate = init_est; }

		void set_factor(float factor) { a = factor; }
		void step(volatile T measurement)
		{
			single_smoothed = a * measurement + (1 - a) * single_smoothed;
			double_smoothed = a * single_smoothed + (1 - a) * double_smoothed;

			T est_a = (2*single_smoothed - double_smoothed);
			T est_b = (a / (1-a) )*(single_smoothed - double_smoothed);
			estimate = est_a + est_b;
		}
		volatile T get(){ return estimate; }

	private:
		T estimate, double_smoothed, single_smoothed;
		float a;
};


BrownLinearExpo<float> alt_filter(0.1, 0);

void output()
{
	switch(output_mode)
	{
		case OUTPUT_EULER:
		{
			imu::Vector<3> euler = uimu_ahrs_get_euler();

			Serial.print("euler: ");
			Serial.print(euler.x());
			Serial.print(" ");
			Serial.print(euler.y());
			Serial.print(" ");
		    Serial.println(euler.z());
		}
		break;
		case OUTPUT_MATRIX:
		{
			imu::Matrix<3> m = uimu_ahrs_get_matrix();
			Serial.print("matrix: ");

			int i, j;
			for(i = 0; i <= 2; i++)
			{
				for(j = 0; j <= 2; j++)
				{
					Serial.print(m.cell(i, j)); Serial.print(" ");
				}
			}
			Serial.print("\n");
		}
		break;
		case OUTPUT_QUATERNION:
		{
			imu::Quaternion rotation;
			rotation = uimu_ahrs_get_quaternion();
			Serial.print("quat: ");
			Serial.print(rotation.w());
			Serial.print(" ");
			Serial.print(rotation.x());
			Serial.print(" ");
			Serial.print(rotation.y());
			Serial.print(" ");
			Serial.println(rotation.z());
		}
		break;
	}

	alt_filter.step(bmp.read_altitude()*0.032808399);
	Serial.print("alt: ");
	Serial.println(alt_filter.get());
	Serial.print("temp: ");
	Serial.println(bmp.read_temp()/10.0);
}


void mag_cal()
{
	for(int i = 0; i < 6; i++)
		hmc.cal[i] = 0;

	for(int i = 0; i < 3000; i++)
	{
		imu::Vector<3> mag = hmc.read_raw();

		if(mag[0] < hmc.cal[0])
		    hmc.cal[0] = mag[0];
		if(mag[0] > hmc.cal[1])
		    hmc.cal[1] = mag[0];

		if(mag[1] < hmc.cal[2])
		    hmc.cal[2] = mag[1];
		if(mag[1] > hmc.cal[3])
		    hmc.cal[3] = mag[1];

		if(mag[2] < hmc.cal[4])
		    hmc.cal[4] = mag[2];
		if(mag[2] > hmc.cal[5])
		    hmc.cal[5] = mag[2];

		delay(20);
	}

	Serial.print("mag_cal:");
	for(int i = 0; i < 6; i++)
	{
		Serial.print(" ");
		Serial.print(int(hmc.cal[i]));
	}
	Serial.print("\n");
	delay(10000);
}

void setup()
{
	I2C_BUS.begin();
	Serial.begin(115200);
	Serial.setTimeout(1);
    Serial.println("Core Electronics 10 DOF AHRS & Alt");
	Serial.println("www.CamelSoftware.com");

	I2C_BUS.begin();

	hmc.init();
	delay(10);
	adxl.init();
	delay(10);
	itg.init();
	delay(10);

	itg.zeroCalibrate(128, 5);

	imu::Vector<3> acc = adxl.read_acc();
	imu::Vector<3> mag = hmc.read_mag();

	uimu_ahrs_init(acc, mag);
	uimu_ahrs_set_beta(0.1);

	bmp.init();
	bmp.zeroCal(0, 0);
}



void loop()
{
    if((millis() - delta_t) < 20)
        return;

	float dt = millis() - delta_t;
	delta_t = millis();
	dt /= 1000.0;

	uimu_ahrs_iterate(itg.read_gyro(), adxl.read_acc(), hmc.read_mag());

	if(auto_output)
		output();

	char buffer[20];
	memset(buffer, '\0', 20);
	Serial.readBytesUntil('\n', buffer, 20);

	if(strncmp(buffer, "output_euler", 12) == 0)
		output_mode= OUTPUT_EULER;
	
	if(strncmp(buffer, "output_mat", 11) == 0)
		output_mode = OUTPUT_MATRIX;

	if(strncmp(buffer, "output_quat", 12) == 0)
		output_mode = OUTPUT_QUATERNION;

	if(strncmp(buffer, "auto_on", 7) == 0)
		auto_output = true;

	if(strncmp(buffer, "auto_off", 8) == 0)
		auto_output = false;

	if(strncmp(buffer, "read", 4) == 0)
		output();

	if(strncmp(buffer, "set_offset", 10) == 0)
	{
		uimu_ahrs_set_offset(uimu_ahrs_get_imu_quaternion());
	}

	if(strncmp(buffer, "mag_cal", 7) == 0)
	{
		mag_cal();
	}

	if(strncmp(buffer, "cal_data:", 9) == 0)
	{
		scanf("cal_data: %i %i %i %i %i %i", hmc.cal[0], hmc.cal[1], hmc.cal[2], hmc.cal[3], hmc.cal[4], hmc.cal[5]);
	}
}

