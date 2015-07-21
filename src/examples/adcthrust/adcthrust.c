/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file test_adc.c
 * Test for the analog to digital converter.
 */

#include <px4_config.h>
#include <nuttx/arch.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <poll.h>
#include <sys/mount.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include "systemlib/systemlib.h"
#include "drivers/drv_pwm_output.h"

#include <nuttx/spi.h>
#include <nuttx/fs/ioctl.h>

#include "tests.h"
#include <arch/board/board.h>

#include <nuttx/analog/adc.h>
#include <drivers/drv_adc.h>
#include "systemlib/systemlib.h"
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#define ADC_AIRSPEED_VOLTAGE_CHANNEL	15
#define ADC_BATTERY_VOLTAGE_CHANNEL		2
#define ADC_BATTERY_CURRENT_CHANNEL		3
#define BATT_V_LOWPASS					0.001
#define BATT_V_IGNORE_THRESHOLD			2.5

__EXPORT int adcthrust_main(int argc, char *argv[]);


//bool continueflag = 0; 

// Setup and global variables

/* make space for a maximum of twelve channels */
struct adc_msg_s data[12];
const char *dev = PWM_OUTPUT0_DEVICE_PATH;
hrt_abstime	_last_adc = 0;						/**< last time we took input from the ADC */
//hrt_abstime _battery_current_timestamp = 0;		/**< timestamp of last battery current reading */


// Function parameters for thrust, voltage and current

double calibgain = 0.1972;						/**< calibration gain for load cell */
double caliboffset = 44.9412;					/**< calibration offset for load cell */
double diff_pres_analog_scale = 1000.0;			/**< analog scale for adc differential pressure sensor */
double battery_voltage_scaling = 0.0082;		/**< analog scale for battery voltage */
double battery_current_scaling = 0.0124;		/**< analog scale for battery current */
double data_filtered_voltage = -1.0f;



// Function Prototypes

double measurement_thrust(int fd);
double measurement_voltage(int fd);
double measurement_current(int fd, double batterystatus_voltage);
double avg(double* data1);
double* medianfilt(double* data1);
void pwm_step_input(void);

// Function Definitions

double avg(double* data1){
	double count = 0.0;
	double accum = 0.0;
	for(int i=0;i<10;i++){
		if(data1[i] <-0.9)
			continue;
		accum = accum+data1[i];
		count = count +1;
	}
	// printf("count = %d \n",count);
	// printf("data = %f \n",data1[9]);
	return accum/count;
}
double* medianfilt(double* data1){
	double a,medianVal;
	for(int i=0;i<10;i++){
		for (int j=i+1;j<10;j++){
			if(data1[i]>data1[j]){
				a =  data1[i];
                data1[i] = data1[j];
                data1[j] = a;
			}
		}
	}
	medianVal = data1[5];
	for(int i=0;i<10;i++){
		if ((medianVal-data1[i])*(medianVal-data1[i]) > 1000.0){
			data1[i] = -1.0;
		}
	}
	// printf("medianVal = %f  \n",medianVal);
	// for(int i=0;i<10;i++){
	// 	printf("%f,",data1[i]);
	// }
	// printf("\n");
	return data1;
}

double measurement_thrust(int fd){
	double data_filtered;
	double data_raw_filtered[12];
	/* read all channels available */
	for(int i=0;i<10;i++){
		ssize_t count = read(fd, data, sizeof(data));

		if (count < 0) {
			if (fd != 0) { close(fd); }
			return -1.0;
		}

		unsigned channels = count / sizeof(data[0]);

		for (unsigned j = 0; j < channels; j++) {
			if (ADC_AIRSPEED_VOLTAGE_CHANNEL == data[j].am_channel) {
				double voltage = (double)(data[j].am_data) * 3.3 / 4096.0 * 2.0; 
				double data_raw = voltage*diff_pres_analog_scale;
				data_raw_filtered[i] =  data_raw;
			}

		}
		usleep(5000);
	}
	// for(int ii=0;ii<10;ii++){
	// 	printf("%f,",data_raw_filtered[ii]);
	// }
	// printf("\n");
	data_filtered = avg(medianfilt(data_raw_filtered));
	double calibratedvalue = calibgain*data_filtered + caliboffset;
	return calibratedvalue;
	
	//return -1;

}

double measurement_voltage(int fd){
	double battery_voltage;
	ssize_t count = read(fd, data, sizeof(data));

		if (count < 0) {
			printf("voltageerror1 \n");
			if (fd != 0) { close(fd); }
			return -1;
		}

		unsigned channels = count / sizeof(data[0]);

		for (unsigned j = 0; j < channels; j++) {
			if (ADC_BATTERY_VOLTAGE_CHANNEL == data[j].am_channel) {
				double voltage = (double)(data[j].am_data);  //* 6.6 / 4096.0;
				double data_raw = voltage * battery_voltage_scaling;
				if (data_raw > BATT_V_IGNORE_THRESHOLD) {
						//printf("gettingrawdata %.2f \n", data_raw);
						battery_voltage = data_raw;				//< unfiltered battery voltage>

						// one-time initialization of low-pass value to avoid long init delays 
						if (data_filtered_voltage < BATT_V_IGNORE_THRESHOLD) {
							data_filtered_voltage = data_raw;
						}

						data_filtered_voltage = (data_filtered_voltage + ((battery_voltage - data_filtered_voltage) * BATT_V_LOWPASS));
				} 
				else {
						//mark status as invalid 
						battery_voltage = -1.0f;
						data_filtered_voltage = -1.0f;
						printf("voltageerror2 %.2f \n",data_raw);
				}
			}
		}
	return data_filtered_voltage;
}

double measurement_current(int fd, double batterystatus_voltage){
	ssize_t count = read(fd, data, sizeof(data));

		if (count < 0) {
			printf("current error1 \n");
			if (fd != 0) { close(fd); }
			return -1;
		}

		unsigned channels = count / sizeof(data[0]);

		for (unsigned j = 0; j < channels; j++) {
			if (ADC_BATTERY_CURRENT_CHANNEL == data[j].am_channel) {
				if(batterystatus_voltage > 0.0){
					double current = data[j].am_data * battery_current_scaling;
					printf("gettingcurrent rawdata %.2f \n", current);

					if(current >= 0.0){
						return current;
					} else {
						return -1.0f;
					}
				}
			}
		}
	return -1.0f;	
}

void pwm_step_input(void){

	unsigned pwm_off = 1000;
	unsigned pwm_low = 1060;
	unsigned pwm_high = 1860;
	unsigned channel = 1;
	unsigned pwm_step_time = 2000;
	unsigned phase = 0;
	unsigned phase_counter = 0;
	unsigned const phase_maxcount = 20;
	int ret;

	

	// Open Device
	int fd = open(dev, 0);

	if (fd < 0) {
		err(1, "can't open %s", dev);
	}

	// Disable Safety and Arm the Device

	/* tell IO/FMU that its ok to disable its safety with the switch */
	ret = ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);
	if (ret != OK)
		err(1, "PWM_SERVO_SET_ARM_OK");
	/* tell IO/FMU that the system is armed (it will output values if safety is off) */
	ret = ioctl(fd, PWM_SERVO_ARM, 0);
	if (ret != OK)
		err(1, "PWM_SERVO_ARM");

	warnx("Outputs armed");

	// Get current servo values
	struct pwm_output_values last_spos;
	ret = ioctl(fd, PWM_SERVO_GET(channel), (unsigned long)&last_spos.values);

			if (ret != OK) {
				err(1, "PWM_SERVO_GET(%d)", channel);
			}

	// Perform PWM output

	/* Open console directly to grab CTRL-C signal */
	struct pollfd fds;
	fds.fd = 0; /* stdin */
	fds.events = POLLIN;

	warnx("Step input (0 to 100%%) over %u us ramp", pwm_step_time);

	while(1){
		unsigned pwm_val;
		if (phase == 0) {
			pwm_val = pwm_low;

		} else if (phase == 1) {
			/* ramp - depending how steep it is this ramp will look instantaneous on the output */
			pwm_val = pwm_low + (pwm_high - pwm_low) /(phase_maxcount / (float)phase_counter);

		} else {
			pwm_val = pwm_off;
		}

		ret = ioctl(fd, PWM_SERVO_SET(channel), pwm_val);
		if (ret != OK) {
			err(1, "PWM_SERVO_SET(%d)", channel);
		}

		/* abort on user request */
		char c;
		ret = poll(&fds, 1, 0);

		if (ret > 0) {
			ret = read(0, &c, 1);
			if (ret > 0) {
				/* reset output to the last value */
				ret = ioctl(fd, PWM_SERVO_SET(channel), last_spos.values);

				if (ret != OK) {
					err(1, "PWM_SERVO_SET(%d)", i);
				}
					
				warnx("User abort\n");
				exit(0);
			}
		}

		if (phase == 1) {
			usleep(pwm_step_time / phase_maxcount);

		} else if (phase == 0) {
			usleep(50000);

		} else if (phase == 2) {
			usleep(50000);

		} else {
			break;
		}

		phase_counter++;

		if (phase_counter > phase_maxcount) {
			phase++;
			phase_counter = 0;
		}
	}
}


int adcthrust_main(int argc, char *argv[])
{
	double newmeasurement_thrust = 0.0;
	double initialvalue_thrust = 0.0,measurementVal_thrust = 0.0; 
	double measurementVal_voltage = 0.0;
	double measurementVal_current = 0.0;
	bool toTare = 1;
	
	
	// Open ADC Device
	int fd = open(ADC0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		warnx("ERROR: can't open ADC device");
		return 1;
	}

	hrt_abstime t = hrt_absolute_time();

	while(true) {
		/*if(continueflag){
			continueflag = 0;
			continue;
		}*/

		// rate limit to 20 Hz
		if(t - _last_adc >= 50000){
			measurementVal_thrust = measurement_thrust(fd);
			measurementVal_voltage = measurement_voltage(fd);
			measurementVal_current = measurement_current(fd,measurementVal_voltage);
			
			if(measurementVal_thrust < -0.9){
				printf("no thrust data \n");
				return OK;
			}
			
			if(measurementVal_voltage< -0.9){
				printf("no voltage data \n");
				return OK;
			}

			if(measurementVal_current < -0.9){
				printf("no current data");
				return OK;
			}


			if(toTare){
				initialvalue_thrust = measurementVal_thrust;
				toTare = 0;
			}
		
			newmeasurement_thrust = measurementVal_thrust- initialvalue_thrust;
			if(newmeasurement_thrust<1100.0){
				//usleep(10000);
				

				printf(" Thrust value is = %.2f ,%d grams. \t", newmeasurement_thrust, (int)newmeasurement_thrust);
				printf(" Voltage value is = %.2f  V. \t", measurementVal_voltage);
				printf(" Current value is = %.2f  A. \n", measurementVal_current);
			}
		}
	}
	_last_adc = t;
	//warnx("\t ADC test successful.\n");
	return OK;
}
