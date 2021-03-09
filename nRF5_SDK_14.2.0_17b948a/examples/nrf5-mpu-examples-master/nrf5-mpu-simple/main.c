
#include <stdio.h>
#include "boards.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_ICM20948.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "SEGGER_RTT.h"

#define PI 3.1415
/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
	
}


void icm_init(void)
{
	
		/*ICM-20948 initialization and configuration */
		
		ret_code_t ret_code;
	
		// Initiate ICM driver
		ret_code = app_icm_init();
		APP_ERROR_CHECK(ret_code); // Check for errors in return value
	
		ICM20948_reset();
	
		ICM20948_selectAutoClockSource();
//		ICM20948_enableI2cMaster(); //da valutare se inserire
				
		ICM20948_enableAccelGyro();
		ICM20948_configAccel();
		ICM20948_configGyro();
		ICM20948_setGyroSrd(21); //19 nella versione MPU. ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]) 1100/(1+21) = 50 Hz
		ICM20948_setAccelSrd(21);//19 nella versione MPU (in ICM sono due registri separati, mentre in MPU è unico )
		//ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]) 1125/(21+1) = 51.136 Hz
		
		ICM20948_resetMag();
		app_icm_magn_config_t magnetometer_config;
		magnetometer_config.mode = MAG_CNTL2_MODE_100HZ; //continuous measurment mode 4 = 100 Hz
		app_icm_magnetometer_init(&magnetometer_config);	
		//APP_ERROR_CHECK(ret_code);
		
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{    
    uint32_t err_code;
    float deltat=0.025;
		float mx;
		float my;
		float mz;
		float quat[4];
	
	  accel_values_t acc_values;
		magn_values_t magn_values;
		gyro_values_t gyro_values;
    accel_values_float_t acc;
	  gyro_values_float_t gyro;
    // Initialize.
    log_init();
	
		NRF_LOG_INFO("PROVA IMU BREAKOUT KIT, questa è la riga di codice prima di icm_init()");
    
    icm_init();
	
	  NRF_LOG_INFO("e questa è quella subito dopo");
    
    // Start execution.

    
    
    while(1)
    {
       for (int i=0;i<4; i++)
						{
							
							//ICM-20948
							//Accelerometer data read
							err_code = app_icm_read_accel(&acc_values);
							APP_ERROR_CHECK(err_code);
							//sensitivity 2g
//							acc.x=((acc_values.x)/16384.)+0.00274279364;
//							acc.y=((acc_values.y)/16384.)+0.0255990699;
//							acc.z=((acc_values.z)/16384.)-0.02146649;
							//sensitivity 4g
							acc.x=((acc_values.x)/4096.)-0.0217083115;
							acc.y=((acc_values.y)/4096.)+0.0340574495;
							acc.z=((acc_values.z)/4096.)-0.02073884;
							
							//Gyroscope data read
							err_code = app_icm_read_gyro(&gyro_values);
							APP_ERROR_CHECK(err_code);	

							//sensitivity 500 dps	
							gyro.x=(gyro_values.x/65.54)*PI/180.0f-0.00743509503;
							gyro.y=(gyro_values.y/65.54)*PI/180.0f+0.0040876139;
							gyro.z=(gyro_values.z/65.54)*PI/180.0f-0.0127511621;
							
							//Magnetometer data read
							err_code = app_icm_read_magnetometer(&magn_values, NULL);
							//APP_ERROR_CHECK(err_code);
							mx=(((magn_values.x)*0.15)+16.0499992)*1.08809519;
							my=(((magn_values.y)*0.15)+47.0999985)*1.00882995;
							mz=(((magn_values.z)*0.15)-72)*0.917670667;
							
							//MadgwickQuaternionUpdate(acc.x, acc.y, acc.z,gyro.x, gyro.y, gyro.z, mx, my, mz,deltat);
//							MadgwickQuaternionUpdate(0.95,0.002, 0.002,0.01, 0.01, 0.01,50.1, 100.2, 75.3,deltat);
							//quat[0]=q[0]*127;
							//quat[1]=q[1]*127;
							//quat[2]=q[2]*127;
							//quat[3]=q[3]*127;

							nrf_delay_ms(25);
						}
    }
}

/** @} */

