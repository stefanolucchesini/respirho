 /* 
  * This example is not extensively tested and only 
  * meant as a simple explanation and for inspiration. 
  * NO WARRANTY of ANY KIND is provided. 
  */

#include <stdio.h>
#include <stdint.h>
#include "boards.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_ICM20948.h"
#include "nrf_drv_mpu.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_clock.h"
#include <app_timer.h>
#include "quat.h"


/**@brief Function for initializing the nrf log module.
 */
 
 
 const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(0);

APP_TIMER_DEF(m_repeated_timer_id); 
 

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
		//ICM20948_enableI2cMaster(); //da valutare se inserire
				
		ICM20948_enableAccelGyro();
		ICM20948_configAccel();
		ICM20948_configGyro();
		ICM20948_setGyroSrd(21); //19 nella versione MPU. ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]) 1100/(1+21) = 50 Hz
		ICM20948_setAccelSrd(21);//19 nella versione MPU (in ICM sono due registri separati, mentre in MPU è unico )
		//ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]) 1125/(21+1) = 51.136 Hz
		/*
		ICM20948_resetMag();
		app_icm_magn_config_t magnetometer_config;
		magnetometer_config.mode = MAG_CNTL2_MODE_100HZ; //continuous measurment mode 4 = 100 Hz
		ret_code = app_icm_magnetometer_init(&magnetometer_config);	

		APP_ERROR_CHECK(ret_code);
		*/
}

/**
 * @brief Function for main application entry.
 */

int main(void)
{    

    uint32_t err_code;
		accel_values_t acc_values;
		accel_values_float_t accelbias;
		gyro_values_t gyro_values;
		gyro_values_float_t gyrobias;
		int count=0;
//		float acc_array_x[1450];
//		float acc_array_y[1450];
//		float acc_array_z[1450];
//		float gyro_array_x[1450];
	
    // Initialize.
    log_init();    
    icm_init();
    
    while(1)
    {
        if(NRF_LOG_PROCESS() == false)
        {
         
					while (count<1450)
						{
							err_code = app_icm_read_accel(&acc_values);
							APP_ERROR_CHECK(err_code);
							accelbias.x=(acc_values.x/16384.)+accelbias.x;
							accelbias.y=(acc_values.y/16384.)+accelbias.y;
							accelbias.z=(acc_values.z/16384.)+accelbias.z;
							
//							acc_array_x[count] = acc_values.x/16384.;
//							acc_array_y[count] = acc_values.y/16384.;
//							acc_array_z[count] = acc_values.z/16384.;
							
							err_code = app_icm_read_gyro(&gyro_values);
							APP_ERROR_CHECK(err_code);
							gyrobias.x=((gyro_values.x/65.54)*PI/180.0f)+gyrobias.x;
							gyrobias.y=((gyro_values.y/65.54)*PI/180.0f)+gyrobias.y;
							gyrobias.z=((gyro_values.z/65.54)*PI/180.0f)+gyrobias.z;
							
//							gyro_array_x[count]=(gyro_values.x/16.4)*PI/180.0f;
							
							count++;

							nrf_delay_ms(25);
						}
						if (count==1450)
						{
							accelbias.x=accelbias.x/1450;
							accelbias.y=accelbias.y/1450;
							accelbias.z=accelbias.z/1450;
							
							gyrobias.x=gyrobias.x/1450;
							gyrobias.y=gyrobias.y/1450;
							gyrobias.z=gyrobias.z/1450;
							
							count=0;

						}          
        }
    }
}

/** @} */

