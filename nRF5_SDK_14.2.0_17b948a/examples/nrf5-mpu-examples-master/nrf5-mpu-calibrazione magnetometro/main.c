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
#include "mpu9150_register_map.h"


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
	
		/*ICM20948_reset();
	
		ICM20948_selectAutoClockSource();
		//ICM20948_enableI2cMaster(); //da valutare se inserire
				
		ICM20948_enableAccelGyro();
		ICM20948_configAccel();
		ICM20948_configGyro();
		ICM20948_setGyroSrd(21); //19 nella versione MPU. ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]) 1100/(1+21) = 50 Hz
		ICM20948_setAccelSrd(21);//19 nella versione MPU (in ICM sono due registri separati, mentre in MPU è unico )
		//ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]) 1125/(21+1) = 51.136 Hz
		*/
		ICM20948_resetMag();
		app_icm_magn_config_t magnetometer_config;
		magnetometer_config.mode = MAG_CNTL2_MODE_100HZ; //continuous measurment mode 4 = 100 Hz
		ret_code = app_icm_magnetometer_init(&magnetometer_config);	

		APP_ERROR_CHECK(ret_code);
}


/**
 * @brief Function for main application entry.
 */

//int main(void)
//{    
//		app_timer_init();
//    uint32_t err_code;
//    
//		float magcalibrex;
//		float magcalibrey;
//		float magcalibrez;
//	  float mx;
//	  float my;
//	  float mz;
//    // Initialize.
//    log_init();
//		NRF_LOG_INFO("\033[2J\033[;H"); // Clear screen
//    
//    icm_init();
//    
//    // Start execution.
//    NRF_LOG_INFO("MPU Free Fall Interrupt example.");
//    
//		
//		magn_values_t magn_values;
//		magn_values_t magnbis_values;
//		
//    
//    const uint8_t MAG_DATA_SIZE = 10;
//    uint8_t magn_data[MAG_DATA_SIZE];
//    memset(magn_data, 0, MAG_DATA_SIZE);
//		err_code=app_mpu_read_magnbis(&magnbis_values,NULL);
//		magcalibrex=(magnbis_values.x-128)/256.0f+1.f;
//    magcalibrey=(magnbis_values.y-128)/256.0f+1.f;
//    magcalibrez=(magnbis_values.z-128)/256.0f+1.f;
//		
//		uint16_t ii=0;
//		
//		
//    while(1)
//    {
//        if(NRF_LOG_PROCESS() == false)
//        {
//           if (ii==0)
//					 {
//						 NRF_LOG_INFO("My magcalibrex " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT((float) magcalibrex));
//						 NRF_LOG_INFO("My magcalibrey " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT((float) magcalibrey));
//						 NRF_LOG_INFO("My magcalibrez " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT((float) magcalibrez));
//						 ii++;
//						 nrf_delay_ms(25);
//					 }
//					 err_code = app_mpu_read_magnetometer(&magn_values, NULL);
//           APP_ERROR_CHECK(err_code);
//					 NRF_LOG_INFO("My magn1 " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT((float) magn_values.x));
//					 NRF_LOG_INFO("My magn2 " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT((float) magn_values.y));
//					 NRF_LOG_INFO("My magn3 " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT((float) magn_values.z));
//					 nrf_delay_ms(25); 
//					/*err_code = app_mpu_read_magnetometer(&magn_values, NULL);
//          APP_ERROR_CHECK(err_code);
//          mx=(((magn_values.x*(-78.82))/0.776)+8398.4)*0.9933;
//					my=(((magn_values.y*(-80.85))/0.776)-4896.8)*1.0315;
//					mz=(((magn_values.z*(-90.81))/0.776)-6202.2)*0.9768;
//					NRF_LOG_INFO("My mx " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT((float) mx));
//					NRF_LOG_INFO("My my " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT((float) my));
//					NRF_LOG_INFO("My mz " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT((float) mz));*/
//					//nrf_delay_ms(50);
//        }
//    }
//}

int main(void)
{    

    uint32_t err_code;
    
	  float mx_bias;
	  float my_bias;
	  float mz_bias;
		float avg_rad;
		float mx_scale;
		float my_scale;
		float mz_scale;
		volatile float x_scale;
		volatile float y_scale;
		volatile float z_scale;
	
		magn_values_t magn_values;
		int16_t max_x;
		int16_t min_x;
		int16_t max_y;
		int16_t min_y;
		int16_t max_z;
		int16_t min_z;
		
//		float magn_array_x[1450];
		
		uint16_t count=0;
    // Initialize.
    log_init();
    
    icm_init();

    while(1)
    {
        if(NRF_LOG_PROCESS() == false)
        {
					while (count<1450)
						{
							
							err_code = app_icm_read_magnetometer(&magn_values, NULL);
							APP_ERROR_CHECK(err_code);
							if (count == 0)
							{
										max_x = magn_values.x;
										min_x = magn_values.x;
										max_y = magn_values.y;
										min_y = magn_values.y;
										max_z = magn_values.z;
										min_z = magn_values.z;								
							}
							else
							{
								if (magn_values.x > max_x)
								{
									max_x = magn_values.x;
								}
								else if (magn_values.x < min_x)
								{
									min_x = magn_values.x;
								}
								
								if (magn_values.y > max_y)
								{
									max_y = magn_values.y;
								}
								else if (magn_values.y < min_y)
								{
									min_y = magn_values.y;
								}
								
								if (magn_values.z > max_z)
								{
									max_z = magn_values.z;
								}
								else if (magn_values.z < min_z)
								{
									min_z = magn_values.z;
								}
							}
							
//							magn_array_x[count]=magn_values.x;
							
							count++;

							nrf_delay_ms(25);
					}
						if (count == 1450)
						{
							mx_bias = (max_x + min_x)/2;
							mx_bias = mx_bias*0.15;
							my_bias = (max_y + min_y)/2;
							my_bias = my_bias*0.15;
							mz_bias = (max_z + min_z)/2;
							mz_bias = mz_bias*0.15;
							
							mx_scale = (max_x - min_x)/2;
							my_scale = (max_y - min_y)/2;
							mz_scale = (max_z - min_z)/2;
							
							avg_rad = mx_scale + my_scale + mz_scale;
							avg_rad = avg_rad/ 3.0;
							
							x_scale = avg_rad/mx_scale;
							y_scale = avg_rad/my_scale;
							z_scale = avg_rad/mz_scale;
							
							count=0;
						}

        }
    }
}
/** @} */

