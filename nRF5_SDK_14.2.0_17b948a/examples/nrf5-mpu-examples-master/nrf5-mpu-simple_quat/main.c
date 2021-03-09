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
#include "app_mpu.h"
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
 
 
// const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(0);

//APP_TIMER_DEF(m_repeated_timer_id); 
 

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}



void mpu_init(void)
{
    ret_code_t ret_code;
    // Initiate MPU driver
    ret_code = app_mpu_init();
    APP_ERROR_CHECK(ret_code); // Check for errors in return value
    
    // Setup and configure the MPU with intial values
    app_mpu_config_t p_mpu_config = MPU_DEFAULT_CONFIG(); // Load default values
    p_mpu_config.smplrt_div = 19;   // Change sampelrate. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). 19 gives a sample rate of 50Hz
    p_mpu_config.accel_config.afs_sel = AFS_2G; // Set accelerometer full scale range to 2G
    ret_code = app_mpu_config(&p_mpu_config); // Configure the MPU with above values
    APP_ERROR_CHECK(ret_code); // Check for errors in return value 
	
	// Enable magnetometer
		app_mpu_magn_config_t magnetometer_config;
		magnetometer_config.mode = CONTINUOUS_MEASUREMENT_100Hz_MODE;
    ret_code = app_mpu_magnetometer_init(&magnetometer_config);
    APP_ERROR_CHECK(ret_code); // Check for errors in return value
}

/**
 * @brief Function for main application entry.
 */


int main(void)
{    
    uint32_t err_code;
    int i=0;
		//float deltat=50.0f;
	  float deltat=100;
		float mx;
		float my;
		float mz;
		float quat[4];
    // Initialize.
    log_init();
		NRF_LOG_INFO("\033[2J\033[;H"); // Clear screen
    
    mpu_init();
    // Start execution.
    //NRF_LOG_INFO("MPU Free Fall Interrupt example.");
    
		accel_values_t acc_values;
		magn_values_t magn_values;
		gyro_values_t gyro_values;
    accel_values_float_t acc;
	  gyro_values_float_t gyro;
		
    while(1)
    {
        if(NRF_LOG_PROCESS() == false)
        {
            if (i==2)
						{
							i=0;
						}
					// Read accelerometer sensor values
            err_code = app_mpu_read_accel(&acc_values);
            APP_ERROR_CHECK(err_code);
					  acc.x=((acc_values.x)/16384.)-0.04;
					  acc.y=((acc_values.y)/16384.)-0.03;
					  acc.z=((acc_values.z)/16384.);
						
						
					// Read magnetometer sensor values
						err_code = app_mpu_read_magnetometer(&magn_values, NULL);
            APP_ERROR_CHECK(err_code);
						//mx=(((magn_values.x*(-78.82))/0.776)+8398.4)*0.9933;
					  //my=(((magn_values.y*(-80.85))/0.776)-4896.8)*1.0315;
					  //mz=(((magn_values.z*(-90.81))/0.776)-6202.2)*0.9768;
						mx=(((magn_values.x*(-78.82))/0.667)+10399.)*0.9933;
					  my=(((magn_values.y*(-80.85))/0.667)-5697.1)*1.0315;
					  mz=(((magn_values.z*(-90.81))/0.667)-7215.8)*0.9768;
            
					
					// Read gyro sensor values
						err_code = app_mpu_read_gyro(&gyro_values);
            APP_ERROR_CHECK(err_code);
					  gyro.x=(gyro_values.x/65.54)*PI/180.0f;
					  gyro.y=(gyro_values.y/65.54)*PI/180.0f;
						gyro.z=(gyro_values.z/65.54)*PI/180.0f;
						
						
						MadgwickQuaternionUpdate(acc.x,acc.y, acc.z,gyro.x, gyro.y, gyro.z,mx, my, mz,deltat);
						//MadgwickQuaternionUpdate(0.002,0.002, 0.95,0.01, 0.01, 0.01,50.1, 100.2, 75.3,deltat);
						quat[0]=q[0]*127;
						quat[1]=q[1]*127;
						quat[2]=q[2]*127;
						quat[3]=q[3]*127;
						NRF_LOG_INFO("My quat0 " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(quat[0]));
						NRF_LOG_INFO("My quat1 " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(quat[1]));
						NRF_LOG_INFO("My quat2 " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(quat[2]));
						NRF_LOG_INFO("My quat3 " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(quat[3]));
						//NRF_LOG_INFO("My gyro1 " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT((float) (acc.x)));
						//NRF_LOG_INFO("My gyro2 " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT((float) (my)));
						//NRF_LOG_INFO("My gyro3 " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT((float) (mz)));
						/*if (i==0)
							{
						NRF_LOG_INFO("My quat0 " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(quat[0]));
						NRF_LOG_INFO("My quat1 " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(quat[1]));
						NRF_LOG_INFO("My quat2 " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(quat[2]));
						NRF_LOG_INFO("My quat3 " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(quat[3]));
						 }
							i++;*/
             nrf_delay_ms(100);
            // Clear terminal and print values
            
						
					
           
        }
    }
}

/** @} */

