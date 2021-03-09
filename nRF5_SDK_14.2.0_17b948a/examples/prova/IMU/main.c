

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "mpu6050.h"



#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"




// main code

int main(void)
{

// initialize the logger
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
	
	
// create arrays which will hold x,y & z co-ordinates values of acc and gyro
    static int16_t AccValue[3], GyroValue[3];

    //bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS); // initialize the leds and buttons

    twi_master_init(); // initialize the twi 
    nrf_delay_ms(1000); // give some delay

    while(mpu6050_init() == false) // wait until MPU6050 sensor is successfully initialized
    {
      NRF_LOG_INFO("MPU_6050 initialization failed!!!"); // if it failed to initialize then print a message
      nrf_delay_ms(1000);
    }

   NRF_LOG_INFO("MPU6050 Init Successfully!!!"); 

   NRF_LOG_INFO("Reading Values from ACC & GYRO"); // display a message to let the user know that the device is starting to read the values
   nrf_delay_ms(2000);


  
    
    while (true)
    {
        if(MPU6050_ReadAcc(&AccValue[0], &AccValue[1], &AccValue[2]) == true) // Read acc value from mpu6050 internal registers and save them in the array
        {
          NRF_LOG_INFO("ACC Values:  x = %d  y = %d  z = %d", AccValue[0], AccValue[1], AccValue[2]); // display the read values
        }
        else
        {
          NRF_LOG_INFO("Reading ACC values Failed!!!"); // if reading was unsuccessful then let the user know about it
        }


        if(MPU6050_ReadGyro(&GyroValue[0], &GyroValue[1], &GyroValue[2]) == true) // read the gyro values from mpu6050's internal registers and save them in another array
        {
          NRF_LOG_INFO("GYRO Values: x = %d  y = %d  z = %d", GyroValue[0], GyroValue[1], GyroValue[2]); // display then values
        }

        else
        {
          NRF_LOG_INFO("Reading GYRO values Failed!!!");
        }

       nrf_delay_ms(100); // give some delay 


    }
}

/** @} */
