/** 
 * File main.c
 * The file contains the functions to initialize the interface with the ICM-20948 and to initialize and set
 * the ANT communication protocol.
 * In the for loop the sensor data reading is performed and, then, the quaternions are created and sent.
 *
 *
 * This software is subject to the ANT+ Shared Source License
 * www.thisisant.com/swlicenses
 * Copyright (c) Dynastream Innovations, Inc. 2014
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * 1) Redistributions of source code must retain the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2) Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 * 
 * 3) Neither the name of Dynastream nor the names of its
 *    contributors may be used to endorse or promote products
 *    derived from this software without specific prior
 *    written permission.
 * 
 * The following actions are prohibited:
 * 1) Redistribution of source code containing the ANT+ Network
 *    Key. The ANT+ Network Key is available to ANT+ Adopters.
 *    Please refer to http://thisisant.com to become an ANT+
 *    Adopter and access the key.
 * 
 * 2) Reverse engineering, decompilation, and/or disassembly of
 *    software provided in binary form under this license.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE HEREBY
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; DAMAGE TO ANY DEVICE, LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE. SOME STATES DO NOT ALLOW
 * THE EXCLUSION OF INCIDENTAL OR CONSEQUENTIAL DAMAGES, SO THE
 * ABOVE LIMITATIONS MAY NOT APPLY TO YOU.
 * 
 */
/**@file
 * @defgroup ant_broadcast_rx_example ANT Broadcast RX Example
 * @{
 * @ingroup nrf_ant_broadcast
 *
 * @brief Example of basic ANT Broadcast RX.
 *
 * Before compiling this example for NRF52, complete the following steps:
 * - Download the S212 SoftDevice from <a href="https://www.thisisant.com/developer/components/nrf52832" target="_blank">thisisant.com</a>.
 * - Extract the downloaded zip file and copy the S212 SoftDevice headers to <tt>\<InstallFolder\>/components/softdevice/s212/headers</tt>.
 * If you are using Keil packs, copy the files into a @c headers folder in your example folder.
 * - Make sure that @ref ANT_LICENSE_KEY in @c nrf_sdm.h is uncommented.
 */
 //Header per ANT e ICM
#include "nrf_drv_mpu.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "bsp.h"
#include "hardfault.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"
#include "nrf_pwr_mgmt.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "ant_channel_config.h"
#include "app_ICM20948.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "quat.h"
#include "nrf_delay.h"
//Header per pulsanti e led
#include "nrf_gpio.h"

#include "nrf_drv_saadc.h"


#define APP_ANT_OBSERVER_PRIO   1    /**< Application's ANT observer priority. You shouldn't need to modify this value. */
#define LED 11
#define SAADC_CHANNEL 0     //Pin A0 (sarebbe il 2)


void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
}


/**
 * @brief Function for confguring SAADC channel 0 for sampling AIN0 (P0.02).
 */
void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(SAADC_CHANNEL, &channel_config);
    APP_ERROR_CHECK(err_code);
}

int count=0, stato=0, var=0, i=0;
/**@brief Function for handling a ANT stack event.
 *
 * @param[in] p_ant_evt  ANT stack event.
 * @param[in] p_context  Context.
 */
 
 
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


//IMU
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
		
		if(MAGNETOMETRO_ABILITATO){
		
		
		ICM20948_resetMag();
		
		app_icm_magn_config_t magnetometer_config;
		
		magnetometer_config.mode = MAG_CNTL2_MODE_100HZ; //continuous measurment mode 4 = 100 Hz
		
		
		ret_code = app_icm_magnetometer_init(&magnetometer_config);	
		APP_ERROR_CHECK(ret_code);
		}
}

/* ANT functions*/
 void ant_message_send(void)
{
    uint8_t         message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE];

    memset(message_payload, 0, ANT_STANDARD_DATA_PAYLOAD_SIZE);
    // Assign a new value to the broadcast data.
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 8] = 1;
		message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 7] = 0;
	
    // Broadcast the data.
    ret_code_t err_code = sd_ant_broadcast_message_tx(BROADCAST_CHANNEL_NUMBER,
                                                      ANT_STANDARD_DATA_PAYLOAD_SIZE,
                                                      message_payload);
    APP_ERROR_CHECK(err_code);

}
void ant_send(int sample, int quat1,int quat2, int quat3, int quat4, int counter)
{
    uint8_t         message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE];
    

    memset(message_payload, 0, ANT_STANDARD_DATA_PAYLOAD_SIZE);
    // Assign a new value to the broadcast data.
		message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 8] = 3;
		message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 7] = (int)sample; 
	  message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 5] = counter;
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 4] = quat1;
		message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 3] = quat2;
		message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 2] = quat3;
		message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 1] = quat4;

    // Broadcast the data.
    ret_code_t err_code = sd_ant_broadcast_message_tx(BROADCAST_CHANNEL_NUMBER,
                                                      ANT_STANDARD_DATA_PAYLOAD_SIZE,
                                                      message_payload);
    APP_ERROR_CHECK(err_code);
    
}


void ant_evt_handler(ant_evt_t * p_ant_evt, void * p_context)
{
    //ret_code_t err_code;

    if (p_ant_evt->channel == BROADCAST_CHANNEL_NUMBER)
    {
        switch (p_ant_evt->event)
        {
            case EVENT_RX:
                if (p_ant_evt->message.ANT_MESSAGE_ucMesgID == MESG_BROADCAST_DATA_ID)
                {
                  if (p_ant_evt->message.ANT_MESSAGE_aucPayload [0x00] == 0x00)   //se il primo byte del payload è zero
									{ 
										if (p_ant_evt->message.ANT_MESSAGE_aucPayload [0x07] == 0x80)   //e se l'ultimo byte del payload è 128
									  {
										  nrf_gpio_pin_set(LED);
										  if (stato==1)									//ferma l'acquisizione
										  {
										   stato=0;
											 count = 0;
										  }
								    }
									  else 		 											//altrimenti avvia l'acquisizione
										nrf_gpio_pin_clear(LED);
										count=0;
										i=0;
										stato = (stato==0) ? 1 : 0;
								    }
				
                }
                break;

            default:
                break;
        }
    }
}

NRF_SDH_ANT_OBSERVER(m_ant_observer, APP_ANT_OBSERVER_PRIO, ant_evt_handler, NULL);

/**@brief Function for the Timer and BSP initialization.
 */
static void utils_setup(void)
{
		ret_code_t err_code;
    
		//err_code = NRF_LOG_INIT(NULL);
    //APP_ERROR_CHECK(err_code);

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = bsp_init(BSP_INIT_LED,
                        NULL);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for ANT stack initialization.
 */
static void softdevice_setup(void)
{
    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    ASSERT(nrf_sdh_is_enabled());

    err_code = nrf_sdh_ant_enable();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for setting up the ANT module to be ready for RX broadcast.
 */
static void ant_channel_rx_broadcast_setup(void)
{
    ant_channel_config_t broadcast_channel_config =
    {
        .channel_number    = BROADCAST_CHANNEL_NUMBER,
        .channel_type      = CHANNEL_TYPE_SHARED_SLAVE,
        .ext_assign        = 0x00,
        .rf_freq           = RF_FREQ,
        .transmission_type = CHAN_ID_TRANS_TYPE,
        .device_type       = CHAN_ID_DEV_TYPE,
        .device_number     = CHAN_ID_DEV_NUM,
        .channel_period    = CHAN_PERIOD,
        .network_number    = ANT_NETWORK_NUM,
    };

    ret_code_t err_code = ant_channel_init(&broadcast_channel_config);
    APP_ERROR_CHECK(err_code);

    // Open channel.
    err_code = sd_ant_channel_open(BROADCAST_CHANNEL_NUMBER);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry. Does not return.
 */
int main(void)
{
	  nrf_gpio_cfg_output(LED);
		nrf_gpio_pin_set(LED);

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
	  nrf_saadc_value_t sample;
  
    // Initialize.
    log_init();
	//	NRF_LOG_INFO("\033[2J\033[;H"); // Clear screen
//    NRF_POWER->DCDCEN = 1;
    icm_init();
		
    saadc_init();

	
    // Start execution  
		NRF_LOG_INFO("MPU Free Fall Interrupt example.");
	
	  utils_setup();
    softdevice_setup();
    ant_channel_rx_broadcast_setup();
		
	  static uint8_t                  m_channel_number=0; 
	  uint8_t         message_addr[ANT_STANDARD_DATA_PAYLOAD_SIZE];
	  memset(message_addr, 3, ANT_STANDARD_DATA_PAYLOAD_SIZE);

	  err_code = sd_ant_broadcast_message_tx(BROADCAST_CHANNEL_NUMBER,
                                                      ANT_STANDARD_DATA_PAYLOAD_SIZE,
                                                      message_addr);	
		nrf_gpio_pin_clear(LED);
		
    // Main loop.
    for (;;)
    {
			if(NRF_LOG_PROCESS() == false)
        {
          if (stato==1)
					{
						for (i=0;i<4; i++)
						{
							
							//ICM-20948
							//Accelerometer data read
							err_code = app_icm_read_accel(&acc_values);
							APP_ERROR_CHECK(err_code);
						
							//Sensitivity 2G
//							acc.x=((acc_values.x)/16384.)-0.0384667963;
//							acc.y=((acc_values.y)/16384.)-0.0059826239;
//							acc.z=((acc_values.z)/16384.)-0.0222609;
							//Sensitivity 4G
							acc.x=((acc_values.x)/4096.)+0.00258519663;
							acc.y=((acc_values.y)/4096.)-0.00990874227;
							acc.z=((acc_values.z)/4096.)-0.01995349;
							if( acc_values.x == 0 && acc_values.y == 0 && acc_values.z == 0 ) nrf_gpio_pin_set(LED);
//							acc.x=((acc_values.x)/2048.);
//							acc.y=((acc_values.y)/2048.);
//							acc.z=((acc_values.z)/2048.);
							
							//Gyroscope data read
							err_code = app_icm_read_gyro(&gyro_values);
							APP_ERROR_CHECK(err_code);			
							//sensitivity 500 dps	
							gyro.x=(gyro_values.x/65.54)*PI/180.0f+0.0176442917;
							gyro.y=(gyro_values.y/65.54)*PI/180.0f-0.0170090254;
							gyro.z=(gyro_values.z/65.54)*PI/180.0f+0.00491240807;
							if( gyro_values.x == 0 && gyro_values.y == 0 && gyro_values.z == 0 ) nrf_gpio_pin_set(LED);
							//sensitivity 2000 dps	
//							gyro.x=(gyro_values.x/16.4)*PI/180.0f+0.13899231;
//							gyro.y=(gyro_values.y/16.4)*PI/180.0f-0.182615861;
//							gyro.z=(gyro_values.z/16.4)*PI/180.0f+0.0688171238;
//							gyro.x=(gyro_values.x/16.4)*PI/180.0f;
//							gyro.y=(gyro_values.y/16.4)*PI/180.0f;
//							gyro.z=(gyro_values.z/16.4)*PI/180.0f;
							
							//Magnetometer data read
							if(MAGNETOMETRO_ABILITATO){
							
							err_code = app_icm_read_magnetometer(&magn_values, NULL);
							APP_ERROR_CHECK(err_code);
							mx=(((magn_values.x)*0.15)+15.6000004)*1.02588999;
							my=(((magn_values.y)*0.15)-12.1499996)*0.990625024;
							mz=(((magn_values.z)*0.15)-11.6999998)*0.984472036;
//							mx=((magn_values.x)*0.15);
//							my=((magn_values.y)*0.15);
//							mz=((magn_values.z)*0.15);
							
							}
			
							err_code = nrf_drv_saadc_sample_convert(SAADC_CHANNEL, &sample);   //lettura ADC
              APP_ERROR_CHECK(err_code);
							
							
							MadgwickQuaternionUpdate(acc.x, acc.y, acc.z,gyro.x, gyro.y, gyro.z, mx, my, mz,deltat);
//							MadgwickQuaternionUpdate(0.002,0.002, 0.95,0.01, 0.01, 0.01,50.1, 100.2, 75.3,deltat);
							quat[0]=q[0]*127;
							quat[1]=q[1]*127;
							quat[2]=q[2]*127;
							quat[3]=q[3]*127;
		
							nrf_delay_ms(25);
						}

						count++;
						
						
						  int tensione = (int)sample/517 * 18;   //18 sono 1,8V
							ant_send( tensione, quat[0],quat[1],quat[2],quat[3], count);
						  nrf_gpio_pin_clear(LED);
					

					}

			NRF_LOG_FLUSH();
			nrf_pwr_mgmt_run();
			
    }
}
}

/**
 *@}
 **/
