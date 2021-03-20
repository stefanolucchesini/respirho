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
#include "app_simple_timer.h"

#include "ant_boot_settings_api.h"

// Version string
#define VERSION_STRING              "BFM1.00B01"
static const uint8_t m_version_string[] = VERSION_STRING; // Version string

#define APP_ANT_OBSERVER_PRIO   1    /**< Application's ANT observer priority. You shouldn't need to modify this value. */
#define LED 11
#define SAADC_CHANNEL 0     //Pin A0 (sarebbe il 2)
#define TIMEOUT_VALUE 25000                          /**< 25 mseconds timer time-out value. */

#define DEVICENUMBER 2     //1, 2 o 3

int count=0, stato=0, i=0;
float deltat=0.025;
float mx;
float my;
float mz;
float quat[4];

int notshown = 1; // per il log
volatile int bootloadenable = 0;

accel_values_t acc_values;
magn_values_t magn_values;
gyro_values_t gyro_values;
accel_values_float_t acc;
gyro_values_float_t gyro;
nrf_saadc_value_t sample;
ret_code_t err_code;		

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
	
}

static void restart_in_bootloader(void)
{
    uint32_t err_code;
    static ant_boot_settings_t ant_boot_settings;

    err_code = ant_boot_settings_clear(&ant_boot_settings); // Clears and set FFs to the memory block
    APP_ERROR_CHECK(err_code);
    memcpy((void*) ant_boot_settings.app_version, m_version_string, sizeof(m_version_string));
    ant_boot_settings.app_size = 0xFFFF;                      // Estimated current application size used to try to preserve itself
    err_code = ant_boot_settings_save(&ant_boot_settings);
    APP_ERROR_CHECK(err_code);
    ant_boot_settings_validate(1);                          // Sets flag to indicate restarting in bootloader mode. Must be done last before the reset!!!
    NVIC_SystemReset();
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
		
		
		ret_code = ICM20948_resetMag();
			
		app_icm_magn_config_t magnetometer_config;
		
		magnetometer_config.mode = MAG_CNTL2_MODE_100HZ; //continuous measurment mode 4 = 100 Hz
		

		ret_code = app_icm_magnetometer_init(&magnetometer_config);
		if (ret_code) NRF_LOG_INFO("Ret code magnetometro: %d", ret_code);	
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
void ant_send(int campione, int quat1,int quat2, int quat3, int quat4, int counter)
{
    uint8_t         message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE];
    

    memset(message_payload, 0, ANT_STANDARD_DATA_PAYLOAD_SIZE);
    // Assign a new value to the broadcast data.
		message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 8] = DEVICENUMBER;
		message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 7] = campione; 
	  message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 6] = 0;
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
                  if (p_ant_evt->message.ANT_MESSAGE_aucPayload [0x00] == 0x00 && p_ant_evt->message.ANT_MESSAGE_aucPayload [0x07] == 0x80 )   //se il primo byte del payload è zero e l'ultimo è 128
									{ 
									
											stato=0;									  //ferma l'acquisizione												
											nrf_gpio_pin_set(LED);
										  bootloadenable = 1;
									
								   }
									 if (p_ant_evt->message.ANT_MESSAGE_aucPayload [0x00] == 0x00 && p_ant_evt->message.ANT_MESSAGE_aucPayload [0x07] != 0x80)   //se il primo byte del payload è zero avvia l'acquisizione
									  {  		 										
										 nrf_gpio_pin_clear(LED);
										 count=0;
										 i=0;
										 stato = 1;
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

void timeout_handler(void * p_context)
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
							if( magn_values.x == 0 && magn_values.y == 0 && magn_values.z == 0 ) nrf_gpio_pin_set(LED);
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
		
						if(stato == 0 && notshown ) { NRF_LOG_INFO("Attesa messaggio di inizio aquisizione"); notshown = 0;}
						i++;
						i = (i > 3) ? 0 : i; 
					
if(stato == 1 && i == 0) {
	ant_send( sample, quat[0],quat[1],quat[2],quat[3], count);
	NRF_LOG_INFO("Count: %d", count);
	count++;
	nrf_gpio_pin_clear(LED);
}

if(bootloadenable)
restart_in_bootloader();   //da spostare in comando dedicato

										                  
}

void SD_EVT_IRQHandler(void)   //handler eventi softdevice, serve per fare il riavvio nel bootloader
{
   uint32_t ulEvent;
   while (sd_evt_get(&ulEvent) != NRF_ERROR_NOT_FOUND) // read out SOC events
   { 	
       ant_boot_settings_event(ulEvent);
		   
   }
  
	 nrf_sdh_evts_poll();
}

int main(void)
{
	  nrf_gpio_cfg_output(LED);
		nrf_gpio_pin_set(LED);
	  log_init();
	  utils_setup();
    softdevice_setup();
    ant_channel_rx_broadcast_setup();
	
	for(int u=0; u<5;u++){
  nrf_gpio_pin_toggle(LED);
	nrf_delay_ms(300);
	}
	
	  uint32_t err_code;
  
    // Initialize.
   
	//	NRF_LOG_INFO("\033[2J\033[;H"); // Clear screen
//    NRF_POWER->DCDCEN = 1;
    icm_init();
		
    saadc_init();
	
    // Start execution  
		NRF_LOG_INFO("Dispositivo RESPIRHO' numero %d", DEVICENUMBER);
		
//	  static uint8_t                  m_channel_number=0; 
	  uint8_t         message_addr[ANT_STANDARD_DATA_PAYLOAD_SIZE];
	  memset(message_addr, DEVICENUMBER, ANT_STANDARD_DATA_PAYLOAD_SIZE);

	  err_code = sd_ant_broadcast_message_tx(BROADCAST_CHANNEL_NUMBER,
                                                      ANT_STANDARD_DATA_PAYLOAD_SIZE,
                                                      message_addr);	
																											
		err_code = app_simple_timer_init();  //Timer1, 1MHz, 16 bit
    APP_ERROR_CHECK(err_code);

    err_code = app_simple_timer_start(APP_SIMPLE_TIMER_MODE_REPEATED, timeout_handler, TIMEOUT_VALUE, NULL);
    APP_ERROR_CHECK(err_code);				
		
		nrf_gpio_pin_clear(LED);
		
    // Main loop.
    while (1)
    {
			if(NRF_LOG_PROCESS() == false)
        {

			NRF_LOG_FLUSH();
			nrf_pwr_mgmt_run();
			
    }
}
}
