#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "bsp.h"
#include "hardfault.h"
#include "app_error.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"
#include "nrf_pwr_mgmt.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "ant_channel_config.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"
//Generazione quaternioni
#include "quat.h"
//Header per pulsanti e led
#include "nrf_gpio.h"
//ADC e timer
#include "nrf_drv_saadc.h"
#include "app_simple_timer.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
//flash memory management
#include "nrf_nvmc.h"
//IMU
#include "nrf_drv_mpu.h"
#include "app_ICM20948.h"


#define APP_ANT_OBSERVER_PRIO   1    /**< Application's ANT observer priority. You shouldn't need to modify this value. */
#define LED 11							//Pin del LED rosso
#define SAADC_BATTERY 0     //Canale tensione della batteria
#define SAADC_REFERENCE 1   //Canale tensione di alimentazione, per vedere se è giusta
//NRF_SAADC_INPUT_AIN0 <----PIN A0 (è il 2), per modificarlo andare in saadc_init()
//ADC = Vsample * (1/6) * (1/Vref=0.6V) * 2^8 (lettura a 8 bit)
//Quindi Vsample = (ADC in decimale) * 9/640

#define TIMEOUT_VALUE 25      /**< 25 mseconds timer time-out value. Interrupt a 40Hz*/
APP_TIMER_DEF(m_repeated_timer_id);     /*Handler for repeated timer */

#define DEVICENUMBER 3     //1 = TORACE, 2 = ADDOME o 3 = REFERENCE
//Il #define MAGNETOMETRO_ABILITATO si trova in quat.h
//I pin che definiscono SCL e SDA sono in nrf_drv_mpu_twi.c, CONTROLLARE CHE SIANO GIUSTI PER PRIMA COSA!!
//!!ATTENZIONE!! L'utilizzo dei log con UART aumenta il consumo di corrente, se non si deve fare debug vanno disabilitati 
//nel file sdk_config.h
//per debug mettere ottimizzazione al livello 0 e direttiva DEBUG nel compilatore
volatile int count=0, stato=0, i=0;
const float deltat = 0.025;
volatile float mx, my, mz;
volatile float quat[4];

//Valori default di calibrazione
volatile float magnetometer_bias[] = {+15.6000004, -12.1499996, -11.6999998};  // X, Y, Z
volatile float magnetometer_scale[] = {1.02588999, 0.990625024, 0.984472036};   //X, Y, Z
volatile float acc_bias[] = {0.00258519663, -0.00990874227, -0.01995349}; //X, Y, Z
volatile float gyro_bias[] = {0.0176442917, -0.0170090254, +0.00491240807}; //X, Y, Z
volatile int cal_rec = 0;  //impedisce che si faccia più di una calibrazione, se se ne vuole fare un'altra bisogna spegnere e riaccendere
volatile int flag_cal = 0;  //flag che definisce calibrazione in corso
#define START_ADDR 0x00011200  //indirizzo di partenza per salvataggio dati in memoria non volatile

int notshown = 1; // per il log

//Definizione variabili che conterranno i valori letti dall'IMU
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

    err_code = nrf_drv_saadc_channel_init(SAADC_BATTERY, &channel_config);
    APP_ERROR_CHECK(err_code);
	
	  nrf_saadc_channel_config_t channel2_config =
    NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
	
	  err_code = nrf_drv_saadc_channel_init(SAADC_REFERENCE, &channel2_config);
    APP_ERROR_CHECK(err_code);
	 
}
  
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


//Inizializzazione IMU
void icm_init(void)
{
	
		/*ICM-20948 initialization and configuration */
		
		ret_code_t ret_code;
	
		// Initiate ICM driver
		ret_code = app_icm_init(); //inizializza i2c
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
		if (ret_code) NRF_LOG_INFO("Ret code magnetometro: %d", ret_code);	
 		APP_ERROR_CHECK(ret_code);
		
    }
}

void calibrazione(){  //funzione di calibrazione dell'IMU (giro, acc e magne)
	flag_cal = 1;
	int16_t max_x;
	int16_t min_x;
	int16_t max_y;
	int16_t min_y;
	int16_t max_z;
	int16_t min_z; 
	int step = 0;
	for(int c = 0; c<3; c++){  //azzera valori di default
	acc_bias[c] = 0; 
  gyro_bias[c] = 0;
	}
						while (step<1450)  //36 secondi di calibrazione, tenere tutto fermo
						{
							//accelerometro
							err_code = app_icm_read_accel(&acc_values);
							APP_ERROR_CHECK(err_code);
							acc_bias[0]=(acc_values.x/16384.)+acc_bias[0];
							acc_bias[1]=(acc_values.y/16384.)+acc_bias[1];
							acc_bias[2]=(acc_values.z/16384.)+acc_bias[2];

							//giroscopio
							err_code = app_icm_read_gyro(&gyro_values);
							APP_ERROR_CHECK(err_code);
							gyro_bias[0]=((gyro_values.x/65.54)*PI/180.0f)+gyro_bias[0];
							gyro_bias[1]=((gyro_values.y/65.54)*PI/180.0f)+gyro_bias[1];
							gyro_bias[2]=((gyro_values.z/65.54)*PI/180.0f)+gyro_bias[2];

							step++;
							nrf_gpio_pin_toggle(LED);
							nrf_delay_ms(25);
					}
							nrf_gpio_pin_clear(LED);
						  acc_bias[0]=acc_bias[0]/1450;
							acc_bias[1]=acc_bias[1]/1450;
							acc_bias[2]=acc_bias[2]/1450;

							gyro_bias[0]=gyro_bias[0]/1450;
							gyro_bias[1]=gyro_bias[1]/1450;
							gyro_bias[2]=gyro_bias[2]/1450;  

step = 0;			        
nrf_gpio_pin_set(LED);							
							while(step<1450){ //36 secondi di calibrazione, muovere nello spazio
							//calib magnetometro 
							
							err_code = app_icm_read_magnetometer(&magn_values, NULL);
							APP_ERROR_CHECK(err_code);
							if (step == 0)
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
						step++;
						nrf_delay_ms(25);	
						}
							
							magnetometer_bias[0] = (max_x + min_x)/2;
							magnetometer_bias[0] = (double)magnetometer_bias[0]*0.15;
							magnetometer_bias[1] = (max_y + min_y)/2;
							magnetometer_bias[1] = (double)magnetometer_bias[1]*0.15;
							magnetometer_bias[2] = (max_z + min_z)/2;
							magnetometer_bias[2] = (double)magnetometer_bias[2]*0.15;

							float mx_scale = (max_x - min_x)/2;
							float my_scale = (max_y - min_y)/2;
							float mz_scale = (max_z - min_z)/2;

							float avg_rad = mx_scale + my_scale + mz_scale;
							avg_rad = (double)avg_rad/ 3.0;

							magnetometer_scale[0] = avg_rad/mx_scale;
							magnetometer_scale[1] = avg_rad/my_scale;
							magnetometer_scale[2] = avg_rad/mz_scale;	

nrf_gpio_pin_clear(LED);						
flag_cal = 0;				
}	

void salva_calib_flash(){   //salva i dati di calibrazione nella flash

		uint32_t f_addr = START_ADDR;   //indirizzo ultima pagina di memoria
//    uint32_t* p_addr = (uint32_t*) f_addr;     //puntatore alla prima cella dell'ultima pagina
		//float val = -0.45924;
		nrf_nvmc_page_erase(f_addr);

	//nrf_nvmc_write_word(f_addr,*(uint32_t*)&val);  
    //uint32_t u= *p_addr;
		//magnetometer_bias[1] = *(float *)&u;
	float vettorone[12];
	for(int u = 0; u<=11; u++){
		if(u<=2) vettorone[u] = magnetometer_bias[u];
		if(u>2 && u<=5) vettorone[u] = magnetometer_scale[u-3];
		if(u>5 && u<=8) vettorone[u] = acc_bias[u-6];
		if(u>8 && u<=11) vettorone[u] = gyro_bias[u-9];
	}	
	nrf_nvmc_write_words(f_addr, (uint32_t*)&vettorone, 12);

}

void leggi_calib_flash(){
 
	uint32_t f_addr = START_ADDR;   
  uint32_t* p_addr = (uint32_t*) f_addr;     //puntatore alla prima cella dell'indirizzo specificato
	uint8_t alreadywrittentoflash = 1;
	float vettorone[12];
	for(int u = 0; u<=11; u++){
	uint32_t c= *(p_addr+u);
	vettorone[u] = *(float *)&c;
	if(c == 0xFFFFFFFF) alreadywrittentoflash = 0;
	}
 if(alreadywrittentoflash){
	for(int u = 0; u<=11; u++){
		if(u<=2)  magnetometer_bias[u] = vettorone[u];
		if(u>2 && u<=5) magnetometer_scale[u-3] = vettorone[u];
		if(u>5 && u<=8) acc_bias[u-6] = vettorone[u];
		if(u>8 && u<=11) gyro_bias[u-9] = vettorone[u];
	}	
 }
 else salva_calib_flash(); //se la memoria non volatile è vuota, ci salva i valori di default
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
void ant_send(int campione, int counter, int quat1, int quat2, int quat3, int quat4)
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
	NRF_LOG_INFO("Messaggio numero %d, ret code ant broadcast: %d", counter, err_code);
}


void ant_evt_handler(ant_evt_t * p_ant_evt, void * p_context)
{

    if (p_ant_evt->channel == BROADCAST_CHANNEL_NUMBER && flag_cal == 0)  //durante la calibrazione ignora tutti i messaggi che arrivano dal master
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
										  sd_ant_pending_transmit_clear (BROADCAST_CHANNEL_NUMBER, NULL); //svuota il buffer, utile per una seconda acquisizione
										  NRF_LOG_INFO("Ricevuto messaggio di stop acquisizione");
										  
									}
									 if (p_ant_evt->message.ANT_MESSAGE_aucPayload [0x00] == 0x00 && p_ant_evt->message.ANT_MESSAGE_aucPayload [0x07] == 0x00)   //se il primo byte del payload è zero avvia l'acquisizione
									  {
										 NRF_LOG_INFO("Inizio acquisizione");	
										 sd_ant_pending_transmit_clear (BROADCAST_CHANNEL_NUMBER, NULL); //svuota il buffer, utile per una seconda acquisizione
										 nrf_gpio_pin_clear(LED);
										 count=0;
										 i=0;
										 stato = 1;
								    }
									if (p_ant_evt->message.ANT_MESSAGE_aucPayload [0x00] == 0x00 && p_ant_evt->message.ANT_MESSAGE_aucPayload [0x07] == 0xFF && cal_rec == 0)   //messaggio di inizio calibrazione
									  {
										 cal_rec = 1;
										 stato = 0;
										 calibrazione();
										 salva_calib_flash();
								    }		
                }
                break;

            default:
                break;
        }
    }
}
NRF_SDH_ANT_OBSERVER(m_ant_observer, APP_ANT_OBSERVER_PRIO, ant_evt_handler, NULL);

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


//void timeout_handler(void * p_context)  //simple timer
static void repeated_timer_handler(void * p_context)  //app timer
{ 
	 if(flag_cal) return; //se la calibrazione è in corso non fa niente
			        //ICM-20948
							//Accelerometer data read
							err_code = app_icm_read_accel(&acc_values);
							APP_ERROR_CHECK(err_code);
						
							//Sensitivity 4G
							acc.x=((acc_values.x)/4096.)+acc_bias[0];
							acc.y=((acc_values.y)/4096.)+acc_bias[1];
							acc.z=((acc_values.z)/4096.)+acc_bias[2];
							if( acc_values.x == 0 && acc_values.y == 0 && acc_values.z == 0 ) nrf_gpio_pin_set(LED);
							
							//Gyroscope data read
							err_code = app_icm_read_gyro(&gyro_values);
							APP_ERROR_CHECK(err_code);			
							//sensitivity 500 dps	
						  gyro.x=(gyro_values.x/65.54)*PI/180.0f+gyro_bias[0];
							gyro.y=(gyro_values.y/65.54)*PI/180.0f+gyro_bias[1];
							gyro.z=(gyro_values.z/65.54)*PI/180.0f+gyro_bias[2];
							if( gyro_values.x == 0 && gyro_values.y == 0 && gyro_values.z == 0 ) nrf_gpio_pin_set(LED);
							
							//Magnetometer data read
							if(MAGNETOMETRO_ABILITATO){
							
							err_code = app_icm_read_magnetometer(&magn_values, NULL);
							APP_ERROR_CHECK(err_code);
							mx=(((magn_values.x)*0.15)+magnetometer_bias[0])*magnetometer_scale[0];
							my=(((magn_values.y)*0.15)+magnetometer_bias[1])*magnetometer_scale[1];
							mz=(((magn_values.z)*0.15)+magnetometer_bias[2])*magnetometer_scale[2];
							if( magn_values.x == 0 && magn_values.y == 0 && magn_values.z == 0 ) nrf_gpio_pin_set(LED);							
							}
			
							err_code = nrf_drv_saadc_sample_convert(SAADC_BATTERY, &sample);   //lettura ADC
              APP_ERROR_CHECK(err_code);
							
							
							MadgwickQuaternionUpdate(acc.x, acc.y, acc.z,gyro.x, gyro.y, gyro.z, mx, my, mz,deltat);
							quat[0]=q[0]*127;
							quat[1]=q[1]*127;
							quat[2]=q[2]*127;
							quat[3]=q[3]*127;
		
						if(stato == 0 && notshown ) { NRF_LOG_INFO("Attesa messaggio di inizio aquisizione"); notshown = 0;}
						i++;
						i = (i > 3) ? 0 : i; 
					
  if(stato == 1 && i == 0) {
	ant_send( sample, count,quat[0] ,quat[1] ,quat[2] ,quat[3] );
	count++;
	nrf_gpio_pin_clear(LED);
  }									                  
}

int main(void)
{
	  //NRF_POWER->DCDCEN = 1;   //Abilita alimentatore DCDC. Attenzione! Devono esserci collegati gli induttori se no non va niente!
	  nrf_gpio_cfg_output(LED);
		nrf_gpio_pin_set(LED);

	  uint32_t err_code;
	  log_init();     //inizializza log
    softdevice_setup();  //abilita softdevice
    ant_channel_rx_broadcast_setup();   //abilita canale ANT
    icm_init();      //inizializza unità IMU
    saadc_init();   //inizializza convertitore analogico digitale

	
    // Start execution  
	  NRF_LOG_INFO("\033[2J\033[;H"); // Clear screen
		NRF_LOG_INFO("Dispositivo RESPIRHO' numero %d", DEVICENUMBER);
	
	
		sd_ant_channel_radio_tx_power_set(BROADCAST_CHANNEL_NUMBER, RADIO_TX_POWER_LVL_4, NULL); 	//potenza trasmissione
	
	  uint8_t  message_addr[ANT_STANDARD_DATA_PAYLOAD_SIZE];
	  memset(message_addr, DEVICENUMBER, ANT_STANDARD_DATA_PAYLOAD_SIZE);
		
		err_code = nrf_drv_saadc_sample_convert(SAADC_REFERENCE, &sample);  //campiona tensione alimentazione (1.8V)
		message_addr[ANT_STANDARD_DATA_PAYLOAD_SIZE - 1] = sample;  //invia campione nell'ultimo byte del payload
		
	  err_code = sd_ant_broadcast_message_tx(BROADCAST_CHANNEL_NUMBER,         //invia messaggio di accensione
                                           ANT_STANDARD_DATA_PAYLOAD_SIZE,
                                           message_addr);	
																											
		//err_code = app_simple_timer_init();  //Inizializza Timer1, 1MHz, 16 bit
    //APP_ERROR_CHECK(err_code);
		app_timer_init();
		err_code = app_timer_create(&m_repeated_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                repeated_timer_handler);
    APP_ERROR_CHECK(err_code);
		
		nrf_gpio_pin_clear(LED);
		leggi_calib_flash();  //recupera valori di calibrazione salvati nella flash 
		err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
		//err_code = app_simple_timer_start(APP_SIMPLE_TIMER_MODE_REPEATED, timeout_handler, TIMEOUT_VALUE, NULL);  //Interrupt timer partito
		err_code = app_timer_start(m_repeated_timer_id, APP_TIMER_TICKS(TIMEOUT_VALUE), NULL);
    //APP_ERROR_CHECK(err_code);	
		
    while (1)
    {
			if(NRF_LOG_PROCESS() == false)
       {

	//		NRF_LOG_FLUSH();
			nrf_pwr_mgmt_run();
				
    }

}
}
