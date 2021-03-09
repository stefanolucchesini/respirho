/*
  * File app_ICM20948.c
  * The file contains the functions to interface with the ICM-20948 sensor, in terms of initialization, setting
  * and reading.
	*
	*
  */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "app_ICM20948.h"
#include "nrf_gpio.h"
#include "nrf_drv_mpu.h"
#include "nrf_error.h"
#include "nrf_peripherals.h"

/* Function to reset the ICM20948 */
uint32_t ICM20948_reset()
{
  ret_code_t err_code;
  app_icm_reg_bank_select(USER_BANK_0);
  err_code = nrf_drv_mpu_write_single_register(UB0_PWR_MGMNT_1, UB0_PWR_MGMNT_1_DEV_RESET); //Reset the internal registers and restores the default settings. 
	//Write a 1 to set the reset, the bit will auto clear (in MPU: SIGNAL_PATH_RESET)
  //APP_ERROR_CHECK(err_code);
	//return 1;
	if(err_code != NRF_SUCCESS) return err_code;
  return NRF_SUCCESS;
}

/* Function to select a USER BANK */
uint32_t app_icm_reg_bank_select(uint8_t userBank) 
{
  ret_code_t err_code;

	uint8_t userBankRegValue = 0x00;

	switch(userBank) {
    case USER_BANK_0: {
    	userBankRegValue = REG_BANK_SEL_USER_BANK_0;
  		break;
    }
    case USER_BANK_1: {
    	userBankRegValue = REG_BANK_SEL_USER_BANK_1;
  		break;
    }
    case USER_BANK_2: {
    	userBankRegValue = REG_BANK_SEL_USER_BANK_2;
  		break;
    }
    case USER_BANK_3: {
    	userBankRegValue = REG_BANK_SEL_USER_BANK_3;
  		break;
    }
  }
  err_code = nrf_drv_mpu_write_single_register(REG_BANK_SEL, userBankRegValue); //write to the register REG_SEL_BANK to srlrct the USER BANK
  //APP_ERROR_CHECK(err_code);
 
  return err_code;
}

/* Function to initialize TWI */
uint32_t app_icm_init(void)
{
    uint32_t err_code;
	
		// Initate TWI or SPI driver dependent on what is defined from the project
		err_code = nrf_drv_mpu_init();
    if(err_code != NRF_SUCCESS) return err_code;

    return NRF_SUCCESS;
}

/* Function to select the clock source */
uint32_t ICM20948_selectAutoClockSource()
{
  ret_code_t err_code;
  app_icm_reg_bank_select(USER_BANK_0);
  err_code = nrf_drv_mpu_write_single_register(UB0_PWR_MGMNT_1, UB0_PWR_MGMNT_1_CLOCK_SEL_AUTO); //write 1 to the PWR_MGMNT_1  auto selects the best available clock source
//  APP_ERROR_CHECK(err_code);
//  return 1;
	if(err_code != NRF_SUCCESS) return err_code;
  return NRF_SUCCESS;
}

/* Function to enable accelerometer and gyroscope */
uint32_t ICM20948_enableAccelGyro()
{
  ret_code_t err_code;
  app_icm_reg_bank_select(USER_BANK_0);
  err_code = nrf_drv_mpu_write_single_register(UB0_PWR_MGMNT_2, UB0_PWR_MGMNT_2_SEN_ENABLE); //write 0x00 to PWR_MGMNT_2 enables acc and gyro (all axes) 
  
//  APP_ERROR_CHECK(err_code);
//  return 1;
	if(err_code != NRF_SUCCESS) return err_code;
  return NRF_SUCCESS;
}

/* Function to disable accelerometer and gyroscope (not used)*/
uint32_t ICM20948_disableAccelGyro()
{
  ret_code_t err_code;
  app_icm_reg_bank_select(USER_BANK_0);
  err_code = nrf_drv_mpu_write_single_register(UB0_PWR_MGMNT_2, UB0_PWR_MGMNT_2_DISABLE); //write 0x3F to PWR_MGMNT_2 diables acc and gyro (all axes)
  
//  APP_ERROR_CHECK(err_code);
//  return 1;
	if(err_code != NRF_SUCCESS) return err_code;
  return NRF_SUCCESS;
}

/* Function to configure accelerometer */
uint32_t ICM20948_configAccel() 
{
  ret_code_t err_code;

  app_icm_reg_bank_select(USER_BANK_2);

  err_code = nrf_drv_mpu_write_single_register(UB2_ACCEL_CONFIG, UB2_ACCEL_CONFIG_FS_SEL_2G);  //[2:1]: Full Scale +-2g. [0]: Bypass accel DLPF
  
//  APP_ERROR_CHECK(err_code);
//  return 1;
	if(err_code != NRF_SUCCESS) return err_code;
  return NRF_SUCCESS;
}

/* Function to configure gyroscope */
uint32_t ICM20948_configGyro()
{
  ret_code_t err_code;
  app_icm_reg_bank_select(USER_BANK_2);

  err_code = nrf_drv_mpu_write_single_register(UB2_GYRO_CONFIG_1, UB2_GYRO_CONFIG_1_FS_2000DPS_DLPFCFG_152HZ); //00001111 [5:3]: BW 151.8 hz, [2:1]:FS +-2000 dps, [0]:DLPF enabled
  
//  APP_ERROR_CHECK(err_code);
//  return 1;
	if(err_code != NRF_SUCCESS) return err_code;
  return NRF_SUCCESS;
}

/* Function to set gyroscope sample rate divider */
uint32_t ICM20948_setGyroSrd(uint8_t srd)
{
  ret_code_t err_code;

  err_code = app_icm_reg_bank_select(USER_BANK_2);
  nrf_drv_mpu_write_single_register(UB2_GYRO_SMPLRT_DIV, srd); //ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0])

//  APP_ERROR_CHECK(err_code);
//  return 1;
	if(err_code != NRF_SUCCESS) return err_code;
  return NRF_SUCCESS;
}

/* Function to set accelerometer sample rate divider */
uint32_t ICM20948_setAccelSrd(uint16_t srd)
{
  ret_code_t err_code;

  app_icm_reg_bank_select(USER_BANK_2);
	
	//ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0])
	
  uint8_t srdHigh = srd >> 8 & 0x0F; // Only last 4 bits can be set 
  
  err_code = nrf_drv_mpu_write_single_register(UB2_ACCEL_SMPLRT_DIV_1, srdHigh); //ACCEL_SMPLRT_DIV[11:8]

  uint8_t srdLow = srd & 0x0F; 
  err_code = nrf_drv_mpu_write_single_register(UB2_ACCEL_SMPLRT_DIV_2, srdLow); //ACCEL_SMPLRT_DIV[7:0]
//  APP_ERROR_CHECK(err_code);
//  return 1;
	if(err_code != NRF_SUCCESS) return err_code;
  return NRF_SUCCESS;
}


/* Function to read accelerometer values */
uint32_t app_icm_read_accel(accel_values_t * accel_values)
{
    uint32_t err_code;
    uint8_t raw_values[6];
		app_icm_reg_bank_select(USER_BANK_0);
	
    err_code = nrf_drv_mpu_read_registers(UB0_ACCEL_XOUT_H, raw_values, 6); //multiple read of acceletometer values from ACCEL_XOUT_H to ACCEL_ZOUT_L
		
    if(err_code != NRF_SUCCESS) return err_code;

    // Reorganize read sensor values and put them into value struct
    uint8_t *data;
    data = (uint8_t*)accel_values;
    for(uint8_t i = 0; i<6; i++)
    {
        *data = raw_values[5-i];
        data++;
    }
    return NRF_SUCCESS;
}

/* Function to read gyroscope values */
uint32_t app_icm_read_gyro(gyro_values_t * gyro_values)
{
    uint32_t err_code;
    uint8_t raw_values[6];
		app_icm_reg_bank_select(USER_BANK_0);
    err_code = nrf_drv_mpu_read_registers(UB0_GYRO_XOUT_H, raw_values, 6); //multiple read of gyroscope values from GYRO_XOUT_H to GYRO_ZOUT_L
		
    if(err_code != NRF_SUCCESS) return err_code;

    // Reorganize read sensor values and put them into value struct
    uint8_t *data;
    data = (uint8_t*)gyro_values;
    for(uint8_t i = 0; i<6; i++)
    {
        *data = raw_values[5-i];
        data++;
    }
    return NRF_SUCCESS;
}

/* Function to read temperature values (not used)*/
uint32_t app_icm_read_temp(temp_value_t * temperature)
{
    uint32_t err_code;
    uint8_t raw_values[2];
		app_icm_reg_bank_select(USER_BANK_0);
    err_code = nrf_drv_mpu_read_registers(UB0_TEMP_OUT_H, raw_values, 2); //multiple read of temperature values from TEMP_OUT_H to TEMP_OUT_L
    if(err_code != NRF_SUCCESS) return err_code;

    *temperature = (temp_value_t)(raw_values[0] << 8) + raw_values[1];

    return NRF_SUCCESS;
}

/* FUNCTIONS FOR MAGNETOMETER */

#if (defined(ICM20948) || defined(MPU9255)) && (MPU_USES_TWI) // Magnetometer only works with TWI so check if TWI is enabled

/* Function to initialize magnetometer */
uint32_t app_icm_magnetometer_init(app_icm_magn_config_t * p_magnetometer_conf)
{	
	uint32_t err_code;
	/*
	// Read out MPU configuration register
	app_icm_int_pin_cfg_t bypass_config;
	app_icm_reg_bank_select(USER_BANK_0);
	err_code = nrf_drv_mpu_read_registers(UB0_INT_PIN_CFG, (uint8_t *)&bypass_config, 1); //Read INT_PIN_CFG register
	
	// Set I2C bypass enable bit to be able to communicate with magnetometer via I2C
	bypass_config.i2c_bypass_en = 1;
	// Write config value back to ICM config register
	err_code = app_icm_int_cfg_pin(&bypass_config);
	if (err_code != NRF_SUCCESS) return err_code;
	//aggiungere funzione di read per vedere se funziona
	app_icm_int_pin_cfg_t prova;
	app_icm_reg_bank_select(USER_BANK_0);
	err_code = nrf_drv_mpu_read_registers(UB0_INT_PIN_CFG, (uint8_t *)&prova, 1);
	*/
	// Write magnetometer config data	
	uint8_t *data;
  data = (uint8_t*)p_magnetometer_conf;	
  return nrf_drv_mpu_write_magnetometer_register(MAG_CNTL2, *data); //write to MAG_CNTL2 register to set the operation mode (bit [4:0])
}

/* Function to write at INT_PIN_CFG register*/
uint32_t app_icm_int_cfg_pin(app_icm_int_pin_cfg_t *cfg)
{
    uint8_t *data;
    data = (uint8_t*)cfg;
    app_icm_reg_bank_select(USER_BANK_0);
    return nrf_drv_mpu_write_single_register(UB0_INT_PIN_CFG, *data);
}

/* Function to read magnetometer values */
uint32_t app_icm_read_magnetometer(magn_values_t * p_magnetometer_values, app_mpu_magn_read_status_t * p_read_status)
{
	uint32_t err_code;
	err_code = nrf_drv_mpu_read_magnetometer_registers(MAG_HXL, (uint8_t *)p_magnetometer_values, 6); //multiple read of magnetometer values from MAG_HXL to MAG_HZH
	if(err_code != NRF_SUCCESS) return err_code;
        
	/* Quote from datasheet: 
	ST2 register has a role as data reading end register, also. When any of measurement data register (HXL to TMPS) is
	read in Continuous measurement mode 1, 2, 3, 4, it means data reading start and taken as data reading until ST2
	register is read. Therefore, when any of measurement data is read, be sure to read ST2 register at the end.*/
        
	if(p_read_status == NULL)
	{
		// If p_read_status equals NULL perform dummy read
		uint8_t status_2_reg;
		err_code = nrf_drv_mpu_read_magnetometer_registers(MAG_ST2, &status_2_reg, 1);
	}
	else
	{
		// If p_read_status NOT equals NULL read and return value of MAG_ST2
		err_code = nrf_drv_mpu_read_magnetometer_registers(MAG_ST2, (uint8_t *)p_read_status, 1);
	}
	return err_code;
}


#endif // (defined(ICM20948) || defined(MPU9255)) && (MPU_USES_TWI) 

/**
  @}
*/

 
uint32_t ICM20948_enableI2cMaster() //serve??
{
  ret_code_t err_code;
  app_icm_reg_bank_select(USER_BANK_0);
  nrf_delay_ms(10);
	err_code = nrf_drv_mpu_write_single_register(UB0_USER_CTRL, 0x2); // reset i2c master [1]: Reset I2C Master module.
  //nrf_delay_ms(2);
	err_code = nrf_drv_mpu_write_single_register(UB0_USER_CTRL, UB0_USER_CTRL_I2C_MST_EN ); /*| 0x2*/// Il creatore della funzione ha scritto: qui ho spostato il ripristino, deve essere fatto insieme a enable master
  // [5]: (0x20) Enable the I2C Master I/F module;
	nrf_delay_ms(10);
  
  app_icm_reg_bank_select(USER_BANK_3);
  err_code = nrf_drv_mpu_write_single_register(UB3_I2C_MST_CTRL, UB3_I2C_MST_CTRL_CLK_400KHZ); //[3:0]: Sets I2C master clock frequency. 0x07 Gives 345.6kHz and is recommended to achieve max 400kHz

//  APP_ERROR_CHECK(err_code);
//  return 1;
	if(err_code != NRF_SUCCESS) return err_code;
  return NRF_SUCCESS;
}

uint8_t ICM20948_whoAmI()
{
  ret_code_t err_code;
  uint8_t data;
  err_code = app_icm_reg_bank_select(USER_BANK_0);
	APP_ERROR_CHECK(err_code);
  // read the WHO AM I register
  err_code = nrf_drv_mpu_read_registers(UB0_WHO_AM_I, &data, 1);
  // return the register value
	APP_ERROR_CHECK(err_code);
  return data;
}

uint32_t ICM20948_resetMag()
{
	ret_code_t err_code;
  nrf_drv_mpu_write_magnetometer_register(MAG_CNTL3, MAG_CNTL3_RESET);
	uint8_t cntl3;
	err_code = nrf_drv_mpu_read_magnetometer_registers(MAG_CNTL3, &cntl3, 1);
	
  return 1;
}

uint8_t ICM20948_whoAmIMag()
{

  ret_code_t err_code;
  uint8_t data;

  err_code = nrf_drv_mpu_read_magnetometer_registers(MAG_WHO_AM_I_1, &data, 1);
  // return the register value
//	APP_ERROR_CHECK(err_code);

  return data;
}