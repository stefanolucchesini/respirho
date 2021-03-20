 /* 
  * File app_ICM20948.h 
  * The file contains constant (ICM-20948 register map and contstant to set the registers), 
  * structs and function definition.
  */

#ifndef APP_MPU_H__
#define APP_MPU_H__


#include <stdbool.h>
#include <stdint.h>

#include "nrf_peripherals.h"
#include "nrf_delay.h"

#define MPU_MPU_BASE_NUM    		0x4000
#define MPU_BAD_PARAMETER       	(MPU_MPU_BASE_NUM + 0) // An invalid paramameter has been passed to function.

//*********************************************************************************************************************ICM DEFINITIONS
#define ICM20948_WHO_AM_I 0xEA;

// ICM20948 registers
    // User bank 0
#define UB0_WHO_AM_I 0x00
#define UB0_USER_CTRL 0x03
#define UB0_USER_CTRL_I2C_MST_EN 0x20

#define UB0_PWR_MGMNT_1 0x06
#define UB0_PWR_MGMNT_1_CLOCK_SEL_AUTO 0x01
#define UB0_PWR_MGMNT_1_DEV_RESET 0x80

#define UB0_PWR_MGMNT_2 0x07
#define UB0_PWR_MGMNT_2_SEN_ENABLE 0x00
#define UB0_PWR_MGMNT_2_DISABLE 0x3F

#define UB0_INT_PIN_CFG 0x0F
#define UB0_INT_PIN_CFG_HIGH_50US 0x00

#define UB0_INT_ENABLE_1 0x11
#define UB0_INT_ENABLE_1_RAW_RDY_EN 0x01
#define UB0_INT_ENABLE_1_DIS 0x00


#define UB0_ACCEL_XOUT_H 0x2D
#define UB0_GYRO_XOUT_H 0x33
#define UB0_TEMP_OUT_H 0x39

#define UB0_EXT_SLV_SENS_DATA_00 0x3B

    // User bank 2
#define UB2_GYRO_SMPLRT_DIV 0x00

#define UB2_GYRO_CONFIG_1 0x01
#define UB2_GYRO_CONFIG_1_FS_SEL_250DPS 0x00
#define UB2_GYRO_CONFIG_1_FS_SEL_500DPS 0x02
#define UB2_GYRO_CONFIG_1_FS_SEL_1000DPS 0x04
#define UB2_GYRO_CONFIG_1_FS_SEL_2000DPS 0x06
#define UB2_GYRO_CONFIG_1_DLPFCFG_12106HZ 0x00
#define UB2_GYRO_CONFIG_1_DLPFCFG_197HZ 0x00 | 0x01
#define UB2_GYRO_CONFIG_1_DLPFCFG_152HZ 0b00001000 | 0x01
#define UB2_GYRO_CONFIG_1_DLPFCFG_120HZ 0b00010000 | 0x01
#define UB2_GYRO_CONFIG_1_DLPFCFG_51HZ  0b00011000 | 0x01
#define UB2_GYRO_CONFIG_1_DLPFCFG_24HZ  0b00100000 | 0x01
#define UB2_GYRO_CONFIG_1_DLPFCFG_12HZ  0b00101000 | 0x01
#define UB2_GYRO_CONFIG_1_DLPFCFG_6HZ   0b00110000 | 0x01
#define UB2_GYRO_CONFIG_1_DLPFCFG_361HZ 0b00111000 | 0x01
#define UB2_GYRO_CONFIG_1_FS_2000DPS_DLPFCFG_152HZ 15

#define UB2_ACCEL_SMPLRT_DIV_1 0x10
#define UB2_ACCEL_SMPLRT_DIV_2 0x11

#define UB2_ACCEL_CONFIG 0x14
#define UB2_ACCEL_CONFIG_FS_SEL_2G 0x00
#define UB2_ACCEL_CONFIG_FS_SEL_4G 0x02
#define UB2_ACCEL_CONFIG_FS_SEL_8G 0x04
#define UB2_ACCEL_CONFIG_FS_SEL_16G 0x06
#define UB2_ACCEL_CONFIG_DLPFCFG_1209HZ 0x00
#define UB2_ACCEL_CONFIG_DLPFCFG_246HZ 0x00 | 0x01
#define UB2_ACCEL_CONFIG_DLPFCFG_111HZ 0b00010000 | 0x01
#define UB2_ACCEL_CONFIG_DLPFCFG_50HZ  0b00011000 | 0x01
#define UB2_ACCEL_CONFIG_DLPFCFG_24HZ  0b00100000 | 0x01
#define UB2_ACCEL_CONFIG_DLPFCFG_12HZ  0b00101000 | 0x01
#define UB2_ACCEL_CONFIG_DLPFCFG_6HZ   0b00110000 | 0x01
#define UB2_ACCEL_CONFIG_DLPFCFG_473HZ 0b00111000 | 0x01

    // User bank 3
#define UB3_I2C_MST_CTRL 0x01
#define UB3_I2C_MST_CTRL_CLK_400KHZ 0x07 // Gives 345.6kHz and is recommended to achieve max 400kHz

#define UB3_I2C_SLV0_ADDR 0x03
#define UB3_I2C_SLV0_ADDR_READ_FLAG 0x80

#define UB3_I2C_SLV0_REG 0x04

#define UB3_I2C_SLV0_CTRL 0x05
#define UB3_I2C_SLV0_CTRL_EN 0x80

#define UB3_I2C_SLV0_DO 0x06

    // Common to all user banks
#define REG_BANK_SEL 0x7F
#define REG_BANK_SEL_USER_BANK_0 0x00
#define REG_BANK_SEL_USER_BANK_1 0x10
#define REG_BANK_SEL_USER_BANK_2 0x20
#define REG_BANK_SEL_USER_BANK_3 0x30

    // Magnetometer ants
#define MAG_AK09916_I2C_ADDR 0x0C
#define MAG_AK09916_WHO_AM_I 0x4809
#define MAG_DATA_LENGTH 8 // Bytes

		// Magnetometer (AK09916) registers
#define MAG_WHO_AM_I_1 0x00
#define MAG_WHO_AM_I_2 0x01

#define MAG_HXL 0x11

#define MAG_CNTL2 0x31
#define MAG_CNTL2_POWER_DOWN 0x00
#define MAG_CNTL2_MODE_10HZ 0x02
#define MAG_CNTL2_MODE_50HZ 0x06
#define MAG_CNTL2_MODE_100HZ 0x08
#define MAG_ST1 0x10
#define MAG_ST2 0x18

#define MAG_CNTL3 0x32
#define MAG_CNTL3_RESET 0x01



enum UserBank
{
	USER_BANK_0,
	USER_BANK_1,
	USER_BANK_2,
	USER_BANK_3,
};


/**@brief Structure to hold acceleromter values. 
 * Sequence of z, y, and x is important to correspond with 
 * the sequence of which z, y, and x data are read from the sensor.
 * All values are unsigned 16 bit integers
*/
typedef struct
{
    int16_t z;
    int16_t y;
    int16_t x;
}accel_values_t;


/**@brief Structure to hold gyroscope values. 
 * Sequence of z, y, and x is important to correspond with 
 * the sequence of which z, y, and x data are read from the sensor.
 * All values are unsigned 16 bit integers
*/
typedef struct
{
    int16_t z;
    int16_t y;
    int16_t x;
}gyro_values_t;

/**@brief Simple typedef to hold temperature values */
typedef int16_t temp_value_t;

/**@brief Structure to hold gyroscope float values. 
 * Sequence of z, y, and x is important to correspond with 
 * the sequence of which z, y, and x data are read from the sensor.
 * All values are unsigned 16 bit integers
*/
typedef struct
{
    float z;
    float y;
    float x;
}gyro_values_float_t;

/**@brief Structure to hold accelerometer float values. 
 * Sequence of z, y, and x is important to correspond with 
 * the sequence of which z, y, and x data are read from the sensor.
 * All values are unsigned 16 bit integers
*/
typedef struct
{
    float z;
    float y;
    float x;
}accel_values_float_t;


/**@brief ICM driver interrupt pin configuration structure. */    
typedef struct
{
    uint8_t reserved        :1;  // reserved
    uint8_t i2c_bypass_en   :1;  // When asserted, the i2c_master interface pins(ES_CL and ES_DA) will go into ‘bypass mode’ when the i2c master interface is disabled. 
    uint8_t fsync_int_en    :1;  // When equal to 0, this bit disables the FSYNC pin from causing an interrupt to the host processor. When equal to 1, this bit enables the FSYNC pin to be used as an interrupt to the host processor.
    uint8_t fsync_int_level :1;  // When this bit is equal to 0, the logic level for the FSYNC pin (when used as an interrupt to the host processor) is active high. When this bit is equal to 1, the logic level for the FSYNC pin (when used as an interrupt to the host processor) is active low.
    uint8_t int_rd_clear    :1;  // When this bit is equal to 0, interrupt status bits are cleared only by reading INT_STATUS (Register 58). When this bit is equal to 1, interrupt status bits are cleared on any read operation.
    uint8_t latch_int_en    :1;  // When this bit is equal to 0, the INT pin emits a 50us long pulse. When this bit is equal to 1, the INT pin is held high until the interrupt is cleared.
    uint8_t int_open        :1;  // When this bit is equal to 0, the INT pin is configured as push-pull. When this bit is equal to 1, the INT pin is configured as open drain.
    uint8_t int_level       :1;  // When this bit is equal to 0, the logic level for the INT pin is active high. When this bit is equal to 1, the logic level for the INT pin is active low.
}app_icm_int_pin_cfg_t;

/* Functions */

/**@brief Function to reset the ICM-20948
 *
 * @retval      uint32_t        Error code
 */
uint32_t ICM20948_reset(void);

/**@brief Function to select a USER BANK
 *
 * @param[in]   userBank    		Variable to select USER BANK
 * @retval      uint32_t        Error code
 */
uint32_t app_icm_reg_bank_select(uint8_t userBank);

/**@brief Function for initiating ICM and ICM library
 *
 * @retval      uint32_t        Error code
 */
uint32_t app_icm_init(void);

/**@brief Function to select the clock source 
 *
 * @retval      uint32_t        Error code
 */
uint32_t ICM20948_selectAutoClockSource(void);

/**@brief Function to enable accelerometer and gyroscope 
 *
 * @retval      uint32_t        Error code
 */
uint32_t ICM20948_enableAccelGyro(void);

/**@brief Function to disable accelerometer and gyroscope 
 *
 * @retval      uint32_t        Error code
 */
uint32_t ICM20948_disableAccelGyro(void);

/**@brief Function to configure accelerometer 
 *
 * @retval      uint32_t        Error code
 */
uint32_t ICM20948_configAccel(void);

/**@brief Function to configure accelerometer 
 *
 * @retval      uint32_t        Error code
 */
uint32_t ICM20948_configGyro(void);

/**@brief Function to set accelerometer sample rate divider
 *
 * @retval      uint32_t        Error code
 */
uint32_t ICM20948_setAccelSrd(uint16_t srd);

/**@brief Function to set gyroscope sample rate divider
 *
 * @retval      uint32_t        Error code
 */
uint32_t ICM20948_setGyroSrd(uint8_t srd);


/**@brief Function for reading ICM accelerometer data.
 *
 * @param[in]   accel_values    Pointer to variable to hold accelerometer data
 * @retval      uint32_t        Error code
 */
uint32_t app_icm_read_accel(accel_values_t * accel_values);


/**@brief Function for reading ICM gyroscope data.
 *
 * @param[in]   gyro_values     Pointer to variable to hold gyroscope data
 * @retval      uint32_t        Error code
 */
uint32_t app_icm_read_gyro(gyro_values_t * gyro_values);


/**@brief Function for reading ICM temperature data.
 *
 * @param[in]   temp_values     Pointer to variable to hold temperature data
 * @retval      uint32_t        Error code
 */
uint32_t app_icm_read_temp(temp_value_t * temp_values);

/**@brief Function for configuring the behaviour of the interrupt pin of the ICM
 *
 * @param[in]   config          Pointer to configuration structure
 * @retval      uint32_t        Error code
 */
uint32_t app_icm_int_cfg_pin(app_icm_int_pin_cfg_t *cfg);

uint8_t ICM20948_whoAmI(void);

/* FUNCTIONS FOR MAGNETOMETER */


#if (defined(ICM20948) || defined(MPU9255)) && (MPU_USES_TWI) // Magnetometer only works with TWI so check if TWI is enabled


typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
}magn_values_t;

/**@brief Configuration structure used to set magnetometer operation mode
 */
typedef struct
{
	uint8_t mode : 5;  

}app_icm_magn_config_t;


/**@brief Structure to hold data read from MAG_ST2 after reading sensor values.
 */
typedef struct
{
	uint8_t 			: 3;  
	uint8_t overflow 	: 1; //  single measurement mode, continuous measurement mode, external trigger measurement mode and self-test mode, magnetic sensor may overflow even though measurement data regiseter is not saturated. 
	uint8_t res_mirror 	: 1; // Output bit setting (mirror) 
}app_mpu_magn_read_status_t;


/**@brief Function for enabling and starting the magnetometer
 *
 * @param[in]   app_mpu_magn_config_t 	Magnetometer config struct
 * @retval      uint32_t        	Error code
 */
uint32_t app_icm_magnetometer_init(app_icm_magn_config_t * p_magnetometer_conf);


/**@brief Function for reading out magnetometer values
 *
 * @param[in]   magn_values_t *				Magnetometer values struct
 * @param[in]   app_mpu_magn_read_status_t *	Value of status register 2 (MAG_ST2) after magnetometer data is read. NULL can be passed as argument if status is not needed
 * @retval      uint32_t     				Error code
 */
uint32_t app_icm_read_magnetometer(magn_values_t * p_magnetometer_values, app_mpu_magn_read_status_t * p_read_status);


#endif

#endif /* APP_MPU_H__ */



uint32_t ICM20948_enableI2cMaster(void); //controllare se serve

uint32_t ICM20948_resetMag(void);
uint8_t ICM20948_whoAmIMag(void);

/**
  @}
*/

