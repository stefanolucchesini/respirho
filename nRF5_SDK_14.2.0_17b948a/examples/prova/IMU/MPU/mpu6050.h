#ifndef AT24C02_H__
#define AT24C02_H__
#include "nrf_delay.h"

//I2C Pins Settings, you change them to any other pins
#define TWI_SCL_M           22         //I2C SCL Pin
#define TWI_SDA_M           23         //I2C SDA Pin


#define MPU6050_ADDRESS_LEN  1         //MPU6050
#define MPU6050_ADDRESS     (0xD0>>1)  //MPU6050 Device Address
#define MPU6050_WHO_AM_I     0x68U     //MPU6050 ID


#define MPU6050_GYRO_OUT        0x43
#define MPU6050_ACC_OUT         0x3B

#define ADDRESS_WHO_AM_I          (0x75U) //  WHO_AM_I register identifies the device. Expected value is 0x68.
#define ADDRESS_SIGNAL_PATH_RESET (0x68U) // 

//MPU6050 Registers addresses, see datasheet for more info and each register's function
#define MPU_SELF_TESTX_REG		0x0D	
#define MPU_SELF_TESTY_REG		0x0E	
#define MPU_SELF_TESTZ_REG		0x0F	
#define MPU_SELF_TESTA_REG		0x10	
#define MPU_SAMPLE_RATE_REG		0x19	
#define MPU_CFG_REG                     0x1A	
#define MPU_GYRO_CFG_REG		0x1B	
#define MPU_ACCEL_CFG_REG		0x1C	
#define MPU_MOTION_DET_REG		0x1F	
#define MPU_FIFO_EN_REG			0x23	
#define MPU_I2CMST_CTRL_REG		0x24	
#define MPU_I2CSLV0_ADDR_REG            0x25	
#define MPU_I2CSLV0_REG			0x26	
#define MPU_I2CSLV0_CTRL_REG            0x27	
#define MPU_I2CSLV1_ADDR_REG            0x28	
#define MPU_I2CSLV1_REG			0x29	
#define MPU_I2CSLV1_CTRL_REG            0x2A	
#define MPU_I2CSLV2_ADDR_REG            0x2B	
#define MPU_I2CSLV2_REG			0x2C	
#define MPU_I2CSLV2_CTRL_REG            0x2D	
#define MPU_I2CSLV3_ADDR_REG            0x2E	
#define MPU_I2CSLV3_REG			0x2F	
#define MPU_I2CSLV3_CTRL_REG            0x30	
#define MPU_I2CSLV4_ADDR_REG            0x31	
#define MPU_I2CSLV4_REG			0x32	
#define MPU_I2CSLV4_DO_REG		0x33	
#define MPU_I2CSLV4_CTRL_REG            0x34	
#define MPU_I2CSLV4_DI_REG		0x35	


#define MPU_PWR_MGMT1_REG		0x6B	
#define MPU_PWR_MGMT2_REG		0x6C	

#define MPU_I2CMST_STA_REG		0x36	
#define MPU_INTBP_CFG_REG		0x37	
#define MPU_INT_EN_REG			0x38	
#define MPU_INT_STA_REG			0x3A	

#define MPU_I2CMST_DELAY_REG            0x67	
#define MPU_SIGPATH_RST_REG		0x68	
#define MPU_MDETECT_CTRL_REG            0x69	
#define MPU_USER_CTRL_REG		0x6A	
#define MPU_PWR_MGMT1_REG		0x6B	
#define MPU_PWR_MGMT2_REG		0x6C	
#define MPU_FIFO_CNTH_REG		0x72	
#define MPU_FIFO_CNTL_REG		0x73	
#define MPU_FIFO_RW_REG			0x74	
#define MPU_DEVICE_ID_REG		0x75	

void twi_master_init(void); // initialize the twi communication
bool mpu6050_init(void);    // initialize the mpu6050

/**
  @brief Function for writing a MPU6050 register contents over TWI.
  @param[in]  register_address Register address to start writing to
  @param[in] value Value to write to register
  @retval true Register write succeeded
  @retval false Register write failed
*/
bool mpu6050_register_write(uint8_t register_address, const uint8_t value);

/**
  @brief Function for reading MPU6050 register contents over TWI.
  Reads one or more consecutive registers.
  @param[in]  register_address Register address to start reading from
  @param[in]  number_of_bytes Number of bytes to read
  @param[out] destination Pointer to a data buffer where read data will be stored
  @retval true Register read succeeded
  @retval false Register read failed
*/
bool mpu6050_register_read(uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes);

/**
  @brief Function for reading and verifying MPU6050 product ID.
  @retval true Product ID is what was expected
  @retval false Product ID was not what was expected
*/
bool mpu6050_verify_product_id(void);


bool MPU6050_ReadGyro(int16_t *pGYRO_X , int16_t *pGYRO_Y , int16_t *pGYRO_Z );
bool MPU6050_ReadAcc( int16_t *pACC_X , int16_t *pACC_Y , int16_t *pACC_Z );

#endif


