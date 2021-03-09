
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include <nrf_twi.h>
#include <nrf_twim.h>
#include "mpu6050.h"





//Initializing TWI0 instance
#define TWI_INSTANCE_ID     0

// A flag to indicate the transfer state
static volatile bool m_xfer_done = false;


// Create a Handle for the twi communication
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);




//Event Handler
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    //Check the event to see what type of event occurred
    switch (p_event->type)
    {
        //If data transmission or receiving is finished
	case NRF_DRV_TWI_EVT_DONE:
        m_xfer_done = true;//Set the flag
        break;
        
        default:
        // do nothing
          break;
    }
}



//Initialize the TWI as Master device
void twi_master_init(void)
{
    ret_code_t err_code;

    // Configure the settings for twi communication
    const nrf_drv_twi_config_t twi_config = {
       .scl                = TWI_SCL_M,  //SCL Pin
       .sda                = TWI_SDA_M,  //SDA Pin
       .frequency          = NRF_TWI_FREQ_400K, //Communication Speed
       .interrupt_priority = 2, //Interrupt Priority(Note: if using Bluetooth then select priority carefully)
       .clear_bus_init     = false //automatically clear bus
    };


    //A function to initialize the twi communication
    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    //APP_ERROR_CHECK(err_code);
    
    //Enable the TWI Communication
    nrf_drv_twi_enable(&m_twi);
}



/*
   A function to write a Single Byte to MPU6050's internal Register
*/ 
bool mpu6050_register_write(uint8_t register_address, uint8_t value)
{
    ret_code_t err_code;
    uint8_t tx_buf[MPU6050_ADDRESS_LEN+1];
	
    //Write the register address and data into transmit buffer
    tx_buf[0] = register_address;
    tx_buf[1] = value;

    //Set the flag to false to show the transmission is not yet completed
    m_xfer_done = false;
    
    //Transmit the data over TWI Bus
    err_code = nrf_drv_twi_tx(&m_twi, MPU6050_ADDRESS, tx_buf, MPU6050_ADDRESS_LEN+1, false);
    
    //Wait until the transmission of the data is finished
    while (m_xfer_done == false)
    {
      }

    // if there is no error then return true else return false
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }
    
    return true;	
}




/*
  A Function to read data from the MPU6050
*/ 
bool mpu6050_register_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
    ret_code_t err_code;

    //Set the flag to false to show the receiving is not yet completed
    m_xfer_done = false;
    
    // Send the Register address where we want to write the data
    err_code = nrf_drv_twi_tx(&m_twi, MPU6050_ADDRESS, &register_address, 1, true);
	  
    //Wait for the transmission to get completed
    while (m_xfer_done == false){}
    
    // If transmission was not successful, exit the function with false as return value
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }

    //set the flag again so that we can read data from the MPU6050's internal register
    m_xfer_done = false;
	  
    // Receive the data from the MPU6050
    err_code = nrf_drv_twi_rx(&m_twi, MPU6050_ADDRESS, destination, number_of_bytes);
		
    //wait until the transmission is completed
    while (m_xfer_done == false){}
	
    // if data was successfully read, return true else return false
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }
    
    return true;
}



/*
  A Function to verify the product id
  (its a basic test to check if we are communicating with the right slave, every type of I2C Device has 
  a special WHO_AM_I register which holds a specific value, we can read it from the MPU6050 or any device
  to confirm we are communicating with the right device)
*/ 
bool mpu6050_verify_product_id(void)
{
    uint8_t who_am_i; // create a variable to hold the who am i value


    // Note: All the register addresses including WHO_AM_I are declared in 
    // MPU6050.h file, you can check these addresses and values from the
    // datasheet of your slave device.
    if (mpu6050_register_read(ADDRESS_WHO_AM_I, &who_am_i, 1))
    {
        if (who_am_i != MPU6050_WHO_AM_I)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}


/*
  Function to initialize the mpu6050
*/ 
bool mpu6050_init(void)
{   
  bool transfer_succeeded = true;
	
  //Check the id to confirm that we are communicating with the right device
  transfer_succeeded &= mpu6050_verify_product_id();
	
  if(mpu6050_verify_product_id() == false)
    {
	return false;
      }

  // Set the registers with the required values, see the datasheet to get a good idea of these values
  (void)mpu6050_register_write(MPU_PWR_MGMT1_REG , 0x00); 
  (void)mpu6050_register_write(MPU_SAMPLE_RATE_REG , 0x07); 
  (void)mpu6050_register_write(MPU_CFG_REG , 0x06); 						
  (void)mpu6050_register_write(MPU_INT_EN_REG, 0x00); 
  (void)mpu6050_register_write(MPU_GYRO_CFG_REG , 0x18); 
  (void)mpu6050_register_write(MPU_ACCEL_CFG_REG,0x00);   		

  return transfer_succeeded;
}



/*
  Read the Gyro values from the MPU6050's internal Registers
*/ 
bool MPU6050_ReadGyro(int16_t *pGYRO_X , int16_t *pGYRO_Y , int16_t *pGYRO_Z )
{
  uint8_t buf[6]; 
  
  bool ret = false;	
	
  if(mpu6050_register_read(MPU6050_GYRO_OUT,  buf, 6) == true)
  {
    *pGYRO_X = (buf[0] << 8) | buf[1];
    if(*pGYRO_X & 0x8000) *pGYRO_X-=65536;
		
    *pGYRO_Y= (buf[2] << 8) | buf[3];
    if(*pGYRO_Y & 0x8000) *pGYRO_Y-=65536;
	
    *pGYRO_Z = (buf[4] << 8) | buf[5];
    if(*pGYRO_Z & 0x8000) *pGYRO_Z-=65536;
		
    ret = true;
	}

  return ret;
}	




/*
  A Function to read accelerometer's values from the internal registers of MPU6050
*/ 
bool MPU6050_ReadAcc( int16_t *pACC_X , int16_t *pACC_Y , int16_t *pACC_Z )
{
  uint8_t buf[6];
  bool ret = false;		
  
  if(mpu6050_register_read(MPU6050_ACC_OUT, buf, 6) == true)
  {
    mpu6050_register_read(MPU6050_ACC_OUT, buf, 6);
    
    *pACC_X = (buf[0] << 8) | buf[1];
    if(*pACC_X & 0x8000) *pACC_X-=65536;

    *pACC_Y= (buf[2] << 8) | buf[3];
    if(*pACC_Y & 0x8000) *pACC_Y-=65536;

    *pACC_Z = (buf[4] << 8) | buf[5];
    if(*pACC_Z & 0x8000) *pACC_Z-=65536;
		
    ret = true;
    }
  
  return ret;
}





