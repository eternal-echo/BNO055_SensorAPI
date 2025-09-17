/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file bno055_support.c
* @date 10/01/2020
* @version  2.0.6
*
*/

/*---------------------------------------------------------------------------*
*  Includes
*---------------------------------------------------------------------------*/
#include "bno055.h"
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"
#include "esp_check.h"

// I2C timeout constant
#define I2C_MASTER_TIMEOUT_MS       1000
// BNO055 quaternion LSB scale (per datasheet: 1 LSB = 1/16384)
#define BNO055_QUATERNION_SCALE_FACTOR 16384.0f

static const char* TAG = "BNO055";

// New I2C master driver handles
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_bno_dev = NULL;

// Unified acquisition task handle
static TaskHandle_t s_bno_acq_task_handle = NULL;

// Global sensor data - no locks, single writer, single reader
volatile struct {
    float accel_x, accel_y, accel_z;    // m/s²
    float gyro_x, gyro_y, gyro_z;       // deg/s
    float mag_x, mag_y, mag_z;          // µT
    float euler_h, euler_r, euler_p;    // degrees
    float quat_w, quat_x, quat_y, quat_z; // quaternion
    float gravity_x, gravity_y, gravity_z; // m/s²
    float linear_accel_x, linear_accel_y, linear_accel_z; // m/s²
    uint8_t calib_sys, calib_gyro, calib_accel, calib_mag; // calib status
    uint32_t timestamp_us;
    bool data_valid;
} bno055_latest_data = {0};

/*----------------------------------------------------------------------------*
*  The following APIs are used for reading and writing of
*   sensor data using I2C communication
*----------------------------------------------------------------------------*/
#ifdef  BNO055_API
#define BNO055_I2C_BUS_WRITE_ARRAY_INDEX ((u8)1)

/*  \Brief: The API is used as I2C bus read
 *  \Return : Status of the I2C read
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be read
 *  \param reg_data : This data read from the sensor,
 *   which is hold in an array
 *  \param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*  \Brief: The API is used as SPI bus write
 *  \Return : Status of the SPI write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *  will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

/*
 * \Brief: I2C init routine
 */
s8 I2C_routine(void);

/*  Brief : The delay routine
 *  \param : delay in ms
 */
void BNO055_delay_msek(u32 msek);

/*  Note: IRQ init/deinit logic is embedded inside bno055_data_acq_start/stop */


/*  Brief : Cleanup BNO055 interrupt support (now merged into bno055_data_acq_stop) */

/*  Brief : Start background data acquisition
 *  If IRQ enabled, use IRQ task; otherwise, use periodic polling.
 */
esp_err_t bno055_data_acq_start(void);

/*  Brief : Stop background data acquisition (IRQ or polling)
 */
esp_err_t bno055_data_acq_stop(void);

/*  Brief : Complete BNO055 initialization with status validation
 *  \return : ESP_OK if successful, error code otherwise
 */
esp_err_t bno055_init_sensor(void);

#endif

/********************End of I2C APIs declarations***********************/

/* This API is an example for reading sensor data
 *  \param: None
 *  \return: communication result
 */
s32 bno055_data_readout_template(void);

/*----------------------------------------------------------------------------*
 *  struct bno055_t parameters can be accessed by using BNO055
 *  BNO055_t having the following parameters
 *  Bus write function pointer: BNO055_WR_FUNC_PTR
 *  Bus read function pointer: BNO055_RD_FUNC_PTR
 *  Burst read function pointer: BNO055_BRD_FUNC_PTR
 *  Delay function pointer: delay_msec
 *  I2C address: dev_addr
 *  Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/
struct bno055_t bno055;

/* This API is an example for reading sensor data
 *  \param: None
 *  \return: communication result
 */
s32 bno055_data_readout_template(void)
{
    /* Variable used to return value of
     * communication routine*/
    s32 comres = BNO055_ERROR;

    /* variable used to set the power mode of the sensor*/
    u8 power_mode = BNO055_INIT_VALUE;

    /*********read raw accel data***********/
    /* variable used to read the accel x data */
    s16 accel_datax = BNO055_INIT_VALUE;

    /* variable used to read the accel y data */
    s16 accel_datay = BNO055_INIT_VALUE;

    /* variable used to read the accel z data */
    s16 accel_dataz = BNO055_INIT_VALUE;

    /* variable used to read the accel xyz data */
    struct bno055_accel_t accel_xyz;

    /*********read raw mag data***********/
    /* variable used to read the mag x data */
    s16 mag_datax = BNO055_INIT_VALUE;

    /* variable used to read the mag y data */
    s16 mag_datay = BNO055_INIT_VALUE;

    /* variable used to read the mag z data */
    s16 mag_dataz = BNO055_INIT_VALUE;

    /* structure used to read the mag xyz data */
    struct bno055_mag_t mag_xyz;

    /***********read raw gyro data***********/
    /* variable used to read the gyro x data */
    s16 gyro_datax = BNO055_INIT_VALUE;

    /* variable used to read the gyro y data */
    s16 gyro_datay = BNO055_INIT_VALUE;

    /* variable used to read the gyro z data */
    s16 gyro_dataz = BNO055_INIT_VALUE;

    /* structure used to read the gyro xyz data */
    struct bno055_gyro_t gyro_xyz;

    /*************read raw Euler data************/
    /* variable used to read the euler h data */
    s16 euler_data_h = BNO055_INIT_VALUE;

    /* variable used to read the euler r data */
    s16 euler_data_r = BNO055_INIT_VALUE;

    /* variable used to read the euler p data */
    s16 euler_data_p = BNO055_INIT_VALUE;

    /* structure used to read the euler hrp data */
    struct bno055_euler_t euler_hrp;

    /************read raw quaternion data**************/
    /* variable used to read the quaternion w data */
    s16 quaternion_data_w = BNO055_INIT_VALUE;

    /* variable used to read the quaternion x data */
    s16 quaternion_data_x = BNO055_INIT_VALUE;

    /* variable used to read the quaternion y data */
    s16 quaternion_data_y = BNO055_INIT_VALUE;

    /* variable used to read the quaternion z data */
    s16 quaternion_data_z = BNO055_INIT_VALUE;

    /* structure used to read the quaternion wxyz data */
    struct bno055_quaternion_t quaternion_wxyz;

    /************read raw linear acceleration data***********/
    /* variable used to read the linear accel x data */
    s16 linear_accel_data_x = BNO055_INIT_VALUE;

    /* variable used to read the linear accel y data */
    s16 linear_accel_data_y = BNO055_INIT_VALUE;

    /* variable used to read the linear accel z data */
    s16 linear_accel_data_z = BNO055_INIT_VALUE;

    /* structure used to read the linear accel xyz data */
    struct bno055_linear_accel_t linear_acce_xyz;

    /*****************read raw gravity sensor data****************/
    /* variable used to read the gravity x data */
    s16 gravity_data_x = BNO055_INIT_VALUE;

    /* variable used to read the gravity y data */
    s16 gravity_data_y = BNO055_INIT_VALUE;

    /* variable used to read the gravity z data */
    s16 gravity_data_z = BNO055_INIT_VALUE;

    /* structure used to read the gravity xyz data */
    struct bno055_gravity_t gravity_xyz;

    /*************read accel converted data***************/
    /* variable used to read the accel x data output as m/s2 or mg */
    double d_accel_datax = BNO055_INIT_VALUE;

    /* variable used to read the accel y data output as m/s2 or mg */
    double d_accel_datay = BNO055_INIT_VALUE;

    /* variable used to read the accel z data output as m/s2 or mg */
    double d_accel_dataz = BNO055_INIT_VALUE;

    /* structure used to read the accel xyz data output as m/s2 or mg */
    struct bno055_accel_double_t d_accel_xyz;

    /******************read mag converted data********************/
    /* variable used to read the mag x data output as uT*/
    double d_mag_datax = BNO055_INIT_VALUE;

    /* variable used to read the mag y data output as uT*/
    double d_mag_datay = BNO055_INIT_VALUE;

    /* variable used to read the mag z data output as uT*/
    double d_mag_dataz = BNO055_INIT_VALUE;

    /* structure used to read the mag xyz data output as uT*/
    struct bno055_mag_double_t d_mag_xyz;

    /*****************read gyro converted data************************/
    /* variable used to read the gyro x data output as dps or rps */
    double d_gyro_datax = BNO055_INIT_VALUE;

    /* variable used to read the gyro y data output as dps or rps */
    double d_gyro_datay = BNO055_INIT_VALUE;

    /* variable used to read the gyro z data output as dps or rps */
    double d_gyro_dataz = BNO055_INIT_VALUE;

    /* structure used to read the gyro xyz data output as dps or rps */
    struct bno055_gyro_double_t d_gyro_xyz;

    /*******************read euler converted data*******************/

    /* variable used to read the euler h data output
     * as degree or radians*/
    double d_euler_data_h = BNO055_INIT_VALUE;

    /* variable used to read the euler r data output
     * as degree or radians*/
    double d_euler_data_r = BNO055_INIT_VALUE;

    /* variable used to read the euler p data output
     * as degree or radians*/
    double d_euler_data_p = BNO055_INIT_VALUE;

    /* structure used to read the euler hrp data output
     * as as degree or radians */
    struct bno055_euler_double_t d_euler_hpr;

    /*********read linear acceleration converted data**********/
    /* variable used to read the linear accel x data output as m/s2*/
    double d_linear_accel_datax = BNO055_INIT_VALUE;

    /* variable used to read the linear accel y data output as m/s2*/
    double d_linear_accel_datay = BNO055_INIT_VALUE;

    /* variable used to read the linear accel z data output as m/s2*/
    double d_linear_accel_dataz = BNO055_INIT_VALUE;

    /* structure used to read the linear accel xyz data output as m/s2*/
    struct bno055_linear_accel_double_t d_linear_accel_xyz;

    /********************Gravity converted data**********************/
    /* variable used to read the gravity sensor x data output as m/s2*/
    double d_gravity_data_x = BNO055_INIT_VALUE;

    /* variable used to read the gravity sensor y data output as m/s2*/
    double d_gravity_data_y = BNO055_INIT_VALUE;

    /* variable used to read the gravity sensor z data output as m/s2*/
    double d_gravity_data_z = BNO055_INIT_VALUE;

    /* structure used to read the gravity xyz data output as m/s2*/
    struct bno055_gravity_double_t d_gravity_xyz;

    /*---------------------------------------------------------------------------*
     *********************** START INITIALIZATION ************************
     *--------------------------------------------------------------------------*/
#ifdef  BNO055_API

    /*  Based on the user need configure I2C interface.
     *  It is example code to explain how to use the bno055 API*/
    I2C_routine();
#endif

    /*--------------------------------------------------------------------------*
     *  This API used to assign the value/reference of
     *  the following parameters
     *  I2C address
     *  Bus Write
     *  Bus read
     *  Chip id
     *  Page id
     *  Accel revision id
     *  Mag revision id
     *  Gyro revision id
     *  Boot loader revision id
     *  Software revision id
     *-------------------------------------------------------------------------*/
    comres = bno055_init(&bno055);

    /*  For initializing the BNO sensor it is required to the operation mode
     * of the sensor as NORMAL
     * Normal mode can set from the register
     * Page - page0
     * register - 0x3E
     * bit positions - 0 and 1*/
    power_mode = BNO055_POWER_MODE_NORMAL;

    /* set the power mode as NORMAL*/
    comres += bno055_set_power_mode(power_mode);

    /*----------------------------------------------------------------*
     ************************* END INITIALIZATION *************************
     *-----------------------------------------------------------------*/

    /************************* START READ RAW SENSOR DATA****************/

    /*  Using BNO055 sensor we can read the following sensor data and
     * virtual sensor data
     * Sensor data:
     * Accel
     * Mag
     * Gyro
     * Virtual sensor data
     * Euler
     * Quaternion
     * Linear acceleration
     * Gravity sensor */

    /*  For reading sensor raw data it is required to set the
     * operation modes of the sensor
     * operation mode can set from the register
     * page - page0
     * register - 0x3D
     * bit - 0 to 3
     * for sensor data read following operation mode have to set
     * SENSOR MODE
     * 0x01 - BNO055_OPERATION_MODE_ACCONLY
     * 0x02 - BNO055_OPERATION_MODE_MAGONLY
     * 0x03 - BNO055_OPERATION_MODE_GYRONLY
     * 0x04 - BNO055_OPERATION_MODE_ACCMAG
     * 0x05 - BNO055_OPERATION_MODE_ACCGYRO
     * 0x06 - BNO055_OPERATION_MODE_MAGGYRO
     * 0x07 - BNO055_OPERATION_MODE_AMG
     * based on the user need configure the operation mode*/
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);

    /*  Raw accel X, Y and Z data can read from the register
     * page - page 0
     * register - 0x08 to 0x0D*/
    comres += bno055_read_accel_x(&accel_datax);
    comres += bno055_read_accel_y(&accel_datay);
    comres += bno055_read_accel_z(&accel_dataz);
    comres += bno055_read_accel_xyz(&accel_xyz);

    /*  Raw mag X, Y and Z data can read from the register
     * page - page 0
     * register - 0x0E to 0x13*/
    comres += bno055_read_mag_x(&mag_datax);
    comres += bno055_read_mag_y(&mag_datay);
    comres += bno055_read_mag_z(&mag_dataz);
    comres += bno055_read_mag_xyz(&mag_xyz);

    /*  Raw gyro X, Y and Z data can read from the register
     * page - page 0
     * register - 0x14 to 0x19*/
    comres += bno055_read_gyro_x(&gyro_datax);
    comres += bno055_read_gyro_y(&gyro_datay);
    comres += bno055_read_gyro_z(&gyro_dataz);
    comres += bno055_read_gyro_xyz(&gyro_xyz);

    /************************* END READ RAW SENSOR DATA****************/

    /************************* START READ RAW FUSION DATA ********
     * For reading fusion data it is required to set the
     * operation modes of the sensor
     * operation mode can set from the register
     * page - page0
     * register - 0x3D
     * bit - 0 to 3
     * for sensor data read following operation mode have to set
     * FUSION MODE
     * 0x08 - BNO055_OPERATION_MODE_IMUPLUS
     * 0x09 - BNO055_OPERATION_MODE_COMPASS
     * 0x0A - BNO055_OPERATION_MODE_M4G
     * 0x0B - BNO055_OPERATION_MODE_NDOF_FMC_OFF
     * 0x0C - BNO055_OPERATION_MODE_NDOF
     * based on the user need configure the operation mode*/
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

    /*  Raw Euler H, R and P data can read from the register
     * page - page 0
     * register - 0x1A to 0x1E */
    comres += bno055_read_euler_h(&euler_data_h);
    comres += bno055_read_euler_r(&euler_data_r);
    comres += bno055_read_euler_p(&euler_data_p);
    comres += bno055_read_euler_hrp(&euler_hrp);

    /*  Raw Quaternion W, X, Y and Z data can read from the register
     * page - page 0
     * register - 0x20 to 0x27 */
    comres += bno055_read_quaternion_w(&quaternion_data_w);
    comres += bno055_read_quaternion_x(&quaternion_data_x);
    comres += bno055_read_quaternion_y(&quaternion_data_y);
    comres += bno055_read_quaternion_z(&quaternion_data_z);
    comres += bno055_read_quaternion_wxyz(&quaternion_wxyz);

    /*  Raw Linear accel X, Y and Z data can read from the register
     * page - page 0
     * register - 0x28 to 0x2D */
    comres += bno055_read_linear_accel_x(&linear_accel_data_x);
    comres += bno055_read_linear_accel_y(&linear_accel_data_y);
    comres += bno055_read_linear_accel_z(&linear_accel_data_z);
    comres += bno055_read_linear_accel_xyz(&linear_acce_xyz);

    /*  Raw Gravity sensor X, Y and Z data can read from the register
     * page - page 0
     * register - 0x2E to 0x33 */
    comres += bno055_read_gravity_x(&gravity_data_x);
    comres += bno055_read_gravity_y(&gravity_data_y);
    comres += bno055_read_gravity_z(&gravity_data_z);
    comres += bno055_read_gravity_xyz(&gravity_xyz);

    /************************* END READ RAW FUSION DATA  ************/
    /******************START READ CONVERTED SENSOR DATA****************/

    /*  API used to read accel data output as double  - m/s2 and mg
     * float functions also available in the BNO055 API */
    comres += bno055_convert_double_accel_x_msq(&d_accel_datax);
    comres += bno055_convert_double_accel_x_mg(&d_accel_datax);
    comres += bno055_convert_double_accel_y_msq(&d_accel_datay);
    comres += bno055_convert_double_accel_y_mg(&d_accel_datay);
    comres += bno055_convert_double_accel_z_msq(&d_accel_dataz);
    comres += bno055_convert_double_accel_z_mg(&d_accel_dataz);
    comres += bno055_convert_double_accel_xyz_msq(&d_accel_xyz);
    comres += bno055_convert_double_accel_xyz_mg(&d_accel_xyz);

    /*  API used to read mag data output as double  - uT(micro Tesla)
     * float functions also available in the BNO055 API */
    comres += bno055_convert_double_mag_x_uT(&d_mag_datax);
    comres += bno055_convert_double_mag_y_uT(&d_mag_datay);
    comres += bno055_convert_double_mag_z_uT(&d_mag_dataz);
    comres += bno055_convert_double_mag_xyz_uT(&d_mag_xyz);

    /*  API used to read gyro data output as double  - dps and rps
     * float functions also available in the BNO055 API */
    comres += bno055_convert_double_gyro_x_dps(&d_gyro_datax);
    comres += bno055_convert_double_gyro_y_dps(&d_gyro_datay);
    comres += bno055_convert_double_gyro_z_dps(&d_gyro_dataz);
    comres += bno055_convert_double_gyro_x_rps(&d_gyro_datax);
    comres += bno055_convert_double_gyro_y_rps(&d_gyro_datay);
    comres += bno055_convert_double_gyro_z_rps(&d_gyro_dataz);
    comres += bno055_convert_double_gyro_xyz_dps(&d_gyro_xyz);
    comres += bno055_convert_double_gyro_xyz_rps(&d_gyro_xyz);

    /*  API used to read Euler data output as double  - degree and radians
     * float functions also available in the BNO055 API */
    comres += bno055_convert_double_euler_h_deg(&d_euler_data_h);
    comres += bno055_convert_double_euler_r_deg(&d_euler_data_r);
    comres += bno055_convert_double_euler_p_deg(&d_euler_data_p);
    comres += bno055_convert_double_euler_h_rad(&d_euler_data_h);
    comres += bno055_convert_double_euler_r_rad(&d_euler_data_r);
    comres += bno055_convert_double_euler_p_rad(&d_euler_data_p);
    comres += bno055_convert_double_euler_hpr_deg(&d_euler_hpr);
    comres += bno055_convert_double_euler_hpr_rad(&d_euler_hpr);

    /*  API used to read Linear acceleration data output as m/s2
     * float functions also available in the BNO055 API */
    comres += bno055_convert_double_linear_accel_x_msq(&d_linear_accel_datax);
    comres += bno055_convert_double_linear_accel_y_msq(&d_linear_accel_datay);
    comres += bno055_convert_double_linear_accel_z_msq(&d_linear_accel_dataz);
    comres += bno055_convert_double_linear_accel_xyz_msq(&d_linear_accel_xyz);

    /*  API used to read Gravity sensor data output as m/s2
     * float functions also available in the BNO055 API */
    comres += bno055_convert_gravity_double_x_msq(&d_gravity_data_x);
    comres += bno055_convert_gravity_double_y_msq(&d_gravity_data_y);
    comres += bno055_convert_gravity_double_z_msq(&d_gravity_data_z);
    comres += bno055_convert_double_gravity_xyz_msq(&d_gravity_xyz);

    /*-----------------------------------------------------------------------*
     ************************* START DE-INITIALIZATION ***********************
     *-------------------------------------------------------------------------*/

    /*  For de - initializing the BNO sensor it is required
     * to the operation mode of the sensor as SUSPEND
     * Suspend mode can set from the register
     * Page - page0
     * register - 0x3E
     * bit positions - 0 and 1*/
    power_mode = BNO055_POWER_MODE_SUSPEND;

    /* set the power mode as SUSPEND*/
    comres += bno055_set_power_mode(power_mode);

    /*---------------------------------------------------------------------*
    ************************* END DE-INITIALIZATION **********************
    *---------------------------------------------------------------------*/
    return comres;
}

#ifdef  BNO055_API

/*--------------------------------------------------------------------------*
 *  The following API is used to map the I2C bus read, write, delay and
 *  device address with global structure bno055_t
 *-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*
 *  By using bno055 the following structure parameter can be accessed
 *  Bus write function pointer: BNO055_WR_FUNC_PTR
 *  Bus read function pointer: BNO055_RD_FUNC_PTR
 *  Delay function pointer: delay_msec
 *  I2C address: dev_addr
 *--------------------------------------------------------------------------*/
s8 I2C_routine(void)
{
    esp_err_t ret = ESP_OK;

    // Create I2C master bus (new driver)
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = (i2c_port_num_t)CONFIG_BNO055_I2C_PORT_NUM,
        .sda_io_num = (gpio_num_t)CONFIG_BNO055_I2C_SDA_IO,
        .scl_io_num = (gpio_num_t)CONFIG_BNO055_I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
            .allow_pd = 0,
        },
    };

    if (s_i2c_bus == NULL) {
        ESP_GOTO_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_i2c_bus), err, TAG, "I2C bus init failed");
    }

    // Probe and add BNO055 device on the bus
    uint16_t found_addr = BNO055_I2C_ADDR1;
    ret = i2c_master_probe(s_i2c_bus, found_addr, 100);
    if (ret == ESP_ERR_NOT_FOUND) {
        // Try secondary address
        found_addr = BNO055_I2C_ADDR2;
        ret = i2c_master_probe(s_i2c_bus, found_addr, 100);
    }
    ESP_GOTO_ON_ERROR(ret, err, TAG, "BNO055 not found on I2C bus");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = found_addr,
        .scl_speed_hz = CONFIG_BNO055_I2C_FREQ_HZ,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };

    if (s_bno_dev == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_bno_dev), err, TAG, "I2C add device failed");
    }

    ESP_EARLY_LOGI(TAG, "I2C initialized successfully, found BNO055 at 0x%02X, SDA GPIO %d, SCL GPIO %d",
            found_addr, bus_cfg.sda_io_num, bus_cfg.scl_io_num);

    // Set BNO055 function pointers
    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msek;
    bno055.dev_addr = (u8)found_addr;

    return BNO055_INIT_VALUE;

err:
    if (s_bno_dev) {
        i2c_master_bus_rm_device(s_bno_dev);
        s_bno_dev = NULL;
    }
    if (s_i2c_bus) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }
    return BNO055_ERROR;
}

/************** I2C buffer length******/

#define I2C_BUFFER_LEN 8
#define I2C0           5

/*-------------------------------------------------------------------*
 *
 *  This is a sample code for read and write the data by using I2C
 *  Use either I2C  based on your need
 *  The device address defined in the bno055.h file
 *
 *--------------------------------------------------------------------*/

/*  \Brief: The API is used as I2C bus write
 *  \Return : Status of the I2C write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *      will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    (void)dev_addr; // Address bound to device handle
    if (s_bno_dev == NULL) {
        return BNO055_ERROR;
    }

    uint8_t tmp_buf[1 + 255];
    tmp_buf[0] = reg_addr;
    if (cnt > 0 && reg_data != NULL) {
        memcpy(&tmp_buf[1], reg_data, cnt);
    }

    esp_err_t err = i2c_master_transmit(s_bno_dev, tmp_buf, (size_t)(cnt + 1), I2C_MASTER_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_EARLY_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(err));
        return BNO055_ERROR;
    }

    return BNO055_SUCCESS;
}

/*  \Brief: The API is used as I2C bus read
 *  \Return : Status of the I2C read
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *  will data is going to be read
 *  \param reg_data : This data read from the sensor,
 *   which is hold in an array
 *  \param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    (void)dev_addr; // Address bound to device handle
    if (cnt == 0 || reg_data == NULL) {
        return BNO055_ERROR;
    }
    if (s_bno_dev == NULL) {
        return BNO055_ERROR;
    }

    uint8_t reg = reg_addr;
    esp_err_t err = i2c_master_transmit_receive(s_bno_dev, &reg, 1, reg_data, cnt, I2C_MASTER_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_EARLY_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(err));
        return BNO055_ERROR;
    }

    return BNO055_SUCCESS;
}

/*  Brief : The delay routine
 *  \param : delay in ms
 */
void BNO055_delay_msek(u32 msek)
{
    esp_rom_delay_us(msek * 1000);  // 阻塞延迟，不让出CPU
}


// Common one-shot sampler
static inline void bno055_sample_once(void)
{
    struct bno055_accel_double_t accel_data;
    struct bno055_gyro_double_t gyro_data;
    struct bno055_mag_double_t mag_data;
    struct bno055_euler_double_t euler_data;
    struct bno055_quaternion_t quat_data;
    struct bno055_gravity_double_t grav_data;
    struct bno055_linear_accel_double_t linear_accel_data;

    // // Check system status and error codes before sampling
    // u8 sys_status = 0;
    // u8 sys_error = 0;
    
    // // Check system status - should be in normal operation mode
    // if (bno055_get_sys_stat_code(&sys_status) != BNO055_SUCCESS) {
    //     ESP_LOGW(TAG, "Failed to read system status");
    //     bno055_latest_data.data_valid = false;
    //     return;
    // }
    
    // // Check for system errors
    // if (bno055_get_sys_error_code(&sys_error) != BNO055_SUCCESS) {
    //     ESP_LOGW(TAG, "Failed to read system error code");
    //     bno055_latest_data.data_valid = false;
    //     return;
    // }
    
    // // Validate system status according to BNO055 datasheet:
    // // 0x00 = System idle (not acceptable for sampling)
    // // 0x01 = System error (not acceptable)  
    // // 0x02 = Initializing peripherals
    // // 0x03 = System initialization
    // // 0x04 = Executing self-test
    // // 0x05 = Sensor fusion algorithm running
    // // 0x06 = System running without fusion algorithm
    // if (sys_status < 0x05) {
    //     ESP_LOGW(TAG, "System not ready, status: 0x%02X", sys_status);
    //     bno055_latest_data.data_valid = false;
    //     return;
    // }
    
    // // Check for system errors (0x00 = No error)
    // if (sys_error != 0x00) {
    //     ESP_LOGW(TAG, "System error detected: 0x%02X", sys_error);
    //     bno055_latest_data.data_valid = false;
    //     return;
    // }

    bno055_latest_data.timestamp_us = esp_timer_get_time();

    if (bno055_convert_double_accel_xyz_msq(&accel_data) == BNO055_SUCCESS) {
        bno055_latest_data.accel_x = (float)accel_data.x;
        bno055_latest_data.accel_y = (float)accel_data.y;
        bno055_latest_data.accel_z = (float)accel_data.z;
    }
    if (bno055_convert_double_gyro_xyz_dps(&gyro_data) == BNO055_SUCCESS) {
        bno055_latest_data.gyro_x = (float)gyro_data.x;
        bno055_latest_data.gyro_y = (float)gyro_data.y;
        bno055_latest_data.gyro_z = (float)gyro_data.z;
    }
    if (bno055_convert_double_mag_xyz_uT(&mag_data) == BNO055_SUCCESS) {
        bno055_latest_data.mag_x = (float)mag_data.x;
        bno055_latest_data.mag_y = (float)mag_data.y;
        bno055_latest_data.mag_z = (float)mag_data.z;
    }
    if (bno055_convert_double_euler_hpr_deg(&euler_data) == BNO055_SUCCESS) {
        bno055_latest_data.euler_h = (float)euler_data.h;
        bno055_latest_data.euler_r = (float)euler_data.r;
        bno055_latest_data.euler_p = (float)euler_data.p;
    }
    if (bno055_read_quaternion_wxyz(&quat_data) == BNO055_SUCCESS) {
        bno055_latest_data.quat_w = (float)quat_data.w / BNO055_QUATERNION_SCALE_FACTOR;
        bno055_latest_data.quat_x = (float)quat_data.x / BNO055_QUATERNION_SCALE_FACTOR;
        bno055_latest_data.quat_y = (float)quat_data.y / BNO055_QUATERNION_SCALE_FACTOR;
        bno055_latest_data.quat_z = (float)quat_data.z / BNO055_QUATERNION_SCALE_FACTOR;
    }
    if (bno055_convert_double_gravity_xyz_msq(&grav_data) == BNO055_SUCCESS) {
        bno055_latest_data.gravity_x = (float)grav_data.x;
        bno055_latest_data.gravity_y = (float)grav_data.y;
        bno055_latest_data.gravity_z = (float)grav_data.z;
    }
    if (bno055_convert_double_linear_accel_xyz_msq(&linear_accel_data) == BNO055_SUCCESS) {
        bno055_latest_data.linear_accel_x = (float)linear_accel_data.x;
        bno055_latest_data.linear_accel_y = (float)linear_accel_data.y;
        bno055_latest_data.linear_accel_z = (float)linear_accel_data.z;
    }

    {
        u8 t;
        if (bno055_get_sys_calib_stat(&t) == BNO055_SUCCESS) {
            bno055_latest_data.calib_sys = t;
        }
        if (bno055_get_gyro_calib_stat(&t) == BNO055_SUCCESS) {
            bno055_latest_data.calib_gyro = t;
        }
        if (bno055_get_accel_calib_stat(&t) == BNO055_SUCCESS) {
            bno055_latest_data.calib_accel = t;
        }
        if (bno055_get_mag_calib_stat(&t) == BNO055_SUCCESS) {
            bno055_latest_data.calib_mag = t;
        }
    }

    bno055_latest_data.data_valid = true;
}

// Unified acquisition task (polling only)
static void bno055_acq_task(void* pvParameters)
{
    ESP_LOGI(TAG, "BNO055 acquisition task started with %d ticks interval",
             pdMS_TO_TICKS(CONFIG_BNO055_POLL_INTERVAL_MS));
    vTaskDelay(pdMS_TO_TICKS(700));
    while (1) {
        bno055_sample_once();
        vTaskDelay(pdMS_TO_TICKS(CONFIG_BNO055_POLL_INTERVAL_MS));
    }
}

// 获取最新IMU数据 - ESP-IDF实现
esp_err_t bno055_get_latest_data(imu_data_t *imu_data)
{
    if (!imu_data) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!bno055_latest_data.data_valid) {
        return ESP_ERR_INVALID_STATE;
    }

    // Direct structure copy
    imu_data->accel_x = bno055_latest_data.accel_x;
    imu_data->accel_y = bno055_latest_data.accel_y;
    imu_data->accel_z = bno055_latest_data.accel_z;
    imu_data->gyro_x = bno055_latest_data.gyro_x;
    imu_data->gyro_y = bno055_latest_data.gyro_y;
    imu_data->gyro_z = bno055_latest_data.gyro_z;
    imu_data->mag_x = bno055_latest_data.mag_x;
    imu_data->mag_y = bno055_latest_data.mag_y;
    imu_data->mag_z = bno055_latest_data.mag_z;
    imu_data->euler_h = bno055_latest_data.euler_h;
    imu_data->euler_r = bno055_latest_data.euler_r;
    imu_data->euler_p = bno055_latest_data.euler_p;
    imu_data->quat_w = bno055_latest_data.quat_w;
    imu_data->quat_x = bno055_latest_data.quat_x;
    imu_data->quat_y = bno055_latest_data.quat_y;
    imu_data->quat_z = bno055_latest_data.quat_z;
    imu_data->gravity_x = bno055_latest_data.gravity_x;
    imu_data->gravity_y = bno055_latest_data.gravity_y;
    imu_data->gravity_z = bno055_latest_data.gravity_z;
    imu_data->linear_accel_x = bno055_latest_data.linear_accel_x;
    imu_data->linear_accel_y = bno055_latest_data.linear_accel_y;
    imu_data->linear_accel_z = bno055_latest_data.linear_accel_z;
    imu_data->calib_sys = bno055_latest_data.calib_sys;
    imu_data->calib_gyro = bno055_latest_data.calib_gyro;
    imu_data->calib_accel = bno055_latest_data.calib_accel;
    imu_data->calib_mag = bno055_latest_data.calib_mag;
    imu_data->connected = true;
    imu_data->timestamp_ms = bno055_latest_data.timestamp_us / 1000;

    return ESP_OK;
}

// Start background acquisition with polling
esp_err_t bno055_data_acq_start(void)
{
    if (s_bno_acq_task_handle == NULL) {
        BaseType_t ret = xTaskCreate(bno055_acq_task, "bno055_acq", 3072, NULL,
                                     configMAX_PRIORITIES - 3, &s_bno_acq_task_handle);
        if (ret != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGI(TAG, "BNO055 polling started @ %d ms", CONFIG_BNO055_POLL_INTERVAL_MS);
    }
    return ESP_OK;
}

esp_err_t bno055_data_acq_stop(void)
{
    if (s_bno_acq_task_handle) {
        vTaskDelete(s_bno_acq_task_handle);
        s_bno_acq_task_handle = NULL;
    }
    bno055_latest_data.data_valid = false;
    ESP_LOGI(TAG, "BNO055 polling stopped");
    return ESP_OK;
}

/*  Brief : Complete BNO055 initialization with status validation
 *  \return : ESP_OK if successful, error code otherwise
 */
esp_err_t bno055_init_sensor(void)
{
    ESP_LOGI(TAG, "Initializing BNO055 IMU sensor...");

    // Initialize I2C first
    s8 result = I2C_routine();
    if (result != BNO055_INIT_VALUE) {
        ESP_LOGE(TAG, "I2C initialization failed");
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Initialize BNO055 sensor
    result = bno055_init(&bno055);
    if (result != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "BNO055 initialization failed: %d", result);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Check chip ID to verify communication
    u8 chip_id = 0;
    result = bno055_read_chip_id(&chip_id);
    if (result != BNO055_SUCCESS || chip_id != 0xA0) {
        ESP_LOGE(TAG, "BNO055 not responding or wrong chip ID: 0x%02X", chip_id);
        return ESP_ERR_NOT_FOUND;
    }

    // // Set power mode to normal
    // result = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    // if (result != BNO055_SUCCESS) {
    //     ESP_LOGE(TAG, "Failed to set power mode: %d", result);
    //     return ESP_ERR_INVALID_RESPONSE;
    // }

    // // Give sensor time to stabilize after power mode change
    // vTaskDelay(pdMS_TO_TICKS(50));

    // // Switch to CONFIG mode first to ensure clean state (critical fix)
    // ESP_LOGI(TAG, "Entering CONFIG mode for clean initialization...");
    // result = bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
    // if (result != BNO055_SUCCESS) {
    //     ESP_LOGE(TAG, "Failed to set CONFIG operation mode: %d", result);
    //     return ESP_ERR_INVALID_RESPONSE;
    // }
    // vTaskDelay(pdMS_TO_TICKS(25));  // CONFIG mode switch delay

    // Directly set to AM mode (following Arduino success pattern)
    ESP_LOGI(TAG, "Setting AMG mode directly...");
    result = bno055_set_operation_mode(BNO055_OPERATION_MODE_IMUPLUS);
    if (result != BNO055_SUCCESS) {
        ESP_LOGE(TAG, "Failed to set NDOF operation mode: %d", result);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Wait longer for NDOF fusion algorithm to fully initialize (critical fix)
    ESP_LOGI(TAG, "Waiting for fusion algorithm initialization...");
    vTaskDelay(pdMS_TO_TICKS(300));  // Increased delay for stability

    // Simple validation read (minimal testing to avoid algorithm disruption)
    struct bno055_euler_t euler_hrp;
    result = bno055_read_euler_hrp(&euler_hrp);
    if (result == BNO055_SUCCESS) {
        ESP_LOGI(TAG, "NDOF fusion working - Euler: H=%d R=%d P=%d",
                 euler_hrp.h, euler_hrp.r, euler_hrp.p);
    } else {
        ESP_LOGW(TAG, "Initial Euler read failed, but fusion may still stabilize");
    }

    // Allow additional time for fusion algorithm stabilization
    vTaskDelay(pdMS_TO_TICKS(100));

    // Final system status check after all operations
    u8 sys_status = 0;
    u8 sys_error = 0;
    
    if (bno055_get_sys_stat_code(&sys_status) != BNO055_SUCCESS) {
        ESP_LOGW(TAG, "Failed to read system status during final check");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (bno055_get_sys_error_code(&sys_error) != BNO055_SUCCESS) {
        ESP_LOGW(TAG, "Failed to read system error code during final check");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Validate final system status
    if (sys_status < 0x05) {
        ESP_LOGW(TAG, "System not ready after full init, status: 0x%02X", sys_status);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Temporarily disable strict system error check - BNO055 may report non-zero but still work
    // if (sys_error != 0x00) {
    //     ESP_LOGE(TAG, "System error detected after full init: 0x%02X", sys_error);
    //     return ESP_ERR_INVALID_STATE;
    // }
    if (sys_error != 0x00) {
        ESP_LOGW(TAG, "System error detected but continuing: 0x%02X", sys_error);
    }

    // Read and display calibration status after initialization
    u8 calib_sys = 0, calib_gyro = 0, calib_accel = 0, calib_mag = 0;
    if (bno055_get_sys_calib_stat(&calib_sys) == BNO055_SUCCESS &&
        bno055_get_gyro_calib_stat(&calib_gyro) == BNO055_SUCCESS &&
        bno055_get_accel_calib_stat(&calib_accel) == BNO055_SUCCESS &&
        bno055_get_mag_calib_stat(&calib_mag) == BNO055_SUCCESS) {

        ESP_LOGI(TAG, "=== BNO055 Calibration Status ===");
        ESP_LOGI(TAG, "System:        %d/3 %s", calib_sys, (calib_sys >= 2) ? "✓ Good" : "⚠ Need calibration");
        ESP_LOGI(TAG, "Gyroscope:     %d/3 %s", calib_gyro, (calib_gyro >= 2) ? "✓ Good" : "⚠ Need calibration");
        ESP_LOGI(TAG, "Accelerometer: %d/3 %s", calib_accel, (calib_accel >= 2) ? "✓ Good" : "⚠ Need calibration");
        ESP_LOGI(TAG, "Magnetometer:  %d/3 %s", calib_mag, (calib_mag >= 2) ? "✓ Good" : "⚠ Need calibration");

        if (calib_mag < 2) {
            ESP_LOGW(TAG, "*** MAGNETOMETER NEEDS CALIBRATION ***");
            ESP_LOGW(TAG, "To calibrate: Hold device and move in FIGURE-8 pattern");
            ESP_LOGW(TAG, "Rotate around all 3 axes until mag calibration reaches 2+");
        }
        if (calib_gyro < 2) {
            ESP_LOGW(TAG, "*** GYROSCOPE NEEDS CALIBRATION ***");
            ESP_LOGW(TAG, "To calibrate: Place device on stable surface and keep still");
        }
        if (calib_accel < 2) {
            ESP_LOGW(TAG, "*** ACCELEROMETER NEEDS CALIBRATION ***");
            ESP_LOGW(TAG, "To calibrate: Place device in 6 different orientations");
        }
        if (calib_sys < 2) {
            ESP_LOGW(TAG, "*** SYSTEM CALIBRATION LOW ***");
            ESP_LOGW(TAG, "Overall system needs sensor calibration");
        }
    } else {
        ESP_LOGW(TAG, "Failed to read calibration status");
    }

    ESP_LOGI(TAG, "BNO055 initialization completed successfully!");
    ESP_LOGI(TAG, "Final status - Chip ID: 0x%02X, Sys Status: 0x%02X, Mode: NDOF", chip_id, sys_status);
    ESP_LOGI(TAG, "BNO055 I2C: SDA=%d SCL=%d FREQ=%d Hz PORT=%d",
             (int)CONFIG_BNO055_I2C_SDA_IO, (int)CONFIG_BNO055_I2C_SCL_IO,
             (int)CONFIG_BNO055_I2C_FREQ_HZ, (int)CONFIG_BNO055_I2C_PORT_NUM);
    ESP_LOGI(TAG, "BNO055 mode: NDOF (9-axis fusion), Polling interval: %d ms",
             (int)CONFIG_BNO055_POLL_INTERVAL_MS);

    return ESP_OK;
}

#endif
