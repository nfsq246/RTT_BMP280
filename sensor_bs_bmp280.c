/*!
 *  @brief Example shows basic application of configuring and reading pressure.
 */
#include "sensor_bs_bmp280.h"

#include "stdio.h"
#include "string.h"
#define DBG_ENABLE
#define DBG_LEVEL DBG_INFO
#define DBG_SECTION_NAME  "sensor.bs.bmp280"
#define DBG_COLOR
#include <rtdbg.h>


#define SENSOR_PRES_RANGE_MAX 110000
#define SENSOR_PRES_RANGE_MIN 30000
#define SENSOR_TEMP_RANGE_MAX 85
#define SENSOR_TEMP_RANGE_MIN -40

static struct rt_i2c_bus_device *i2c_bus_dev;
static void delay_ms(uint32_t period_ms);
static int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
static int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
static int _rt_bmp280_init(struct rt_sensor_intf *intf);
static void print_rslt(const char api_name[], int8_t rslt);

struct bmp280_dev bmp;
static rt_size_t _bmp280_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{

    if (sensor->info.type == RT_SENSOR_CLASS_BARO)
    {

        struct bmp280_uncomp_data ucomp_data;
        uint32_t pres32;

        /* Reading the raw data from sensor */
        if(bmp280_get_uncomp_data(&ucomp_data, &bmp)!=BMP280_OK)
        {
            LOG_E("Reading the raw data from sensor error");
            return 0;
        }

        /* Getting the compensated pressure using 32 bit precision */
        bmp280_get_comp_pres_32bit(&pres32, ucomp_data.uncomp_press, &bmp);


        data->type = RT_SENSOR_CLASS_BARO;
        data->data.baro = pres32;
        data->timestamp = rt_sensor_get_ts();

    }

    else if (sensor->info.type == RT_SENSOR_CLASS_TEMP)
    {

        struct bmp280_uncomp_data ucomp_data;
        int32_t temp32;

        /* Reading the raw data from sensor */
        if(bmp280_get_uncomp_data(&ucomp_data, &bmp)!=BMP280_OK)
        {
            LOG_E("Reading the raw data from sensor error");
            return 0;
        }
        /* Getting the compensated pressure using 32 bit precision */
        bmp280_get_comp_temp_32bit(&temp32, ucomp_data.uncomp_temp, &bmp);


        data->type = RT_SENSOR_CLASS_TEMP;
        data->data.temp = temp32/10;
        data->timestamp = rt_sensor_get_ts();
    }
    else
    {
        LOG_E("only RT_SENSOR_CLASS_BARO, RT_SENSOR_CLASS_TEMP could get");
        return 0;			
    }
    return 1;
}
static rt_size_t _bmp280_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _bmp280_polling_get_data(sensor, buf);
    }
    else if (sensor->config.mode == RT_SENSOR_MODE_INT)
    {
        LOG_E("only RT_SENSOR_MODE_POLLING could set");
        return 0;
    }
    else if (sensor->config.mode == RT_SENSOR_MODE_FIFO)
    {
        LOG_E("only RT_SENSOR_MODE_POLLING could get");
        return 0;
    }
    else
    {
        LOG_E("only RT_SENSOR_MODE_POLLING could get");
        return 0;
    }
        
}

static rt_err_t _bmp280_set_odr(rt_sensor_t sensor, rt_int32_t args)
{

    int8_t rslt;
    struct bmp280_config conf;
    if(args==1||args==2||args==4||args==8||args==16||args==2048||args==BMP280_ODR_2000_MS||args==BMP280_ODR_4000_MS)
    {
        rslt = bmp280_get_config(&conf, &bmp);
        if(rslt!=BMP280_OK)
        {
            print_rslt(" bmp280_get_config status", rslt);			
            return -RT_ERROR;
        }
        switch(args)
        {
            case 1 :    conf.odr = BMP280_ODR_1000_MS;  break;
            case 2 :    conf.odr = BMP280_ODR_500_MS;  break;
            case 4 :    conf.odr = BMP280_ODR_250_MS;  break;
            case 8 :    conf.odr = BMP280_ODR_125_MS;  break;
            case 16 :   conf.odr = BMP280_ODR_62_5_MS;  break;
            case 2048 : conf.odr = BMP280_ODR_0_5_MS;  break;
            case BMP280_ODR_2000_MS :   conf.odr = BMP280_ODR_2000_MS;  break;
            case BMP280_ODR_4000_MS :   conf.odr = BMP280_ODR_4000_MS;  break;

            default: return -RT_ERROR;
        }

        rslt = bmp280_set_config(&conf, &bmp);

        if(rslt!=BMP280_OK)
        {
            print_rslt(" bmp280_set_config status", rslt);			
            return -RT_ERROR;
        }
        return RT_EOK;
    }
    else
    {
        LOG_E("only 1,2,4,8,16,2048,BMP280_ODR_2000_MS,BMP280_ODR_4000_MS could set");
        return -RT_ERROR;
    }
    

}

static rt_err_t _bmp280_set_POWER(rt_sensor_t sensor, rt_int32_t args)
{
    int8_t rslt;
    if(args==RT_SENSOR_POWER_DOWN||args==RT_SENSOR_POWER_NORMAL)
    {
        /* Always set the power mode after setting the configuration */
        if(args==RT_SENSOR_POWER_DOWN)
        {
            rslt = bmp280_set_power_mode(BMP280_SLEEP_MODE, &bmp);
        }
        else if(args==RT_SENSOR_POWER_NORMAL)
        {
            rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
        }
        else
        {
            LOG_E("only RT_SENSOR_POWER_DOWN,RT_SENSOR_POWER_NORMAL could set");
            return -RT_ERROR;
        }
        if(rslt!=BMP280_OK)
        {
            print_rslt(" bmp280_set_power_mode status", rslt);			
            return -RT_ERROR;
        }
        else
        {
            return RT_EOK;
        }
    }
    else
    {
        LOG_E("only RT_SENSOR_POWER_DOWN,RT_SENSOR_POWER_NORMAL could set");
        return -RT_ERROR;
    }
}
static rt_err_t _bmp280_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        result = -RT_ERROR;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = -RT_ERROR;
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = _bmp280_set_odr(sensor,(rt_uint32_t)args & 0xffff);
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = -RT_ERROR;
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _bmp280_set_POWER(sensor,(rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        result = -RT_ERROR;
        break;
    default:
        LOG_E("only RT_SENSOR_CTRL_SET_POWER,RT_SENSOR_CTRL_SET_ODR could set");
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    _bmp280_fetch_data,
    _bmp280_control
};
int rt_hw_bmp280_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_pres = RT_NULL, sensor_temp = RT_NULL;


    result = _rt_bmp280_init(&cfg->intf);
    if (result != RT_EOK)
    {
        LOG_E("_rt_bmp280_init err code: %d", result);
        return -RT_ERROR;
    }
    else
    {

        sensor_pres = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_pres == RT_NULL)
        {
            LOG_E("rt_calloc error");
            return -RT_ERROR;
        }
            
        sensor_pres->info.type       = RT_SENSOR_CLASS_BARO;
        sensor_pres->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
        sensor_pres->info.model      = "bmp280_pres";
        sensor_pres->info.unit       = RT_SENSOR_UNIT_PA;
        sensor_pres->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_pres->info.range_max  = SENSOR_PRES_RANGE_MAX;
        sensor_pres->info.range_min  = SENSOR_PRES_RANGE_MIN;
        sensor_pres->info.period_min = 5;

        rt_memcpy(&sensor_pres->config, cfg, sizeof(struct rt_sensor_config));
        sensor_pres->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_pres, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
                LOG_E("device register err code: %d", result);
                goto __exit;
        }

        sensor_temp = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_temp == RT_NULL)
        {
            LOG_E("rt_calloc error");
            goto __exit;
        }

        sensor_temp->info.type       = RT_SENSOR_CLASS_TEMP;
        sensor_temp->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
        sensor_temp->info.model      = "bmp280_temp";
        sensor_temp->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
        sensor_temp->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_temp->info.range_max  = SENSOR_TEMP_RANGE_MAX;
        sensor_temp->info.range_min  = SENSOR_TEMP_RANGE_MIN;
        sensor_temp->info.period_min = 5;

        rt_memcpy(&sensor_temp->config, cfg, sizeof(struct rt_sensor_config));
        sensor_temp->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_temp, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
                LOG_E("device register err code: %d", result);
                goto __exit;
        }

    }
    

    LOG_I("bmp280_sensor init success");
    return RT_EOK;

__exit:
    if (sensor_pres)
        rt_free(sensor_pres);
    if (sensor_temp)
        rt_free(sensor_temp);

    return -RT_ERROR;
}
static int _rt_bmp280_init(struct rt_sensor_intf *intf)
{
    int8_t rslt;
		rt_uint8_t  i2c_addr = (rt_uint32_t)(intf->user_data) & 0xff;
    struct bmp280_config conf;
		

    i2c_bus_dev = rt_i2c_bus_device_find(intf->dev_name); 
    if (i2c_bus_dev == RT_NULL)
    {
        LOG_E("can't find bmp280 %s device\r\n",intf->dev_name);
        return -RT_ERROR;
    }		
    /* Map the delay function pointer with the function responsible for implementing the delay */
    bmp.delay_ms = delay_ms;

    /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
    bmp.dev_id = i2c_addr;

    /* Select the interface mode as I2C */
    bmp.intf = BMP280_I2C_INTF;

    /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
    bmp.read = i2c_reg_read;
    bmp.write = i2c_reg_write;

    rslt = bmp280_init(&bmp);

    if(rslt!=BMP280_OK)
    {
        print_rslt(" bmp280_init status", rslt);	
            return -RT_ERROR;
    }
    /* Always read the current settings before writing, especially when
     * all the configuration is not modified
     */
    rslt = bmp280_get_config(&conf, &bmp);
    if(rslt!=BMP280_OK)
    {
        print_rslt(" bmp280_get_config status", rslt);			
        return -RT_ERROR;
    }

    /* configuring the temperature oversampling, filter coefficient and output data rate */
    /* Overwrite the desired settings */

    conf.filter = BMP280_FILTER_COEFF_2;
    /* Temperature oversampling set at 4x */

    conf.os_temp = BMP280_OS_4X;

    /* Pressure oversampling set at 4x */
    conf.os_pres = BMP280_OS_4X;

    /* Setting the output data rate as 1HZ(1000ms) */
    conf.odr = BMP280_ODR_1000_MS;
    rslt = bmp280_set_config(&conf, &bmp);

    if(rslt!=BMP280_OK)
    {
        print_rslt(" bmp280_set_config status", rslt);			
            return -RT_ERROR;
    }
/* Always set the power mode after setting the configuration */
    rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);

    if(rslt!=BMP280_OK)
    {
            print_rslt(" bmp280_set_power_mode status", rslt);			
            return -RT_ERROR;
    }
    return RT_EOK;


}

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs such as "bmg250_soft_reset",
 *      "bmg250_set_foc", "bmg250_perform_self_test"  and so on.
 *
 *  @param[in] period_ms  : the required wait time in milliseconds.
 *  @return void.
 *
 */
static void delay_ms(uint32_t period_ms)
{
    /* Implement the delay routine according to the target machine */
		rt_thread_mdelay(period_ms);
}

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
static int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Implement the I2C write routine according to the target machine. */
    rt_uint8_t tmp = reg_addr;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = i2c_addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = i2c_addr;             /* Slave address */
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;        /* Read flag */
    msgs[1].buf   = reg_data;             /* Read data pointer */
    msgs[1].len   = length;              /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
static int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Implement the I2C read routine according to the target machine. */
    rt_uint8_t tmp = reg_addr;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = i2c_addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = i2c_addr;             /* Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Read flag */
    msgs[1].buf   = reg_data;             /* Read data pointer */
    msgs[1].len   = length;              /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
static void print_rslt(const char api_name[], int8_t rslt)
{
    if (rslt != BMP280_OK)
    {
        rt_kprintf("%s\t", api_name);
        if (rslt == BMP280_E_NULL_PTR)
        {
            LOG_E("Error [%d] : Null pointer error\r\n", rslt);
        }
        else if (rslt == BMP280_E_COMM_FAIL)
        {
            LOG_E("Error [%d] : Bus communication failed\r\n", rslt);
        }
        else if (rslt == BMP280_E_IMPLAUS_TEMP)
        {
            LOG_E("Error [%d] : Invalid Temperature\r\n", rslt);
        }
        else if (rslt == BMP280_E_DEV_NOT_FOUND)
        {
            LOG_E("Error [%d] : Device not found\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            LOG_E("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}
