# bmp280

## 简介

本软件包是为 BoschSensortec 公司的BMP280，高精度大气压强传感器。并且本软件包新的版本已经对接到了 Sensor 框架，通过 Sensor 框架，开发者可以快速的将此传感器驱动起来。若想查看**BoschSensortec**官方的 驱动信息 请点击[这里](https://github.com/BoschSensortec/BMP280_driver)。

想看中文详细性能信息淘宝BMP280就行。

BMP280支持IIC和SPI通信，但测试板子只有ali-developer-kit，只提供了IIC接口，如果想要支持SPI参考上面的官方驱动改下就行。

## 支持情况

| 包含设备         | 大气压 | 温度 |
| ---------------- | -------- | ------ |
| **通讯接口**     |          |        |
| IIC              | √        | √      |
| SPI              |          |        |
| **工作模式**     |          |        |
| 轮询             | √        | √      |
| 中断             |          |        |
| FIFO             |          |        |
| **电源模式**     |          |        |
| 掉电             | √        | √      |
| 低功耗           |          |        |
| 普通             | √        | √      |
| 高功耗           |          |        |
| **获取id**      |   √      |  √     |
| **数据输出速率** |   √      |  √     |
| **测量范围**     |          |        |
| **自检**         |          |        |
| **多实例**       |          |        |

## 使用说明

### 依赖

- RT-Thread 4.0.0+
- Sensor 组件
- IIC 驱动：bmp280 设备使用 IIC 进行数据通讯，需要系统 IIC 驱动支持；

### 获取软件包

使用 bmp280 软件包需要在 RT-Thread 的包管理中选中它，具体路径如下：

```
RT-Thread online packages  --->
  peripheral libraries and drivers  --->
    sensors drivers  --->
      BME280 sensor driver package, support: barometric, humidity.
              Version (latest)  --->
```

**Version**：软件包版本选择

不定时随缘更新

### 使用软件包

bmp280 软件包初始化函数如下所示：

```
int rt_hw_bmp280_init(const char *name, struct rt_sensor_config *cfg);
```

该函数需要由用户调用，函数主要完成的功能有，

- 设备配置和初始化（根据传入的配置信息，配置接口设备）；
- 注册相应的传感器设备，完成 bmp280 设备的注册；

#### 初始化示例

```
#include "sensor_bs_bmp280.h"

int bmp280_port(void)
{
    struct rt_sensor_config cfg;
    
    cfg.intf.dev_name = BMP280_I2CBUS_NAME;
    cfg.intf.user_data = (void *)BMP280_ADDR_DEFAULT;

    rt_hw_bmp280_init("bmp280", &cfg);
    return 0;
}
INIT_APP_EXPORT(bmp280_port);
```
可以先在msh控制台，输入 list_device ，来查看驱动（temp_bmp和baro_bmp）是否挂载成功，成功之后可以使用 sensor_polling baro_bmp 来查看大气压信息。
## 注意事项

暂无

## 联系人信息

维护人:

- [ChenHN_246](https://github.com/nfsq246) 

- 主页：<https://github.com/nfsq246/RTT_BMP280>

- 邮箱：<ChenHN_246@163.com>

- 第一次写驱动，有bug或者问题的欢迎轰炸我