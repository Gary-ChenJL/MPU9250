# MPU9250

This project is created with C for the **9-axis** MPU9250 IMU. The project is created on the STM32Cube IDE platform and a functional example is provided.\
The library is based on the I2C communication protocol. It is not fully tested, but it works well. If you find any bugs, please let me know.
\

# Setup

## Hardware

The provided example is designed to work with the STM32F103C8T6 microcontroller. If you want to use another microcontroller, you need to update the Handlers configuration accordingly.\
For STM32F103C8T6 configuration, the following channels are used:\
### I2C1 - PB6, PB7
![i2c1_setup](https://github.com/Gary-ChenJL/MPU9250/blob/main/images/i2c_setup.PNG)\
### UART1 - PA9, PA10
![uart1_setup](https://github.com/Gary-ChenJL/MPU9250/blob/main/images/uart_setup.PNG)\
The MPU9250 is connected to the I2C1 bus.\
The I2C1 bus is used and connected to the following pins:

- PB6 - SCL
- PB7 - SDA

The UART1 bus is used and connected to the following pins:

- PA9 - RX
- PA10 - TX

## Software

The project is created on the STM32Cube IDE platform. Thus, I am using the provided **HAL** libraries.\
You can use the STM32CubeMX to generate the project for your microcontroller.\
In the **main.c** file, you can find the following code:

### Initialization of MPU9250

---

```c
    for (;;) {
	  if(MPU6500_init(&hi2c1) == 1){
		  HAL_Delay(2000);
		  snprintf(txbuf, sizeof(txbuf),"%s%s",success, "MPU6500\r\n");
		  HAL_UART_Transmit(&huart1,txbuf ,(uint16_t) strlen(txbuf), 10);
		  break;
	  } else {
		  HAL_Delay(2000);
		  snprintf(txbuf, sizeof(txbuf),"%s%s",fail, "MPU6500\r\n");
		  HAL_UART_Transmit(&huart1,txbuf,(uint16_t) strlen(txbuf),10);
		  HAL_Delay(100);
	  }
  }
```

### Initialization of AK8963

---

```c
    for(;;) {
	  uint8_t x = MPU9250_initMagnetometer(&hi2c1, &Magnetometer);
	  if(x == 1){
		  snprintf(txbuf, sizeof(txbuf),"%s%s",success, "Magnetometer\r\n");
		  HAL_UART_Transmit(&huart1,txbuf,(uint16_t)strlen(txbuf),10);
		  break;
	  } else {
		  snprintf(txbuf, sizeof(txbuf),"%s%s",fail, "Magnetometer\r\n");
		  HAL_UART_Transmit(&huart1,txbuf,strlen(txbuf),100);
	  }
  }
```

### Calibration of Accelerometer and Gyroscope

---

```c
    HAL_UART_Transmit(&huart1,hold,sizeof(hold),10);
    HAL_Delay(100);
    MPU6500_calibrate(&hi2c1, &MPU6500);
    HAL_UART_Transmit(&huart1,holdf,sizeof(holdf),10);
```

In the main loop, you can find the following code:

### Reading 9-axis data from MPU9250

---

```c
    MPU6500_readAll(&hi2c1, &MPU6500);
    MPU6500_getValues(&MPU6500);
    MPU9250_Read_AK8963Data(&hi2c1, &Magnetometer);
```

### Printing 9-axis data to UART

```c
    MPU9250_transmitUART(&huart1, txbuf, &MPU6500, &Magnetometer);
	HAL_Delay(1000);
```

## Important Notes

Make sure to enable floating point support for printf in the project settings. This can be done by enabling an option in the project settings. \
![setting](https://github.com/Gary-ChenJL/MPU9250/blob/main/images/proj_setting.PNG)\
Alternatively, you can add "-u \_printf_float" as linker option\
If you experience related issues, please refer to [this link](https://community.st.com/s/question/0D50X0000AldaPzSQI/cubeide-sprintf-does-not-work-with-f) about more detailed info.
If you wish to add Kalman filtering to the project, please refer to [this project](https://github.com/TKJElectronics/KalmanFilter) from TKJElectronics for more info.
# References
The project is inspired by [MPU6050](https://github.com/leech001/MPU6050) project from leech001 and [MPU9250](https://github.com/wollewald/MPU9250_WE) project from wollewald.\
# License
MIT License