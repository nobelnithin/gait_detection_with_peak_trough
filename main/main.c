#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_timer.h"



#define PIN_SDA 1
#define PIN_CLK 2
#define I2C_ADDRESS 0x68 // I2C address of MPU6050

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_PWR_MGMT_1   0x6B

#define WINDOW_SIZE 10

#define THRESH_LOW -1.5
#define THRESH_HIGH 0

/*
 * The following registers contain the primary data we are interested in
 * 0x3B MPU6050_ACCEL_XOUT_H
 * 0x3C MPU6050_ACCEL_XOUT_L
 * 0x3D MPU6050_ACCEL_YOUT_H
 * 0x3E MPU6050_ACCEL_YOUT_L
 * 0x3F MPU6050_ACCEL_ZOUT_H
 * 0x50 MPU6050_ACCEL_ZOUT_L
 * 0x41 MPU6050_TEMP_OUT_H
 * 0x42 MPU6050_TEMP_OUT_L
 * 0x43 MPU6050_GYRO_XOUT_H
 * 0x44 MPU6050_GYRO_XOUT_L
 * 0x45 MPU6050_GYRO_YOUT_H
 * 0x46 MPU6050_GYRO_YOUT_L
 * 0x47 MPU6050_GYRO_ZOUT_H
 * 0x48 MPU6050_GYRO_ZOUT_L
 */

static char tag[] = "mpu6050";

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);



float moving_average_filter(float *window, int window_size) {
    float sum = 0;
    for (int i = 0; i < window_size; i++) {
        sum += window[i];
    }
    return sum / window_size;
}

bool peak_detected(float curr_val, float prev_val)
{
	if(curr_val < prev_val)
		return true;
	return false;
}
bool trough_detected(float curr_val, float prev_val)
{
	if(curr_val > prev_val)
		return true;
	return false;
}

void MPU6050_initialize(void *params)
{
	ESP_LOGD(tag, ">> mpu6050");
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = PIN_SDA;
	conf.scl_io_num = PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	conf.clk_flags = 0;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

	i2c_cmd_handle_t cmd;
	vTaskDelay(200/portTICK_PERIOD_MS);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_GYRO_XOUT_H, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, 1);
	i2c_master_write_byte(cmd, 0, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	printf("MPU6050 - Gyroscope initialised successfuly");
	while(1)
	{
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}
void detect_gait(void *params)
{
	uint8_t data[6];
	float GyroX, GyroY, GyroZ;
	short gyro_x, gyro_y, gyro_z;
	float gyroAngleX, gyroAngleY;
	gyroAngleX = 0;
	gyroAngleY = 0;
	unsigned long oldmillis;
	oldmillis = esp_timer_get_time();
	float dt;
	float window[WINDOW_SIZE] = {0};
    int window_index = 0;
	i2c_cmd_handle_t cmd;
	float prev_value = 0.0;
	float curr_value = 0.0;
	bool p_detect_flag = true;
	bool t_detect_flag = true;
	while(1) {
		
		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_GYRO_XOUT_H, 1));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);
		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1));

		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data,   0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+1, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+2, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+3, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+4, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+5, 1));

		//i2c_master_read(cmd, data, sizeof(data), 1);
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);

		gyro_x = (data[0] << 8) | data[1];
		gyro_y = (data[2] << 8) | data[3];
		gyro_z = (data[4] << 8) | data[5];
		GyroX = (float)gyro_x/131.0;
		GyroY =  (float)gyro_y/131.0;
		GyroZ =  (float)gyro_z/131.0;
		

		dt = (esp_timer_get_time() - oldmillis)/1000; 
		oldmillis = esp_timer_get_time();
		
		gyroAngleX = GyroX * (dt/1000);
  		gyroAngleY = GyroY * (dt/1000);
		window[window_index] = gyroAngleY;
		float filtered_gyroAngleY = moving_average_filter(window, WINDOW_SIZE);
		window_index = (window_index + 1) % WINDOW_SIZE;
		prev_value = curr_value;
		curr_value = filtered_gyroAngleY;
		//printf("%f\n",filtered_gyroAngleY);

		if(curr_value > 3)
		{
			if(trough_detected(curr_value, prev_value) && t_detect_flag)
			{

					printf("->->->->->->->->Fire ON<-<-<-<-<-<-<-<-\n");
					t_detect_flag = false;
				    p_detect_flag = true;
		
				
		

			}
			//gpio_set_level(green_led, 0);
		}
		if(curr_value < THRESH_HIGH)
		{
			if(peak_detected(curr_value, prev_value) && p_detect_flag)
			{
				//stim off
				printf("Fire OFF\n");
				
				p_detect_flag = false;
				t_detect_flag = true;
			}
			//gpio_set_level(blue_led, 0);
		}

		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}



void app_main(void) 
{

	xTaskCreate(MPU6050_initialize, "MPU6050_initialize", 2048, NULL, 1, NULL);
	xTaskCreate(detect_gait, "detect_gait",8192, NULL, 1, NULL);
	
}
