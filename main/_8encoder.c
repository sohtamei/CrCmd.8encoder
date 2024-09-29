#include "esp_log.h"
#include "driver/i2c.h"

#include "_8encoder.h"

static const char *TAG = "I2C";

#define _8ENCODER_ADDR		0x41
#define GPIO_SCL			1
#define GPIO_SDA			2

#define I2C_TIMEOUT_MS		1000
#define I2C_PORT			0

int _8encoder_init(void)
{
	const i2c_config_t i2c_conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = GPIO_SDA,
		.scl_io_num = GPIO_SCL,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 10000,
	};
	i2c_param_config(I2C_PORT, &i2c_conf);
	ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, i2c_conf.mode, 0/*I2C_MASTER_RX_BUF_DISABLE*/, 0/*I2C_MASTER_TX_BUF_DISABLE*/, 0));
	i2c_set_timeout(I2C_PORT, 31);	// necessary for _8ENCODER_REG_BUTTON
	return 0;
}

int _8encoder_delete(void)
{
	ESP_ERROR_CHECK(i2c_driver_delete(I2C_PORT));
	return 0;
}

int _8encoder_read(int reg, int32_t* data, int num)
{
	int regSize = 1;
	if((reg&0xE0) == _8ENCODER_REG_COUNTER || (reg&0xE0) == _8ENCODER_REG_INCREMENT)
		regSize = 4;

	int i;
	for(i = 0; i < num; i++) {
		int ret;
		uint8_t sendBuf;
		uint8_t recvBuf[4];

		sendBuf = reg+i*regSize;
		ret = i2c_master_write_to_device(I2C_PORT, _8ENCODER_ADDR, &sendBuf, 1, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
		if(ret) {
			ESP_LOGE(TAG, "Error(%d,%x)", __LINE__, ret); 
			return ret;
		}

		ret = i2c_master_read_from_device(I2C_PORT, _8ENCODER_ADDR, recvBuf, regSize, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
		if(ret) {
			ESP_LOGE(TAG, "Error(%d,%x)", __LINE__, ret); 
			return ret;
		}

		data[i] = 0;
		for(int j = 0; j < regSize; j++) {
			data[i] |= recvBuf[j]<<(j*8);
		}
	}
	return 0;
}

int _8encoder_write(int reg, int index, uint32_t data)
{
	int regSize = 1;
	if((reg&0xF0) == _8ENCODER_REG_RGB || (reg&0xF0) == _8ENCODER_REG_RGB+0x10)
		regSize = 3;

	int ret;
	uint8_t sendBuf[1+4];
	sendBuf[0] = reg+index*regSize;

	for(int j = 0; j < regSize; j++) {
		sendBuf[1+j] = (data>>(j*8))&0xFF;
	}

	ret = i2c_master_write_to_device(I2C_PORT, _8ENCODER_ADDR, sendBuf, 1+regSize, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
	if(ret) {
		ESP_LOGE(TAG, "Error(%d,%x)", __LINE__, ret); 
		return ret;
	}
	return 0;
}
