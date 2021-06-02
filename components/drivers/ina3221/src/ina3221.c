/*
 *
 */

#include <ina3221.h>

#define INA3221_ACK_CHECK_EN 0x1
#define INA3221_ACK_VAL      0x0
#define INA3221_NACK_VAL     0x1

/************************************************************************
 ************** P R I V A T E  I M P L E M E N T A T I O N **************
 ************************************************************************/
esp_err_t ina3221_init(ina3221_handle_t ina3221_handle) {
	return ESP_OK;
}
esp_err_t ina3221_write8(ina3221_handle_t ina3221_handle, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, INA3221_ADDRESS | I2C_MASTER_WRITE, INA3221_ACK_CHECK_EN);
    i2c_master_write(cmd, &value, 1, INA3221_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(ina3221_handle->i2c_master_port, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}
esp_err_t ina3221_read16(ina3221_handle_t ina3221_handle, uint16_t* value) {
	uint8_t buff[3];
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, INA3221_ADDRESS | I2C_MASTER_READ, INA3221_ACK_CHECK_EN);
    i2c_master_read(cmd, (uint8_t*)buff, 2, INA3221_ACK_VAL);
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(ina3221_handle->i2c_master_port, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    *value= (buff[0] << 8) | buff[1];
	return ESP_OK;
}

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t ina3221_read_register(ina3221_handle_t ina3221_handle, uint8_t address, uint16_t* value) {
	ESP_ERROR_CHECK(ina3221_write8(ina3221_handle, address));
	ESP_ERROR_CHECK(ina3221_read16(ina3221_handle, value));
	return ESP_OK;
}

esp_err_t ina3221_test_connection(ina3221_handle_t ina3221_handle) {
    uint16_t data_rd;
    ESP_ERROR_CHECK(ina3221_read_register(ina3221_handle, INA3221_REG_MANUFACTURER_ID, &data_rd));

    if(data_rd == INA3221_REG_MANUFACTURER_ID_VALUE) {
    	return ESP_OK;
    }
	return ESP_FAIL;
}
