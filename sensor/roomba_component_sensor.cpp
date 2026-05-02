#include "esphome/core/log.h"
#include "roomba_component_sensor.h"

namespace esphome {
namespace roomba_component {

static const char *TAG = "roomba_component.sensor";

float RoombaComponentSensor::get_setup_priority() const { return setup_priority::DATA; }

void RoombaComponentSensor::update() {
	uint8_t charging;
	uint16_t voltage;
	int16_t current;
	uint16_t batteryCharge;
	uint16_t batteryCapacity;

	this->parent_->flush();

	uint8_t sensors[] = {
		SensorChargingState,
		SensorVoltage,
		SensorCurrent,
		SensorBatteryCharge,
		SensorBatteryCapacity
	};

	uint8_t values[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

	bool success = this->parent_->get_sensors_list(sensors, sizeof(sensors), values, sizeof(values));
	if (!success) {
		ESP_LOGD("custom", "Could not get sensor values from serial");
		return;
	}

	// uint16_t voltage2 = (values[1] << 8) | values[2];

	voltage = values[1] * 256 + values[2];
	current = values[3] * 256 + values[4];

	float voltageInVolts = (1.0 * voltage) / 1000.0;

	if (this->voltage_sensor_ != nullptr && this->voltage_sensor_->state != voltageInVolts) {
		this->voltage_sensor_->publish_state(voltageInVolts);
	}

	float currentInAmps = (1.0 * current) / 1000.0;
	if (this->current_sensor_ != nullptr && this->current_sensor_->state != currentInAmps) {
		this->current_sensor_->publish_state(currentInAmps);
	}
}

void RoombaComponentSensor::dump_config(){
	ESP_LOGCONFIG(TAG, "Roomba Component");
}

}  // namespace roomba_component
}  // namespace esphome
