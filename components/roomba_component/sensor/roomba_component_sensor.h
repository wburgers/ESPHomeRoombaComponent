#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

#include "../roomba_component.h"

namespace esphome {
namespace roomba_component {

class RoombaComponentSensor : public Parented<RoombaComponent>, public sensor::Sensor, public PollingComponent {
	public:
		float get_setup_priority() const override;
        void update() override;
		void dump_config() override;
		void set_voltage_sensor(sensor::Sensor *voltage_sensor) { voltage_sensor_ = voltage_sensor; }
		void set_current_sensor(sensor::Sensor *current_sensor) { current_sensor_ = current_sensor; }
	private:
		sensor::Sensor *voltage_sensor_{nullptr};
		sensor::Sensor *current_sensor_{nullptr};
};

}  // namespace roomba_component
}  // namespace esphome
