#pragma once

#include "esphome/core/component.h"
#include "esphome/components/api/custom_api_device.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/log.h"
#include "roomba_constants.h"
#include "roomba_sensor.h"

#include "sensor/roomba_charge_sensor.h"
#include "sensor/roomba_capacity_sensor.h"
#include "sensor/roomba_voltage_sensor.h"
#include "sensor/roomba_battery_temp_sensor.h"
#include "sensor/roomba_current_sensor.h"
#include "sensor/roomba_battery_charge_sensor.h"
#include "text_sensor/roomba_activity_sensor.h"
#include "text_sensor/roomba_charging_state_text_sensor.h"

#include <vector>
#include <string>
#include <map>

namespace esphome
{
	namespace roomba_component
	{

		class RoombaComponent : public api::CustomAPIDevice, public PollingComponent, public uart::UARTDevice
		{
		public:
			void register_sensor(RoombaSensor *s) { this->sensors_.push_back(s); }

			void setup() override;
			void update() override;
			void dump_config() override;

			void on_command(std::string command);

			void start_cleaning();
			void seek_dock();
			void play_locate_beep();
			void set_song(uint8_t song_number, const uint8_t *data, uint8_t len);
			void play_song(uint8_t song_number);
			void spot_clean();
			void wake_up_device();
			void set_safe_mode();
			void set_full_mode();

		protected:
			void request_packet(uint8_t packet_id);
			void flush_uart_buffer();
			bool has_read_timeout_occurred(uint32_t start_time);
			void collect_sensor_requirements(std::map<uint8_t, std::vector<RoombaSensor *>> &packet_to_sensors);
			std::map<uint8_t, std::vector<RoombaSensor *>> optimize_packet_requests(const std::map<uint8_t, std::vector<RoombaSensor *>> &requested_packets);

			std::vector<RoombaSensor *> sensors_;
			static const char *const TAG;
		};

	}
}