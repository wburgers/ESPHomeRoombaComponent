#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/log.h"
#include "roomba_constants.h"
#include "roomba_sensor.h"

#include "sensor/battery_sensors.h"
#include "text_sensor/power_sensors.h"
#include "text_sensor/activity_sensor.h"

#include <vector>
#include <string>

namespace esphome
{
	namespace roomba_component
	{

		class RoombaComponent : public PollingComponent, public uart::UARTDevice
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
			void spot_clean();
			void wake_up_device();
			void set_safe_mode();
			void set_full_mode();

		protected:
			void request_packet(uint8_t packet_id);
			void flush_uart_buffer();
			bool has_read_timeout_occurred(uint32_t start_time);

			std::vector<RoombaSensor *> sensors_;
			static const char *const TAG;
		};

	}
}