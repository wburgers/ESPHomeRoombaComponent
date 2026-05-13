#pragma once
#include "../roomba_sensor.h"

namespace esphome
{
    namespace roomba_component
    {

        class RoombaBatteryTempSensor : public RoombaSensor, public sensor::Sensor
        {
        public:
            uint8_t get_packet_id() override { return SENSOR_BATTERY_TEMP; }

            void process_packet(const std::vector<uint8_t> &data) override
            {
                if (data.empty())
                    return;

                float raw_temp = static_cast<float>(static_cast<int8_t>(data[0]));

                float filtered_temp = this->validate_and_filter(raw_temp, 30.0f, 1.0f);

                if (!std::isnan(filtered_temp))
                {
                    this->publish_state(filtered_temp);
                }
            }

            void dump_config() override
            {
                ESP_LOGCONFIG("roomba.sensor", "Roomba Battery Temperature Sensor:");
                ESP_LOGCONFIG("roomba.sensor", "  Packet ID: %u, Expected size: %u", this->get_packet_id(), this->get_expected_size());
            }
        };

    }
}