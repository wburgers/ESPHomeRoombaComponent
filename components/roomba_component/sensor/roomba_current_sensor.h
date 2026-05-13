#pragma once
#include "../roomba_sensor.h"

namespace esphome
{
    namespace roomba_component
    {

        class RoombaCurrentSensor : public RoombaSensor, public sensor::Sensor
        {
        public:
            uint8_t get_packet_id() override { return SENSOR_CURRENT; }
            void process_packet(const std::vector<uint8_t> &data) override
            {
                if (data.size() == 2)
                {
                    int16_t current_ma = static_cast<int16_t>(this->combine_bytes(data[0], data[1]));
                    this->publish_state(current_ma);
                }
            }

            void dump_config() override
            {
                ESP_LOGCONFIG("roomba.sensor", "Roomba Current Sensor:");
                ESP_LOGCONFIG("roomba.sensor", "  Packet ID: %u, Expected size: %u", this->get_packet_id(), this->get_expected_size());
            }
        };

    }
}