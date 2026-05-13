#pragma once
#include "../roomba_sensor.h"

namespace esphome
{
    namespace roomba_component
    {

        class RoombaCapacitySensor : public RoombaSensor, public sensor::Sensor
        {
        public:
            uint8_t get_packet_id() override { return SENSOR_BATTERY_CAPACITY; }
            void process_packet(const std::vector<uint8_t> &data) override
            {
                if (data.size() == 2)
                {
                    uint16_t capacity = this->combine_bytes(data[0], data[1]);

                    // Validate capacity reading - Roomba batteries are typically 2000-4000 mAh
                    if (capacity >= 500 && capacity <= 10000)
                    {
                        float filtered_capacity = this->validate_and_filter(capacity, 10000.0f, 500.0f);
                        if (!std::isnan(filtered_capacity))
                        {
                            this->publish_state(filtered_capacity);
                        }
                    }
                }
            }

            void dump_config() override
            {
                ESP_LOGCONFIG("roomba.sensor", "Roomba Battery Capacity Sensor:");
                ESP_LOGCONFIG("roomba.sensor", "  Packet ID: %u, Expected size: %u", this->get_packet_id(), this->get_expected_size());
            }
        };

    }
}