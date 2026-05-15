#pragma once
#include "../roomba_sensor.h"

namespace esphome
{
    namespace roomba_component
    {

        class RoombaBatteryChargeSensor : public RoombaSensor, public sensor::Sensor
        {
        public:
            uint8_t get_packet_id() override { return SENSOR_BATTERY_CHARGE; }
            void process_packet(const std::vector<uint8_t> &data) override
            {
                if (data.size() == 2)
                {
                    uint16_t charge = this->combine_bytes(data[0], data[1]);

                    // Validate charge reading - Roomba batteries are typically 2000-4000 mAh
                    if (charge >= 500 && charge <= 10000)
                    {
                        float filtered_charge = this->validate_and_filter(charge, 10000.0f, 500.0f);
                        if (!std::isnan(filtered_charge))
                        {
                            this->publish_state(filtered_charge);
                        }
                    }
                }
            }

            void dump_config() override
            {
                ESP_LOGCONFIG("roomba.sensor", "Roomba Battery Charge Sensor:");
                ESP_LOGCONFIG("roomba.sensor", "  Packet ID: %u, Expected size: %u", this->get_packet_id(), this->get_expected_size());
            }
        };

    }
}