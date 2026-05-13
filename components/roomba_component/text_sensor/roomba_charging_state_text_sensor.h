#pragma once
#include "esphome/components/text_sensor/text_sensor.h"
#include "../roomba_sensor.h"
#include "../roomba_constants.h"

namespace esphome
{
    namespace roomba_component
    {

        class RoombaChargingStateTextSensor : public RoombaSensor, public text_sensor::TextSensor
        {
        public:
            uint8_t get_packet_id() override { return SENSOR_CHARGING_STATE; }

            void process_packet(const std::vector<uint8_t> &data) override
            {
                if (data.empty())
                    return;

                uint8_t code = data[0];
                if (CHARGE_STATE_STRINGS.count(code))
                {
                    this->publish_state(CHARGE_STATE_STRINGS.at(code));
                }
                else
                {
                    this->publish_state("Unknown (" + std::to_string(code) + ")");
                }
            }

            void dump_config() override
            {
                text_sensor::TextSensor::dump_config();
            }
        };

    }
}