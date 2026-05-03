#pragma once
#include "../roomba_sensor.h"

namespace esphome
{
    namespace roomba_component
    {

        class RoombaChargingStateSensor : public RoombaSensor, public sensor::Sensor
        {
        public:
            uint8_t get_packet_id() override { return SENSOR_CHARGING_STATE; }
            void process_packet(const std::vector<uint8_t> &data) override
            {
                if (!data.empty())
                {
                    this->publish_state(data[0]);
                }
            }
        };

    }
}