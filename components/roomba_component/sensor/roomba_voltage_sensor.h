#pragma once
#include "../roomba_sensor.h"

namespace esphome
{
    namespace roomba_component
    {

        class RoombaVoltageSensor : public RoombaSensor, public sensor::Sensor
        {
        public:
            uint8_t get_packet_id() override { return SENSOR_VOLTAGE; }
            void process_packet(const std::vector<uint8_t> &data) override
            {
                if (data.size() == 2)
                {
                    this->publish_state(this->combine_bytes(data[0], data[1]) / 1000.0f);
                }
            }
        };

    }
}