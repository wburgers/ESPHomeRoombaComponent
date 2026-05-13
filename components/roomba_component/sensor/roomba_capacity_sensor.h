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
                    this->publish_state(this->combine_bytes(data[0], data[1]));
                }
            }

            void dump_config() override
            {
                sensor::Sensor::dump_config();
            }
        };

    }
}