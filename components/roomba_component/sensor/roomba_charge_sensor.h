#pragma once
#include "../roomba_sensor.h"

namespace esphome
{
    namespace roomba_component
    {

        class RoombaChargeSensor : public RoombaSensor, public sensor::Sensor
        {
        public:
            uint8_t get_packet_id() override { return SENSOR_GRP_21_TO_26; }

            void process_packet(const std::vector<uint8_t> &data) override
            {
                if (data.size() >= 10)
                {
                    uint16_t charge = (uint16_t)data[6] << 8 | data[7];
                    uint16_t capacity = (uint16_t)data[8] << 8 | data[9];

                    // Validate capacity reading - Roomba batteries are typically 2000-4000 mAh
                    if (capacity >= 500 && capacity <= 10000)
                    {
                        this->last_valid_capacity_ = capacity;
                    }

                    // Use last valid capacity for percentage calculation
                    if (this->last_valid_capacity_ > 0)
                    {
                        float pct = ((float)charge / (float)this->last_valid_capacity_) * 100.0f;
                        if (pct > 100.0f)
                            pct = 100.0f;
                        this->publish_state(pct);
                    }
                }
            }

            void dump_config() override
            {
                sensor::Sensor::dump_config();
            }

        private:
            uint16_t last_valid_capacity_ = 0;
        };

    }
}