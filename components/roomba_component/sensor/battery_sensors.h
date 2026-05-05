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

                    if (capacity > 0)
                    {
                        float pct = ((float)charge / (float)capacity) * 100.0f;
                        if (pct > 100.0f)
                            pct = 100.0f;
                        this->publish_state(pct);
                    }
                }
            }
        };

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
        };

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
        };

    }
}