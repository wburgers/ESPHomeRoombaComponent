#pragma once
#include "../roomba_sensor.h"

namespace esphome
{
    namespace roomba_component
    {

        class RoombaChargeSensor : public RoombaSensor, public sensor::Sensor
        {
        public:
            uint8_t get_packet_id() override { return SENSOR_BATTERY_CHARGE; }
            void process_packet(const std::vector<uint8_t> &data) override
            {
                if (data.size() == 2)
                {
                    this->publish_state(this->combine_bytes(data[0], data[1]));
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