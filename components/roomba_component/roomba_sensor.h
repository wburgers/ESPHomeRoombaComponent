#pragma once
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "roomba_constants.h"
#include <vector>
#include <cmath>

namespace esphome
{
    namespace roomba_component
    {

        class RoombaSensor
        {
        public:
            virtual uint8_t get_packet_id() = 0;

            uint8_t get_expected_size()
            {
                auto it = PACKET_SIZES.find(this->get_packet_id());
                return (it != PACKET_SIZES.end()) ? it->second : 0;
            }

            virtual void process_packet(const std::vector<uint8_t> &data) = 0;

        protected:
            float last_valid_value_ = NAN;
            int missed_reading_count_ = 0;

            uint16_t combine_bytes(uint8_t high, uint8_t low)
            {
                return (static_cast<uint16_t>(high) << 8) | low;
            }

            bool is_reading_plausible(float new_value, float max_limit)
            {
                return std::isnan(last_valid_value_) || new_value <= max_limit;
            }

            bool is_stable_transition(float new_value, float max_diff)
            {
                if (std::isnan(last_valid_value_))
                    return true;
                return std::abs(new_value - last_valid_value_) < max_diff;
            }

            bool should_force_update_after_misses(int limit)
            {
                return missed_reading_count_ >= limit;
            }

            float validate_and_filter(float new_value, float max_limit, float max_diff)
            {
                if (!this->is_reading_plausible(new_value, max_limit))
                {
                    return last_valid_value_;
                }

                if (this->is_stable_transition(new_value, max_diff) || this->should_force_update_after_misses(5))
                {
                    this->missed_reading_count_ = 0;
                    this->last_valid_value_ = new_value;
                    return new_value;
                }

                this->missed_reading_count_++;
                return last_valid_value_;
            }
        };

    }
}