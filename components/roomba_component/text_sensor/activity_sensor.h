#pragma once
#include "../roomba_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome
{
    namespace roomba_component
    {

        class RoombaActivitySensor : public RoombaSensor, public text_sensor::TextSensor
        {
        public:
            uint8_t get_packet_id() override { return SENSOR_GRP_21_TO_26; }

            void process_packet(const std::vector<uint8_t> &data) override
            {
                if (data.size() < 5)
                    return;

                uint8_t charging_code = data[0];
                int16_t current_ma = static_cast<int16_t>(this->combine_bytes(data[3], data[4]));

                this->publish_state(this->infer_activity_from_state(charging_code, current_ma));
            }

        protected:
            std::string infer_activity_from_state(uint8_t charging_code, int16_t current_ma)
            {
                if (current_ma > -50)
                {
                    return "Docked";
                }

                if (this->is_in_active_charging_state(charging_code))
                {
                    return "Charging";
                }

                if (current_ma < -300)
                {
                    return "Cleaning";
                }

                return "Lost";
            }

            bool is_in_active_charging_state(uint8_t code)
            {
                return code == static_cast<uint8_t>(ChargeState::CHARGE_STATE_RECONDITIONING) ||
                       code == static_cast<uint8_t>(ChargeState::CHARGE_STATE_FULL) ||
                       code == static_cast<uint8_t>(ChargeState::CHARGE_STATE_TRICKLE);
            }
        };

    }
}