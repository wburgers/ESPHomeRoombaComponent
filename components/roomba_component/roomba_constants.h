#pragma once
#include <map>
#include <vector>
#include <string>
#include <cstdint>

namespace esphome
{
    namespace roomba_component
    {

        enum RoombaPacket : uint8_t
        {
            SENSOR_GRP_7_TO_26 = 0,
            SENSOR_GRP_7_TO_16 = 1,
            SENSOR_GRP_17_TO_20 = 2,
            SENSOR_GRP_21_TO_26 = 3,
            SENSOR_GRP_27_TO_34 = 4,
            SENSOR_GRP_35_TO_42 = 5,
            SENSOR_GRP_7_TO_42 = 6,
            SENSOR_BUMPS_WHEEL_DROPS = 7,
            SENSOR_WALL = 8,
            SENSOR_CLIFF_LEFT = 9,
            SENSOR_CLIFF_FRONT_LEFT = 10,
            SENSOR_CLIFF_FRONT_RIGHT = 11,
            SENSOR_CLIFF_RIGHT = 12,
            SENSOR_VIRTUAL_WALL = 13,
            SENSOR_OVERCURRENTS = 14,
            SENSOR_IR_BYTE = 17,
            SENSOR_BUTTONS = 18,
            SENSOR_DISTANCE = 19,
            SENSOR_ANGLE = 20,
            SENSOR_CHARGING_STATE = 21,
            SENSOR_VOLTAGE = 22,
            SENSOR_CURRENT = 23,
            SENSOR_BATTERY_TEMP = 24,
            SENSOR_BATTERY_CHARGE = 25,
            SENSOR_BATTERY_CAPACITY = 26,
            SENSOR_WALL_SIGNAL = 27,
            SENSOR_CLIFF_LEFT_SIGNAL = 28,
            SENSOR_CLIFF_FL_SIGNAL = 29,
            SENSOR_CLIFF_FR_SIGNAL = 30,
            SENSOR_CLIFF_R_SIGNAL = 31,
            SENSOR_USER_DIGITAL_IN = 32,
            SENSOR_USER_ANALOG_IN = 33,
            SENSOR_CHARGING_SOURCES = 34,
            SENSOR_OI_MODE = 35,
            SENSOR_SONG_NUMBER = 36,
            SENSOR_SONG_PLAYING = 37,
            SENSOR_NUM_STREAM_PACKETS = 38,
            SENSOR_VELOCITY = 39,
            SENSOR_RADIUS = 40,
            SENSOR_RIGHT_VELOCITY = 41,
            SENSOR_LEFT_VELOCITY = 42,
            SENSOR_LEFT_ENCODER = 43,
            SENSOR_RIGHT_ENCODER = 44,
            SENSOR_LIGHT_BUMPER = 45,
            SENSOR_LIGHT_BUMP_L_SIG = 46,
            SENSOR_LIGHT_BUMP_FL_SIG = 47,
            SENSOR_LIGHT_BUMP_CL_SIG = 48,
            SENSOR_LIGHT_BUMP_CR_SIG = 49,
            SENSOR_LIGHT_BUMP_FR_SIG = 50,
            SENSOR_LIGHT_BUMP_R_SIG = 51,
            SENSOR_LEFT_MOTOR_CURR = 54,
            SENSOR_RIGHT_MOTOR_CURR = 55,
            SENSOR_MAIN_BRUSH_CURR = 56,
            SENSOR_SIDE_BRUSH_CURR = 57,
            SENSOR_STASIS = 58,
            SENSOR_GRP_7_TO_58 = 100,
            SENSOR_GRP_43_TO_58 = 101,
            SENSOR_GRP_46_TO_51 = 106,
            SENSOR_GRP_54_TO_58 = 107
        };

        static const std::map<uint8_t, uint8_t> PACKET_SIZES = {
            {SENSOR_GRP_7_TO_26, 26}, {SENSOR_GRP_7_TO_16, 10}, {SENSOR_GRP_17_TO_20, 6}, {SENSOR_GRP_21_TO_26, 10}, {SENSOR_GRP_27_TO_34, 12}, {SENSOR_GRP_35_TO_42, 12}, {SENSOR_GRP_7_TO_42, 52}, {SENSOR_BUMPS_WHEEL_DROPS, 1}, {SENSOR_WALL, 1}, {SENSOR_CLIFF_LEFT, 1}, {SENSOR_CLIFF_FRONT_LEFT, 1}, {SENSOR_CLIFF_FRONT_RIGHT, 1}, {SENSOR_CLIFF_RIGHT, 1}, {SENSOR_VIRTUAL_WALL, 1}, {SENSOR_OVERCURRENTS, 1}, {SENSOR_IR_BYTE, 1}, {SENSOR_BUTTONS, 1}, {SENSOR_DISTANCE, 2}, {SENSOR_ANGLE, 2}, {SENSOR_CHARGING_STATE, 1}, {SENSOR_VOLTAGE, 2}, {SENSOR_CURRENT, 2}, {SENSOR_BATTERY_TEMP, 1}, {SENSOR_BATTERY_CHARGE, 2}, {SENSOR_BATTERY_CAPACITY, 2}, {SENSOR_WALL_SIGNAL, 2}, {SENSOR_CLIFF_LEFT_SIGNAL, 2}, {SENSOR_CLIFF_FL_SIGNAL, 2}, {SENSOR_CLIFF_FR_SIGNAL, 2}, {SENSOR_CLIFF_R_SIGNAL, 2}, {SENSOR_USER_DIGITAL_IN, 1}, {SENSOR_USER_ANALOG_IN, 2}, {SENSOR_CHARGING_SOURCES, 1}, {SENSOR_OI_MODE, 1}, {SENSOR_SONG_NUMBER, 1}, {SENSOR_SONG_PLAYING, 1}, {SENSOR_NUM_STREAM_PACKETS, 1}, {SENSOR_VELOCITY, 2}, {SENSOR_RADIUS, 2}, {SENSOR_RIGHT_VELOCITY, 2}, {SENSOR_LEFT_VELOCITY, 2}, {SENSOR_LEFT_ENCODER, 2}, {SENSOR_RIGHT_ENCODER, 2}, {SENSOR_LIGHT_BUMPER, 1}, {SENSOR_LIGHT_BUMP_L_SIG, 2}, {SENSOR_LIGHT_BUMP_FL_SIG, 2}, {SENSOR_LIGHT_BUMP_CL_SIG, 2}, {SENSOR_LIGHT_BUMP_CR_SIG, 2}, {SENSOR_LIGHT_BUMP_FR_SIG, 2}, {SENSOR_LIGHT_BUMP_R_SIG, 2}, {SENSOR_LEFT_MOTOR_CURR, 2}, {SENSOR_RIGHT_MOTOR_CURR, 2}, {SENSOR_MAIN_BRUSH_CURR, 2}, {SENSOR_SIDE_BRUSH_CURR, 2}, {SENSOR_STASIS, 1}, {SENSOR_GRP_7_TO_58, 80}, {SENSOR_GRP_43_TO_58, 28}, {SENSOR_GRP_46_TO_51, 12}, {SENSOR_GRP_54_TO_58, 9}};

        enum ChargeState
        {
            CHARGE_STATE_NOT_CHARGING = 0,
            CHARGE_STATE_RECONDITIONING = 1,
            CHARGE_STATE_FULL = 2,
            CHARGE_STATE_TRICKLE = 3,
            CHARGE_STATE_WAITING = 4,
            CHARGE_STATE_FAULT = 5
        };

        static const std::map<uint8_t, std::string> CHARGE_STATE_STRINGS = {
            {CHARGE_STATE_NOT_CHARGING, "Not Charging"},
            {CHARGE_STATE_RECONDITIONING, "Reconditioning Charging"},
            {CHARGE_STATE_FULL, "Full Charging"},
            {CHARGE_STATE_TRICKLE, "Trickle Charging"},
            {CHARGE_STATE_WAITING, "Waiting"},
            {CHARGE_STATE_FAULT, "Charging Fault"}};

        struct PacketMember
        {
            uint8_t packet_id;
            uint8_t start_byte;
            uint8_t byte_count;
        };

        static const std::map<uint8_t, std::vector<PacketMember>> GROUP_PACKET_MEMBERS = {
            {SENSOR_GRP_21_TO_26,
             {
                 {SENSOR_CHARGING_STATE, 0, 1},
                 {SENSOR_VOLTAGE, 1, 2},
                 {SENSOR_CURRENT, 3, 2},
                 {SENSOR_BATTERY_TEMP, 5, 1},
                 {SENSOR_BATTERY_CHARGE, 6, 2},
                 {SENSOR_BATTERY_CAPACITY, 8, 2},
             }},
            {SENSOR_GRP_7_TO_16,
             {
                 {SENSOR_BUMPS_WHEEL_DROPS, 0, 1},
                 {SENSOR_WALL, 1, 1},
                 {SENSOR_CLIFF_LEFT, 2, 1},
                 {SENSOR_CLIFF_FRONT_LEFT, 3, 1},
                 {SENSOR_CLIFF_FRONT_RIGHT, 4, 1},
                 {SENSOR_CLIFF_RIGHT, 5, 1},
                 {SENSOR_VIRTUAL_WALL, 6, 1},
                 {SENSOR_OVERCURRENTS, 7, 1},
                 {SENSOR_IR_BYTE, 8, 1},
                 {SENSOR_BUTTONS, 9, 1},
             }},
        };

        static const std::map<uint8_t, uint8_t> PACKET_TO_GROUP = {
            {SENSOR_CHARGING_STATE, SENSOR_GRP_21_TO_26},
            {SENSOR_VOLTAGE, SENSOR_GRP_21_TO_26},
            {SENSOR_CURRENT, SENSOR_GRP_21_TO_26},
            {SENSOR_BATTERY_TEMP, SENSOR_GRP_21_TO_26},
            {SENSOR_BATTERY_CHARGE, SENSOR_GRP_21_TO_26},
            {SENSOR_BATTERY_CAPACITY, SENSOR_GRP_21_TO_26},
            {SENSOR_BUMPS_WHEEL_DROPS, SENSOR_GRP_7_TO_16},
            {SENSOR_WALL, SENSOR_GRP_7_TO_16},
            {SENSOR_CLIFF_LEFT, SENSOR_GRP_7_TO_16},
            {SENSOR_CLIFF_FRONT_LEFT, SENSOR_GRP_7_TO_16},
            {SENSOR_CLIFF_FRONT_RIGHT, SENSOR_GRP_7_TO_16},
            {SENSOR_CLIFF_RIGHT, SENSOR_GRP_7_TO_16},
            {SENSOR_VIRTUAL_WALL, SENSOR_GRP_7_TO_16},
            {SENSOR_OVERCURRENTS, SENSOR_GRP_7_TO_16},
            {SENSOR_IR_BYTE, SENSOR_GRP_7_TO_16},
            {SENSOR_BUTTONS, SENSOR_GRP_7_TO_16},
        };

        enum RoombaCommand : uint8_t
        {
            CMD_START = 128,
            CMD_BAUD = 129,
            CMD_CONTROL = 130,
            CMD_SAFE = 131,
            CMD_FULL = 132,
            CMD_POWER = 133,
            CMD_SPOT = 134,
            CMD_CLEAN = 135,
            CMD_MAX = 136,
            CMD_DRIVE = 137,
            CMD_MOTORS = 138,
            CMD_SONG = 140,
            CMD_PLAY = 141,
            CMD_QUERY = 142,
            CMD_DOCK = 143
        };

    }
}