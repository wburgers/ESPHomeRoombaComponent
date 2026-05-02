#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/api/custom_api_device.h"

namespace esphome {
namespace roomba_component {

typedef enum {
	SensorGroup7to26					= 0,  //00
	SensorGroup7to16					= 1,  //01
	SensorGroup17to20					= 2,  //02
	SensorGroup21to26					= 3,  //03
	SensorGroup27to34					= 4,  //04
	SensorGroup35to42					= 5,  //05
	SensorGroup7to42					= 6,  //06
	SensorBumpsAndWheelDrops			= 7,  //07
	SensorWall							= 8,  //08
	SensorCliffLeft						= 9,  //09
	SensorCliffFrontLeft				= 10, //0A
	SensorCliffFrontRight				= 11, //0B
	SensorCliffRight					= 12, //0C
	SensorVirtualWall					= 13, //0D
	SensorOvercurrents					= 14, //0E
	//SensorUnused1						= 15, //0F
	//SensorUnused2						= 16, //10
	SensorIRByte						= 17, //11
	SensorButtons						= 18, //12
	SensorDistance						= 19, //13
	SensorAngle							= 20, //14
	SensorChargingState					= 21, //15
	SensorVoltage						= 22, //16
	SensorCurrent						= 23, //17
	SensorBatteryTemperature			= 24, //18
	SensorBatteryCharge					= 25, //19
	SensorBatteryCapacity				= 26, //1A
	SensorWallSignal					= 27, //1B
	SensoCliffLeftSignal				= 28, //1C
	SensoCliffFrontLeftSignal			= 29, //1D
	SensoCliffFrontRightSignal			= 30, //1E
	SensoCliffRightSignal				= 31, //1F
	SensorUserDigitalInputs				= 32, //20
	SensorUserAnalogInput				= 33, //21
	SensorChargingSourcesAvailable		= 34, //22
	SensorOIMode						= 35, //23
	SensorSongNumber					= 36, //24
	SensorSongPlaying					= 37, //25
	SensorNumberOfStreamPackets			= 38, //26
	SensorVelocity						= 39, //27
	SensorRadius						= 40, //28
	SensorRightVelocity					= 41, //29
	SensorLeftVelocity					= 42, //2A
	SensorLeftEncoderCounts				= 43, //2B
	SensorRightEncoderCounts			= 44, //2C
	SensorLightBumper					= 45, //2D
	SensorLightBumperLeftSignal			= 46, //2E
	SensorLightBumperFrontLeftSignal	= 47, //2F
	SensorLightBumperCenterLeftSignal	= 48, //30
	SensorLightBumperCenterRightSignal	= 49, //31
	SensorLightBumperFrontRightSignal	= 50, //32
	SensorLightBumperRightSignal		= 51, //33
	SensorLeftMotorCurrent				= 54, //36
	SensorRightMotorCurrent				= 55, //37
	SensorMainBrushMotorCurrent			= 56, //38
	SensorSideBrushMotorCurrent			= 57, //39
	SensorStasis						= 58, //3A
	SensorGroup7to58					= 100, //64
	SensorGroup43to58					= 101, //65
	SensorGroup46to51					= 106, //6A
	SensorGroup54to58					= 107, //6B
} SensorCode;

typedef enum {
	ChargeStateNotCharging			= 0,
	ChargeStateReconditioningCharging	= 1,
	ChargeStateFullCharging			= 2,
	ChargeStateTrickleCharging		= 3,
	ChargeStateWaiting			= 4,
	ChargeStateFault			= 5,
} ChargeState;

class RoombaComponent : public api::CustomAPIDevice, public Component, public uart::UARTDevice {
	public:
		void setup() override;
		void loop() override;
		void dump_config() override;
		void flush();
		bool get_sensor(uint8_t packet_id, uint8_t* dest, uint8_t len);
		bool get_sensors_list(uint8_t* packet_ids, uint8_t number_packet_ids, uint8_t* dest, uint8_t len);
		/* void set_text_sensors(std::array<text_sensor::TextSensor*, 2> sensors) = {
			charging_state_sensor_ = sensors[0];
			activity_sensor_ = sensors[1];
		}; */
	private:
		// text_sensor::TextSensor *charging_state_sensor_ = nullptr;
		// text_sensor::TextSensor *activity_sensor_ = nullptr;
		void on_command(std::string command);
		void start_serial();
		void cover();
		void dock();
		void spot();
		void safe_mode();
		void locate();
		void set_song(uint8_t songNumber, uint8_t data[], uint8_t len);
		void play_song(uint8_t songNumber);
		bool get_data(uint8_t* dest, uint8_t len);
};

}  // namespace roomba_component
}  // namespace esphome
