#include "esphome.h"

#define ROOMBA_READ_TIMEOUT 200

class RoombaComponent : public UARTDevice, public CustomAPIDevice, public PollingComponent {
	public:
		//Sensor *distanceSensor;
		Sensor *voltageSensor;
		Sensor *currentSensor;
		Sensor *batteryChargeSensor;
		Sensor *batteryCapacitySensor;
		Sensor *batteryPercentSensor;
		TextSensor *chargingSensor;
		TextSensor *activitySensor;

		static RoombaComponent* instance(uint8_t brcPin, UARTComponent *parent, uint32_t updateInterval) {
			static RoombaComponent* INSTANCE = new RoombaComponent(brcPin, parent, updateInterval);
			return INSTANCE;
		}

		void setup() override {
			pinMode(this->brcPin, OUTPUT);
			digitalWrite(this->brcPin, HIGH);

			register_service(&RoombaComponent::on_command, "command", {"command"});
		}

    	void update() override {
			uint8_t charging;
			uint16_t voltage;
			int16_t current;
			uint16_t batteryCharge;
			uint16_t batteryCapacity;

			flush();

			uint8_t sensors[] = {
				SensorChargingState,
				SensorVoltage,
				SensorCurrent,
				SensorBatteryCharge,
				SensorBatteryCapacity
			};
			uint8_t values[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

			bool success = getSensorsList(sensors, sizeof(sensors), values, sizeof(values));
      		if (!success) {
				ESP_LOGD("custom", "Could not get sensor values from serial");
				return;
			}

			charging = values[0];
			voltage = values[1] * 256 + values[2];
			current = values[3] * 256 + values[4];
			batteryCharge = values[5] * 256 + values[6];
      		batteryCapacity = values[7] * 256 + values[8];
			
      		std::string activity = this->get_activity(charging, current);

			float voltageInVolts = (1.0 * voltage) / 1000.0;
			if (this->voltageSensor->state != voltageInVolts) {
				this->voltageSensor->publish_state(voltageInVolts);
			}

			float currentInAmps = (1.0 * current) / 1000.0;
			if (this->currentSensor->state != currentInAmps) {
				this->currentSensor->publish_state(currentInAmps);
			}

			float chargeInAmpHours = (1.0 * batteryCharge) / 1000.0;
			if (this->batteryChargeSensor->state != chargeInAmpHours) {
				this->batteryChargeSensor->publish_state(chargeInAmpHours);
			}

			float capacityInAmpHours = (1.0 * batteryCapacity) / 1000.0;
			if (this->batteryCapacitySensor->state != capacityInAmpHours) {
				this->batteryCapacitySensor->publish_state(capacityInAmpHours);
			}

			float battery_level = 100.0 * ((1.0 * batteryCharge) / (1.0 * batteryCapacity));
			if (this->batteryPercentSensor->state != battery_level) {
				this->batteryPercentSensor->publish_state(battery_level);
			}

			if (this->chargingState != charging) {
				this->chargingState = charging;
				this->chargingSensor->publish_state(ToString(charging));
			}

			if (activity.compare(this->activitySensor->state) != 0) {
				this->activitySensor->publish_state(activity);
			}
		}

	private:
		uint8_t brcPin;
		uint8_t chargingState;
		
    	RoombaComponent(uint8_t brcPin, UARTComponent *parent, uint32_t updateInterval) : UARTDevice(parent), PollingComponent(updateInterval) {
			this->brcPin = brcPin;
			this->voltageSensor = new Sensor();
			this->currentSensor = new Sensor();
			this->batteryChargeSensor = new Sensor();
	        this->batteryCapacitySensor = new Sensor();
	        this->batteryPercentSensor = new Sensor();

			this->chargingSensor = new TextSensor();
			this->activitySensor = new TextSensor();
		}

		typedef enum {
			Sensors7to26					= 0,  //00
			Sensors7to16					= 1,  //01
			Sensors17to20					= 2,  //02
			Sensors21to26					= 3,  //03
			Sensors27to34					= 4,  //04
			Sensors35to42					= 5,  //05
			Sensors7to42					= 6,  //06
			SensorBumpsAndWheelDrops		= 7,  //07
			SensorWall						= 8,  //08
			SensorCliffLeft					= 9,  //09
			SensorCliffFrontLeft			= 10, //0A
			SensorCliffFrontRight			= 11, //0B
			SensorCliffRight				= 12, //0C
			SensorVirtualWall				= 13, //0D
			SensorOvercurrents				= 14, //0E
			//SensorUnused1					= 15, //0F
			//SensorUnused2					= 16, //10
			SensorIRByte					= 17, //11
			SensorButtons					= 18, //12
			SensorDistance					= 19, //13
			SensorAngle						= 20, //14
			SensorChargingState				= 21, //15
			SensorVoltage					= 22, //16
			SensorCurrent					= 23, //17
			SensorBatteryTemperature		= 24, //18
			SensorBatteryCharge				= 25, //19
			SensorBatteryCapacity			= 26, //1A
			SensorWallSignal				= 27, //1B
			SensoCliffLeftSignal			= 28, //1C
			SensoCliffFrontLeftSignal		= 29, //1D
			SensoCliffFrontRightSignal		= 30, //1E
			SensoCliffRightSignal			= 31, //1F
			SensorUserDigitalInputs			= 32, //20
			SensorUserAnalogInput			= 33, //21
			SensorChargingSourcesAvailable	= 34, //22
			SensorOIMode					= 35, //23
			SensorSongNumber				= 36, //24
			SensorSongPlaying				= 37, //25
			SensorNumberOfStreamPackets		= 38, //26
			SensorVelocity					= 39, //27
			SensorRadius					= 40, //28
			SensorRightVelocity				= 41, //29
			SensorLeftVelocity				= 42, //2A
		} SensorCode;

		typedef enum {
			ChargeStateNotCharging				= 0,
			ChargeStateReconditioningCharging	= 1,
			ChargeStateFullCharging				= 2,
			ChargeStateTrickleCharging			= 3,
			ChargeStateWaiting					= 4,
			ChargeStateFault					= 5,
		} ChargeState;

		void brc_wakeup() {
			digitalWrite(this->brcPin, LOW);
			delay(1000);
			digitalWrite(this->brcPin, HIGH);
			delay(100);
		}

		void on_command(std::string command) {
			if (command == "turn_on" || command == "turn_off" || command == "start" || command == "stop") {
				cover();
			}
			else if (command == "dock" || command == "return_to_base") {
				dock();
			}
			else if (command == "locate") {
				//fix later
			}
			else if (command == "spot" || command == "clean_spot") {
				spot();
			}
			else if (command == "wakeup") {
				this->brc_wakeup();
			}
			else if (command == "sleep") {
				sleep();
			}
    	}

		void cover() {
			write(135);
		}

		void dock() {
			write(143);
		}

		void spot() {
			write(134);
		}

		void sleep() {
			write(133);
		}

		void flush() {
			while (available())
			{
				read();
			}
		}

		bool getSensorsList(uint8_t* packetIDs, uint8_t numPacketIDs, uint8_t* dest, uint8_t len){
			write(149);
			write(numPacketIDs);
			write_array(packetIDs, numPacketIDs);
			return getData(dest, len);
		}

		bool getData(uint8_t* dest, uint8_t len) {
			while (len-- > 0) {
				unsigned long startTime = millis();
				while (!available()) {
					yield();
					// Look for a timeout
					if (millis() > startTime + ROOMBA_READ_TIMEOUT)
					return false;
				}
				*dest++ = read();
			}
			return true;
		}

		std::string get_activity(uint8_t charging, int16_t current) {
			bool isCharging = charging == ChargeStateReconditioningCharging || charging == ChargeStateFullCharging || charging == ChargeStateTrickleCharging;
			
			if (current > -50)
				return "Docked";
			else if (isCharging)
				return "Charging";
			else if (current < -300)
				return "Cleaning";
			return "Lost";
		}

		inline const char* ToString(uint8_t chargeState) {
			switch (chargeState) {
				case ChargeStateNotCharging:			return "NotCharging";
				case ChargeStateReconditioningCharging:	return "ReconditioningCharging";
				case ChargeStateFullCharging:			return "FullCharging";
				case ChargeStateTrickleCharging:		return "TrickleCharging";
				case ChargeStateWaiting:				return "Waiting";
				case ChargeStateFault:					return "Fault";
				default:								return "Unknow Charging State";
			}
		}
};
