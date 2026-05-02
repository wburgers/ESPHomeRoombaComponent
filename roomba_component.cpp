#include "esphome/core/log.h"
#include "roomba_component.h"

namespace esphome {
namespace roomba_component {

#define ROOMBA_READ_TIMEOUT 200

static const char *TAG = "roomba_component";

void RoombaComponent::setup() {
	register_service(&RoombaComponent::on_command, "command", {"command"});
	start_serial();
}

void RoombaComponent::loop() {
}

void RoombaComponent::on_command(std::string command) {
	ESP_LOGI(TAG, "Command: %s", command.c_str());
	if (command == "turn_on" || command == "turn_off" || command == "start" || command == "stop") {
		cover();
	}
	else if (command == "dock" || command == "return_to_base") {
		dock();
	}
	else if (command == "locate") {
		locate();
	}
	else if (command == "spot" || command == "clean_spot") {
		spot();
	}
	else if (command == "wakeup") {
		//this->brc_wakeup();
	}
	else if (command == "sleep") {
		//sleep();
	}
}

void RoombaComponent::start_serial() {
	write(128);
}

void RoombaComponent::cover() {
	write(135);
}

void RoombaComponent::dock() {
	write(143);
}

void RoombaComponent::spot() {
	write(134);
}

void RoombaComponent::safe_mode() {
	write(131);
}

void RoombaComponent::locate() {
	uint8_t song[] = {62, 12, 66, 12, 69, 12, 74, 36};
	safe_mode();
	delay(500);
	set_song(0, song, sizeof(song));
	play_song(0);
}

void RoombaComponent::set_song(uint8_t songNumber, uint8_t data[], uint8_t len) {
	write(140);
	write(songNumber);
	write(len >> 1); // 2 bytes per note
	write_array(data, len);
}

void RoombaComponent::play_song(uint8_t songNumber) {
	write(141);
	write(songNumber);
}

void RoombaComponent::flush() {
	while (available())
	{
		read();
	}
}

bool RoombaComponent::get_sensor(uint8_t packet_id, uint8_t* dest, uint8_t len) {
	write(148);
	write(packet_id);
	return get_data(dest, len);
}

bool RoombaComponent::get_sensors_list(uint8_t* packet_ids, uint8_t number_packet_ids, uint8_t* dest, uint8_t len) {
	write(149);
	write(number_packet_ids);
	write_array(packet_ids, number_packet_ids);
	return get_data(dest, len);
}

bool RoombaComponent::get_data(uint8_t* dest, uint8_t len) {
	while (len-- > 0) {
		unsigned long startTime = millis();
		while (!available()) {
			yield();
			// Look for a timeout
			if (millis() > startTime + ROOMBA_READ_TIMEOUT) {
				return false;
			}
		}
		*dest++ = read();
	}
	return true;
}

void RoombaComponent::dump_config(){
	ESP_LOGCONFIG(TAG, "Roomba Component:");
}

}  // namespace roomba_component
}  // namespace esphome
