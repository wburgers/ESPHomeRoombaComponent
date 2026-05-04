#include "roomba_component.h"

namespace esphome
{
	namespace roomba_component
	{

		const char *const RoombaComponent::TAG = "roomba.component";

		void RoombaComponent::setup()
		{
			register_service(&RoombaComponent::on_command, "command", {"command"});
			this->write_byte(CMD_START);
		}

		void RoombaComponent::dump_config()
		{
			ESP_LOGCONFIG(TAG, "Roomba Component:");
			ESP_LOGCONFIG(TAG, "  Sensors: %zu", this->sensors_.size());
		}

		void RoombaComponent::on_command(std::string command)
		{
			if (command == "turn_on" || command == "turn_off" || command == "start" || command == "stop")
			{
				this->start_cleaning();
			}
			else if (command == "dock" || command == "return_to_base")
			{
				this->seek_dock();
			}
			else if (command == "locate")
			{
				this->play_locate_beep();
			}
			else if (command == "spot" || command == "clean_spot")
			{
				this->spot_clean();
			}
			else if (command == "wakeup")
			{
				this->wake_up_device();
			}
		}

		void RoombaComponent::start_cleaning()
		{
			this->write_byte(CMD_CLEAN);
		}

		void RoombaComponent::seek_dock()
		{
			this->write_byte(CMD_DOCK);
		}

		void RoombaComponent::spot_clean()
		{
			this->write_byte(CMD_SPOT);
		}

		void RoombaComponent::play_locate_beep()
		{
			uint8_t song[] = {62, 12, 66, 12, 69, 12, 74, 36};

			this->set_safe_mode();
			delay(500);
			this->set_song(0, song, sizeof(song));
			this->play_song(0);
		}

		void RoombaComponent::set_song(uint8_t song_number, const uint8_t *data, uint8_t len)
		{
			this->write_byte(CMD_SONG);
			this->write_byte(song_number);
			this->write_byte(len >> 1);
			for (size_t i = 0; i < len; i++)
			{
				this->write_byte(data[i]);
			}
		}

		void RoombaComponent::play_song(uint8_t song_number)
		{
			this->write_byte(CMD_PLAY); // 141
			this->write_byte(song_number);
		}

		void RoombaComponent::wake_up_device()
		{
			this->write_byte(CMD_START);
		}

		void RoombaComponent::set_safe_mode()
		{
			this->write_byte(CMD_SAFE);
		}

		void RoombaComponent::set_full_mode()
		{
			this->write_byte(CMD_FULL);
		}

		void RoombaComponent::request_packet(uint8_t packet_id)
		{
			this->write_byte(CMD_QUERY);
			this->write_byte(packet_id);
		}

		void RoombaComponent::flush_uart_buffer()
		{
			while (this->available())
			{
				this->read();
			}
		}

		bool RoombaComponent::has_read_timeout_occurred(uint32_t start_time)
		{
			return (millis() - start_time > 150);
		}

		void RoombaComponent::update()
		{
			if (this->sensors_.empty())
				return;

			for (auto *sensor : this->sensors_)
			{
				uint8_t packet_id = sensor->get_packet_id();
				uint8_t expected_len = sensor->get_expected_size();

				if (expected_len == 0)
					continue;

				this->request_packet(packet_id);

				std::vector<uint8_t> buffer;
				uint32_t start_time = millis();

				while (buffer.size() < expected_len && !this->has_read_timeout_occurred(start_time))
				{
					if (this->available())
					{
						buffer.push_back(this->read());
					}
					yield();
				}

				if (buffer.size() == expected_len)
				{
					sensor->process_packet(buffer);
				}
				else
				{
					this->flush_uart_buffer();
				}
			}
		}

	}
}