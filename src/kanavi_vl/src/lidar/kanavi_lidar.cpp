#include "kanavi_lidar.h"

/**
 * @brief Construct a new kanavi lidar::kanavi lidar object
 *
 * @param model_ LiDAR Model ref include/common.h
 */
kanavi_lidar::kanavi_lidar(int model_)
{
	datagram_ = new kanaviDatagram(model_);
	checked_onGoing = false;
	checked_pares_end = false;
	checked_ch = 0;

	checked_lidar_inputed = false;

	checked_model = -1;

	total_size = 0;
}

/**
 * @brief Destroy the kanavi lidar::kanavi lidar object
 *
 */
kanavi_lidar::~kanavi_lidar()
{
}

/**
 * @brief check LiDAR Sensor
 *
 * @param data
 * @return int
 */
int kanavi_lidar::classification(const std::vector<u_char> &data)
{
	if ((data[KANAVI::COMMON::PROTOCOL_POS::HEADER] & 0xFF) == KANAVI::COMMON::PROTOCOL_VALUE::HEADER) // industrial header detected
	{
		u_char indus_M = data[KANAVI::COMMON::PROTOCOL_POS::PRODUCT_LINE];
		switch (indus_M)
		{
		case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R2:
			printf("********R2**********\n");
			return KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R2;
		case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R4:
			printf("********R4**********\n");
			return KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R4;
		case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R270:
			printf("********R270**********\n");
			return KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R270;
		default:
			return -1;
		}
	}

	return -1;
}

int kanavi_lidar::process(const std::vector<u_char> &data)
{
	u_char mode = data[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::MODE)];
	u_char ch = data[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];

	// header value Check in data
	if ((data[KANAVI::COMMON::PROTOCOL_POS::HEADER] & 0xFF) != KANAVI::COMMON::PROTOCOL_VALUE::HEADER) 
	{
		if(checked_onGoing)
		{
			r270(data, datagram_);
			return 0;
		}
		else
		{
			printf("[LiDAR] Invalid header\n");
			return -1;
		}
	}

	// check Model
	if (!checked_lidar_inputed)
	{
		checked_model = classification(data);
		if (checked_model == -1)
		{
			perror("LiDAR Classification Error!");
			return -1;
		}

		// check Model, one more time
		if (checked_model != datagram_->model)
		{
			perror("LiDAR Model not Matched");
			return -1;
		}
		checked_lidar_inputed = true;
		checked_ch = ch;
	}
	

	// Process data based on mode
	if (mode == KANAVI::COMMON::PROTOCOL_VALUE::COMMAND::MODE::DISTANCE_DATA) {
		if(ch == checked_ch)
		{
			printf("---------KANAVI PROCESS------------\n");
		}
		
		try {
			// Process the data directly
			switch (checked_model) {
				case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R2:
					r2(data, datagram_);
					break;
				case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R4:
					r4(data, datagram_);
					break;
				case KANAVI::COMMON::PROTOCOL_VALUE::MODEL::R270:
					r270(data, datagram_);
					break;
				default:
					printf("[LiDAR] Unknown model\n");
					return -1;
			}
			
		} catch (const std::exception& e) {
			printf("[LiDAR] Error in process: %s\n", e.what());
			return -1;
		}
	}
	return 0;
}

std::string kanavi_lidar::getLiDARModel()
{
	return std::string();
}

void kanavi_lidar::parse(const std::vector<u_char> &data)
{
	if (data.size() < KANAVI::COMMON::PROTOCOL_POS::RAWDATA_START)
	{
		printf("[LiDAR] Invalid data size\n");
		return;
	}

	switch (datagram_->model)
	{
	case KANAVI::COMMON::PROTOCOL_VALUE::R2:
		r2(data, datagram_);
		break;
	case KANAVI::COMMON::PROTOCOL_VALUE::R4:
		r4(data, datagram_);
		break;
	case KANAVI::COMMON::PROTOCOL_VALUE::R270:
		r270(data, datagram_);
		break;
	}
}

void kanavi_lidar::r2(const std::vector<u_char> &input, kanaviDatagram *output)
{
	u_char mode = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::MODE)];
	u_char ch = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];
	int channel = static_cast<int>(ch & 0x0F);

	if (mode == KANAVI::COMMON::PROTOCOL_VALUE::COMMAND::MODE::DISTANCE_DATA)
	{
		// set specification
		{
			output->v_resolution = KANAVI::COMMON::SPECIFICATION::R4::VERTICAL_RESOLUTION;
			output->h_resolution = KANAVI::COMMON::SPECIFICATION::R4::HORIZONTAL_RESOLUTION;
			output->current_ch = channel;
		}

		parseLength(input, output, channel); // convert byte to length
		checked_pares_end = true;
	}
}

void kanavi_lidar::r4(const std::vector<u_char> &input, kanaviDatagram *output)
{
	try {
		u_char ch = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];
		int channel = static_cast<int>(ch & 0x0F);

		// Validate channel number
		if (channel >= KANAVI::COMMON::SPECIFICATION::R4::VERTICAL_CHANNEL) {
			printf("[LiDAR] Invalid channel number: %d\n", channel);
			return;
		}

		// Ensure len_buf is properly initialized
		if (output->len_buf.empty() || output->len_buf.size() <= channel) {
			printf("[LiDAR] len_buf not properly initialized for channel %d\n", channel);
			return;
		}

		// set specification for any channel
		output->v_resolution = KANAVI::COMMON::SPECIFICATION::R4::VERTICAL_RESOLUTION;
		output->h_resolution = KANAVI::COMMON::SPECIFICATION::R4::HORIZONTAL_RESOLUTION;
		output->current_ch = channel;

		// process the current channel data
		parseLength(input, output, channel);
		checked_pares_end = true;
	} catch (const std::exception& e) {
		printf("[LiDAR] Error in r4: %s\n", e.what());
	}
}

void kanavi_lidar::r270(const std::vector<u_char> &input, kanaviDatagram *output)
{
	u_char mode = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::MODE)];
	u_char ch = input[static_cast<int>(KANAVI::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];

	// 첫 번째 패킷인 경우 (채널값 + 데이터)
	if (temp_buf_.empty()) // == !checked_onGoing
	{
		checked_onGoing = true;

		// set specification
		output->v_resolution = KANAVI::COMMON::SPECIFICATION::R270::VERTICAL_RESOLUTION;
		output->h_resolution = KANAVI::COMMON::SPECIFICATION::R270::HORIZONTAL_RESOLUTION;
		
		// 첫 번째 패킷 데이터 저장
		temp_buf_ = input;
	}
	// 두 번째 패킷인 경우 (나머지 데이터)
	else
	{
		// 두 번째 패킷은 헤더 체크를 하지 않음
		// 두 패킷의 데이터를 합쳐서 처리
		std::vector<u_char> combined_data;
		combined_data.insert(combined_data.end(), temp_buf_.begin(), temp_buf_.end());
		combined_data.insert(combined_data.end(), input.begin(), input.end());
		
		// 합쳐진 데이터 처리
		parseLength(combined_data, output, 0);
		checked_pares_end = true;
		
		// 다음 프레임을 위해 초기화
		temp_buf_.clear();
		checked_onGoing = false;
	}
}

void kanavi_lidar::parseLength(const std::vector<u_char> &input, kanaviDatagram *output, int ch)
{
	try {
		// Calculate expected size
		int start = KANAVI::COMMON::PROTOCOL_POS::RAWDATA_START;
		int end = input.size() - KANAVI::COMMON::PROTOCOL_SIZE::CHECKSUM;
		int expected_size = (end - start) / 2;

		if (expected_size <= 0) {
			printf("[LiDAR] Invalid data size: start=%d, end=%d\n", start, end);
			return;
		}

		// Create new vector with proper size
		std::vector<float> len_;
		len_.reserve(expected_size);

		// Process data
		for (int i = start; i < end; i += 2) {
			if (i + 1 >= static_cast<int>(input.size())) {
				printf("[LiDAR] Data size error at index %d\n", i);
				break;
			}
			float up = input[i];
			float low = input[i + 1];
			float len = up + low / 100; // convert 2 byte to length[m]
			len_.push_back(len);
		}

		// Verify the processed data
		if (len_.empty()) {
			printf("[LiDAR] No valid data processed for channel %d\n", ch);
			return;
		}

		// Update the output buffer
		output->len_buf[ch] = std::move(len_);
	} catch (const std::exception& e) {
		printf("[LiDAR] Error in parseLength: %s\n", e.what());
	}
}

bool kanavi_lidar::checkedProcessEnd()
{
	return checked_pares_end;
}

void kanavi_lidar::initProcessEnd()
{
	checked_pares_end = false;
}

kanaviDatagram kanavi_lidar::getDatagram()
{
	return *datagram_;
}