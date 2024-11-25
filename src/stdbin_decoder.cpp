#include <numeric>
#include <sstream>
#include <stdexcept>

#include <ixblue_stdbin_decoder/stdbin_decoder.h>

/* Navigation data blocs */
#include <ixblue_stdbin_decoder/memory_blocs_parsers/extended_navigation_data/rotation_acceleration_vessel_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/extended_navigation_data/rotation_acceleration_vessel_frame_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/acceleration_geographic_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/acceleration_vessel_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/acceleration_vessel_frame_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/ahrs_algorithm_status.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/ahrs_system_status.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/ahrs_user_status.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/attitude_heading.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/attitude_heading_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/attitude_quaternion.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/attitude_quaternion_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/course_speed_over_ground.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/current_geographic_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/current_geographic_frame_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/heading_roll_pitch_rate.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/heave_surge_sway_speed.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/ins_algorithm_status.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/ins_system_status.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/ins_user_status.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/position.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/position_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/raw_acceleration_vessel_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/realtime_heave_surge_sway.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/rotation_rate_vessel_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/rotation_rate_vessel_frame_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/sensor_status.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/smart_heave.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/speed_geographic_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/speed_geographic_frame_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/speed_vessel_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/system_date.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/temperatures.h>

/* Extended navigation data blocs */
#include <ixblue_stdbin_decoder/memory_blocs_parsers/extended_navigation_data/raw_rotation_rate_vessel_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/extended_navigation_data/rotation_acceleration_vessel_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/extended_navigation_data/rotation_acceleration_vessel_frame_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/extended_navigation_data/vehicle_attitude_heading.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/extended_navigation_data/vehicle_attitude_heading_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/extended_navigation_data/vehicle_position.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/extended_navigation_data/vehicle_position_deviation.h>

/* External data blocs */
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/depth.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/dmi.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/dvl_ground_speed.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/dvl_water_speed.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/emlog.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/eventmarker.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/gnss.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/lbl.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/logbook.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/sound_velocity.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/turret_angles.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/usbl.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/utc.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/vtg.h>

using namespace boost::asio;

namespace ixblue_stdbin_decoder
{

StdBinDecoder::StdBinDecoder()
    : navigationParsers(
          {std::make_shared<Parser::AttitudeHeading>(),
           std::make_shared<Parser::AttitudeHeadingDeviation>(),
           std::make_shared<Parser::RealTimeHeaveSurgeSway>(),
           std::make_shared<Parser::SmartHeave>(),
           std::make_shared<Parser::HeadingRollPitchRate>(),
           std::make_shared<Parser::RotationRateVesselFrame>(),
           std::make_shared<Parser::AccelerationVesselFrame>(),
           std::make_shared<Parser::Position>(),
           std::make_shared<Parser::PositionDeviation>(),
           std::make_shared<Parser::SpeedGeographicFrame>(),
           std::make_shared<Parser::SpeedGeographicFrameDeviation>(),
           std::make_shared<Parser::CurrentGeographicFrame>(),
           std::make_shared<Parser::CurrentGeographicFrameDeviation>(),
           std::make_shared<Parser::SystemDate>(),
           std::make_shared<Parser::SensorStatus>(),
           std::make_shared<Parser::INSAlgorithmStatus>(),
           std::make_shared<Parser::INSSystemStatus>(),
           std::make_shared<Parser::INSUserStatus>(),
           std::make_shared<Parser::AHRSAlgorithmStatus>(),
           std::make_shared<Parser::AHRSSystemStatus>(),
           std::make_shared<Parser::AHRSUserStatus>(),
           std::make_shared<Parser::HeaveSurgeSwaySpeed>(),
           std::make_shared<Parser::SpeedVesselFrame>(),
           std::make_shared<Parser::AccelerationGeographicFrame>(),
           std::make_shared<Parser::CourseSpeedoverGround>(),
           std::make_shared<Parser::Temperatures>(),
           std::make_shared<Parser::AttitudeQuaternion>(),
           std::make_shared<Parser::AttitudeQuaternionDeviation>(),
           std::make_shared<Parser::RawAccelerationVesselFrame>(),
           std::make_shared<Parser::AccelerationVesselFrameDeviation>(),
           std::make_shared<Parser::RotationRateVesselFrameDeviation>()},
          [](const MemoryBlockParserPtr& lhs, const MemoryBlockParserPtr& rhs) -> bool {
              return lhs->getOffsetInMask() < rhs->getOffsetInMask();
          }),
      extendedNavigationParsers(
          {std::make_shared<Parser::RotationAccelerationVesselFrame>(),
           std::make_shared<Parser::RotationAccelerationVesselFrameDeviation>(),
           std::make_shared<Parser::RawRotationRateVesselFrame>(),
           std::make_shared<Parser::VehicleAttitudeHeading>(),
           std::make_shared<Parser::VehiclePosition>(),
           std::make_shared<Parser::VehiclePositionDeviation>()},
          [](const MemoryBlockParserPtr& lhs, const MemoryBlockParserPtr& rhs) -> bool {
              return lhs->getOffsetInMask() < rhs->getOffsetInMask();
          }),
      externalDataParsers(
          {std::make_shared<Parser::Utc>(),
           std::make_shared<Parser::Gnss1>(),
           std::make_shared<Parser::Gnss2>(),
           std::make_shared<Parser::GnssManual>(),
           std::make_shared<Parser::Emlog1>(),
           std::make_shared<Parser::Emlog2>(),
           std::make_shared<Parser::Depth>(),
           std::make_shared<Parser::Usbl1>(),
           std::make_shared<Parser::Usbl2>(),
           std::make_shared<Parser::Usbl3>(),
           std::make_shared<Parser::DvlGroundSpeed1>(),
           std::make_shared<Parser::DvlWaterSpeed1>(),
           std::make_shared<Parser::SoundVelocity>(),
           std::make_shared<Parser::Dmi>(),
           std::make_shared<Parser::Lbl1>(),
           std::make_shared<Parser::Lbl2>(),
           std::make_shared<Parser::Lbl3>(),
           std::make_shared<Parser::Lbl4>(),
           std::make_shared<Parser::EventMarkerA>(),
           std::make_shared<Parser::EventMarkerB>(),
           std::make_shared<Parser::EventMarkerC>(),
           std::make_shared<Parser::DvlGroundSpeed2>(),
           std::make_shared<Parser::DvlWaterSpeed2>(),
           std::make_shared<Parser::TurretAngles>(),
           std::make_shared<Parser::Vtg1>(),
           std::make_shared<Parser::Vtg2>(),
           std::make_shared<Parser::LogBook>()},
          [](const MemoryBlockParserPtr& lhs, const MemoryBlockParserPtr& rhs) -> bool {
              return lhs->getOffsetInMask() < rhs->getOffsetInMask();
          })

{}


bool StdBinDecoder::parseNextFrame(const std::vector<uint8_t>& data, std::size_t& consumed)
{
	return parseNextFrame(data.data(), data.size(), consumed);
}

bool StdBinDecoder::parseNextFrame(const uint8_t* data, const std::size_t length, std::size_t& consumed)
{
	consumed = 0;
    boost::asio::const_buffer buffer(data, length);
    Data::NavHeader::MessageType header_type = Data::NavHeader::MessageType::Unknown;

    // Skip some bytes if there is no header signature
    while(buffer.size() > HEADER_MINIMAL_SIZE)
    {
    	header_type = haveHeader(buffer);
    	if(header_type == Data::NavHeader::MessageType::Unknown)
    		buffer += 1;
    	else
    		break;
    }

    // Indicate as consumed all bytes that have been skipped so far
    consumed = length - buffer.size();

    // Have not found a valid header signature, return
    if(header_type == Data::NavHeader::MessageType::Unknown)
        return false;

    // Advance buffer by signature size
    buffer += 2;
    // Get protocol version
    uint8_t protocol_version = 0;
    buffer >> protocol_version;

    try
    {
    	if (!haveEnoughBytesToParseHeader(buffer.size(), header_type, protocol_version))
    		return false;
    }
    catch(...)
    {
    	// Rethrow, but tell to advance the buffer before
    	consumed += 1;
    	throw;
    }

    lastHeader = parseHeader(buffer, header_type, protocol_version);
    // if we didn't receive the whole frame, we return false
    if(length - consumed < lastHeader.telegramSize)
    {
        return false;
    }

    // Compare checksum before going further (will throw if bad)
    try
    {
    	compareChecksum(data + consumed, length - consumed);
    }
    catch(...)
    {
        // Rethow, but tell to remove the parsed telegram from the buffer
        consumed += lastHeader.telegramSize;
        throw;
    }

    if(lastHeader.messageType == Data::NavHeader::MessageType::NavData)
    {
        for(const auto& parser : navigationParsers)
        {
            parser->parse(buffer, lastHeader.navigationBitMask, lastParsed);
        }

        if(lastHeader.extendedNavigationBitMask.is_initialized())
        {
            for(const auto& parser : extendedNavigationParsers)
            {
                parser->parse(buffer, lastHeader.extendedNavigationBitMask.get(),
                              lastParsed);
            }
        }

        for(const auto& parser : externalDataParsers)
        {
            parser->parse(buffer, lastHeader.externalSensorBitMask, lastParsed);
        }
    }
    else if(lastHeader.messageType == Data::NavHeader::MessageType::Answer)
    {
        lastAnswer.clear();
        const size_t answerSize =
            lastHeader.telegramSize - ANSWER_HEADER_SIZE - CHECKSUM_SIZE;
        lastAnswer.resize(answerSize);
        buffer_copy(boost::asio::buffer(lastAnswer), buffer, answerSize);
        buffer = buffer + answerSize;
    }

    // Remove the parsed telegram from the buffer
    consumed += lastHeader.telegramSize;
    return true;
}

Data::NavHeader::MessageType StdBinDecoder::haveHeader(const_buffer buffer)
{
	uint8_t h1, h2;

	buffer >> h1;
	if(h1 != 'I' && h1 != 'A')
		return Data::NavHeader::MessageType::Unknown;

	buffer >> h2;

	if(h1 == 'I' && h2 == 'X')
	{
		return Data::NavHeader::MessageType::NavData;
	}

	if(h1 == 'A' && h2 == 'N')
	{
		return Data::NavHeader::MessageType::Answer;
	}
    return Data::NavHeader::MessageType::Unknown;
}

bool StdBinDecoder::haveEnoughBytesToParseHeader(const std::size_t length,
		const Data::NavHeader::MessageType header_type, const uint8_t protocol_version)
{
	if(header_type == Data::NavHeader::MessageType::NavData)
	{
		switch(protocol_version)
		{
		case 0x02: return length >= HEADER_SIZE_V2;
		case 0x03: return length >= HEADER_SIZE_V3;
		case 0x04: return length >= HEADER_SIZE_V4;
		case 0x05: return length >= HEADER_SIZE_V5;
		default:
			throw std::runtime_error("Unhandled protocol version");
		}
	}
	else if(header_type == Data::NavHeader::MessageType::Answer)
	{
		if(protocol_version >= 3 && protocol_version <= 5)
		{
			return length >= ANSWER_HEADER_SIZE;
		}
		else
		{
			throw std::runtime_error("Unhandled protocol version for an answer");
		}
	}
	else
	{
		// Should have been checked before
		throw std::runtime_error("No valid header found");
	}
	return false;
}

void StdBinDecoder::compareChecksum(const uint8_t* data, std::size_t length)
{
    // Create a new buffer to start from start of frame
    boost::asio::const_buffer buffer(data, length);
    const std::size_t checksumPositionInFrame = lastHeader.telegramSize - CHECKSUM_SIZE;
    buffer = buffer + checksumPositionInFrame;
    uint32_t receivedChecksum = 0;
    buffer >> receivedChecksum;

    const uint32_t computedChecksum = std::accumulate(
    		data, data + checksumPositionInFrame, 0);

    if(receivedChecksum != computedChecksum)
    {
        std::ostringstream ss;
        ss << "Bad checksum. Received: 0x" << std::hex << receivedChecksum
           << ", computed: 0x" << computedChecksum;
        throw std::runtime_error(ss.str());
    }
}

Data::NavHeader StdBinDecoder::parseHeader(const_buffer& buffer,
		const Data::NavHeader::MessageType header_type, const uint8_t protocol_version) const
{
    Data::NavHeader res;
    if(buffer.size() < HEADER_MINIMAL_SIZE)
    {
        throw std::runtime_error("Not enough bytes in buffer to parse header");
    }

    res.messageType = header_type;
    if(res.messageType == Data::NavHeader::MessageType::Unknown)
    {
        throw std::runtime_error("Incorrect frame header, expected 'I', 'X' or 'A', 'N'");
    }

    res.protocolVersion = protocol_version;
    if(res.protocolVersion < 2 || res.protocolVersion > 5)
    {
        throw std::runtime_error(
            "Unknown protocol version. Supported protocol are version 2->5");
    }

    if(res.messageType == Data::NavHeader::MessageType::NavData)
    {
        buffer >> res.navigationBitMask;
        if(res.protocolVersion > 2)
        {
            uint32_t extendedNavigationMask;
            buffer >> extendedNavigationMask;
            res.extendedNavigationBitMask = extendedNavigationMask;
        }
        buffer >> res.externalSensorBitMask;
        uint16_t navigationSize = 0;
        if(res.protocolVersion > 3)
        {
            buffer >> navigationSize;
        }
        buffer >> res.telegramSize;
        buffer >> res.navigationDataValidityTime_100us;
        uint32_t counter;
        buffer >> counter;
    }
    else
    {
        buffer >> res.telegramSize;
    }
    return res;
}

} // namespace ixblue_stdbin_decoder
