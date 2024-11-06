#include <stdexcept>

#include <ixblue_stdbin_decoder/memory_block_parser.h>

namespace ixblue_stdbin_decoder
{

void MemoryBlockParser::parse(boost::asio::const_buffer& buffer, const tBitMask& mask,
                              Data::BinaryNav& outBinaryNav)
{
    // If this memory block is present in the frame,
    if(mask.test(offsetInMask))
    {
        // And there is enough bytes in buffer we can parse it :
        if(buffer.size() >= bytesCount)
        {
            parse(buffer, outBinaryNav);
        }
        else
        {
            throw std::runtime_error(
                "Not enough byes in buffer to parse this memory block");
        }
    }
}

boost::asio::const_buffer& operator>>(boost::asio::const_buffer& buf, double& res)
{
    uint64_t bytes = BIG_TO_NATIVE64(*boost::asio::buffer_cast<const uint64_t*>(buf));
    std::memcpy(&res, &bytes, sizeof(double));
    buf = buf + sizeof(double);
    return buf;
}

boost::asio::const_buffer& operator>>(boost::asio::const_buffer& buf, float& res)
{
    uint32_t bytes = BIG_TO_NATIVE32(*boost::asio::buffer_cast<const uint32_t*>(buf));
    std::memcpy(&res, &bytes, sizeof(float));
    buf = buf + sizeof(float);
    return buf;
}

boost::asio::const_buffer& operator>>(boost::asio::const_buffer& buf, int32_t& res)
{
    res = BIG_TO_NATIVE32(*boost::asio::buffer_cast<const int32_t*>(buf));
    buf = buf + sizeof(int32_t);
    return buf;
}

boost::asio::const_buffer& operator>>(boost::asio::const_buffer& buf, uint64_t& res)
{
    res = BIG_TO_NATIVE64(*boost::asio::buffer_cast<const uint64_t*>(buf));
    buf = buf + sizeof(uint64_t);
    return buf;
}

boost::asio::const_buffer& operator>>(boost::asio::const_buffer& buf, uint32_t& res)
{
    res = BIG_TO_NATIVE32(*boost::asio::buffer_cast<const uint32_t*>(buf));
    buf = buf + sizeof(uint32_t);
    return buf;
}

boost::asio::const_buffer& operator>>(boost::asio::const_buffer& buf, uint16_t& res)
{
    res = BIG_TO_NATIVE16(*boost::asio::buffer_cast<const uint16_t*>(buf));
    buf = buf + sizeof(uint16_t);
    return buf;
}

boost::asio::const_buffer& operator>>(boost::asio::const_buffer& buf, uint8_t& res)
{
    res = *boost::asio::buffer_cast<const uint8_t*>(buf);
    buf = buf + sizeof(uint8_t);
    return buf;
}

} // namespace ixblue_stdbin_decoder
