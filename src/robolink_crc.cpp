#include "foray_platform/robolink_crc.hpp"

namespace foray_platform
{
    std::uint32_t crc32_iso_hdlc(const std::uint8_t* data, std::size_t size)
    {
        std::uint32_t crc = 0xFFFFFFFFu;

        for(std::size_t i = 0; i < size ; ++i)
        {
            crc ^= data[i];
            for(int bit = 0; bit < 8 ; ++bit)
            {
                if((crc & 1u) != 0u){ // 最右为1
                    crc = (crc >> 1) ^ 0xEDB88320u;
                } else { // 最右为0
                    crc = crc >> 1;
                }
            }
        }

        return crc ^ 0xFFFFFFFFu;
    }

} // namespace foray_platform
