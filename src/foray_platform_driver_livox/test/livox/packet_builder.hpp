#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

namespace foray::platform::driver::livox::test {

constexpr std::size_t kHeaderBytes = 36;

inline void put_u16(std::vector<std::uint8_t>& buffer, std::size_t offset, std::uint16_t value) {
    buffer[offset] = static_cast<std::uint8_t>(value & 0xFFU);
    buffer[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFFU);
}

inline void put_u32(std::vector<std::uint8_t>& buffer, std::size_t offset, std::uint32_t value) {
    for (std::size_t index = 0; index < 4; ++index) {
        buffer[offset + index] = static_cast<std::uint8_t>((value >> (8 * index)) & 0xFFU);
    }
}

inline void put_u64(std::vector<std::uint8_t>& buffer, std::size_t offset, std::uint64_t value) {
    for (std::size_t index = 0; index < 8; ++index) {
        buffer[offset + index] = static_cast<std::uint8_t>((value >> (8 * index)) & 0xFFU);
    }
}

inline void put_f32(std::vector<std::uint8_t>& buffer, std::size_t offset, float value) {
    std::memcpy(buffer.data() + offset, &value, sizeof(value));
}

inline std::vector<std::uint8_t> make_packet(std::uint8_t data_type, std::uint8_t time_type,
                                             std::uint16_t dot_num, std::uint8_t frame_cnt,
                                             std::uint16_t udp_cnt, std::uint64_t timestamp_ns,
                                             std::uint16_t time_interval,
                                             const std::vector<std::uint8_t>& payload) {
    std::vector<std::uint8_t> packet(kHeaderBytes + payload.size(), 0);
    packet[0] = 1; // version
    put_u16(packet, 1, static_cast<std::uint16_t>(packet.size()));
    put_u16(packet, 3, time_interval);
    put_u16(packet, 5, dot_num);
    put_u16(packet, 7, udp_cnt);
    packet[9] = frame_cnt;
    packet[10] = data_type;
    packet[11] = time_type;
    put_u64(packet, 28, timestamp_ns);
    std::memcpy(packet.data() + kHeaderBytes, payload.data(), payload.size());
    return packet;
}

#pragma pack(push, 1)
struct HighPointWire {
    std::int32_t x;
    std::int32_t y;
    std::int32_t z;
    std::uint8_t reflectivity;
    std::uint8_t tag;
};

struct LowPointWire {
    std::int16_t x;
    std::int16_t y;
    std::int16_t z;
    std::uint8_t reflectivity;
    std::uint8_t tag;
};

struct SphericalPointWire {
    std::uint32_t depth;
    std::uint16_t theta;
    std::uint16_t phi;
    std::uint8_t reflectivity;
    std::uint8_t tag;
};
#pragma pack(pop)

inline void append(std::vector<std::uint8_t>& payload, const void* data, std::size_t size) {
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    payload.insert(payload.end(), bytes, bytes + size);
}

inline std::vector<std::uint8_t> high_point_payload(std::int32_t x_mm, std::int32_t y_mm,
                                                    std::int32_t z_mm, std::uint8_t reflectivity,
                                                    std::uint8_t tag) {
    HighPointWire point{x_mm, y_mm, z_mm, reflectivity, tag};
    std::vector<std::uint8_t> payload;
    append(payload, &point, sizeof(point));
    return payload;
}

} // namespace foray::platform::driver::livox::test
