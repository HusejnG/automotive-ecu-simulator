// can_frame.h
// A single CAN message: an ID, a data length, and up to 8 payload bytes
// -- the classic CAN 2.0 frame shape. Extended (29-bit) IDs aren't
// modeled here; 11-bit standard IDs cover everything this project needs.

#pragma once

#include <array>
#include <cstdint>
#include <vector>

struct CanFrame {
    std::uint32_t id = 0;      // 11-bit standard ID (0-2047)
    std::uint8_t dlc = 0;      // data length, 0-8 bytes
    std::array<std::uint8_t, 8> data{};

    CanFrame() = default;

    CanFrame(std::uint32_t frameId, const std::vector<std::uint8_t>& payload)
        : id(frameId) {
        dlc = static_cast<std::uint8_t>(payload.size() > 8 ? 8 : payload.size());
        for (std::uint8_t i = 0; i < dlc; ++i) {
            data[i] = payload[i];
        }
    }
};
