// external/brotli_g_sdk/inc/common/BrotligDeswizzler.h
// Brotli-G SDK 1.1 (Full Google Brotli v1.2.0 Support)
//
// Copyright(c) 2022 - 2024 Advanced Micro Devices, Inc. All rights reserved.
// MIT License

#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "common/BrotligConstants.h"

namespace BrotliG {
static const uint32_t BrotligBitMask[33] = {
    0x00000000, 0x00000001, 0x00000003, 0x00000007, 0x0000000F, 0x0000001F, 0x0000003F, 0x0000007F, 0x000000FF,
    0x000001FF, 0x000003FF, 0x000007FF, 0x00000FFF, 0x00001FFF, 0x00003FFF, 0x00007FFF, 0x0000FFFF, 0x0001FFFF,
    0x0003FFFF, 0x0007FFFF, 0x000FFFFF, 0x001FFFFF, 0x003FFFFF, 0x007FFFFF, 0x00FFFFFF, 0x01FFFFFF, 0x03FFFFFF,
    0x07FFFFFF, 0x0FFFFFFF, 0x1FFFFFFF, 0x3FFFFFFF, 0x7FFFFFFF, 0xFFFFFFFF};

class BrotligDeswizzler
{
public:
    BrotligDeswizzler() { Reset(); }

    ~BrotligDeswizzler() { Reset(); }

    void Reset()
    {
        for (size_t i = 0; i < BROTLIG_MAX_NUM_BITSTREAMS; ++i) {
            m_bufs[i] = 0;
            m_validBits[i] = 0;
            m_nexts[i] = nullptr;
        }
        m_curindex = 0;
        m_numbitstreams = BROLTIG_DEFAULT_NUM_BITSTREAMS;
    }

    void Initialize(size_t num_bitstreams)
    {
        m_numbitstreams = num_bitstreams;
        m_curindex = 0;
    }

    void SetReader(size_t index, const uint8_t* stream)
    {
        m_nexts[index] = stream;
        uint64_t initial_val = 0;
        if (stream) {
            std::memcpy(&initial_val, stream, 8);
            m_nexts[index] += 8;
        }
        m_bufs[index] = initial_val;
        m_validBits[index] = 64;
    }

    void SetReader(size_t index, const uint8_t* stream, size_t) { SetReader(index, stream); }

    inline uint32_t ReadNoConsume(uint32_t n)
    {
        if (n == 0)
            return 0;
        return static_cast<uint32_t>(m_bufs[m_curindex]) & BrotligBitMask[n];
    }

    inline void Consume(uint32_t n)
    {
        m_bufs[m_curindex] >>= n;
        m_validBits[m_curindex] -= n;
        if (m_validBits[m_curindex] <= 32) {
            uint32_t nextDword = 0;
            if (m_nexts[m_curindex]) {
                std::memcpy(&nextDword, m_nexts[m_curindex], 4);
                m_nexts[m_curindex] += 4;
            }

            uint64_t mask = (m_validBits[m_curindex] >= 64) ? ~0ULL : ((1ULL << m_validBits[m_curindex]) - 1ULL);
            m_bufs[m_curindex] =
                (m_bufs[m_curindex] & mask) | (static_cast<uint64_t>(nextDword) << m_validBits[m_curindex]);

            m_validBits[m_curindex] += 32;
        }
    }

    inline uint32_t ReadAndConsume(uint32_t n)
    {
        if (n == 0)
            return 0;
        uint32_t v = static_cast<uint32_t>(m_bufs[m_curindex]) & BrotligBitMask[n];
        Consume(n);
        return v;
    }

    inline uint32_t ReadNoConsume16() { return static_cast<uint32_t>(m_bufs[m_curindex]) & 0x0000FFFF; }

    inline uint32_t ReadNoConsume15() { return static_cast<uint32_t>(m_bufs[m_curindex]) & 0x00007FFF; }

    inline uint32_t ReadNoConsume10() { return static_cast<uint32_t>(m_bufs[m_curindex]) & 0x000003FF; }

    inline uint32_t ReadNoConsume9() { return static_cast<uint32_t>(m_bufs[m_curindex]) & 0x000001FF; }

    inline void BSSwitch()
    {
        ++m_curindex;
        if (m_curindex == m_numbitstreams)
            m_curindex = 0;
    }

    inline void BSReset() { m_curindex = 0; }

private:
    size_t m_numbitstreams;
    uint64_t m_bufs[BROTLIG_MAX_NUM_BITSTREAMS];
    size_t m_validBits[BROTLIG_MAX_NUM_BITSTREAMS];
    const uint8_t* m_nexts[BROTLIG_MAX_NUM_BITSTREAMS];
    uint32_t m_curindex;
};
} // namespace BrotliG
