// external/brotli_g_sdk/inc/common/BrotligBitReader.h
// Brotli-G SDK 1.1
//
// Copyright(c) 2022 - 2024 Advanced Micro Devices, Inc. All rights reserved.
// MIT License

#pragma once

#include <cassert>
#include <cstring>

extern "C" {
#include "brotli/c/common/platform.h"
}

#include "common/BrotligCommon.h"
#include "common/BrotligConstants.h"

namespace BrotliG
{
    class BrotligBitReaderLSB
    {
    public:
        BrotligBitReaderLSB()
        {
            Reset();
        }

        ~BrotligBitReaderLSB()
        {
            Reset();
        }

        void Reset()
        {
            m_buf = 0;
            m_bitcnt = 0;
            m_input = m_next = m_end = nullptr;
            m_inputsize = 0;
        }

        void Initialize(const uint8_t* input, size_t inputsize)
        {
            m_input = m_next = input;
            m_inputsize = inputsize;
            m_end = input + inputsize;

            uint32_t val = 0;
            size_t copy_bytes = 0;
            if (m_next < m_end) {
                size_t remaining = static_cast<size_t>(m_end - m_next);
                copy_bytes = (remaining < 4) ? remaining : 4;
                std::memcpy(&val, m_next, copy_bytes);
                m_next += copy_bytes;
            }

            m_buf = (uint64_t)val;
            m_bitcnt = copy_bytes * 8;
        }

        inline uint32_t ReadNoConsume(uint32_t n) {
            return (uint32_t)m_buf & BrotligBitMask[n];
        }

        inline uint32_t ReadNoConsume() { return (uint32_t)m_buf; }

        inline uint32_t ReadAndConsume(uint32_t n)
        {
            uint32_t bits = (uint32_t)m_buf & BrotligBitMask[n];
            Consume(n);
            return bits;
        }

        inline void Consume(uint32_t n)
        {
            m_buf >>= n;
            m_bitcnt -= n;

            if (m_bitcnt < minbitcnt && m_next < m_end)
            {
                uint32_t val = 0;
                size_t remaining = static_cast<size_t>(m_end - m_next);
                size_t copy_bytes = (remaining < 4) ? remaining : 4;
                std::memcpy(&val, m_next, copy_bytes);
                m_next += copy_bytes;

                m_buf |= (uint64_t)val << m_bitcnt;
                m_bitcnt += copy_bytes * 8;
            }
        }

        inline uint16_t ReadNoConsume16() { return (uint16_t)m_buf; }

        inline const uint8_t* GetInput()
        {
            return m_input;
        }

    private:
        const uint32_t BrotligBitMask[33] = { 0x00000000,
            0x00000001, 0x00000003, 0x00000007, 0x0000000F,
            0x0000001F, 0x0000003F, 0x0000007F, 0x000000FF,
            0x000001FF, 0x000003FF, 0x000007FF, 0x00000FFF,
            0x00001FFF, 0x00003FFF, 0x00007FFF, 0x0000FFFF,
            0x0001FFFF, 0x0003FFFF, 0x0007FFFF, 0x000FFFFF,
            0x001FFFFF, 0x003FFFFF, 0x007FFFFF, 0x00FFFFFF,
            0x01FFFFFF, 0x03FFFFFF, 0x07FFFFFF, 0x0FFFFFFF,
            0x1FFFFFFF, 0x3FFFFFFF, 0x7FFFFFFF, 0xFFFFFFFF
        };

        static constexpr uint32_t minbitcnt = 32;

        uint64_t m_buf;
        size_t m_bitcnt;

        const uint8_t* m_input;
        const uint8_t* m_next;
        const uint8_t* m_end;
        size_t m_inputsize;
    };
}
