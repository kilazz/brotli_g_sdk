// external/brotli_g_sdk/inc/common/BrotligCommand.h
// Brotli-G SDK 1.1
//
// Copyright(c) 2022 - 2024 Advanced Micro Devices, Inc. All rights reserved.
// MIT License

#pragma once

extern "C" {
#include "brotli/c/enc/command.h"
}

#include "common/BrotligBitWriter.h"
#include "common/BrotligCommon.h"

namespace BrotliG {
typedef struct BrotligCommand
{
    uint32_t insert_pos;
    uint32_t insert_len;
    uint32_t copy_len;
    uint32_t dist_extra;
    uint16_t cmd_prefix;
    uint16_t dist_prefix;
    uint32_t dist;
    int32_t dist_code;

    BrotligCommand()
    {
        insert_pos = 0;
        insert_len = 0;
        copy_len = 0;
        dist_extra = 0;
        cmd_prefix = 0;
        dist_prefix = 0;
        dist = 0;
        dist_code = 0;
    }

    Command* ToBroliCommand()
    {
        Command* out = new Command;
        out->insert_len_ = insert_len;
        out->copy_len_ = copy_len;
        out->cmd_prefix_ = cmd_prefix;
        out->dist_prefix_ = dist_prefix;
        out->dist_extra_ = dist_extra;

        return out;
    }

    void Copy(Command* in)
    {
        insert_len = in->insert_len_;
        copy_len = in->copy_len_ & 0x1FFFFFF;
        cmd_prefix = in->cmd_prefix_;
        dist_prefix = in->dist_prefix_;
        dist_extra = in->dist_extra_ & 0x00FFFFFFu;
        dist = 0;
        dist_code = 0;
    }

    uint32_t CopyLen() { return copy_len & 0x1FFFFFF; }

    uint32_t DistanceContext()
    {
        uint32_t r = cmd_prefix >> 6;
        uint32_t c = cmd_prefix & 7;
        if ((r == 0 || r == 2 || r == 4 || r == 7) && (c <= 2)) {
            return c;
        }
        return 3;
    }

    uint16_t Distance() { return dist_prefix & 0x3FF; }

    uint32_t CopyLenCode()
    {
        uint32_t modifier = copy_len >> 25;
        int32_t delta = (int8_t)((uint8_t)(modifier | ((modifier & 0x40) << 1)));
        return (uint32_t)((int32_t)(copy_len & 0x1FFFFFF) + delta);
    }

    uint16_t InsertLengthCode()
    {
        if (insert_len < 6) {
            return (uint16_t)insert_len;
        } else if (insert_len < 130) {
            uint32_t nbits = Log2FloorNonZero(insert_len - 2) - 1u;
            return (uint16_t)((nbits << 1) + ((insert_len - 2) >> nbits) + 2);
        } else if (insert_len < 2114) {
            uint32_t nbits = Log2FloorNonZero(insert_len - 66) - 1u;
            return (uint16_t)((nbits << 1) + ((insert_len - 66) >> nbits) + 10);
        } else if (insert_len < 6210) {
            return 21u;
        } else if (insert_len < 22594) {
            return 22u;
        } else {
            return 23u;
        }
    }

    uint16_t GetCopyLengthCode(size_t copylen)
    {
        // Prevent unsigned underflow (1 - 2 = 65535) when copylen is 0 or 1
        if (copylen < 2) {
            return 0;
        } else if (copylen < 10) {
            return (uint16_t)(copylen - 2);
        } else if (copylen < 134) {
            uint32_t nbits = Log2FloorNonZero(copylen - 6) - 1u;
            return (uint16_t)((nbits << 1u) + ((copylen - 6) >> nbits) + 4);
        } else if (copylen < 2118) {
            uint32_t nbits = Log2FloorNonZero(copylen - 70) - 1u;
            return (uint16_t)((nbits << 1u) + ((copylen - 70) >> nbits) + 12);
        } else {
            return 23u;
        }
    }

    void GetExtra(uint32_t& n_bits, uint64_t& bits)
    {
        if (cmd_prefix <= BROTLI_NUM_COMMAND_SYMBOLS) {
            uint32_t copylen_code = CopyLenCode();
            uint16_t inscode = InsertLengthCode();
            uint16_t copycode = GetCopyLengthCode(copylen_code);
            uint32_t insnumextra = GetInsertExtra(inscode);
            uint64_t insextraval = insert_len - GetInsertBase(inscode);
            uint64_t copyextraval = (copycode > 1) ? copylen_code - GetCopyBase(copycode) : copylen_code;
            bits = (copyextraval << insnumextra) | insextraval;
            n_bits = insnumextra + GetCopyExtra(copycode);
        } else {
            uint16_t inscode = InsertLengthCode();
            uint32_t insnumextra = GetInsertExtra(inscode);
            uint64_t insextraval = insert_len - GetInsertBase(inscode);
            bits = insextraval;
            n_bits = insnumextra;
        }
    }

    void StoreExtra(BrotligBitWriterLSB* bw)
    {
        if (cmd_prefix <= BROTLI_NUM_COMMAND_SYMBOLS) {
            uint32_t copylen_code = CopyLenCode();
            uint16_t inscode = InsertLengthCode();
            uint16_t copycode = GetCopyLengthCode(copylen_code);
            uint32_t insnumextra = GetInsertExtra(inscode);
            uint64_t insextraval = insert_len - GetInsertBase(inscode);
            uint64_t copyextraval = (copycode > 1) ? copylen_code - GetCopyBase(copycode) : copylen_code;
            uint64_t bits = (copyextraval << insnumextra) | insextraval;
            bw->Write(insnumextra + GetCopyExtra(copycode), bits);
        } else {
            uint16_t inscode = InsertLengthCode();
            uint32_t insnumextra = GetInsertExtra(inscode);
            uint64_t insextraval = insert_len - GetInsertBase(inscode);
            uint64_t bits = insextraval;
            bw->Write(insnumextra, bits);
        }
    }

    uint16_t CombineLengthCodes(uint16_t inscode, uint16_t copycode, bool use_last_distance)
    {
        uint16_t bits64 = (uint16_t)((copycode & 0x7u) | ((inscode & 0x7u) << 3u));
        if (use_last_distance && inscode < 8u && copycode < 16u) {
            uint16_t combinedcode = (copycode < 8u) ? bits64 : (bits64 | 64u);
            assert(combinedcode < BROTLI_NUM_COMMAND_SYMBOLS);
            return combinedcode;
        } else {
            uint32_t offset = 2u * ((copycode >> 3u) + 3u * (inscode >> 3u));
            offset = (offset << 5u) + 0x40u + ((0x520D40u >> offset) & 0xC0u);
            uint16_t combinedcode = (uint16_t)(offset | bits64);
            assert(combinedcode < BROTLI_NUM_COMMAND_SYMBOLS);
            return combinedcode;
        }
    }
} BrotligCommand;
} // namespace BrotliG
