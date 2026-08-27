// external/brotli_g_sdk/src/decoder/PageDecoder.cpp
// Brotli-G SDK 1.1 (Brotli v1.2.0)
//
// Copyright(c) 2022 - 2024 Advanced Micro Devices, Inc. All rights reserved.
// MIT License

#include "PageDecoder.h"

#include <algorithm>
#include <cassert>
#include <cstring>

#include "common/BrotligBitReader.h"
#include "common/BrotligCommandLut.h"
#include "common/BrotligConstants.h"
#include "common/BrotligUtils.h"
#include "decoder/BrotligHuffmanTable.h"

#define OVERLAP(x1, x2, y1, y2) (x1 < y2 && y1 < x2)

using namespace BrotliG;

PageDecoder::PageDecoder()
{
    for (size_t i = 0; i < BROTLIG_NUM_HUFFMAN_TREES; ++i) {
        m_symbols[i] = nullptr;
        m_codelens[i] = nullptr;
    }
}

PageDecoder::~PageDecoder()
{
    Cleanup();
}

bool PageDecoder::Setup(const BrotligDecoderParams& params, const BrotligDataconditionParams& dcParams)
{
    m_params = params;
    m_dcparams = dcParams;

    for (size_t i = 0; i < BROTLIG_NUM_HUFFMAN_TREES; ++i) {
        if (!m_symbols[i])
            m_symbols[i] = new uint16_t[BROTLIG_HUFFMAN_TABLE_SIZE];
        if (!m_codelens[i])
            m_codelens[i] = new uint16_t[BROTLIG_HUFFMAN_TABLE_SIZE];

        memset(m_symbols[i], 0, sizeof(uint16_t) * BROTLIG_HUFFMAN_TABLE_SIZE);
        memset(m_codelens[i], 0, sizeof(uint16_t) * BROTLIG_HUFFMAN_TABLE_SIZE);
    }

    m_pReader.Reset();
    m_pReader.Initialize(m_params.num_bitstreams);

    return true;
}

void PageDecoder::Cleanup()
{
    for (size_t i = 0; i < BROTLIG_NUM_HUFFMAN_TREES; ++i) {
        if (m_symbols[i]) {
            delete[] m_symbols[i];
            m_symbols[i] = nullptr;
        }
        if (m_codelens[i]) {
            delete[] m_codelens[i];
            m_codelens[i] = nullptr;
        }
    }
}

bool PageDecoder::Run(const uint8_t* input, size_t inputSize, size_t inputOffset, uint8_t* output, size_t outputSize,
                      size_t outputOffset)
{
    const uint8_t* p_inPtr = input + inputOffset;
    uint8_t* p_outPtr = output + outputOffset;

    if (outputSize == inputSize) {
        memcpy(p_outPtr, p_inPtr, outputSize);
        return true;
    } else {
        for (size_t i = 0; i < BROTLIG_NUM_HUFFMAN_TREES; ++i) {
            if (m_symbols[i])
                memset(m_symbols[i], 0, sizeof(uint16_t) * BROTLIG_HUFFMAN_TABLE_SIZE);
            if (m_codelens[i])
                memset(m_codelens[i], 0, sizeof(uint16_t) * BROTLIG_HUFFMAN_TABLE_SIZE);
        }

        BrotligBitReaderLSB br;
        br.Initialize(p_inPtr, inputSize);

        m_params.distance_postfix_bits = br.ReadAndConsume(BROTLIG_PAGE_HEADER_NPOSTFIX_BITS);
        m_params.num_direct_distance_codes = br.ReadAndConsume(BROTLIG_PAGE_HEADER_NDIST_BITS)
                                          << m_params.distance_postfix_bits;
        bool Isdeltaencoded = (br.ReadAndConsume(BROTLIG_PAGE_HEADER_ISDELTAENCODED_BITS) == 1);
        br.Consume(BROTLIG_PAGE_HEADER_RESERVED_BITS);

        uint32_t compressedOffsetBits = BROTLIG_PAGE_HEADER_SIZE_BITS;

        uint32_t rAvgBSSizeInBytes =
            static_cast<uint32_t>((inputSize + (m_params.num_bitstreams - 1)) / m_params.num_bitstreams);
        uint32_t baseSizeBits = Log2FloorNonZero(rAvgBSSizeInBytes) + 1;

        uint32_t logSize = Log2FloorNonZero(static_cast<uint32_t>(inputSize - 1)) + 1;
        uint32_t deltaBitsSizeBits = Log2FloorNonZero(logSize) + 1;

        uint32_t baseSize = br.ReadAndConsume(baseSizeBits);
        uint32_t deltaSizeBits = br.ReadAndConsume(deltaBitsSizeBits);
        compressedOffsetBits +=
            (baseSizeBits + deltaBitsSizeBits + static_cast<uint32_t>(m_params.num_bitstreams) * deltaSizeBits);
        compressedOffsetBits =
            ((compressedOffsetBits + BROTLIG_DWORD_SIZE_BITS - 1) / BROTLIG_DWORD_SIZE_BITS) * BROTLIG_DWORD_SIZE_BITS;
        size_t inIndex = compressedOffsetBits / 8;

        m_pReader.Reset();
        m_pReader.Initialize(m_params.num_bitstreams);

        for (size_t i = 0; i < m_params.num_bitstreams; ++i) {
            uint32_t delta = br.ReadAndConsume(deltaSizeBits);
            uint32_t bslength = baseSize + delta;
            m_pReader.SetReader(i, &p_inPtr[inIndex]);
            inIndex += bslength;
        }

        m_pReader.BSReset();

        LoadHuffmanTable(m_pReader, BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE, m_symbols[BROTLIG_ICP_TREE_INDEX],
                         m_codelens[BROTLIG_ICP_TREE_INDEX]);

        LoadHuffmanTable(m_pReader, BROTLIG_NUM_DISTANCE_SYMBOLS, m_symbols[BROTLIG_DIST_TREE_INDEX],
                         m_codelens[BROTLIG_DIST_TREE_INDEX]);

        LoadHuffmanTable(m_pReader, BROTLI_NUM_LITERAL_SYMBOLS, m_symbols[BROTLIG_LIT_TREE_INDEX],
                         m_codelens[BROTLIG_LIT_TREE_INDEX]);

        m_distring[0] = 4;
        m_distring[1] = 11;
        m_distring[2] = 15;
        m_distring[3] = 16;

        size_t num_bitstreams = m_params.num_bitstreams;
        uint8_t* wPtr = p_outPtr;
        uint8_t* wEnd = p_outPtr + outputSize;

        size_t litQueueCap = (outputSize * 2) + 262144;
        uint8_t* litQueue = new uint8_t[litQueueCap];
        uint8_t* lqfront = litQueue;
        uint8_t* lqback = litQueue;

        size_t cmdQueueCap = (outputSize * 2) + 131072;
        BrotligCommand* cmdQueue = new BrotligCommand[cmdQueueCap];
        BrotligCommand* cqfront = cmdQueue;
        BrotligCommand* cqback = cmdQueue;

        size_t bs_processed = 0, litcount = 0, aclitcount = 0, mult = 0, rlitcount = 0, prev_tail = 0;
        bool foundSentinel = false;
        BrotligCommand cmd;

        while (!foundSentinel && wPtr < wEnd) {
            if (lqfront < lqback) {
                size_t remaining = static_cast<size_t>(lqback - lqfront);
                memmove(litQueue, lqfront, remaining);
                lqfront = litQueue;
                lqback = litQueue + remaining;
            } else {
                lqfront = lqback = litQueue;
            }

            litcount = 0;
            bs_processed = 0;

            while (bs_processed < num_bitstreams) {
                if (DecodeCommand(cmd)) {
                    foundSentinel = true;
                }

                if (!foundSentinel) {
                    litcount += cmd.insert_len;
                    if (cqback < cmdQueue + cmdQueueCap) {
                        *cqback++ = cmd;
                    }
                }

                ++bs_processed;
                m_pReader.BSSwitch();
            }
            m_pReader.BSReset();

            aclitcount = (litcount > prev_tail) ? litcount - prev_tail : 0;
            mult = (aclitcount + num_bitstreams - 1) / num_bitstreams;
            rlitcount = num_bitstreams * mult;
            prev_tail = rlitcount + prev_tail - litcount;

            while (rlitcount--) {
                if (lqback < litQueue + litQueueCap) {
                    *lqback++ = DecodeLiteral();
                } else {
                    DecodeLiteral();
                }
                m_pReader.BSSwitch();
            }
            m_pReader.BSReset();

            while (cqfront != cqback) {
                cmd = *cqfront++;

                // 1. Copy literals
                while (cmd.insert_len > 0 && wPtr < wEnd && lqfront < lqback) {
                    *wPtr++ = *lqfront++;
                    cmd.insert_len--;
                }

                if (cmd.dist == 0)
                    continue;

                // 2. LZ77 Match Copy (Bounded strictly to current page)
                while (cmd.copy_len > 0 && wPtr < wEnd) {
                    if (wPtr >= p_outPtr + cmd.dist) {
                        *wPtr = *(wPtr - cmd.dist);
                    } else {
                        *wPtr = 0;
                    }
                    wPtr++;
                    cmd.copy_len--;
                }
            }

            cqfront = cqback = cmdQueue;
        }

        delete[] litQueue;
        delete[] cmdQueue;

        if (Isdeltaencoded) {
            DeltaDecode(outputOffset, outputOffset + outputSize, p_outPtr);
        }

        return true;
    }
}

bool PageDecoder::DecodeCommand(BrotligCommand& cmd)
{
    uint16_t bits = sBrotligReverseBits15[m_pReader.ReadNoConsume15()];
    m_pReader.Consume(m_codelens[BROTLIG_ICP_TREE_INDEX][bits]);
    cmd.cmd_prefix = m_symbols[BROTLIG_ICP_TREE_INDEX][bits];

    if (cmd.cmd_prefix == BROTLI_NUM_COMMAND_SYMBOLS) // 704 = EOS Sentinel
    {
        cmd.insert_len = 0;
        cmd.copy_len = 0;
        cmd.dist = 0;
        cmd.dist_code = 0;
        return true;
    } else if (cmd.cmd_prefix < BROTLI_NUM_COMMAND_SYMBOLS) // 0..703
    {
        BrotligCmdLutElement clut = sBrotligCmdLut[cmd.cmd_prefix];
        cmd.insert_len = clut.insert_len_offset + m_pReader.ReadAndConsume(clut.insert_len_extra_bits);
        cmd.copy_len = clut.copy_len_offset + m_pReader.ReadAndConsume(clut.copy_len_extra_bits);
        cmd.dist_code = (cmd.cmd_prefix >= 128) ? DecodeDistance() : 0;
        TranslateDistance(cmd);
        return false;
    } else // 705..728 = Insert-only command
    {
        uint16_t inscode = cmd.cmd_prefix - BROTLI_NUM_COMMAND_SYMBOLS;
        uint32_t insnumextra = GetInsertExtra(inscode);
        uint32_t insert_base = GetInsertBase(inscode);
        uint32_t insert_extra_val = m_pReader.ReadAndConsume(insnumextra);
        cmd.insert_len = insert_base + insert_extra_val;
        cmd.copy_len = 0;
        cmd.dist = 0;
        cmd.dist_code = 0;
        return false;
    }
}

uint8_t PageDecoder::DecodeLiteral()
{
    uint16_t bits = sBrotligReverseBits15[m_pReader.ReadNoConsume15()];
    m_pReader.Consume(m_codelens[BROTLIG_LIT_TREE_INDEX][bits]);
    return static_cast<uint8_t>(m_symbols[BROTLIG_LIT_TREE_INDEX][bits]);
}

uint8_t PageDecoder::DecodeNFetchLiteral(uint16_t& code, size_t& codelen)
{
    uint16_t bits = sBrotligReverseBits15[m_pReader.ReadNoConsume15()];
    codelen = m_codelens[BROTLIG_LIT_TREE_INDEX][bits];
    m_pReader.Consume(static_cast<uint32_t>(codelen));
    code = bits;
    return static_cast<uint8_t>(m_symbols[BROTLIG_LIT_TREE_INDEX][bits]);
}

uint32_t PageDecoder::DecodeDistance()
{
    uint16_t bits = BrotligReverseBits15(static_cast<uint16_t>(m_pReader.ReadNoConsume15()));
    m_pReader.Consume(m_codelens[BROTLIG_DIST_TREE_INDEX][bits]);
    return m_symbols[BROTLIG_DIST_TREE_INDEX][bits];
}

void PageDecoder::TranslateDistance(BrotligCommand& cmd)
{
    uint32_t ndistbits = 0;
    uint32_t dist_code = static_cast<uint32_t>(cmd.dist_code);
    switch (dist_code) {
    case 0:
        cmd.dist = m_distring[0];
        return;
    case 1: {
        cmd.dist = m_distring[1];
        uint32_t temp = m_distring[1];
        m_distring[1] = m_distring[0];
        m_distring[0] = temp;
        return;
    }
    case 2: {
        cmd.dist = m_distring[2];
        uint32_t temp = m_distring[2];
        m_distring[2] = m_distring[1];
        m_distring[1] = m_distring[0];
        m_distring[0] = temp;
        return;
    }
    case 3: {
        cmd.dist = m_distring[3];
        uint32_t temp = m_distring[3];
        m_distring[3] = m_distring[2];
        m_distring[2] = m_distring[1];
        m_distring[1] = m_distring[0];
        m_distring[0] = temp;
        return;
    }
    case 4:
        cmd.dist = m_distring[0] - 1;
        break;
    case 5:
        cmd.dist = m_distring[0] + 1;
        break;
    case 6:
        cmd.dist = m_distring[0] - 2;
        break;
    case 7:
        cmd.dist = m_distring[0] + 2;
        break;
    case 8:
        cmd.dist = m_distring[0] - 3;
        break;
    case 9:
        cmd.dist = m_distring[0] + 3;
        break;
    case 10:
        cmd.dist = m_distring[1] - 1;
        break;
    case 11:
        cmd.dist = m_distring[1] + 1;
        break;
    case 12:
        cmd.dist = m_distring[1] - 2;
        break;
    case 13:
        cmd.dist = m_distring[1] + 2;
        break;
    case 14:
        cmd.dist = m_distring[1] - 3;
        break;
    case 15:
        cmd.dist = m_distring[1] + 3;
        break;
    default: {
        if (m_params.num_direct_distance_codes > 0 && dist_code < 16 + m_params.num_direct_distance_codes) {
            cmd.dist = dist_code - 15;
        } else {
            ndistbits =
                1 + ((dist_code - m_params.num_direct_distance_codes - 16) >> (m_params.distance_postfix_bits + 1));

            cmd.dist_extra = m_pReader.ReadAndConsume(ndistbits);

            uint32_t hcode = (dist_code - m_params.num_direct_distance_codes - 16) >> m_params.distance_postfix_bits;

            uint32_t lcode =
                (dist_code - m_params.num_direct_distance_codes - 16) & Mask32(m_params.distance_postfix_bits);

            uint32_t offset = ((2 + (hcode & 1)) << ndistbits) - 4;
            cmd.dist = ((offset + cmd.dist_extra) << m_params.distance_postfix_bits) + lcode +
                       m_params.num_direct_distance_codes + 1;
        }
        break;
    }
    }

    m_distring[3] = m_distring[2];
    m_distring[2] = m_distring[1];
    m_distring[1] = m_distring[0];
    m_distring[0] = cmd.dist;
}

uint32_t PageDecoder::DeconditionBC1_5(uint32_t offsetAddr, uint32_t sub)
{
    uint32_t sbsize = m_dcparams.subBlockSizes[sub];
    uint32_t sboffset = m_dcparams.subBlockOffsets[sub];
    uint32_t mip = 0;
    for (uint32_t i = 0; i < m_dcparams.numMipLevels; ++i) {
        if (offsetAddr >= m_dcparams.mipOffsetBlocks[i] * sbsize)
            mip = i;
    }

    uint32_t moffset_block = m_dcparams.mipOffsetBlocks[mip];
    uint32_t mip_pos = m_dcparams.mipOffsetsBytes[mip];
    uint32_t mip_width = m_dcparams.widthInBlocks[mip];
    uint32_t mip_pitch = m_dcparams.pitchInBytes[mip];

    uint32_t localOffset = offsetAddr - (moffset_block * sbsize);
    uint32_t block = localOffset / sbsize;
    uint32_t row = block / mip_width;
    uint32_t col = block % mip_width;

    if (m_dcparams.swizzle && mip_width >= BROTLIG_PRECON_SWIZZLE_REGION_SIZE &&
        m_dcparams.heightInBlocks[mip] >= BROTLIG_PRECON_SWIZZLE_REGION_SIZE) {
        uint32_t rem_width = mip_width % BROTLIG_PRECON_SWIZZLE_REGION_SIZE;
        uint32_t eff_width = mip_width - rem_width;
        uint32_t eff_height =
            m_dcparams.heightInBlocks[mip] - (m_dcparams.heightInBlocks[mip] % BROTLIG_PRECON_SWIZZLE_REGION_SIZE);

        if (row < eff_height && col < eff_width) {
            uint32_t eff_block = block - (row * rem_width);
            uint32_t blockGrp_width = eff_width / BROTLIG_PRECON_SWIZZLE_REGION_SIZE;
            uint32_t blockGrp = eff_block / (BROTLIG_PRECON_SWIZZLE_REGION_SIZE * BROTLIG_PRECON_SWIZZLE_REGION_SIZE);
            uint32_t blockInGrp = eff_block % (BROTLIG_PRECON_SWIZZLE_REGION_SIZE * BROTLIG_PRECON_SWIZZLE_REGION_SIZE);

            uint32_t oblockGrpRow = blockGrp / blockGrp_width;
            uint32_t oblockGrpCol = blockGrp % blockGrp_width;

            uint32_t oblockRowInGrp = blockInGrp / BROTLIG_PRECON_SWIZZLE_REGION_SIZE;
            uint32_t oblockColInGrp = blockInGrp % BROTLIG_PRECON_SWIZZLE_REGION_SIZE;

            row = BROTLIG_PRECON_SWIZZLE_REGION_SIZE * oblockGrpRow + oblockRowInGrp;
            col = BROTLIG_PRECON_SWIZZLE_REGION_SIZE * oblockGrpCol + oblockColInGrp;
        }
    }

    uint32_t block_pos = (row * mip_pitch) + (col * m_dcparams.blockSizeBytes);
    uint32_t byte_pos = localOffset % sbsize;

    return mip_pos + block_pos + sboffset + byte_pos;
}

void PageDecoder::DeltaDecode(size_t page_start, size_t page_end, uint8_t* data)
{
    if (!m_dcparams.precondition)
        return;

    uint32_t sub = 0;
    size_t color_start = 0, color_end = 0, p_sub_start = 0, p_sub_end = 0, p_sub_size = 0;
    for (uint32_t i = 0; i < m_dcparams.numColorSubBlocks; ++i) {
        sub = m_dcparams.colorSubBlocks[i];
        color_start = static_cast<size_t>(m_dcparams.subStreamOffsets[sub]);
        color_end = static_cast<size_t>(m_dcparams.subStreamOffsets[sub + 1]);

        if (OVERLAP(color_start, color_end, page_start, page_end)) {
            p_sub_start = (color_start > page_start) ? color_start - page_start : 0;
            p_sub_end = (color_end < page_end) ? color_end - page_start : page_end - page_start;
            p_sub_size = p_sub_end - p_sub_start;

            DeltaDecodeByte(p_sub_size, data + p_sub_start);
        }
    }
}

void PageDecoder::DeltaDecodeByte(size_t inSize, uint8_t* inData)
{
    if (inSize <= 1 || !inData)
        return;
    uint8_t prev = inData[0];
    for (size_t el = 1; el < inSize; ++el) {
        inData[el] += prev;
        prev = inData[el];
    }
}
