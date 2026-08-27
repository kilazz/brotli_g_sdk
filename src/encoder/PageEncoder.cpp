// external/brotli_g_sdk/src/encoder/PageEncoder.cpp
// Brotli-G SDK 1.1 (Brotli v1.2.0)
//
// Copyright(c) 2022 - 2024 Advanced Micro Devices, Inc. All rights reserved.
// MIT License

extern "C" {
#include "brotli/c/enc/backward_references.h"
#include "brotli/c/enc/backward_references_hq.h"
#include "brotli/c/enc/bit_cost.h"
#include "brotli/c/enc/cluster.h"
#include "brotli/c/enc/command.h"
#include "brotli/c/enc/encoder_dict.h"
#include "brotli/c/enc/metablock.h"
#include "brotli/c/enc/params.h"
#include "brotli/c/enc/prefix.h"
#include "brotli/c/enc/quality.h"
#include "brotli/c/enc/utf8_util.h"
}

#include "PageEncoder.h"
#include "common/BrotligBitWriter.h"
#include "common/BrotligCommand.h"
#include "common/BrotligCommandLut.h"
#include "common/BrotligDataConditioner.h"
#include "common/BrotligUtils.h"
#include "encoder/BrotligHuffman.h"

#define MAX(a, b)               ((a > b) ? a : b)
#define MIN(a, b)               ((a < b) ? a : b)
#define OVERLAP(x1, x2, y1, y2) (x1 < y2 && y1 < x2)

using namespace BrotliG;

static inline uint32_t BrotligDecodeRawDistance(uint16_t dist_prefix, uint32_t dist_extra,
                                                const BrotliDistanceParams* dist)
{
    uint32_t dist_code = dist_prefix & 0x3FFu;
    if (dist->num_direct_distance_codes > 0 && dist_code < 16 + dist->num_direct_distance_codes) {
        return dist_code - 15;
    }
    uint32_t dist_extra_val = dist_extra & 0x00FFFFFFu;
    uint32_t ndistbits = 1 + ((dist_code - dist->num_direct_distance_codes - 16) >> (dist->distance_postfix_bits + 1));
    uint32_t hcode = (dist_code - dist->num_direct_distance_codes - 16) >> dist->distance_postfix_bits;
    uint32_t lcode = (dist_code - dist->num_direct_distance_codes - 16) & ((1U << dist->distance_postfix_bits) - 1U);
    uint32_t offset = ((2U + (hcode & 1U)) << ndistbits) - 4U;
    return ((offset + dist_extra_val) << dist->distance_postfix_bits) + lcode + dist->num_direct_distance_codes + 1U;
}

static ContextType ChooseContextMode(const BrotliEncoderParams* params, const uint8_t* data, const size_t pos,
                                     const size_t mask, const size_t length)
{
    if (params->quality >= MIN_QUALITY_FOR_HQ_BLOCK_SPLITTING &&
        !BrotliIsMostlyUTF8(data, pos, mask, length, kMinUTF8Ratio)) {
        return CONTEXT_SIGNED;
    }
    return CONTEXT_UTF8;
}

static BROTLI_BOOL ShouldCompress(const uint8_t* data, const size_t mask, const uint64_t last_flush_pos,
                                  const size_t bytes, const size_t num_literals, const size_t num_commands)
{
    if (bytes <= 2)
        return BROTLI_FALSE;
    if (num_commands < (bytes >> 8) + 2) {
        if (static_cast<double>(num_literals) > 0.99 * static_cast<double>(bytes)) {
            uint32_t literal_histo[256] = {0};
            static const uint32_t kSampleRate = 13;
            static const double kMinEntropy = 7.92;
            const double bit_cost_threshold = static_cast<double>(bytes) * kMinEntropy / kSampleRate;
            size_t t = (bytes + kSampleRate - 1) / kSampleRate;
            uint32_t pos = static_cast<uint32_t>(last_flush_pos);
            size_t i;
            for (i = 0; i < t; i++) {
                ++literal_histo[data[pos & mask]];
                pos += kSampleRate;
            }
            if (BrotliBitsEntropy(literal_histo, 256) > bit_cost_threshold) {
                return BROTLI_FALSE;
            }
        }
    }
    return BROTLI_TRUE;
}

static void BrotligCreateBackwardReferences(BrotligEncoderState* state, const uint8_t* input, size_t input_size,
                                            ContextLut literal_context_lut, bool isLast)
{
    state->commands_ = BROTLI_ALLOC(&state->memory_manager_, Command, (input_size * 2) + 4096);

    InitOrStitchToPreviousBlock(&state->memory_manager_, &state->hasher_, input, BROTLIG_INPUT_BIT_MASK, &state->params,
                                0, input_size, isLast);

    // Fast, exact backward references without Zopfli context-model command corruption
    BrotliCreateBackwardReferences(input_size, 0, input, BROTLIG_INPUT_BIT_MASK, literal_context_lut, &state->params,
                                   &state->hasher_, state->dist_cache_, &state->last_insert_len_, state->commands_,
                                   &state->num_commands_, &state->num_literals_);

    // Filter out completely empty placeholder commands (insert == 0 && copy == 0)
    size_t valid_cmds = 0;
    for (size_t i = 0; i < state->num_commands_; ++i) {
        if (state->commands_[i].insert_len_ > 0 || (state->commands_[i].copy_len_ & 0x1FFFFFF) > 0) {
            state->commands_[valid_cmds++] = state->commands_[i];
        }
    }
    state->num_commands_ = valid_cmds;

    int dist_ring[4] = {4, 11, 15, 16};
    size_t cur_pos = 0;

    for (size_t i = 0; i < state->num_commands_; ++i) {
        Command* c = &state->commands_[i];
        c->dist_extra_ &= 0x00FFFFFFu;

        uint32_t actual_insert_len = c->insert_len_;
        uint32_t actual_copy_len = c->copy_len_ & 0x1FFFFFF;

        cur_pos += actual_insert_len;

        // Convert invalid copy lengths (< 2) into literal insertions
        if (actual_copy_len < 2) {
            actual_insert_len += actual_copy_len;
            cur_pos += actual_copy_len;
            actual_copy_len = 0;
            c->insert_len_ = actual_insert_len;
            c->copy_len_ = 0;
            c->dist_prefix_ = 0;
            c->dist_extra_ = 0;
            uint16_t inscode = GetInsertLengthCode(actual_insert_len);
            if (inscode > 23)
                inscode = 23;
            c->cmd_prefix_ = static_cast<uint16_t>(BROTLI_NUM_COMMAND_SYMBOLS) + inscode;
            continue;
        }

        uint32_t dist_code = 0;
        if (c->cmd_prefix_ < 128) {
            dist_code = 0;
            c->dist_prefix_ = 0;
            c->dist_extra_ = 0;
        } else {
            dist_code = c->dist_prefix_ & 0x3FFu;
        }

        uint32_t raw_dist = 0;
        if (dist_code == 0) {
            raw_dist = dist_ring[0];
        } else if (dist_code == 1) {
            raw_dist = dist_ring[1];
        } else if (dist_code == 2) {
            raw_dist = dist_ring[2];
        } else if (dist_code == 3) {
            raw_dist = dist_ring[3];
        } else if (dist_code == 4) {
            raw_dist = dist_ring[0] - 1;
        } else if (dist_code == 5) {
            raw_dist = dist_ring[0] + 1;
        } else if (dist_code == 6) {
            raw_dist = dist_ring[0] - 2;
        } else if (dist_code == 7) {
            raw_dist = dist_ring[0] + 2;
        } else if (dist_code == 8) {
            raw_dist = dist_ring[0] - 3;
        } else if (dist_code == 9) {
            raw_dist = dist_ring[0] + 3;
        } else if (dist_code == 10) {
            raw_dist = dist_ring[1] - 1;
        } else if (dist_code == 11) {
            raw_dist = dist_ring[1] + 1;
        } else if (dist_code == 12) {
            raw_dist = dist_ring[1] - 2;
        } else if (dist_code == 13) {
            raw_dist = dist_ring[1] + 2;
        } else if (dist_code == 14) {
            raw_dist = dist_ring[1] - 3;
        } else if (dist_code == 15) {
            raw_dist = dist_ring[1] + 3;
        } else {
            raw_dist = BrotligDecodeRawDistance(c->dist_prefix_, c->dist_extra_, &state->params.dist);
        }

        // Unconditionally update distance ring buffer to match standard Brotli LRU state
        if (dist_code == 1) {
            int temp = dist_ring[1];
            dist_ring[1] = dist_ring[0];
            dist_ring[0] = temp;
        } else if (dist_code == 2) {
            int temp = dist_ring[2];
            dist_ring[2] = dist_ring[1];
            dist_ring[1] = dist_ring[0];
            dist_ring[0] = temp;
        } else if (dist_code == 3) {
            int temp = dist_ring[3];
            dist_ring[3] = dist_ring[2];
            dist_ring[2] = dist_ring[1];
            dist_ring[1] = dist_ring[0];
            dist_ring[0] = temp;
        } else if (dist_code >= 4) {
            dist_ring[3] = dist_ring[2];
            dist_ring[2] = dist_ring[1];
            dist_ring[1] = dist_ring[0];
            dist_ring[0] = raw_dist;
        }

        // Convert distances pointing outside current page or to static dictionary into literal insertions
        if (raw_dist == 0 || raw_dist > cur_pos) {
            actual_insert_len += actual_copy_len;
            cur_pos += actual_copy_len;
            actual_copy_len = 0;
            c->insert_len_ = actual_insert_len;
            c->copy_len_ = 0;
            c->dist_prefix_ = 0;
            c->dist_extra_ = 0;
            uint16_t inscode = GetInsertLengthCode(actual_insert_len);
            if (inscode > 23)
                inscode = 23;
            c->cmd_prefix_ = static_cast<uint16_t>(BROTLI_NUM_COMMAND_SYMBOLS) + inscode;
            continue;
        } else {
            if (c->cmd_prefix_ < 128) {
                c->dist_prefix_ = 0;
                c->dist_extra_ = 0;
            }
            cur_pos += actual_copy_len;
        }

        c->copy_len_ = actual_copy_len;

        if (actual_copy_len == 0) {
            uint16_t inscode = GetInsertLengthCode(actual_insert_len);
            if (inscode > 23)
                inscode = 23;
            c->cmd_prefix_ = static_cast<uint16_t>(BROTLI_NUM_COMMAND_SYMBOLS) + inscode;
        } else {
            uint16_t inscode = GetInsertLengthCode(actual_insert_len);
            uint16_t copycode = GetCopyLengthCode(actual_copy_len);
            c->cmd_prefix_ = CombineLengthCodes(inscode, copycode, BROTLI_FALSE);
        }
    }

    size_t endPos = 0;
    for (size_t i = 0; i < state->num_commands_; ++i) {
        endPos += state->commands_[i].insert_len_ + (state->commands_[i].copy_len_ & 0x1FFFFFF);
    }

    if (endPos < input_size) {
        size_t litsLeft = input_size - endPos;

        Command extra = {};
        extra.insert_len_ = static_cast<uint32_t>(litsLeft);
        extra.copy_len_ = 0;
        extra.dist_prefix_ = 0;
        extra.dist_extra_ = 0;

        uint16_t inscode = GetInsertLengthCode(extra.insert_len_);
        if (inscode > 23)
            inscode = 23;
        extra.cmd_prefix_ = static_cast<uint16_t>(BROTLI_NUM_COMMAND_SYMBOLS) + inscode;

        state->commands_[state->num_commands_++] = extra;
        state->num_literals_ += litsLeft;
    }

    Command sentinel = {};
    sentinel.cmd_prefix_ = BROTLI_NUM_COMMAND_SYMBOLS; // 704
    state->commands_[state->num_commands_++] = sentinel;
}

static bool StoreUncompressed(size_t inputSize, const uint8_t* input, size_t* outputSize, uint8_t* output)
{
    memset(output, 0, *outputSize);
    memcpy(output, input, inputSize);
    *outputSize = inputSize;
    return true;
}

PageEncoder::PageEncoder()
{
    m_state = nullptr;
    m_dcparams = nullptr;
    m_pWriter = nullptr;
}

PageEncoder::~PageEncoder()
{
    Cleanup();
}

bool PageEncoder::Setup(BrotligEncoderParams& params, BrotligDataconditionParams* dcparams)
{
    m_params = params;
    m_dcparams = dcparams;
    return true;
}

bool PageEncoder::Run(const uint8_t* input, size_t inputSize, size_t inputOffset, uint8_t* output, size_t* outputSize,
                      size_t outputOffset, bool isLast)
{
    bool Isdeltaencoded = false;
    uint8_t* pageCopy = nullptr;

    const uint8_t* p_inPtr = input + inputOffset;
    size_t inSize = inputSize;
    if (m_dcparams && m_dcparams->precondition && m_dcparams->delta_encode) {
        pageCopy = new uint8_t[inSize];
        memcpy(pageCopy, input + inputOffset, inSize);

        Isdeltaencoded = DeltaEncode(inputOffset, inputOffset + inputSize, pageCopy);

        if (Isdeltaencoded)
            p_inPtr = pageCopy;
        else {
            delete[] pageCopy;
            pageCopy = nullptr;
        }
    }

    uint8_t* p_outPtr = output + outputOffset;

    m_state = new BrotligEncoderState(0, 0, 0);
    if (!m_state) {
        if (pageCopy)
            delete[] pageCopy;
        return false;
    }

    m_state->SetParameter(BROTLI_PARAM_QUALITY, static_cast<uint32_t>(m_params.quality));
    m_state->SetParameter(BROTLI_PARAM_LGWIN, static_cast<uint32_t>(m_params.lgwin));
    m_state->SetParameter(BROTLI_PARAM_MODE, static_cast<uint32_t>(m_params.mode));
    m_state->SetParameter(BROTLI_PARAM_SIZE_HINT, static_cast<uint32_t>(inSize));

    if (m_params.lgwin > BROTLI_MAX_WINDOW_BITS) {
        m_state->SetParameter(BROTLI_PARAM_LARGE_WINDOW, BROTLI_TRUE);
    }

    if (!m_state->EnsureInitialized()) {
        delete m_state;
        m_state = nullptr;
        if (pageCopy)
            delete[] pageCopy;
        return false;
    }

    m_state->params.dist.distance_postfix_bits = 0;
    m_state->params.dist.num_direct_distance_codes = 0;

    ContextType literal_context_mode = ChooseContextMode(&m_state->params, p_inPtr, 0, BROTLIG_INPUT_BIT_MASK, inSize);

    ContextLut literal_context_lut = BROTLI_CONTEXT_LUT(literal_context_mode);

    BrotligCreateBackwardReferences(m_state, p_inPtr, inSize, literal_context_lut, isLast);

    if (!ShouldCompress(p_inPtr, BROTLIG_INPUT_BIT_MASK, 0, inSize, m_state->num_literals_, m_state->num_commands_)) {
        delete m_state;
        m_state = nullptr;
        if (pageCopy)
            delete[] pageCopy;
        return StoreUncompressed(inputSize, input + inputOffset, outputSize, p_outPtr);
    }

    memset(m_histDistances, 0, sizeof(m_histDistances));
    memset(m_histCommands, 0, sizeof(m_histCommands));
    memset(m_histLiterals, 0, sizeof(m_histLiterals));

    size_t cmdIndex = 0, pos = 0, numbitstreams = m_params.num_bitstreams;
    Command cmd;
    uint32_t insertLen = 0, distContext = 0;

    size_t litQueueCap = (inSize * 2) + 131072;
    uint8_t* litqueue = new uint8_t[litQueueCap];
    uint8_t* litqfront = litqueue;
    uint8_t* litqback = litqueue;

    HistogramDistance distCtxHists[BROTLIG_NUM_DIST_CONTEXT_HISTOGRAMS];
    ClearHistogramsDistance(distCtxHists, BROTLIG_NUM_DIST_CONTEXT_HISTOGRAMS);

    while (cmdIndex < m_state->num_commands_) {
        cmd = m_state->commands_[cmdIndex++];
        if (cmd.cmd_prefix_ < BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE) {
            ++m_histCommands[cmd.cmd_prefix_];
        }
        if (cmd.cmd_prefix_ >= 128 && cmd.cmd_prefix_ < BROTLI_NUM_COMMAND_SYMBOLS) {
            distContext = CommandDistanceContext(&cmd);
            uint16_t dsym = cmd.dist_prefix_ & 0x3FF;
            if (dsym < BROTLIG_NUM_DISTANCE_SYMBOLS) {
                HistogramAddDistance(&distCtxHists[distContext % BROTLIG_NUM_DIST_CONTEXT_HISTOGRAMS], dsym);
            }
        }
        insertLen = cmd.insert_len_;
        while (insertLen-- && pos < inSize) {
            ++m_histLiterals[p_inPtr[pos]];
            if (litqback < litqueue + litQueueCap) {
                *litqback++ = p_inPtr[pos++];
            } else {
                pos++;
            }
        }
        pos += (cmd.copy_len_ & 0x1FFFFFF);
    }

    HistogramDistance out[BROTLIG_NUM_DIST_CONTEXT_HISTOGRAMS];
    size_t num_out = 0;
    uint32_t dist_context_map[BROTLIG_NUM_DIST_CONTEXT_HISTOGRAMS];
    BrotliClusterHistogramsDistance(&m_state->memory_manager_, distCtxHists, BROTLIG_NUM_DIST_CONTEXT_HISTOGRAMS,
                                    BROTLIG_MAX_NUM_DIST_HISTOGRAMS, out, &num_out, dist_context_map);

    memcpy(m_histDistances, out[0].data_, sizeof(out[0].data_));

    uint8_t mostFreqLit = static_cast<uint8_t>(
        std::max_element(m_histLiterals, m_histLiterals + BROTLI_NUM_LITERAL_SYMBOLS) - m_histLiterals);

    memset(p_outPtr, 0, *outputSize);
    BrotligBitWriterLSB bw;
    bw.SetStorage(p_outPtr);
    bw.SetPosition(0);

    m_pWriter = new BrotligSwizzler(m_params.num_bitstreams, m_params.page_size * 2 + 65536);
    m_pWriter->SetOutWriter(&bw, *outputSize);

    BuildStoreHuffmanTable(m_histCommands, BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE, *m_pWriter, m_cmdCodes,
                           m_cmdCodelens);

    BuildStoreHuffmanTable(m_histDistances, BROTLIG_NUM_DISTANCE_SYMBOLS, *m_pWriter, m_distCodes, m_distCodelens);

    BuildStoreHuffmanTable(m_histLiterals, BROTLI_NUM_LITERAL_SYMBOLS, *m_pWriter, m_litCodes, m_litCodelens);

    Command sentinel = {};
    sentinel.cmd_prefix_ = BROTLI_NUM_COMMAND_SYMBOLS; // 704

    size_t bsindex = 0;
    bool sentinelFound = false;

    size_t nRounds = (m_state->num_commands_ + numbitstreams - 1) / numbitstreams;
    Command* cqfront = m_state->commands_;
    Command* cqend = m_state->commands_ + m_state->num_commands_;
    size_t litcount = 0, aclitcount = 0, mult = 0, rlitcount = 0, prev_tail = 0;

    while (nRounds--) {
        bsindex = 0;
        litcount = 0;

        while (bsindex < numbitstreams) {
            if (cqfront < cqend && !sentinelFound) {
                cmd = *cqfront++;

                if (cmd.cmd_prefix_ == BROTLI_NUM_COMMAND_SYMBOLS) {
                    sentinelFound = true;
                } else {
                    litcount += cmd.insert_len_;
                }

                if (cmd.cmd_prefix_ < BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE) {
                    StoreCommand(cmd);
                }

                if (cmd.cmd_prefix_ >= 128 && cmd.cmd_prefix_ < BROTLI_NUM_COMMAND_SYMBOLS) {
                    StoreDistance(cmd.dist_prefix_, cmd.dist_extra_);
                }
            } else {
                StoreCommand(sentinel);
            }

            ++bsindex;
            m_pWriter->BSSwitch();
        }

        m_pWriter->BSReset();

        aclitcount = (litcount > prev_tail) ? litcount - prev_tail : 0;
        mult = (aclitcount + numbitstreams - 1) / numbitstreams;
        rlitcount = numbitstreams * mult;
        prev_tail = rlitcount + prev_tail - litcount;

        while (rlitcount--) {
            if (litqfront < litqback) {
                StoreLiteral(*litqfront++);
            } else {
                StoreLiteral(mostFreqLit);
            }
            m_pWriter->BSSwitch();
        }

        m_pWriter->BSReset();

        if (sentinelFound && cqfront >= cqend) {
            break;
        }
    }

    delete[] litqueue;

    m_pWriter->AppendToHeader(BROTLIG_PAGE_HEADER_NPOSTFIX_BITS, 0);
    m_pWriter->AppendToHeader(BROTLIG_PAGE_HEADER_NDIST_BITS, 0);
    m_pWriter->AppendToHeader(BROTLIG_PAGE_HEADER_ISDELTAENCODED_BITS, Isdeltaencoded ? 1 : 0);
    m_pWriter->AppendToHeader(BROTLIG_PAGE_HEADER_RESERVED_BITS, 0);

    m_pWriter->AppendBitstreamSizes();
    m_pWriter->SerializeHeader();
    m_pWriter->SerializeBitstreams();

    if (pageCopy)
        delete[] pageCopy;

    delete m_pWriter;
    m_pWriter = nullptr;
    delete m_state;
    m_state = nullptr;

    size_t newsize = (bw.GetPosition() + 8 - 1) / 8;

    if (newsize >= inputSize)
        return StoreUncompressed(inputSize, input + inputOffset, outputSize, p_outPtr);
    else {
        *outputSize = newsize;
        return true;
    }
}

bool PageEncoder::DeltaEncode(size_t page_start, size_t page_end, uint8_t* data)
{
    if (!m_dcparams)
        return false;
    uint32_t sub = 0;
    size_t color_start = 0, color_end = 0, p_sub_start = 0, p_sub_end = 0, p_sub_size = 0;
    bool iseconded = false;
    for (uint32_t i = 0; i < m_dcparams->numColorSubBlocks; ++i) {
        sub = m_dcparams->colorSubBlocks[i];
        color_start = static_cast<size_t>(m_dcparams->subStreamOffsets[sub]);
        color_end = static_cast<size_t>(m_dcparams->subStreamOffsets[sub + 1]);

        if (OVERLAP(color_start, color_end, page_start, page_end)) {
            p_sub_start = (color_start > page_start) ? color_start - page_start : 0;
            p_sub_end = (color_end < page_end) ? color_end - page_start : page_end - page_start;
            p_sub_size = p_sub_end - p_sub_start;

            DeltaEncodeByte(p_sub_size, data + p_sub_start);
            iseconded |= true;
        }
    }

    return iseconded;
}

void PageEncoder::DeltaEncodeByte(size_t inSize, uint8_t* inData)
{
    if (inSize <= 1 || !inData)
        return;
    uint8_t ref = inData[0];
    uint8_t prev = ref, cur = 0;
    for (size_t el = 1; el < inSize; ++el) {
        cur = inData[el];
        inData[el] -= prev;
        prev = cur;
    }
}

void PageEncoder::StoreCommand(Command& cmd)
{
    if (cmd.cmd_prefix_ >= BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE)
        return;
    uint16_t nbits = m_cmdCodelens[cmd.cmd_prefix_];
    m_pWriter->Append(nbits, m_cmdCodes[cmd.cmd_prefix_]);

    if (cmd.cmd_prefix_ < BROTLI_NUM_COMMAND_SYMBOLS) // 0..703
    {
        BrotligCmdLutElement clut = sBrotligCmdLut[cmd.cmd_prefix_];
        uint32_t insnumextra = clut.insert_len_extra_bits;
        uint32_t copynumextra = clut.copy_len_extra_bits;
        uint64_t insextraval = cmd.insert_len_ - clut.insert_len_offset;
        uint64_t copyextraval = ((cmd.copy_len_ & 0x1FFFFFF) >= clut.copy_len_offset)
                                  ? ((cmd.copy_len_ & 0x1FFFFFF) - clut.copy_len_offset)
                                  : 0;
        uint64_t bits = (copyextraval << insnumextra) | insextraval;
        m_pWriter->Append(insnumextra + copynumextra, bits);
    } else if (cmd.cmd_prefix_ == BROTLI_NUM_COMMAND_SYMBOLS) // 704 = EOS Sentinel
    {
        // Sentinel has 0 extra bits
    } else // 705..728 = Insert-only commands
    {
        uint16_t inscode = cmd.cmd_prefix_ - BROTLI_NUM_COMMAND_SYMBOLS;
        uint32_t insnumextra = GetInsertExtra(inscode);
        uint64_t insextraval = cmd.insert_len_ - GetInsertBase(inscode);
        m_pWriter->Append(insnumextra, insextraval);
    }
}

void PageEncoder::StoreLiteral(uint8_t literal)
{
    uint16_t nbits = m_litCodelens[literal];
    m_pWriter->Append(nbits, m_litCodes[literal]);
}

void PageEncoder::StoreDistance(uint16_t dist_prefix, uint32_t distextra)
{
    uint16_t dist_code = dist_prefix & 0x3FF;
    if (dist_code >= BROTLIG_NUM_DISTANCE_SYMBOLS)
        return;
    uint16_t nbits = m_distCodelens[dist_code];
    uint32_t distnumextra = (dist_code < 16) ? 0 : (1 + ((dist_code - 16) >> 1));
    m_pWriter->Append(nbits, m_distCodes[dist_code]);
    if (distnumextra > 0) {
        m_pWriter->Append(distnumextra, distextra & 0x00FFFFFFu);
    }
}

void PageEncoder::Cleanup()
{
    if (m_state != nullptr) {
        delete m_state;
        m_state = nullptr;
    }
    if (m_pWriter != nullptr) {
        delete m_pWriter;
        m_pWriter = nullptr;
    }
}