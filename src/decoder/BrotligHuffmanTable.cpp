// external/brotli_g_sdk/src/decoder/BrotligHuffmanTable.cpp
// Brotli-G SDK 1.1
//
// Copyright(c) 2022 - 2024 Advanced Micro Devices, Inc. All rights reserved.
// MIT License

#include "BrotligHuffmanTable.h"

#include "common/BrotligUtils.h"

using namespace BrotliG;

static const uint16_t FixedCodes[4][4] = {{0, 1, 0, 0}, {0, 2, 3, 0}, {0, 1, 2, 3}, {0, 2, 6, 7}};

static const uint16_t FixedCodelengths[4][4] = {{1, 1, 0, 0}, {1, 2, 2, 0}, {2, 2, 2, 2}, {1, 2, 3, 3}};

static const uint16_t DyanmicCodeLenReadOrder[BROTLI_CODE_LENGTH_CODES] = {1, 2, 3, 4,  0,  5,  17, 6,  16,
                                                                           7, 8, 9, 10, 11, 12, 13, 14, 15};

void GenerateHuffmanTable(uint16_t lens[], size_t size, uint16_t counts[], uint16_t next_code[],
                          uint16_t numcodelengths, uint16_t symbols[], uint16_t codelens[], size_t max_table_entries)
{
    if (!lens || !counts || !next_code || !symbols || !codelens || numcodelengths == 0)
        return;

    counts[0] = 0;
    for (uint16_t i = 1; i < numcodelengths; ++i) {
        next_code[i] = (next_code[i - 1] + counts[i - 1]) << 1;
    }

    uint16_t maxcodelength = numcodelengths - 1;
    for (size_t i = 0; i < size; ++i) {
        uint16_t codelen = lens[i];
        if (codelen == 0 || codelen >= numcodelengths)
            continue;

        uint16_t leftbits = maxcodelength - codelen;
        uint16_t code = next_code[codelen]++;
        uint16_t startIndex = code << leftbits;
        uint16_t symcount = uint16_t(1) << leftbits;

        if (startIndex >= max_table_entries)
            continue;
        if (startIndex + symcount > max_table_entries) {
            symcount = static_cast<uint16_t>(max_table_entries - startIndex);
        }

        uint16_t* symPtr = symbols + startIndex;
        uint16_t* lenPtr = codelens + startIndex;
        for (uint16_t k = 0; k < symcount; ++k) {
            symPtr[k] = static_cast<uint16_t>(i);
            lenPtr[k] = codelen;
        }
    }
}

void BrotliG::LoadHuffmanTable(BrotligDeswizzler& reader, size_t alphabet_size, uint16_t symbols[], uint16_t codelens[])
{
    if (alphabet_size == 0 || !symbols || !codelens)
        return;

    uint32_t max_bits = Log2Floor(static_cast<uint32_t>(alphabet_size - 1));
    uint32_t ttype = reader.ReadAndConsume(2);

    switch (ttype) {
    case 0: {
        reader.Consume(4);
        uint16_t symbol = (uint16_t)reader.ReadAndConsume(max_bits);

        for (size_t i = 0; i < BROTLIG_HUFFMAN_TABLE_SIZE; ++i) {
            symbols[i] = symbol;
            codelens[i] = 0;
        }

        reader.BSReset();
        break;
    }
    case 1: {
        uint32_t num_symbols = reader.ReadAndConsume(2) + 1;
        uint32_t tree_select = (num_symbols == 4) ? reader.ReadAndConsume(1) : 0;
        reader.Consume((num_symbols == 4) ? 1 : 2);

        size_t table_idx = num_symbols < 4 ? num_symbols - 2 : tree_select ? 3 : 2;
        for (size_t i = 0; i < num_symbols; ++i) {
            uint16_t codelen = FixedCodelengths[table_idx][i];
            uint16_t code = FixedCodes[table_idx][i];
            uint16_t symbol = (uint16_t)reader.ReadAndConsume(max_bits);

            uint16_t leftbits = BROTLIG_HUFFMAN_MAX_CODE_LENGTH - codelen;
            uint16_t startIndex = code << leftbits;
            uint16_t symcount = uint16_t(1) << leftbits;

            for (size_t k = 0; k < symcount && (startIndex + k) < BROTLIG_HUFFMAN_TABLE_SIZE; ++k) {
                symbols[startIndex + k] = symbol;
                codelens[startIndex + k] = codelen;
            }
            reader.BSSwitch();
        }

        reader.BSReset();
        break;
    }
    case 2: {
        uint32_t num_len_symbols = reader.ReadAndConsume(4) + 4;
        if (num_len_symbols > BROTLI_CODE_LENGTH_CODES)
            num_len_symbols = BROTLI_CODE_LENGTH_CODES;

        uint16_t len_sym_data[BROTLI_CODE_LENGTH_CODES] = {0};
        uint16_t len_blCounts[BROTLIG_HUFFMAN_NUM_CODE_LENGTH_CODE_LENGTH] = {0};
        uint16_t len_next_code[BROTLIG_HUFFMAN_NUM_CODE_LENGTH_CODE_LENGTH] = {0};

        for (uint32_t i = 0; i < num_len_symbols; ++i) {
            uint16_t codelen = reader.ReadAndConsume(5);
            if (codelen < BROTLIG_HUFFMAN_NUM_CODE_LENGTH_CODE_LENGTH) {
                len_sym_data[DyanmicCodeLenReadOrder[i]] = codelen;
                ++len_blCounts[codelen];
            }
            reader.BSSwitch();
        }

        uint16_t len_symbols[BROTLIG_HUFFMAN_CODE_LENGTH_TABLE_SIZE] = {0};
        uint16_t len_codelens[BROTLIG_HUFFMAN_CODE_LENGTH_TABLE_SIZE] = {0};

        GenerateHuffmanTable(len_sym_data, BROTLI_CODE_LENGTH_CODES, len_blCounts, len_next_code,
                             BROTLIG_HUFFMAN_NUM_CODE_LENGTH_CODE_LENGTH, len_symbols, len_codelens,
                             BROTLIG_HUFFMAN_CODE_LENGTH_TABLE_SIZE);

        reader.BSReset();

        uint16_t len_symbol = 0, prev_len_symbol = BROTLI_INITIAL_REPEATED_CODE_LENGTH;
        uint16_t len_code = 0, rev_len_code = 0;

        uint16_t data[BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE + 32] = {0};
        uint16_t blCounts[BROTLIG_HUFFMAN_NUM_CODE_LENGTH] = {0};
        uint16_t next_code[BROTLIG_HUFFMAN_NUM_CODE_LENGTH] = {0};

        size_t symbols_written = 0;
        while (symbols_written < alphabet_size) {
            len_code = static_cast<uint16_t>(reader.ReadNoConsume9());
            rev_len_code = sBrotligReverseBits9[len_code & 0x1FF];
            uint16_t clen = len_codelens[rev_len_code];
            if (clen == 0) {
                clen = 1;
            }
            reader.Consume(clen);
            len_symbol = len_symbols[rev_len_code];

            if (len_symbol == BROTLI_REPEAT_PREVIOUS_CODE_LENGTH) {
                uint32_t num_reps = reader.ReadAndConsume(2) + 3;
                if (symbols_written + num_reps > alphabet_size) {
                    num_reps = static_cast<uint32_t>(alphabet_size - symbols_written);
                }
                if (prev_len_symbol < BROTLIG_HUFFMAN_NUM_CODE_LENGTH) {
                    blCounts[prev_len_symbol] += num_reps;
                }
                while (num_reps-- && symbols_written < alphabet_size &&
                       symbols_written < (BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE + 32)) {
                    data[symbols_written++] = prev_len_symbol;
                }
            } else if (len_symbol == BROTLI_REPEAT_ZERO_CODE_LENGTH) {
                uint32_t num_reps = reader.ReadAndConsume(3) + 3;
                if (symbols_written + num_reps > alphabet_size) {
                    num_reps = static_cast<uint32_t>(alphabet_size - symbols_written);
                }
                blCounts[0] += num_reps;
                while (num_reps-- && symbols_written < alphabet_size &&
                       symbols_written < (BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE + 32)) {
                    data[symbols_written++] = 0;
                }
            } else {
                if (len_symbol < BROTLIG_HUFFMAN_NUM_CODE_LENGTH) {
                    prev_len_symbol = len_symbol;
                    ++blCounts[len_symbol];
                }
                if (symbols_written < alphabet_size && symbols_written < (BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE + 32)) {
                    data[symbols_written++] = len_symbol;
                }
            }
            reader.BSSwitch();
        }

        GenerateHuffmanTable(data, alphabet_size, blCounts, next_code, BROTLIG_HUFFMAN_NUM_CODE_LENGTH, symbols,
                             codelens, BROTLIG_HUFFMAN_TABLE_SIZE);

        reader.BSReset();
        break;
    }
    default:
        reader.BSReset();
        break;
    }
}
