// external/brotli_g_sdk/src/encoder/BrotligHuffman.cpp
// Brotli-G SDK 1.1
//
// Copyright(c) 2022 - 2024 Advanced Micro Devices, Inc. All rights reserved.
// MIT License

extern "C" {
#include "brotli/c/enc/entropy_encode.h"
}

#include "BrotligHuffman.h"
#include "common/BrotligConstants.h"
#include "common/BrotligUtils.h"

using namespace BrotliG;

struct BrotligHuffmanNode
{
    uint16_t symbol;
    uint32_t code;
    uint8_t depth;
    uint32_t total_count;
    BrotligHuffmanNode* left;
    BrotligHuffmanNode* right;

    BrotligHuffmanNode()
    {
        symbol = 0;
        code = 0;
        depth = 0;
        total_count = 0;
        left = nullptr;
        right = nullptr;
    }

    ~BrotligHuffmanNode()
    {
        symbol = 0;
        code = 0;
        depth = 0;
        total_count = 0;
        left = nullptr;
        right = nullptr;
    }
};

typedef BrotligHuffmanNode* BrotligHuffmanNodePtr;

class compare
{
public:
    bool operator()(const BrotligHuffmanNodePtr& c1, const BrotligHuffmanNodePtr& c2) const
    {
        if (c1->total_count != c2->total_count)
            return c1->total_count > c2->total_count;
        else
            return c1->symbol < c2->symbol;
    }
};

static bool TraverseTree(BrotligHuffmanNodePtr pNode, uint8_t depth)
{
    if (depth > BROTLIG_HUFFMAN_MAX_DEPTH)
        return false;

    pNode->depth = depth;

    bool ret = true;
    if (pNode->left)
        ret &= TraverseTree(pNode->left, depth + 1);
    if (pNode->right)
        ret &= TraverseTree(pNode->right, depth + 1);

    return ret;
}

static void DeleteTree(BrotligHuffmanNodePtr pNode)
{
    if (pNode == nullptr)
        return;

    DeleteTree(pNode->left);
    DeleteTree(pNode->right);

    delete pNode;
}

void BuildHuffman(uint32_t* hist, size_t alphabet_size, uint16_t codes[], uint8_t codelens[])
{
    size_t i = 0;
    BrotligHuffmanNodePtr temp = nullptr, root = nullptr, temp2 = nullptr;
    std::priority_queue<BrotligHuffmanNodePtr, std::vector<BrotligHuffmanNodePtr>, compare> pqueue;
    std::vector<BrotligHuffmanNodePtr> nodeList;

    for (uint32_t count_limit = 1;; count_limit *= 2) {
        for (i = 0; i < alphabet_size; ++i) {
            if (hist[i]) {
                temp = new BrotligHuffmanNode;
                temp->symbol = (uint16_t)i;
                temp->total_count = hist[i];

                nodeList.push_back(temp);
                pqueue.push(temp);
            }
        }

        if (pqueue.size() > 1) {
            while (pqueue.size() > 1) {
                temp = pqueue.top();
                pqueue.pop();

                temp2 = pqueue.top();
                pqueue.pop();

                root = new BrotligHuffmanNode;
                root->total_count = temp->total_count + temp2->total_count;
                root->left = temp;
                root->right = temp2;

                pqueue.push(root);
            }
        } else if (!pqueue.empty()) {
            root = pqueue.top();
            pqueue.pop();
        }

        if (root && TraverseTree(root, 0))
            break;

        DeleteTree(root);
        root = nullptr;
        while (!pqueue.empty())
            pqueue.pop();
        nodeList.clear();
        for (i = 0; i < alphabet_size; ++i)
            if (hist[i])
                hist[i] = std::max(hist[i], count_limit);
    }

    uint16_t blCounts[BROTLIG_HUFFMAN_NUM_CODE_LENGTH] = {0};
    uint16_t next_code[BROTLIG_HUFFMAN_NUM_CODE_LENGTH] = {0};

    for (auto node : nodeList)
        ++blCounts[node->depth];
    blCounts[0] = 0;
    for (uint16_t bits = 1; bits < BROTLIG_HUFFMAN_NUM_CODE_LENGTH; ++bits)
        next_code[bits] = (next_code[bits - 1] + blCounts[bits - 1]) << 1;

    for (auto node : nodeList) {
        codelens[node->symbol] = node->depth;
        codes[node->symbol] = BrotligReverseBits(node->depth, next_code[node->depth]++);
    }

    DeleteTree(root);
    while (!pqueue.empty())
        pqueue.pop();
    nodeList.clear();
}

static void StoreComplexHuffman(uint16_t codes[], uint8_t codelens[], size_t alphabet_size, BrotligSwizzler& writer)
{
    uint8_t rle_codes[BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE + 32],
        rle_extra_bits[BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE + 32];
    size_t num_rle_codes = 0, num_rle_extra_bits = 0;

    ComputeRLECodes(alphabet_size, codelens, rle_codes, num_rle_codes, rle_extra_bits, num_rle_extra_bits);

    size_t i = 0;
    uint32_t rle_hist[BROTLI_CODE_LENGTH_CODES] = {0};
    for (i = 0; i < num_rle_codes; ++i)
        ++rle_hist[rle_codes[i]];

    uint16_t rle_huffman_codes[BROTLI_CODE_LENGTH_CODES] = {0};
    uint8_t rle_huffman_codelens[BROTLI_CODE_LENGTH_CODES] = {0};

    BuildHuffman(rle_hist, BROTLI_CODE_LENGTH_CODES, rle_huffman_codes, rle_huffman_codelens);

    static const uint8_t kStorageOrder[BROTLI_CODE_LENGTH_CODES] = {1, 2, 3, 4,  0,  5,  17, 6,  16,
                                                                    7, 8, 9, 10, 11, 12, 13, 14, 15};

    for (i = 0; i < BROTLI_CODE_LENGTH_CODES; ++i) {
        uint8_t order = kStorageOrder[i];
        writer.Append(5, rle_huffman_codelens[order]);
        writer.BSSwitch();
    }

    writer.BSReset();

    for (i = 0; i < num_rle_codes; ++i) {
        uint8_t ix = rle_codes[i];
        writer.Append(rle_huffman_codelens[ix], rle_huffman_codes[ix]);

        switch (ix) {
        case BROTLI_REPEAT_PREVIOUS_CODE_LENGTH:
            writer.Append(2, rle_extra_bits[i]);
            break;
        case BROTLI_REPEAT_ZERO_CODE_LENGTH:
            writer.Append(3, rle_extra_bits[i]);
            break;
        default:
            break;
        }
        writer.BSSwitch();
    }
}

void BrotliG::BuildStoreHuffmanTable(uint32_t* hist, size_t alphabet_size, BrotligSwizzler& writer, uint16_t codes[],
                                     uint8_t codelens[])
{
    memset(codes, 0, sizeof(uint16_t) * alphabet_size);
    memset(codelens, 0, sizeof(uint8_t) * alphabet_size);

    size_t count = 0;
    size_t s4[4] = {0};
    for (size_t i = 0; i < alphabet_size; ++i) {
        if (hist[i]) {
            if (count < 4)
                s4[count] = i;
            else if (count > 4)
                break;
            ++count;
        }
    }

    size_t max_bits = 0;
    {
        size_t max_bits_counter = alphabet_size - 1;
        while (max_bits_counter) {
            max_bits_counter >>= 1;
            ++max_bits;
        }
    }

    if (count <= 1) {
        writer.Append(2, 0);
        writer.Append(2, 1);
        writer.Append(2, 0);
        writer.Append((uint32_t)max_bits, (uint32_t)s4[0]);
        writer.BSReset();

        codes[s4[0]] = 0;
        codelens[s4[0]] = 0;

        writer.BSReset();
        return;
    }

    memset(codelens, 0, sizeof(uint8_t) * alphabet_size);
    memset(codes, 0, sizeof(uint16_t) * alphabet_size);

    BuildHuffman(hist, alphabet_size, codes, codelens);

    if (count <= 4) {
        writer.Append(2, 1);
        writer.Append(2, (uint32_t)count - 1);

        for (size_t i = 0; i < count; ++i)
            for (size_t j = i + 1; j < count; ++j)
                if ((codelens[s4[j]] < codelens[s4[i]]) || (codelens[s4[j]] == codelens[s4[i]] && s4[j] < s4[i]))
                    BROTLI_SWAP(size_t, s4, j, i);

        uint8_t orig_codelen_s4_0 = codelens[s4[0]];

        // Clear and assign exact matching FixedCodes and FixedCodelengths
        memset(codes, 0, sizeof(uint16_t) * alphabet_size);
        memset(codelens, 0, sizeof(uint8_t) * alphabet_size);

        switch (count) {
        case 2:
            writer.Append(2, 0);
            writer.Append((uint32_t)max_bits, (uint32_t)s4[0]);
            writer.BSSwitch();
            writer.Append((uint32_t)max_bits, (uint32_t)s4[1]);
            writer.BSSwitch();

            codes[s4[0]] = 0;
            codelens[s4[0]] = 1;
            codes[s4[1]] = 1;
            codelens[s4[1]] = 1;
            break;
        case 3:
            writer.Append(2, 0);
            writer.Append((uint32_t)max_bits, (uint32_t)s4[0]);
            writer.BSSwitch();
            writer.Append((uint32_t)max_bits, (uint32_t)s4[1]);
            writer.BSSwitch();
            writer.Append((uint32_t)max_bits, (uint32_t)s4[2]);
            writer.BSSwitch();

            codes[s4[0]] = 0;
            codelens[s4[0]] = 1;
            codes[s4[1]] = 1;
            codelens[s4[1]] = 2; // binary 10 reversed = 01 (1)
            codes[s4[2]] = 3;
            codelens[s4[2]] = 2; // binary 11 reversed = 11 (3)
            break;
        case 4: {
            uint32_t tree_select = (orig_codelen_s4_0 == 1 ? 1 : 0);
            writer.Append(1, tree_select);
            writer.Append(1, 0);
            writer.Append((uint32_t)max_bits, (uint32_t)s4[0]);
            writer.BSSwitch();
            writer.Append((uint32_t)max_bits, (uint32_t)s4[1]);
            writer.BSSwitch();
            writer.Append((uint32_t)max_bits, (uint32_t)s4[2]);
            writer.BSSwitch();
            writer.Append((uint32_t)max_bits, (uint32_t)s4[3]);
            writer.BSSwitch();

            if (tree_select == 0) {
                codes[s4[0]] = 0;
                codelens[s4[0]] = 2;
                codes[s4[1]] = 2;
                codelens[s4[1]] = 2;
                codes[s4[2]] = 1;
                codelens[s4[2]] = 2;
                codes[s4[3]] = 3;
                codelens[s4[3]] = 2;
            } else {
                codes[s4[0]] = 0;
                codelens[s4[0]] = 1;
                codes[s4[1]] = 1;
                codelens[s4[1]] = 2;
                codes[s4[2]] = 3;
                codelens[s4[2]] = 3;
                codes[s4[3]] = 7;
                codelens[s4[3]] = 3;
            }
            break;
        }
        default:
            writer.BSReset();
            return;
        }

        writer.BSReset();
    } else {
        writer.Append(2, 2);
        writer.Append(4, BROTLI_CODE_LENGTH_CODES - 4);

        StoreComplexHuffman(codes, codelens, alphabet_size, writer);

        writer.BSReset();
    }
}
