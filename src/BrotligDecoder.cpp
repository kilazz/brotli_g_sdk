// external/brotli_g_sdk/src/BrotligDecoder.cpp
// Brotli-G SDK 1.1
//
// Copyright(c) 2022 - 2024 Advanced Micro Devices, Inc. All rights reserved.
// MIT License

#include "BrotligDecoder.h"

#include <string.h>

#include "BrotliG.h"
#include "DataStream.h"
#include "decoder/PageDecoder.h"

using namespace BrotliG;

void DecodeCPUNoPreconSingleThread(uint32_t input_size, const uint8_t* src, BrotligDecoderParams& params,
                                   uint32_t numPages, uint32_t lastPageSize, uint32_t output_size, uint8_t* output,
                                   BROTLIG_Feedback_Proc feedbackProc)
{
    const uint8_t* srcPtr = src;
    uint8_t* outPtr = output;

    const uint32_t* pageTable = reinterpret_cast<const uint32_t*>(srcPtr);
    srcPtr += numPages * sizeof(uint32_t);

    BrotligDataconditionParams dcParams = {};

    PageDecoder pDecoder;
    pDecoder.Setup(params, dcParams);

    uint32_t pageIndex = 0;
    uint32_t curInOffset = 0, curOutOffset = 0;
    size_t inPageSize = 0, outPageSize = 0;

    while (pageIndex < numPages) {
        curInOffset = (pageIndex == 0) ? 0 : pageTable[pageIndex];
        inPageSize = (pageIndex < numPages - 1) ? (pageTable[pageIndex + 1] - curInOffset) : pageTable[0];

        curOutOffset = pageIndex * static_cast<uint32_t>(params.page_size);
        outPageSize = ((pageIndex == numPages - 1) && (lastPageSize != 0)) ? lastPageSize : params.page_size;

        pDecoder.Run(srcPtr, inPageSize, curInOffset, outPtr, outPageSize, curOutOffset);

        if (feedbackProc) {
            feedbackProc(static_cast<BROTLIG_MESSAGE_TYPE>(0), "");
        }

        ++pageIndex;
    }
}

void DecodeCPUWithPreconSingleThread(uint32_t input_size, const uint8_t* src, BrotligDecoderParams& params,
                                     uint32_t numPages, uint32_t lastPageSize, uint32_t output_size, uint8_t* output,
                                     BrotligDataconditionParams& dcParams, BROTLIG_Feedback_Proc feedbackProc)
{
    const uint8_t* srcPtr = src;
    uint8_t* outPtr = output;

    // Properly parse and advance past PreconditionHeader
    const PreconditionHeader* pHeader = reinterpret_cast<const PreconditionHeader*>(srcPtr);
    dcParams.swizzle = (pHeader->Swizzled != 0);
    dcParams.pitchd3d12aligned = (pHeader->PitchD3D12Aligned != 0);
    dcParams.format = pHeader->DataFormat();
    dcParams.numMipLevels = pHeader->NumMips + 1;
    dcParams.widthInBlocks[0] = pHeader->WidthInBlocks + 1;
    dcParams.heightInBlocks[0] = pHeader->HeightInBlocks + 1;
    dcParams.pitchInBytes[0] = pHeader->PitchInBytes + 1;
    dcParams.Initialize(output_size);

    srcPtr += sizeof(PreconditionHeader);

    const uint32_t* pageTable = reinterpret_cast<const uint32_t*>(srcPtr);
    srcPtr += numPages * sizeof(uint32_t);

    PageDecoder pDecoder;
    pDecoder.Setup(params, dcParams);

    uint32_t pageIndex = 0;
    uint32_t curInOffset = 0, curOutOffset = 0;
    size_t inPageSize = 0, outPageSize = 0;

    while (pageIndex < numPages) {
        curInOffset = (pageIndex == 0) ? 0 : pageTable[pageIndex];
        inPageSize = (pageIndex < numPages - 1) ? (pageTable[pageIndex + 1] - curInOffset) : pageTable[0];

        curOutOffset = pageIndex * static_cast<uint32_t>(params.page_size);
        outPageSize = ((pageIndex == numPages - 1) && (lastPageSize != 0)) ? lastPageSize : params.page_size;

        pDecoder.Run(srcPtr, inPageSize, curInOffset, outPtr, outPageSize, curOutOffset);

        if (feedbackProc) {
            feedbackProc(static_cast<BROTLIG_MESSAGE_TYPE>(0), "");
        }

        ++pageIndex;
    }
}

BROTLIG_ERROR DecodeCPUSingleThreaded(uint32_t input_size, const uint8_t* src, uint32_t* output_size, uint8_t* output,
                                      BROTLIG_Feedback_Proc feedbackProc)
{
    if (!src || !output || !output_size || input_size < sizeof(StreamHeader)) {
        return BROTLIG_ERROR_CORRUPT_STREAM;
    }

    const uint8_t* srcPtr = src;
    uint32_t srcSize = input_size;

    const StreamHeader* sHeader = reinterpret_cast<const StreamHeader*>(srcPtr);
    if (!sHeader->Validate()) {
        return BROTLIG_ERROR_CORRUPT_STREAM;
    }

    if (sHeader->Id != BROTLIG_STREAM_ID) {
        return BROTLIG_ERROR_INCORRECT_STREAM_FORMAT;
    }

    uint32_t lastPageSize = sHeader->LastPageSize;
    uint32_t numPages = sHeader->NumPages;
    uint32_t outSize = static_cast<uint32_t>(sHeader->UncompressedSize());

    if (numPages == 0 || outSize == 0) {
        return BROTLIG_ERROR_CORRUPT_STREAM;
    }

    memset(output, 0, *output_size);

    BrotligDecoderParams params = {};
    params.num_bitstreams = BROLTIG_DEFAULT_NUM_BITSTREAMS;
    params.page_size = static_cast<uint32_t>(sHeader->PageSize());

    BrotligDataconditionParams dcParams = {};
    dcParams.precondition = sHeader->IsPreconditioned();

    srcPtr += sizeof(StreamHeader);
    srcSize -= sizeof(StreamHeader);

    if (dcParams.precondition) {
        DecodeCPUWithPreconSingleThread(srcSize, srcPtr, params, numPages, lastPageSize, outSize, output, dcParams,
                                        feedbackProc);
    } else {
        DecodeCPUNoPreconSingleThread(srcSize, srcPtr, params, numPages, lastPageSize, outSize, output, feedbackProc);
    }

    *output_size = outSize;
    return BROTLIG_OK;
}

BROTLIG_ERROR BrotliG::DecodeCPU(uint32_t input_size, const uint8_t* src, uint32_t* output_size, uint8_t* output,
                                 BROTLIG_Feedback_Proc feedbackProc)
{
    return DecodeCPUSingleThreaded(input_size, src, output_size, output, feedbackProc);
}

uint32_t BrotliG::DecompressedSize(uint8_t* src)
{
    if (!src)
        return 0;
    const StreamHeader* sHeader = reinterpret_cast<const StreamHeader*>(src);
    if (!sHeader->Validate()) {
        return 0;
    }

    if (sHeader->Id != BROTLIG_STREAM_ID) {
        return 0;
    }

    return static_cast<uint32_t>(sHeader->UncompressedSize());
}
