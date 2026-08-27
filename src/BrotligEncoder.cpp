// external/brotli_g_sdk/src/BrotligEncoder.cpp
// Brotli-G SDK 1.1 (Brotli v1.2.0)
//
// Copyright(c) 2022 - 2024 Advanced Micro Devices, Inc. All rights reserved.
// MIT License

#include "BrotligEncoder.h"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <thread>

#include "BrotliG.h"
#include "DataStream.h"
#include "common/BrotligConstants.h"
#include "encoder/PageEncoder.h"

using namespace BrotliG;

uint32_t BROTLIG_API BrotliG::MaxCompressedSize(uint32_t input_size, bool precondition, bool deltaencode)
{
    uint32_t numPages = (input_size + BROTLIG_DEFAULT_PAGE_SIZE - 1) / (BROTLIG_DEFAULT_PAGE_SIZE);
    if (numPages == 0)
        numPages = 1;
    uint32_t compressedPagesSize = static_cast<uint32_t>(PageEncoder::MaxCompressedSize(BROTLIG_DEFAULT_PAGE_SIZE));
    uint32_t estimatedSize = (numPages * compressedPagesSize) + (numPages * BROTLIG_PAGE_HEADER_SIZE_BYTES) +
                             sizeof(StreamHeader) + (numPages * sizeof(uint32_t));

    if (precondition) {
        estimatedSize += sizeof(PreconditionHeader);
        if (deltaencode)
            estimatedSize += numPages * BROTLIG_PRECON_DELTA_ENCODING_BASES_SIZE_BYTES;
    }

    return estimatedSize + 65536;
}

BROTLIG_ERROR BROTLIG_API BrotliG::CheckParams(uint32_t page_size, BrotligDataconditionParams dcParams)
{
    if (page_size < BROTLIG_MIN_PAGE_SIZE)
        return BROTLIG_ERROR_MIN_PAGE_SIZE;

    if (page_size > BROTLIG_MAX_PAGE_SIZE)
        return BROTLIG_ERROR_MAX_PAGE_SIZE;

    if (dcParams.precondition)
        return dcParams.CheckParams();

    return BROTLIG_OK;
}

void EncodeWithPreconSinglethreaded(uint32_t input_size, const uint8_t* src, uint32_t* output_size, uint8_t*& output,
                                    uint32_t page_size, BrotligDataconditionParams& dcParams,
                                    BROTLIG_Feedback_Proc feedbackProc)
{
    uint8_t* srcConditioned = nullptr;
    uint32_t srcCondSize = 0;

    BrotliG::Condition(input_size, src, dcParams, srcCondSize, srcConditioned);

    uint32_t numPages = (srcCondSize + page_size - 1) / page_size;
    if (numPages == 0)
        numPages = 1;
    size_t maxOutPageSize = PageEncoder::MaxCompressedSize(page_size);
    uint8_t* tOutput = new uint8_t[maxOutPageSize * numPages];
    size_t* tOutpageSizes = new size_t[numPages];

    BrotligEncoderParams params = {BROTLI_MAX_QUALITY, BROTLI_MAX_WINDOW_BITS, page_size};

    PageEncoder pEncoder;
    pEncoder.Setup(params, &dcParams);

    uint32_t pageIndex = 0;
    uint32_t sizeLeftToRead = srcCondSize, sizeToRead = 0, curInOffset = 0, curOutOffset = 0;
    uint8_t* srcPtr = srcConditioned;
    uint8_t* outPtr = tOutput;

    while (pageIndex < numPages) {
        sizeToRead = (sizeLeftToRead > page_size) ? page_size : sizeLeftToRead;
        tOutpageSizes[pageIndex] = maxOutPageSize;
        pEncoder.Run(srcPtr, sizeToRead, curInOffset, outPtr, &tOutpageSizes[pageIndex], curOutOffset,
                     (pageIndex == numPages - 1));

        sizeLeftToRead -= sizeToRead;
        curInOffset += sizeToRead;
        curOutOffset += static_cast<uint32_t>(maxOutPageSize);
        ++pageIndex;
    }

    size_t tcompressedSize = 0;
    outPtr = output;

    StreamHeader header = {};
    header.SetId(BROTLIG_STREAM_ID);
    header.SetPageSize(page_size);
    header.SetUncompressedSize(input_size);
    header.SetPreconditioned(dcParams.precondition);
    size_t headersize = sizeof(StreamHeader);
    memcpy(outPtr, reinterpret_cast<char*>(&header), headersize);
    outPtr += headersize;
    tcompressedSize += headersize;

    if (dcParams.precondition) {
        PreconditionHeader preconHeader = {};
        preconHeader.Swizzled = dcParams.swizzle;
        preconHeader.PitchD3D12Aligned = dcParams.pitchd3d12aligned;
        preconHeader.WidthInBlocks = (dcParams.widthInBlocks[0] > 0) ? dcParams.widthInBlocks[0] - 1 : 0;
        preconHeader.HeightInBlocks = (dcParams.heightInBlocks[0] > 0) ? dcParams.heightInBlocks[0] - 1 : 0;
        preconHeader.SetDataFormat(dcParams.format);
        preconHeader.NumMips = (dcParams.numMipLevels > 0) ? dcParams.numMipLevels - 1 : 0;
        preconHeader.PitchInBytes = (dcParams.pitchInBytes[0] > 0) ? dcParams.pitchInBytes[0] - 1 : 0;

        size_t preconHeaderSize = sizeof(PreconditionHeader);
        memcpy(outPtr, reinterpret_cast<char*>(&preconHeader), preconHeaderSize);
        outPtr += preconHeaderSize;
        tcompressedSize += preconHeaderSize;
    }

    uint32_t* pageTable = reinterpret_cast<uint32_t*>(outPtr);
    size_t tablesize = numPages * sizeof(uint32_t);
    outPtr += tablesize;
    tcompressedSize += tablesize;
    srcPtr = tOutput;
    curInOffset = 0, curOutOffset = 0;

    for (size_t pindex = 0; pindex < numPages; ++pindex) {
        memcpy(outPtr + curOutOffset, srcPtr + curInOffset, tOutpageSizes[pindex]);
        pageTable[pindex] = curOutOffset;
        tcompressedSize += tOutpageSizes[pindex];
        curOutOffset += static_cast<uint32_t>(tOutpageSizes[pindex]);
        curInOffset += static_cast<uint32_t>(maxOutPageSize);
    }

    pageTable[0] = static_cast<uint32_t>(tOutpageSizes[numPages - 1]);

    delete[] tOutpageSizes;
    delete[] tOutput;
    delete[] srcConditioned;

    *output_size = static_cast<uint32_t>(tcompressedSize);
}

void EncodeNoPreconSinglethreaded(uint32_t input_size, const uint8_t* src, uint32_t* output_size, uint8_t*& output,
                                  uint32_t page_size, BROTLIG_Feedback_Proc feedbackProc)
{
    const uint8_t* srcPtr = src;
    uint32_t numPages = (input_size + page_size - 1) / page_size;
    if (numPages == 0)
        numPages = 1;

    size_t maxOutPageSize = PageEncoder::MaxCompressedSize(page_size);
    uint8_t* tOutput = new uint8_t[maxOutPageSize * numPages];
    size_t* tOutpageSizes = new size_t[numPages];

    BrotligEncoderParams params = {BROTLI_MAX_QUALITY, BROTLI_MAX_WINDOW_BITS, page_size};

    BrotligDataconditionParams dcParams = {};
    PageEncoder pEncoder;
    pEncoder.Setup(params, &dcParams);

    uint32_t pageIndex = 0;
    uint32_t sizeLeftToRead = input_size, sizeToRead = 0, curInOffset = 0, curOutOffset = 0;
    uint8_t* outPtr = tOutput;

    while (pageIndex < numPages) {
        sizeToRead = (sizeLeftToRead > page_size) ? page_size : sizeLeftToRead;
        tOutpageSizes[pageIndex] = maxOutPageSize;
        pEncoder.Run(srcPtr, sizeToRead, curInOffset, outPtr, &tOutpageSizes[pageIndex], curOutOffset,
                     (pageIndex == numPages - 1));

        sizeLeftToRead -= sizeToRead;
        curInOffset += sizeToRead;
        curOutOffset += static_cast<uint32_t>(maxOutPageSize);
        ++pageIndex;
    }

    size_t tcompressedSize = 0;
    outPtr = output;

    StreamHeader header = {};
    header.SetId(BROTLIG_STREAM_ID);
    header.SetPageSize(page_size);
    header.SetUncompressedSize(input_size);
    header.SetPreconditioned(dcParams.precondition);
    size_t headersize = sizeof(StreamHeader);
    memcpy(outPtr, reinterpret_cast<char*>(&header), headersize);
    outPtr += headersize;
    tcompressedSize += headersize;

    uint32_t* pageTable = reinterpret_cast<uint32_t*>(outPtr);
    size_t tablesize = numPages * sizeof(uint32_t);
    outPtr += tablesize;
    tcompressedSize += tablesize;
    srcPtr = tOutput;
    curInOffset = 0, curOutOffset = 0;

    for (size_t pindex = 0; pindex < numPages; ++pindex) {
        memcpy(outPtr + curOutOffset, srcPtr + curInOffset, tOutpageSizes[pindex]);
        pageTable[pindex] = curOutOffset;
        tcompressedSize += tOutpageSizes[pindex];
        curOutOffset += static_cast<uint32_t>(tOutpageSizes[pindex]);
        curInOffset += static_cast<uint32_t>(maxOutPageSize);
    }

    pageTable[0] = static_cast<uint32_t>(tOutpageSizes[numPages - 1]);

    delete[] tOutpageSizes;
    delete[] tOutput;

    *output_size = static_cast<uint32_t>(tcompressedSize);
}

BROTLIG_ERROR BrotliG::Encode(uint32_t input_size, const uint8_t* src, uint32_t* output_size, uint8_t*& output,
                              uint32_t page_size, BrotligDataconditionParams dcParams,
                              BROTLIG_Feedback_Proc feedbackProc)
{
    BROTLIG_ERROR status = BrotliG::CheckParams(page_size, dcParams);
    if (status != BROTLIG_OK)
        return status;

    if (dcParams.precondition) {
        if (!dcParams.Initialize(input_size)) {
            dcParams.precondition = false;
        }
    }

    if (dcParams.precondition)
        EncodeWithPreconSinglethreaded(input_size, src, output_size, output, page_size, dcParams, feedbackProc);
    else
        EncodeNoPreconSinglethreaded(input_size, src, output_size, output, page_size, feedbackProc);

    return BROTLIG_OK;
}
