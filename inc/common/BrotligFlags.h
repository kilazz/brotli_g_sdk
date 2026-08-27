// external/brotli_g_sdk/inc/common/BrotligFlags.h
// Brotli-G SDK 1.1
//
// Copyright(c) 2022 - 2024 Advanced Micro Devices, Inc. All rights reserved.
// MIT License

#pragma once

// Header flags
#define PAD_HEADER                  0
#define LGWIN_FIELD                 0
#define ISLAST_FLAG                 0
#define ISEMPTY_FLAG                0
#define ISUNCOMPRESSED_FLAG         0
#define DIST_POSTFIX_BITS_FIELD     1
#define NUM_DIRECT_DIST_CODES_FIELD 1
#define RESERVE_BITS                1
#define UNCOMPLEN_FIELD             0
#define PREAMBLE                    0
#define BS_SIZES_STORE              1

// Command generation flags
#define USE_INSERT_ONLY_COMMANDS 1 // 0 - off, 1 - on

// Stream serialization flags
#define USE_COMPACT_SERIALIZTION 1 // 0 - off, 1 - on
#define REDISTRIBUTE_LITERALS    1 // 0 - off, 1 - on

// Display flags: default off
#define SHOW_PROGRESS 0 // 0 - off, 1 - on

// Multithread flags - MUST BE 0 for thread safety under external threadpools (Rayon)
#define BROTLIG_ENCODER_MULTITHREADING_MODE     0 // 0 - single threaded, 1 - multi-threaded
#define BROTLIG_CPU_DECODER_MULTITHREADING_MODE 0 // 0 - single threaded, 1 - multi-threaded
