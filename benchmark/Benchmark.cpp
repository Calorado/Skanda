/*
MIT License

Copyright (c) 2023-2026 Carlos de Diego

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#if defined(_MSC_VER)
    #define VC_EXTRALEAN
    #define WIN32_LEAN_AND_MEAN
    #include <Windows.h>
    #define SET_REALTIME_PRIO SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS)
    #undef min
    #undef max
#else
    #include <sys/time.h>
    #include <sys/resource.h>
    #define SET_REALTIME_PRIO setpriority(PRIO_PROCESS, 0, -20)
#endif

#include "lib/threadpool/threadpool.h"
#include "lib/parallel_hashmap/btree.h"
#include "lib/cachepool.h"

#include <fstream>
#include <string>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <filesystem>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <iomanip>
#include <vector>
#include <cmath>
#include <random>
#include <atomic>
#include <future>
#include <new>

using namespace std;

/*************************************************************
 *                         HELPERS                           *
 ************************************************************/

#define NO_SPECTRUM false
#define NO_STRIDER false
#define NO_SKANDA false
#define NO_LZ4 false
#define NO_LIZARD true
#define NO_ZSTD false
#define NO_LIBDEFLATE false
#define NO_LZMA false
#define NO_LZHAM false
#define NO_LZAV false
#define NO_BROTLI false

cachepool::VariableCachePool<std::mutex> cachePool(1024 * 65536);
dp::thread_pool globalThreadPool(4);

size_t compress_bound(size_t inSize) {
    return inSize + inSize / 16 + 1024;
}

/*************************************************************
 *                         SPECTRUM                          *
 ************************************************************/

#if !NO_SPECTRUM

#define SPECTRUM_IMPLEMENTATION
#include "spectrum/spectrum.h"

class SpectrumAllocatorCallback : public spectrum::AllocatorCallback {
public:
    void* allocate(size_t size) {
        return cachePool.allocate(size, true);
    }
    void free(void* ptr) {
        cachePool.free(ptr);
    }
};

class SpectrumThreadPoolCallback : public spectrum::ThreadCallback {
public:
    std::future<size_t> enqueue(size_t(*func)(void*), void* arg) {
        return globalThreadPool.enqueue(func, arg);
    }
};

SpectrumAllocatorCallback spectrumAllocatorCallback;
SpectrumThreadPoolCallback spectrumThreadCallback;

size_t spectrum_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    spectrum::EncoderOptions spectrumOptionsA = { nullptr, 16384, false, false, false, 512, 256, false, false };
    spectrum::EncoderOptions spectrumOptionsB = { nullptr, 16384, false, false, false, 512, 256, true, false  };
    return spectrum::encode(in, inSize, out, level ? spectrumOptionsB : spectrumOptionsA, &spectrumThreadCallback, &spectrumAllocatorCallback);
}

int spectrum_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    return spectrum::decode(com, comSize, dec, decSize, &spectrumThreadCallback);
}

#endif

/*************************************************************
 *                         SKANDA                            *
 ************************************************************/

#if !NO_SKANDA

#define SKANDA_IMPLEMENTATION
#include "Skanda.h"

class SkandaAllocatorCallback : public skanda::AllocatorCallback {
public:
    void* allocate(size_t size) {
        return cachePool.allocate(size, true);
    }
    void free(void* ptr) {
        cachePool.free(ptr);
    }
};

class SkandaThreadPoolCallback : public skanda::ThreadCallback {
public:
    std::future<size_t> enqueue(size_t(*func)(void*), void* arg) {
        return globalThreadPool.enqueue(func, arg);
    }
};

class SkandaProgressCallback : public skanda::ProgressCallback {
public:
    bool progress(size_t processedBytes, size_t compressedSize) {
        std::cout << "\nProcessed: " << processedBytes;
        std::cout << "\nCompressed: " << compressedSize;
        //if (compressedSize > 10'000'000)  //Test stoping
        //    return true;
        return false;
    }
};

SkandaProgressCallback skandaProgressCallback;
SkandaAllocatorCallback skandaAllocatorCallback;
SkandaThreadPoolCallback skandaThreadCallback;

size_t skanda_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    return skanda::compress(in, inSize, out, level % 10, (float)(level / 10) / 100, &skandaAllocatorCallback);
}

int skanda_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    return skanda::decompress(com, comSize, dec, decSize, &skandaThreadCallback, &skandaAllocatorCallback);
}

#endif

#if !NO_SKANDA && !NO_SPECTRUM

size_t spectrum_skanda_backend(const uint8_t* data, const size_t size) {
    try {
        uint8_t* buffer = (uint8_t*)cachePool.allocate(compress_bound(size), true);
        size_t compressed = skanda::compress(data, size, buffer, 2, 0);
        cachePool.free(buffer);
        return compressed;
    }
    catch (std::bad_alloc& e) {
        return 0;
    }
}

size_t spectrum_skanda_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    spectrum::EncoderOptions spectrumOptionsA = { nullptr,                  16384, false, false, false, 640, 256, false, true, 2 };
    spectrum::EncoderOptions spectrumOptionsB = { &spectrum_skanda_backend, 16384, false, false, false, 640, 256, true , true, 2 };

    uint8_t* spectrumBuffer = (uint8_t*)cachePool.allocate(spectrum::encode_bound(inSize), true);
    size_t spectrumSize = spectrum::encode(in, inSize, spectrumBuffer,
        level % 10 >= 5 ? spectrumOptionsB : spectrumOptionsA, &spectrumThreadCallback, &spectrumAllocatorCallback);
    size_t skandaSize;

    if (spectrum::is_error(spectrumSize)) {
        spectrumSize = 0;
        skandaSize = skanda::compress(in, inSize, out + sizeof(size_t), level % 10, (float)(level / 10) / 100);
    }
    else 
        skandaSize = skanda::compress(spectrumBuffer, spectrumSize, out + sizeof(size_t), level % 10, (float)(level / 10) / 100);
    memcpy(out, &spectrumSize, sizeof(size_t));

    cachePool.free(spectrumBuffer);
    return skandaSize + sizeof(size_t);
}

int spectrum_skanda_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    size_t spectrumSize;
    memcpy(&spectrumSize, com, sizeof(size_t));

    if (spectrumSize == 0) {
        if (skanda::decompress(com + sizeof(size_t), comSize - sizeof(size_t), dec, decSize, &skandaThreadCallback, &skandaAllocatorCallback))
            return -1;
    }
    else {
        if (spectrumSize > decSize * 4)  // Too big, probably corrupted
            return -1;
        uint8_t* spectrumBuffer = (uint8_t*)cachePool.allocate(spectrumSize, true);
        if (skanda::decompress(com + sizeof(size_t), comSize - sizeof(size_t), spectrumBuffer, spectrumSize, &skandaThreadCallback, &skandaAllocatorCallback))
            return -1;
        if (spectrum::decode(spectrumBuffer, spectrumSize, dec, decSize, &spectrumThreadCallback, &spectrumAllocatorCallback))
            return -1;
        cachePool.free(spectrumBuffer);
    }
    return 0;
}

#endif

/*************************************************************
 *                        STRIDER                            *
 ************************************************************/

#if !NO_STRIDER

#define STRIDER_IMPLEMENTATION
#include "Strider.h"

size_t strider_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    return strider::compress(in, inSize, out, level);
}

int strider_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    return strider::decompress(com, comSize, dec, decSize);
}

#endif

#if !NO_STRIDER && !NO_SPECTRUM

size_t spectrum_strider_backend(const uint8_t* data, const size_t size) {
    try {
        uint8_t* buffer = (uint8_t*)cachePool.allocate(compress_bound(size), true);
        size_t compressed = strider::compress(data, size, buffer, 4);
        cachePool.free(buffer);
        return compressed;
    }
    catch (std::bad_alloc& e) {
        return 0;
    }
}

size_t spectrum_strider_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    spectrum::EncoderOptions spectrumOptionsA = { nullptr                  , 16384, false, false, false, 1024, 256, false };
    spectrum::EncoderOptions spectrumOptionsB = { &spectrum_strider_backend, 16384, false, false, false, 512 , 256, true  };

    uint8_t* spectrumBuffer = (uint8_t*)cachePool.allocate(spectrum::encode_bound(inSize), true);
    size_t spectrumSize = spectrum::encode(in, inSize, spectrumBuffer,
        level >= 5 ? spectrumOptionsB : spectrumOptionsA, &spectrumThreadCallback, &spectrumAllocatorCallback);
    memcpy(out, &spectrumSize, sizeof(size_t));
    size_t striderSize = strider::compress(spectrumBuffer, spectrumSize, out + 8, level);
    cachePool.free(spectrumBuffer);
    return striderSize + sizeof(size_t);
}

int spectrum_strider_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    size_t spectrumSize;
    memcpy(&spectrumSize, com, sizeof(size_t));
    uint8_t* spectrumBuffer = (uint8_t*)cachePool.allocate(spectrumSize, true);

    if (strider::decompress(com + 8, comSize - 8, spectrumBuffer, spectrumSize))
        return -1;
    if (spectrum::decode(spectrumBuffer, spectrumSize, dec, decSize))
        return -1;
    cachePool.free(spectrumBuffer);
    return 0;
}

#endif

/*************************************************************
 *                          LZ4                              *
 ************************************************************/

#if !NO_LZ4

#include "lz4/lz4.h"
#include "lz4/lz4hc.h"

size_t lz4_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    if (level < 0)
        return LZ4_compress_fast((char*)in, (char*)out, inSize, compress_bound(inSize), -level);
    else if (level <= 1)
        return LZ4_compress_default((char*)in, (char*)out, inSize, compress_bound(inSize));
    return LZ4_compress_HC((char*)in, (char*)out, inSize, compress_bound(inSize), level);
}

int lz4_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    return LZ4_decompress_safe((char*)com, (char*)dec, comSize, decSize) != decSize;
}

#endif

/*************************************************************
 *                         LIZARD                            *
 ************************************************************/

#if !NO_LIZARD

#include "lizard/lizard_compress.h"
#include "lizard/lizard_decompress.h"

size_t lizard_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    return Lizard_compress((char*)in, (char*)out, inSize, compress_bound(inSize), level);
}

int lizard_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    return Lizard_decompress_safe((char*)com, (char*)dec, comSize, decSize) != decSize;
}

#endif

/*************************************************************
 *                          ZSTD                             *
 ************************************************************/

#if !NO_ZSTD

#include "zstd/lib/zstd.h"

size_t zstd_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    return ZSTD_compress(out, compress_bound(inSize), in, inSize, level);
}

int zstd_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    return ZSTD_decompress(dec, decSize, com, comSize) != decSize;
}

#endif

#if !NO_ZSTD && !NO_SPECTRUM

size_t spectrum_zstd_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    uint8_t* spectrumBuffer = (uint8_t*)cachePool.allocate(spectrum::encode_bound(inSize), true);
    size_t spectrumSize = spectrum::encode(in, inSize, spectrumBuffer, spectrum::EncoderOptions(),
        &spectrumThreadCallback, &spectrumAllocatorCallback);
    memcpy(out, &spectrumSize, sizeof(size_t));
    size_t zstdSize = zstd_compress(spectrumBuffer, spectrumSize, out + 8, level);
    cachePool.free(spectrumBuffer);
    return zstdSize + sizeof(size_t);
}

int spectrum_zstd_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    size_t spectrumSize;
    memcpy(&spectrumSize, com, sizeof(size_t));
    uint8_t* spectrumBuffer = (uint8_t*)cachePool.allocate(spectrumSize, true);

    if (zstd_decompress(com + 8, comSize - 8, spectrumBuffer, spectrumSize))
        return -1;
    if (spectrum::decode(spectrumBuffer, spectrumSize, dec, decSize))
        return -1;
    cachePool.free(spectrumBuffer);
    return 0;
}

#endif

/*************************************************************
 *                       LIBDEFLATE                          *
 ************************************************************/

#if !NO_LIBDEFLATE

#include "libdeflate/libdeflate.h"

size_t libdeflate_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    struct libdeflate_compressor* compressor = libdeflate_alloc_compressor(level);
    int64_t res = libdeflate_deflate_compress(compressor, in, inSize, out, compress_bound(inSize));
    libdeflate_free_compressor(compressor);
    return res;
}

int libdeflate_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    struct libdeflate_decompressor* decompressor = libdeflate_alloc_decompressor();
    size_t res = 0;
    libdeflate_deflate_decompress(decompressor, com, comSize, dec, decSize, &res);
    return res != decSize;
}

#endif

/*************************************************************
 *                          LZMA                             *
 ************************************************************/

#if !NO_LZMA

#include <string.h>
#include "lzma/Alloc.h"
#include "lzma/LzmaDec.h"
#include "lzma/LzmaEnc.h"

size_t lzma_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    CLzmaEncProps props;
    int res;
    size_t headerSize = LZMA_PROPS_SIZE;
    SizeT out_len = compress_bound(inSize) - LZMA_PROPS_SIZE;
    LzmaEncProps_Init(&props);
    props.level = level;
    props.numThreads = 1;
    LzmaEncProps_Normalize(&props);
    res = LzmaEncode((uint8_t*)out + LZMA_PROPS_SIZE, &out_len, (uint8_t*)in, inSize, &props, (uint8_t*)out, &headerSize, 0/*int writeEndMark*/, NULL, &g_Alloc, &g_Alloc);
    return LZMA_PROPS_SIZE + out_len;
}

int lzma_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    int res;
    SizeT out_len = decSize;
    SizeT src_len = comSize - LZMA_PROPS_SIZE;
    ELzmaStatus status;
    res = LzmaDecode((uint8_t*)dec, &out_len, (uint8_t*)com + LZMA_PROPS_SIZE, &src_len, (uint8_t*)com, LZMA_PROPS_SIZE, LZMA_FINISH_END, &status, &g_Alloc);
    return out_len != decSize;
}

#endif

/*************************************************************
 *                         LZHAM                             *
 ************************************************************/

#if !NO_LZHAM

#include "lzham/lzham.h"
#include <memory.h>

size_t lzham_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    lzham_compress_params comp_params;
    memset(&comp_params, 0, sizeof(comp_params));
    comp_params.m_struct_size = sizeof(lzham_compress_params);
    comp_params.m_dict_size_log2 = 26;
    comp_params.m_max_helper_threads = 0;
    comp_params.m_level = (lzham_compress_level)level;
    lzham_compress_status_t comp_status;
    lzham_uint32 comp_adler32 = 0;
    size_t outsize;
    lzham_compress_memory(&comp_params, (uint8_t*)out, &outsize, (const lzham_uint8*)in, inSize, &comp_adler32);
    return outsize;
}

int lzham_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    lzham_uint32 comp_adler32 = 0;
    lzham_decompress_params decomp_params;
    memset(&decomp_params, 0, sizeof(decomp_params));
    decomp_params.m_struct_size = sizeof(decomp_params);
    decomp_params.m_dict_size_log2 = 26;
    lzham_decompress_status_t status = lzham_decompress_memory(&decomp_params, (uint8_t*)dec, &decSize, (const lzham_uint8*)com, comSize, &comp_adler32);
    return 0;
}

#endif

/*************************************************************
 *                          LZAV                             *
 ************************************************************/

#if !NO_LZAV

#include "lzav.h"

size_t lzav_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    if (level == 0)
        return lzav_compress_default(in, out, inSize, compress_bound(inSize));
    return lzav_compress_hi(in, out, inSize, compress_bound(inSize));
}

int lzav_decode(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    return lzav_decompress(com, dec, comSize, decSize) != decSize;
}

#endif

/*************************************************************
 *                         BROTLI                            *
 ************************************************************/

#if !NO_BROTLI

#include <brotli/encode.h>
#include <brotli/decode.h>

size_t brotli_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    int windowLog = 24; // sliding window size. Range is 10 to 24
    size_t actual_osize = compress_bound(inSize);
    return BrotliEncoderCompress(level, windowLog, BROTLI_DEFAULT_MODE, inSize, in, &actual_osize, out) == 0 ? 0 : actual_osize;
}

int brotli_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    size_t actual_osize = decSize;
    return BrotliDecoderDecompress(comSize, com, &actual_osize, dec) == BROTLI_DECODER_RESULT_ERROR ? -1 : 0;
}

#endif

#if !NO_BROTLI && !NO_SPECTRUM

size_t spectrum_brotli_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    uint8_t* spectrumBuffer = (uint8_t*)cachePool.allocate(spectrum::encode_bound(inSize), true);
    size_t spectrumSize = spectrum::encode(in, inSize, spectrumBuffer,
        spectrum::EncoderOptions(), &spectrumThreadCallback, &spectrumAllocatorCallback);
    memcpy(out, &spectrumSize, sizeof(size_t));
    size_t brotliSize = brotli_compress(spectrumBuffer, spectrumSize, out + 8, level);
    cachePool.free(spectrumBuffer);
    return brotliSize + sizeof(size_t);
}

int spectrum_brotli_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    size_t spectrumSize;
    memcpy(&spectrumSize, com, sizeof(size_t));
    uint8_t* spectrumBuffer = (uint8_t*)cachePool.allocate(spectrumSize, true);

    if (brotli_decompress(com + 8, comSize - 8, spectrumBuffer, spectrumSize))
        return -1;
    if (spectrum::decode(spectrumBuffer, spectrumSize, dec, decSize))
        return -1;
    cachePool.free(spectrumBuffer);
    return 0;
}

#endif

/*************************************************************
 *                  COMPRESSOR DICTIONARY                    *
 ************************************************************/

struct Compressor {
    string version;
    size_t(*compress)(uint8_t*, size_t, uint8_t*, int);
    int(*decompress)(uint8_t*, size_t, uint8_t*, size_t);
};

unordered_map<string, Compressor> availableCompressors = {
#if !NO_SPECTRUM
	{ "spectrum", { "0.2", &spectrum_compress, &spectrum_decompress } },
#endif
#if !NO_SKANDA
	{ "skanda", { "1.0", &skanda_compress, &skanda_decompress } },
#endif
#if !NO_SKANDA && !NO_SPECTRUM
    { "spectrum_skanda", { "1.0/0.2", &spectrum_skanda_compress, &spectrum_skanda_decompress } },
#endif
#if !NO_STRIDER
	{ "strider", { "0.5", &strider_compress, &strider_decompress } },
#endif
#if !NO_STRIDER && !NO_SPECTRUM
    { "spectrum_strider", { "0.5/0.2", &spectrum_strider_compress, &spectrum_strider_decompress } },
#endif
#if !NO_BROTLI
    { "brotli", { "1.2.0", &brotli_compress, &brotli_decompress } },
#endif
#if !NO_BROTLI && !NO_SPECTRUM
    { "spectrum_brotli", { "1.2.0/0.2", &spectrum_brotli_compress, &spectrum_brotli_decompress } },
#endif
#if !NO_LZ4
	{ "lz4", { "1.10.0", &lz4_compress, &lz4_decompress } },
#endif
#if !NO_LIZARD
	{ "lizard", { "1.0", &lizard_compress, &lizard_decompress } },
#endif
#if !NO_ZSTD
	{ "zstd", { "1.5.6", &zstd_compress, &zstd_decompress } },
#endif
#if !NO_ZSTD && !NO_SPECTRUM
    { "spectrum_zstd", { "1.5.6/0.2", &spectrum_zstd_compress, &spectrum_zstd_decompress } },
#endif
#if !NO_LIBDEFLATE
	{ "libdeflate", { "1.20", &libdeflate_compress, &libdeflate_decompress } },
#endif
#if !NO_LZMA
	{ "lzma", { "22.01", &lzma_compress, &lzma_decompress } },
#endif
#if !NO_LZHAM
	{ "lzham", { "1.0", &lzham_compress, &lzham_decompress } },
#endif
#if !NO_LZAV
    { "lzav", { "4.1", &lzav_compress, &lzav_decode } },
#endif
};

/*************************************************************
 *                        BENCHMARK                          *
 ************************************************************/

enum {
    TEST_TIME,
    TEST_MULTITHREAD,
    TEST_FUZZER,
};

uint64_t generate_random_number() {
    std::random_device r;
    std::default_random_engine e(r());
    std::uniform_int_distribution<uint64_t> dist(0, UINT64_MAX);
    return dist(e);
}

void test_file(string path, uint64_t off, string compressorName, int level, int testMode, size_t blockSize, 
    float testTimeMod, size_t* compressedSize, uint64_t* compressTime, uint64_t* decompressTime, mutex* mtx)
{
    fstream in(path, fstream::in | fstream::binary);
    if (!in.is_open()) {
        mtx->lock();
        std::cout << "\n Could not open file " << path;
        exit(0);
    }
    in.seekg(off);
    uint8_t* data = new uint8_t[blockSize];
    in.read((char*)data, blockSize);

    uint8_t* compressed = new uint8_t[compress_bound(blockSize)];
    //Have the buffers be physically allocated
    memset(compressed, 0, compress_bound(blockSize));

    auto compressor = availableCompressors.find(compressorName);
    if (compressor == availableCompressors.end()) {
        mtx->lock();
        std::cout << "\n Compressor " << compressorName << " not found";
        exit(0);
    }

    uint64_t maxTestTime = 0;
    if (testMode == TEST_TIME)
        maxTestTime = std::max(blockSize * 100, (uint64_t)50'000);  //Give 1 second for every 10MB of file and at least 50 microseconds
    maxTestTime *= testTimeMod;

    uint64_t testTime = 0;
    *compressTime = UINT64_MAX;
    do {
        auto timeStart = std::chrono::high_resolution_clock::now();
        *compressedSize = compressor->second.compress(data, blockSize, compressed, level);
        auto timeEnd = std::chrono::high_resolution_clock::now();
        uint64_t time = (timeEnd - timeStart).count();

        testTime += time;
        bool breakLoop = testTime > maxTestTime || testMode != TEST_TIME;
        if (time < *compressTime) 
            *compressTime = time;
        if (breakLoop)
            break;
    } while (true);

    if (testMode == TEST_FUZZER) {
        //The original compressed buffer is bigger than usually needed, which might cause out of bounds reads to go undetected
        uint8_t* tmp = new uint8_t[*compressedSize];
        memcpy(tmp, compressed, *compressedSize);
        delete[] compressed;
        compressed = tmp;

        uint64_t seed = generate_random_number();
        std::cout << "\nTesting file: " << path << " Algo: " << compressorName + "," + to_string(level) + " Offset: " << off << " Length: " << blockSize << " Seed: " << seed;
        //Introduce errors
        int maxErrors = 3 + seed % (std::max(*compressedSize, (size_t)8) / 4);
        for (int i = 0; i < maxErrors; i++) {
            seed *= 0xff51afd7ed558ccd;
            compressed[seed % *compressedSize] = seed % 256;
        }
    }

    uint8_t* decompressed = new uint8_t[blockSize];
    //Have the buffers be physically allocated
    memset(decompressed, 0, blockSize);

    testTime = 0;
    int decError = 0;
    *decompressTime = UINT64_MAX;
    do {
        auto timeStart = std::chrono::high_resolution_clock::now();
        decError = compressor->second.decompress(compressed, *compressedSize, decompressed, blockSize);
        auto timeEnd = std::chrono::high_resolution_clock::now();
        uint64_t time = (timeEnd - timeStart).count();

        testTime += time;
        bool breakLoop = testTime > maxTestTime || testMode != TEST_TIME;
        if (time < *decompressTime) 
            *decompressTime = time;
        if (breakLoop)
            break;
    } while (true);

    if (testMode != TEST_FUZZER) {
        if (decError || !std::equal(data, data + blockSize, decompressed)) {
            mtx->lock();
            std::cout << "\n Error at file " << path << " (off " << off << "), decoder return code: " << decError << ", correct bytes : " << 
                std::mismatch(data, data + blockSize, decompressed).first - data << " / " << blockSize << "\n";
            system("pause");
            exit(-1);
        }
    }

    delete[] compressed;
    delete[] data;
    delete[] decompressed;
}

struct CompressorResults {
    string compressor;
    int level = 0;
    uint64_t processedData = 0;
    uint64_t compressedSize = 0;
    uint64_t totalCTime = 0;
    uint64_t totalDTime = 0;
};

void benchmark_thread(vector<string>* fileList, size_t* fileIt, vector<CompressorResults>* compressorsToRun, 
    vector<CompressorResults>::iterator compressor, int testMode, size_t testBlockLog, float testTimeMod, std::mutex* mtx) 
{
    while (true) {
        mtx->lock();
        if (*fileIt == fileList->size()) {
            if (testMode != TEST_FUZZER) {
                mtx->unlock();
                break;
            }
            *fileIt = 0;
        }
        size_t thisFile = *fileIt;
        *fileIt += 1;
        uint64_t fileSize = filesystem::file_size(fileList->at(thisFile));
        std::cout << "\nFile: " << fileList->at(thisFile);
        mtx->unlock();

        uint64_t compressedSize = 0;
        double compressTime = 0;
        double decompressTime = 0;

        for (uint64_t pos = 0; pos < fileSize; pos += (1ull << testBlockLog)) {
            size_t blockCompSize;
            uint64_t blockEncTime, blockDecTime;
            test_file(fileList->at(thisFile), pos, compressor->compressor, compressor->level, testMode, 
                std::min(fileSize - pos, (uint64_t)1 << testBlockLog), testTimeMod, &blockCompSize, &blockEncTime, &blockDecTime, mtx);

            compressedSize += blockCompSize;
            compressTime += blockEncTime;
            decompressTime += blockDecTime;
        }

        if (testMode != TEST_FUZZER) {
            mtx->lock();
            compressor->processedData += fileSize;
            compressor->compressedSize += compressedSize;
            compressor->totalCTime += compressTime;
            compressor->totalDTime += decompressTime;

            for (auto i = compressorsToRun->begin(); i <= compressor; i++) {
                std::cout << std::fixed << std::setprecision(2) << "\n| " << i->compressor << " " << availableCompressors[i->compressor].version << " -" << i->level << " | "
                    << i->processedData / (i->totalCTime / 1e9) / 1024 / 1024 << "MiB/s | "
                    << i->processedData / (i->totalDTime / 1e9) / 1024 / 1024 << "MiB/s | "
                    << i->compressedSize << " | " << (double)i->compressedSize / i->processedData * 100 << " |";
            }
            mtx->unlock();
        }
    }
}

vector<string> split_text(string text, char ch) 
{
    vector<string> res;
    size_t nextCh = 0;
    while (true) {
        size_t pos = nextCh;
        nextCh = text.find(ch, nextCh);
        if (nextCh == string::npos)
            nextCh = text.size();
        res.push_back(string());
        for (; pos < nextCh; pos++)
            res.back().push_back(text[pos]);
        if (nextCh == text.size())
            break;
        nextCh++;
    }
    return res;
}

void parse_compressor_string(string text, vector<CompressorResults>* compressorsToRun) 
{
    vector<string> slashSplit = split_text(text, '/');
    for (size_t i = 0; i < slashSplit.size(); i++) {
        vector<string> commaSplit = split_text(slashSplit[i], ',');
        for (size_t j = 1; j < commaSplit.size(); j++)
            compressorsToRun->push_back({ commaSplit[0], stoi(commaSplit[j]) });
    }
}

int main(int argc, char** argv)
{
    string path = "";
    string compressorText = "";
    int testMode = TEST_TIME;
    int testBlockLog = 24;
    int threads = 1;
    float testTimeMod = 1;

    if (argc == 1) {
        std::cout << "Usage: benchmark [OPTIONS...]\n";
        std::cout << "-p: path to file or directory to test (directories are scanned recursively for files).\n";
        std::cout << "-c: compressors to test. Individual compressors are separated by '/', while levels are separated by ','. For example: skanda,2,5,7/zstd,6,9.\n";
        std::cout << "-m: test mode. 0 to test (de)compression speed, 1 to test compression ratio by running multithreading, 2 for fuzzer.\n";
        std::cout << "-l: divide files into blocks of size 2^l to compress individually. Default 24.";
        std::cout << "-t: number of threads to use for test mode 1. Default 1.\n";
        std::cout << "-e: test time multiplier for test mode 0. Default 1.\n";
        exit(0);
    }

    for (int i = 1; i < argc; i += 2) {
        if (argv[i][1] == 'p')
            path = argv[i + 1];
        else if (argv[i][1] == 'c')
            compressorText = argv[i + 1];
        else if (argv[i][1] == 'm')
            testMode = stoi(argv[i + 1]);
        else if (argv[i][1] == 'l')
            testBlockLog = stoi(argv[i + 1]);
        else if (argv[i][1] == 't')
            threads = stoi(argv[i + 1]);
        else if (argv[i][1] == 'e')
            testTimeMod = stof(argv[i + 1]);
    }

    if (testMode == TEST_TIME)
        SET_REALTIME_PRIO;

    vector<CompressorResults> compressorsToRun;
    parse_compressor_string(compressorText, &compressorsToRun);

    vector<string> fileList;
    try {
        if (!filesystem::is_directory(path))
            fileList.push_back(path);
        else {
            for (auto entry : filesystem::recursive_directory_iterator(path)) {
                if (!entry.is_regular_file() || entry.file_size() == 0)
                    continue;
                fileList.push_back(entry.path().string());
            }
        }
    }
    catch (std::filesystem::filesystem_error& e) {
        std::cout << "\n" << e.what();
        return -1;
    }
    catch (std::system_error& e) {
        std::cout << "\n" << e.what();
        return -1;
    }
    catch (...) {
        std::cout << "\nUnknown error";
        return -1;
    }

    for (auto compressorsIt = compressorsToRun.begin(); compressorsIt != compressorsToRun.end(); compressorsIt++) 
    {
        size_t fileIt = 0;
        mutex mtx;
        if (testMode == TEST_MULTITHREAD) {
            if (threads < 1)
                threads = 1;
            thread* cpu = new thread[threads];
            for (int i = 1; i < threads; i++)
                cpu[i] = thread(benchmark_thread, &fileList, &fileIt, &compressorsToRun, compressorsIt, testMode, testBlockLog, testTimeMod, &mtx);
            benchmark_thread(&fileList, &fileIt, &compressorsToRun, compressorsIt, testMode, testBlockLog, testTimeMod, &mtx);
            for (int i = 1; i < threads; i++)
                cpu[i].join();
            delete[] cpu;
        }
        else {
            benchmark_thread(&fileList, &fileIt, &compressorsToRun, compressorsIt, testMode, testBlockLog, testTimeMod, &mtx);
        }
    }

    return 0;
}
