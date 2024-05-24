// Public domain 
// Carlos de Diego 2023

#if defined(_MSC_VER)
#define VC_EXTRALEAN
#define WIN32_LEAN_AND_MEAN
#define SET_REALTIME_PRIO SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS)
#include <Windows.h>
#undef min
#undef max
#else
#include <sys/time.h>
#include <sys/resource.h>
#define SET_REALTIME_PRIO setpriority(PRIO_PROCESS, 0, -20)
#endif

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

using namespace std;

size_t compress_bound(size_t inSize) {
    return inSize + inSize / 16 + 1024;
}

#define NO_SPECTRUM
//#define NO_STRIDER
//#define NO_SKANDA
//#define NO_LZ4
#define NO_LIZARD
//#define NO_ZSTD
//#define NO_LIBDEFLATE
//#define NO_LZMA
//#define NO_LZHAM
//#define NO_LZAV

#ifndef NO_SKANDA
#define SKANDA_IMPLEMENTATION
#include "Skanda.h"
size_t skanda_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    return skanda::compress(in, inSize, out, level % 10, (float)(level / 10) / 100);
}
int skanda_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    return skanda::decompress(com, comSize, dec, decSize);
}
#endif

#ifndef NO_STRIDER
#define STRIDER_IMPLEMENTATION
#include "Strider.h"
size_t strider_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    return strider::compress(in, inSize, out, level);
}
int strider_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    return strider::decompress(com, comSize, dec, decSize);
}
#endif

#ifndef NO_SPECTRUM
#define SPECTRUM_IMPLEMENTATION
#include "Spectrum.h"
size_t spectrum_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    return spectrum::encode(in, inSize, out);
}
int spectrum_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    return spectrum::decode(com, comSize, dec, decSize);
}
#endif

#if !defined(NO_SKANDA) && !defined(NO_SPECTRUM)
size_t spectrum_skanda_backend(const uint8_t* data, const size_t size) {
    uint8_t* out;
    try {
        out = new uint8_t[compress_bound(size)];
    }
    catch (std::bad_alloc& e) {
        return 0;
    }
    size_t compressed = skanda::compress(data, size, out, 2);
    delete[] out;
    return compressed;
}
size_t spectrum_skanda_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    spectrum::EncoderOptions spectrumOptionsA = { nullptr, 16384, 512, 256, false, true };
    spectrum::EncoderOptions spectrumOptionsB = { &spectrum_skanda_backend, 16384, 512, 256, true, true };

    uint8_t* spectrumOut = new uint8_t[spectrum::encode_bound(inSize)];
    size_t spectrumSize = spectrum::encode(in, inSize, spectrumOut, level % 10 >= 5 ? spectrumOptionsB : spectrumOptionsA);
    size_t skandaSize;

    if (spectrum::is_error(spectrumSize)) {
        spectrumSize = 0;
        memcpy(out, &spectrumSize, sizeof(size_t));
        skandaSize = skanda::compress(in, inSize, out + sizeof(size_t), level % 10, (float)(level / 10) / 100);
    }
    else {
        memcpy(out, &spectrumSize, sizeof(size_t));
        skandaSize = skanda::compress(spectrumOut, spectrumSize, out + sizeof(size_t), level % 10, (float)(level / 10) / 100);
    }
    
    delete[] spectrumOut;
    return skandaSize + sizeof(size_t);
}
int spectrum_skanda_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    size_t spectrumSize;
    memcpy(&spectrumSize, com, sizeof(size_t));

    if (spectrumSize == 0) {
        if (skanda::decompress(com + sizeof(size_t), comSize - sizeof(size_t), dec, decSize))
            return -1;
    }
    else {
        uint8_t* spectrumOut = new uint8_t[spectrumSize];
        if (skanda::decompress(com + sizeof(size_t), comSize - sizeof(size_t), spectrumOut, spectrumSize))
            return -1;
        if (spectrum::decode(spectrumOut, spectrumSize, dec, decSize))
            return -1;
        delete[] spectrumOut;
    }
    return 0;
}
#endif

#if !defined(NO_STRIDER) && !defined(NO_SPECTRUM)
size_t spectrum_strider_backend(const uint8_t* data, const size_t size) {
    uint8_t* out;
    try {
        out = new uint8_t[compress_bound(size)];
    }
    catch (std::bad_alloc& e) {
        return 0;
    }
    size_t compressed = strider::compress(data, size, out, 4);
    delete[] out;
    return compressed;
}
size_t spectrum_strider_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    spectrum::EncoderOptions spectrumOptionsA = { nullptr, 16384, 1024, 256, false };
    spectrum::EncoderOptions spectrumOptionsB = { &spectrum_strider_backend, 16384, 512, 256, true };
    uint8_t* spectrumOut = new uint8_t[spectrum::encode_bound(inSize)];
    size_t spectrumSize = spectrum::encode(in, inSize, spectrumOut, level >= 5 ? spectrumOptionsB : spectrumOptionsA);
    memcpy(out, &spectrumSize, sizeof(size_t));
    size_t striderSize = strider::compress(spectrumOut, spectrumSize, out + 8, level);
    delete[] spectrumOut;
    return striderSize + sizeof(size_t);
}
int spectrum_strider_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    size_t spectrumSize;
    memcpy(&spectrumSize, com, sizeof(size_t));
    uint8_t* spectrumOut = new uint8_t[spectrumSize];

    if (strider::decompress(com + 8, comSize - 8, spectrumOut, spectrumSize))
        return -1;
    if (spectrum::decode(spectrumOut, spectrumSize, dec, decSize))
        return -1;
    delete[] spectrumOut;
    return 0;
}
#endif

#ifndef NO_LZ4
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

#ifndef NO_LIZARD
#include "lizard/lizard_compress.h"
#include "lizard/lizard_decompress.h"
size_t lizard_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    return Lizard_compress((char*)in, (char*)out, inSize, compress_bound(inSize), level);
}
int lizard_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    return Lizard_decompress_safe((char*)com, (char*)dec, comSize, decSize) != decSize;
}
#endif

#ifndef NO_ZSTD
#include "zstd/lib/zstd.h"
size_t zstd_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    return ZSTD_compress(out, compress_bound(inSize), in, inSize, level);
}
int zstd_decompress(uint8_t* com, size_t comSize, uint8_t* dec, size_t decSize) {
    return ZSTD_decompress(dec, decSize, com, comSize) != decSize;
}
#endif

#ifndef NO_LIBDEFLATE
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

#ifndef NO_LZMA
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

#ifndef NO_LZHAM
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

#ifndef NO_LZAV
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

struct Compressor {
    string version;
    size_t(*compress)(uint8_t*, size_t, uint8_t*, int);
    int(*decompress)(uint8_t*, size_t, uint8_t*, size_t);
};

unordered_map<string, Compressor> availableCompressors = {
#ifndef NO_SKANDA
	{ "skanda", { "0.9", &skanda_compress, &skanda_decompress } },
#endif
#ifndef NO_STRIDER
	{ "strider", { "0.5", &strider_compress, &strider_decompress } },
#endif
#ifndef NO_SPECTRUM
	{ "spectrum", { "0.2", &spectrum_compress, &spectrum_decompress } },
#endif
#if !defined(NO_SKANDA) && !defined(NO_SPECTRUM)
    { "spectrum_skanda", { "0.9/0.2", &spectrum_skanda_compress, &spectrum_skanda_decompress } },
#endif
#if !defined(NO_STRIDER) && !defined(NO_SPECTRUM)
	{ "spectrum_strider", { "0.5/0.2", &spectrum_strider_compress, &spectrum_strider_decompress } },
#endif
#ifndef NO_LZ4
	{ "lz4", { "1.9.4", &lz4_compress, &lz4_decompress } },
#endif
#ifndef NO_LIZARD
	{ "lizard", { "1.0", &lizard_compress, &lizard_decompress } },
#endif
#ifndef NO_ZSTD
	{ "zstd", { "1.5.6", &zstd_compress, &zstd_decompress } },
#endif
#ifndef NO_LIBDEFLATE
	{ "libdeflate", { "1.20", &libdeflate_compress, &libdeflate_decompress } },
#endif
#ifndef NO_LZMA
	{ "lzma", { "22.01", &lzma_compress, &lzma_decompress } },
#endif
#ifndef NO_LZHAM
	{ "lzham", { "1.0", &lzham_compress, &lzham_decompress } },
#endif
#ifndef NO_LZAV
    { "lzav", { "4.0", &lzav_compress, &lzav_decode } },
#endif
};

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

void test_file(string path, size_t off, string compressorName, int level, int testMode, size_t blockSize, 
    size_t* compressedSize, double* compressTime, double* decompressTime, mutex* mtx) 
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
    in.close();

    uint8_t* compressed = new uint8_t[compress_bound(blockSize)];
    uint8_t* decompressed = new uint8_t[blockSize];
    //Have the buffers be physically allocated
    memset(compressed, 0, compress_bound(blockSize));
    memset(decompressed, 0, blockSize);

    auto compressor = availableCompressors.find(compressorName);
    if (compressor == availableCompressors.end()) {
        mtx->lock();
        std::cout << "\n Compressor " << compressorName << " not found";
        exit(0);
    }

    double maxTimeSinceBestTime = 0;
    if (testMode == TEST_TIME)
        maxTimeSinceBestTime = std::max((double)blockSize * 25, 5e4);  //Give 1 second for every 40MB of file and at least 50 microseconds

    double timeSinceBestTime = 0;
    *compressTime = 1e100;
    do {
        auto timeStart = std::chrono::high_resolution_clock::now();
        *compressedSize = compressor->second.compress(data, blockSize, compressed, level);
        auto timeEnd = std::chrono::high_resolution_clock::now();
        double time = (timeEnd - timeStart).count();

        timeSinceBestTime += time;
        bool breakLoop = timeSinceBestTime > maxTimeSinceBestTime || testMode != TEST_TIME;
        if (time < *compressTime) {
            *compressTime = time;
            timeSinceBestTime = 0;
        }
        if (breakLoop)
            break;
    } while (true);

    if (testMode == TEST_FUZZER) {
        uint64_t seed = generate_random_number();
        std::cout << "\nTesting file: " << path << " Algo: " << compressorName + "," + to_string(level) + " Offset: " << off << " Length: " << blockSize << " Seed: " << seed;
        //Introduce errors
        int maxErrors = 3 + seed % (std::max(*compressedSize, (size_t)8) / 4);
        for (int i = 0; i < maxErrors; i++) {
            seed *= 0xff51afd7ed558ccd;
            compressed[seed % *compressedSize] = seed % 256;
        }
    }

    timeSinceBestTime = 0;
    int decError = 0;
    *decompressTime = 1e100;
    do {
        auto timeStart = std::chrono::high_resolution_clock::now();
        decError = compressor->second.decompress(compressed, *compressedSize, decompressed, blockSize);
        auto timeEnd = std::chrono::high_resolution_clock::now();
        double time = (timeEnd - timeStart).count();

        timeSinceBestTime += time;
        bool breakLoop = timeSinceBestTime > maxTimeSinceBestTime || testMode != TEST_TIME;
        if (time < *decompressTime) {
            *decompressTime = time;
            timeSinceBestTime = 0;
        }
        if (breakLoop)
            break;
    } while (true);

    if (testMode != TEST_FUZZER) {
        if (decError || !std::equal(data, data + blockSize, decompressed)) {
            mtx->lock();
            std::cout << "\n Error at file " << path << ", decoder return code: " << decError << ", correct bytes: " << std::mismatch(data, data + blockSize, decompressed).first - data << "\n";
            exit(-1);
        }
    }

    delete[] data;
    delete[] compressed;
    delete[] decompressed;
}

struct CompressorResults {
    string compressor;
    int level = 0;
    uint64_t processedData = 0;
    uint64_t compressedSize = 0;
    double totalCTime = 0;
    double totalDTime = 0;
};

void benchmark_thread(vector<string>* fileList, size_t* fileIt, vector<CompressorResults>* compressorsToRun, 
    vector<CompressorResults>::iterator compressor, int testMode, size_t testBlockLog, std::mutex* mtx) 
{
    while (true) {
        mtx->lock();
        if (*fileIt == fileList->size()) {
            mtx->unlock();
            break;
        }
        size_t thisFile = *fileIt;
        *fileIt += 1;
        if (testMode == TEST_FUZZER && *fileIt == fileList->size())
            *fileIt = 0;
        if (testMode != TEST_FUZZER)
            std::cout << "\n\n File: " << fileList->at(thisFile);
        mtx->unlock();

        uint64_t fileSize = filesystem::file_size(fileList->at(thisFile));
        uint64_t compressedSize = 0;
        double compressTime = 0;
        double decompressTime = 0;

        for (uint64_t pos = 0; pos < fileSize; pos += (1ull << testBlockLog)) {
            size_t blockCompSize;
            double blockEncTime, blockDecTime;
            test_file(fileList->at(thisFile), pos, compressor->compressor, compressor->level, testMode, std::min(fileSize - pos, (size_t)1 << testBlockLog), &blockCompSize, &blockEncTime, &blockDecTime, mtx);

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

vector<string> split_text(string text, char ch) {

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

void parse_compressor_string(string text, vector<CompressorResults>* compressorsToRun) {

    vector<string> slashSplit = split_text(text, '/');
    for (size_t i = 0; i < slashSplit.size(); i++) {
        vector<string> commaSplit = split_text(slashSplit[i], ',');
        for (size_t j = 1; j < commaSplit.size(); j++)
            compressorsToRun->push_back({ commaSplit[0], stoi(commaSplit[j]) });
    }
}

int main()
{
    string path;
    std::cout << "Path: ";
    std::getline(std::cin, path);
    string compressorText;
    std::cout << "Compressors: ";
    std::getline(std::cin, compressorText);
    int testMode;
    std::cout << "Test mode (0 - Time, 1 - Multithread, 2 - Fuzzer): ";
    std::cin >> testMode;
    size_t testBlockLog;
    std::cout << "Block log: ";
    std::cin >> testBlockLog;

    if (testMode == TEST_TIME)
        SET_REALTIME_PRIO;

    vector<CompressorResults> compressorsToRun;
    parse_compressor_string(compressorText, &compressorsToRun);

    vector<string> fileList;
    if (!filesystem::is_directory(path)) 
        fileList.push_back(path);
    else {
        try {
            for (auto entry : filesystem::recursive_directory_iterator(path)) {
                if (!entry.is_regular_file() || entry.file_size() == 0)
                    continue;
                fileList.push_back(entry.path().string());
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
    }

    for (auto compressorsIt = compressorsToRun.begin(); compressorsIt != compressorsToRun.end(); compressorsIt++) 
    {
        size_t fileIt = 0;
        mutex mtx;
        if (testMode == TEST_MULTITHREAD) {
            int concurrency = thread::hardware_concurrency() / 2 - 2;
            thread* cpu = new thread[concurrency];
            for (int i = 0; i < concurrency; i++)
                cpu[i] = thread(benchmark_thread, &fileList, &fileIt, &compressorsToRun, compressorsIt, testMode, testBlockLog, &mtx);
            for (int i = 0; i < concurrency; i++)
                cpu[i].join();
            delete[] cpu;
        }
        else {
            benchmark_thread(&fileList, &fileIt, &compressorsToRun, compressorsIt, testMode, testBlockLog, &mtx);
        }
    }

    std::cout << "\n\nFinished";
    std::cin >> path;

    return 0;
}
