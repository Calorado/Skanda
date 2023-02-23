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

using namespace std;

size_t compress_bound(size_t inSize) {
    return inSize + inSize / 2 + 1024;
}

#ifndef NO_SKANDA
#define SKANDA_IMPLEMENTATION
#include "Skanda.h"
size_t skanda_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    return skanda::compress(in, inSize, out, level);
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

#ifndef NO_SPECTRUM_STRIDER
size_t spectrum_strider_backend(const uint8_t* data, const size_t size) {
    uint8_t* out;
    try {
        out = new uint8_t[compress_bound(size)];
    }
    catch (std::bad_alloc& e) {
        return 0;
    }
    size_t compressed = strider::compress(data, size, out, 0);
    delete[] out;
    return compressed;
}
size_t spectrum_strider_compress(uint8_t* in, size_t inSize, uint8_t* out, int level) {
    spectrum::EncoderOptions spectrumOptionsA = { nullptr, 16384, 16384, 768, 256, 96, true };
    spectrum::EncoderOptions spectrumOptionsB = { &spectrum_strider_backend, 16384, 32768, 384, 128, 0, true };
    uint8_t* spectrumOut = new uint8_t[spectrum::bound(inSize)];
    size_t spectrumSize = spectrum::encode(in, inSize, spectrumOut, level >= 6 ? spectrumOptionsB : spectrumOptionsA);
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
    return LZ4_compress_HC((char*)in, (char*)out, inSize, inSize, level);
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

struct Compressor {
    string version;
    size_t(*compress)(uint8_t*, size_t, uint8_t*, int);
    int(*decompress)(uint8_t*, size_t, uint8_t*, size_t);
};

unordered_map<string, Compressor> availableCompressors = {
#ifndef NO_SKANDA
	{ "skanda", { "0.6", &skanda_compress, &skanda_decompress } },
#endif
#ifndef NO_STRIDER
	{ "strider", { "0.3", &strider_compress, &strider_decompress } },
#endif
#ifndef NO_SPECTRUM
	{ "spectrum", { "0.2_dev1", &spectrum_compress, &spectrum_decompress } },
#endif
#ifndef NO_SPECTRUM_STRIDER
	{ "spectrum_strider", { "0.3/0.2_dev1", &spectrum_strider_compress, &spectrum_strider_decompress } },
#endif
#ifndef NO_LZ4
	{ "lz4", { "1.9.4", &lz4_compress, &lz4_decompress } },
#endif
#ifndef NO_LIZARD
	{ "lizard", { "1.0", &lizard_compress, &lizard_decompress } },
#endif
#ifndef NO_ZSTD
	{ "zstd", { "1.5.4", &zstd_compress, &zstd_decompress } },
#endif
#ifndef NO_LIBDEFLATE
	{ "libdeflate", { "1.17", &libdeflate_compress, &libdeflate_decompress } },
#endif
#ifndef NO_LZMA
	{ "lzma", { "22.01", &lzma_compress, &lzma_decompress } },
#endif
#ifndef NO_LZHAM
	{ "lzham", { "1.0", &lzham_compress, &lzham_decompress } },
#endif
};

enum {
    TEST_TIME_IMMEDIATE = 0,
    TEST_TIME_SHORT,
    TEST_TIME_LONG
};

void test_file(string path, string compressor, int level, int testTimeMode, size_t* compressedSize, double* compressTime, double* decompressTime, int* err) {

    fstream in(path, fstream::in | fstream::binary);
    size_t fileSize = filesystem::file_size(path);
    uint8_t* data = new uint8_t[fileSize];
    in.read((char*)data, fileSize);

    uint8_t* compressed = new uint8_t[compress_bound(fileSize)];
    uint8_t* decompressed = new uint8_t[fileSize];
    //Have the buffers be physically allocated
    for (size_t i = 0; i < fileSize; i++) {
        compressed[i] = i;
        decompressed[i] = i;
    }

    double nextWait = 0;
    double testTime = 0;
    if (testTimeMode != TEST_TIME_IMMEDIATE) {
        if (testTimeMode == TEST_TIME_SHORT) {
            const double MAX_TEST_TIME = 5e7; //0.05 seconds
            const double ESTIMATED_SPEED = 2.5e7;  //25MB/s
            testTime = std::min(MAX_TEST_TIME, (double)fileSize / ESTIMATED_SPEED * 1e9);
        }
        else {
            testTime = 16 * 1e9; //16 seconds
        }
    }

    double elapsedCTime = 0;
    *compressTime = 1e100;
    do {
        //Give the processor to other processes every now and then, so
        // that we can have it all for the benchmark
        if (testTimeMode != TEST_TIME_IMMEDIATE && elapsedCTime > nextWait) {
            std::this_thread::sleep_for(chrono::milliseconds(1));
            nextWait += ceil(elapsedCTime * 10) / 10;  //Every 100 millis
        }
        auto timeStart = chrono::high_resolution_clock::now();
        *compressedSize = availableCompressors[compressor].compress(data, fileSize, compressed, level);
        double time = (chrono::high_resolution_clock::now() - timeStart).count();

        elapsedCTime += time;
        if (time < *compressTime)
            *compressTime = time;
    } while (elapsedCTime < testTime);

    nextWait = 0;
    double elapsedDTime = 0;
    *decompressTime = 1e100;
    do {
        if (testTimeMode != TEST_TIME_IMMEDIATE && elapsedCTime > nextWait) {
            std::this_thread::sleep_for(chrono::milliseconds(1));
            nextWait += ceil(elapsedDTime * 10) / 10;  //Every 100 millis
        }
        auto timeStart = chrono::high_resolution_clock::now();
        *err = availableCompressors[compressor].decompress(compressed, *compressedSize, decompressed, fileSize);
        double time = (chrono::high_resolution_clock::now() - timeStart).count();

        elapsedDTime += time;
        if (time < *decompressTime)
            *decompressTime = time;
    } while (elapsedDTime < testTime);

    if (!std::equal(data, data + fileSize, decompressed))
        *err = true;

    delete[] data;
    delete[] compressed;
    delete[] decompressed;
}

struct CompressorResults {
    string compressor;
    int level = 0;
    size_t processedData = 0;
    size_t compressedSize = 0;
    double totalCTime = 0;
    double totalDTime = 0;
};

void benchmark_thread(vector<string>* fileList, size_t* fileIt, vector<CompressorResults>* compressorsToRun, 
    vector<CompressorResults>::iterator compressor, int testTimeMode, std::mutex* mtx) 
{
    while (true) {
        mtx->lock();
        if (*fileIt == fileList->size()) {
            mtx->unlock();
            break;
        }
        size_t thisFile = *fileIt;
        *fileIt += 1;
        mtx->unlock();

        size_t compressedSize;
        double compressTime;
        double decompressTime;
        int err;
        test_file(fileList->at(thisFile), compressor->compressor, compressor->level, testTimeMode, &compressedSize, &compressTime, &decompressTime, &err);
        
        if (err) {
            mtx->lock();
            cout << "\n\n Error in file: " << fileList->at(thisFile);
            mtx->unlock();
            exit(0);
        }

        mtx->lock();
        cout << "\n\n File: " << fileList->at(thisFile);
        compressor->processedData += filesystem::file_size(fileList->at(thisFile));
        compressor->compressedSize += compressedSize;
        compressor->totalCTime += compressTime;
        compressor->totalDTime += decompressTime;

        for (auto i = compressorsToRun->begin(); i <= compressor; i++) {
            cout << std::fixed << std::setprecision(2) << "\n| " << i->compressor << " " << availableCompressors[i->compressor].version << " -" << i->level << " | "
                << i->processedData / (i->totalCTime / 1e9) / 1024 / 1024 << "MiB/s | "
                << i->processedData / (i->totalDTime / 1e9) / 1024 / 1024 << "MiB/s | "
                << i->compressedSize << " | " << (double)i->compressedSize / i->processedData * 100 << " |";
        }
        mtx->unlock();
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
    cout << "Path: ";
    std::getline(std::cin, path);
    string compressorText;
    cout << "Compressors: ";
    std::getline(std::cin, compressorText);
    bool multithread;
    cout << "Multithread: ";
    cin >> multithread;

    vector<CompressorResults> compressorsToRun;
    parse_compressor_string(compressorText, &compressorsToRun);
    int testTimeMode;

    vector<string> fileList;
    if (!filesystem::is_directory(path)) {
        multithread = false;
        testTimeMode = TEST_TIME_LONG;
        fileList.push_back(path);
    }
    else {
        testTimeMode = multithread ? TEST_TIME_IMMEDIATE : TEST_TIME_SHORT;
        for (auto entry : filesystem::recursive_directory_iterator(path)) {
            if (!entry.is_directory())
                fileList.push_back(entry.path().string());
        }
    }

    if (!multithread)
        SET_REALTIME_PRIO;

    for (auto compressorsIt = compressorsToRun.begin(); compressorsIt != compressorsToRun.end(); compressorsIt++) {
        int concurrency = multithread ? thread::hardware_concurrency() : 1;
        thread* cpu = new thread[concurrency];

        size_t fileIt = 0;
        mutex mtx;
        for (int i = 0; i < concurrency; i++)
            cpu[i] = thread(benchmark_thread, &fileList, &fileIt, &compressorsToRun, compressorsIt, testTimeMode, &mtx);
        for (int i = 0; i < concurrency; i++)
            cpu[i].join();
        delete[] cpu;
    }

    cout << "\n\nFinished";
    cin >> path;

    return 0;
}
