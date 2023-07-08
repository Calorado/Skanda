/*
 * Skanda Compression Algorithm v0.8
 * Copyright (c) 2023 Carlos de Diego
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef __SKANDA__

#define __SKANDA__

#include <cstdint>
#include <stdio.h> //size_t

namespace skanda {
	//Base class that allows to track progress of compression and decompression of
	// a Skanda stream. You will have to create a child class which implements the functions.
	// - progress(): the algorithm will pass the number of bytes that have been compressed/decompressed,
	//				 and its current compressed size. The function will return whether to stop encoding/decoding.
	class ProgressCallback {
	public:
		virtual bool progress(size_t processedBytes, size_t compressedSize) {
			return false;
		}
	};

	//Compresses "size" bytes of data present in "input", and stores it in "output".
	//"level" is a tradeoff between compressed size and compression speed, and must be <= 9.
	//"decSpeedBias" is a tradeoff between compressed size and decompressed speed,
	// and must have a value between 0 and 1, with higher sacrificing ratio for speed.
	//You may pass a pointer to an object with base class ProgressCallback, to track progress.
	//Returns the size of the compressed stream or -1 on failure.
	size_t compress(const uint8_t* input, size_t size, uint8_t* output, int level = 1, float decSpeedBias = 0.5f, ProgressCallback* progress = nullptr);
	//Decompresses contents in "compressed" to "decompressed".
	//You may also pass a pointer to an object with base class ProgressCallback, to track progress.
	//Returns 0 on success or -1 on failure or corrupted data.
	int decompress(const uint8_t* compressed, size_t compressedSize, uint8_t* decompressed,
		size_t decompressedSize, ProgressCallback* progress = nullptr);

	//For a given input size, returns a size for the output buffer that is big enough to
	// contain the compressed stream even if it expands.
	size_t compress_bound(size_t size);
	//Returns the amount of memory the algorithm will consume on compression.
	size_t estimate_memory(size_t size, int level = 1, float decSpeedBias = 0.35f);
}

#ifdef SKANDA_IMPLEMENTATION

#include <algorithm>
#include <cstring>
#include <vector>
#include <cmath>

#if defined(_MSC_VER)
  #define FORCE_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
  #define FORCE_INLINE inline __attribute__((always_inline))
#else
  #define FORCE_INLINE inline
#endif

#if defined(__GNUC__) || defined(__clang__)
  #define expect(expr,value)    (__builtin_expect ((expr),(value)) )
  #define likely(expr)     expect((expr) != 0, 1)
  #define unlikely(expr)   expect((expr) != 0, 0)
#else
  #define likely(expr)     (expr)
  #define unlikely(expr)   (expr)
#endif

//Probably not the correct way to do it but bleh
#define IS_64BIT (UINTPTR_MAX > UINT32_MAX)

#if defined(_MSC_VER)
  #if defined(_M_AMD64)
	#define x64
    #include <intrin.h>
  #endif
#elif defined(__GNUC__) || defined(__clang__)
  #if defined(__amd64__)
	#define x64
	#include <x86intrin.h>
  #endif
#endif

namespace skanda {

	FORCE_INLINE bool is_little_endian() {
		const union { uint16_t u; uint8_t c[2]; } LITTLE_ENDIAN_CHECK = { 1 };
		return LITTLE_ENDIAN_CHECK.c[0];
	}

	//Undefined behaviour if value == 0
	FORCE_INLINE size_t unsafe_int_log2(size_t value) {
#if defined(_MSC_VER)
		unsigned long result;
	#if IS_64BIT
		_BitScanReverse64(&result, value);
	#else
		_BitScanReverse(&result, value);
	#endif
		return result;
#elif defined(__GNUC__) || defined(__clang__)
	#if IS_64BIT
		return 63 - __builtin_clzll(value);
	#else
		return 31 - __builtin_clz(value);
	#endif
#else
	#if IS_64BIT
		const uint8_t tab64[64] = {
			 0, 58,  1, 59, 47, 53,  2, 60,
			39, 48, 27, 54, 33, 42,  3, 61,
			51, 37, 40, 49, 18, 28, 20, 55,
			30, 34, 11, 43, 14, 22,  4, 62,
			57, 46, 52, 38, 26, 32, 41, 50,
			36, 17, 19, 29, 10, 13, 21, 56,
			45, 25, 31, 35, 16,  9, 12, 44,
			24, 15,  8, 23,  7,  6,  5, 63
		};
		value |= value >> 1;
		value |= value >> 2;
		value |= value >> 4;
		value |= value >> 8;
		value |= value >> 16;
		value |= value >> 32;
		return tab64[value * 0x03f6eaf2cd271461 >> 58];
	#else
		const uint8_t tab32[32] = {
			 0,  9,  1, 10, 13, 21,  2, 29,
			11, 14, 16, 18, 22, 25,  3, 30,
			 8, 12, 20, 28, 15, 17, 24,  7,
			19, 27, 23,  6, 26,  5,  4, 31
		};
		value |= value >> 1;
		value |= value >> 2;
		value |= value >> 4;
		value |= value >> 8;
		value |= value >> 16;
		return tab32[value * 0x07C4ACDDU >> 27];
	#endif
#endif
	}

	//Way faster than using log2(double), also returns 0 for a value of 0
	FORCE_INLINE size_t int_log2(const size_t value) {
#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
		if (likely(value != 0))
			return unsafe_int_log2(value);
		return 0;
#endif  //Fallback already returns 0 when value == 0
		return unsafe_int_log2(value);
	}

	//Undefined behaviour when value == 0
	FORCE_INLINE size_t unsafe_bit_scan_forward(const size_t value) {
#if defined(_MSC_VER)
		unsigned long result;
	#if IS_64BIT
		_BitScanForward64(&result, value);
	#else
		_BitScanForward(&result, value);
	#endif
		return result;
#elif defined(__GNUC__) || defined(__clang__)
	#if IS_64BIT
		return __builtin_ctzll(value);
	#else
		return __builtin_ctz(value);
	#endif
#else
	#if IS_64BIT
		const unsigned int tab64[64] = {
			 0,  1,  2, 53,  3,  7, 54, 27,
			 4, 38, 41,  8, 34, 55, 48, 28,
			62,  5, 39, 46, 44, 42, 22,  9,
			24, 35, 59, 56, 49, 18, 29, 11,
			63, 52,  6, 26, 37, 40, 33, 47,
			61, 45, 43, 21, 23, 58, 17, 10,
			51, 25, 36, 32, 60, 20, 57, 16,
			50, 31, 19, 15, 30, 14, 13, 12
		};
		return tab64[(value & (0 - value)) * 0x022FDD63CC95386D >> 58];
	#else
		static uint8_t tab32[32] = {
			 0,  1, 28,  2, 29, 14, 24,  3,
			30, 22, 20, 15, 25, 17,  4,  8,
			31, 27, 13, 23, 21, 19, 16,  7,
			26, 12, 18,  6, 11,  5, 10,  9
		};
		return tab32[(value & (0 - value)) * 0x077CB531U >> 27];
	#endif
#endif
	}

	//Returns the index of the first set bit, starting from the least significant, or 0 if the input is null
	FORCE_INLINE size_t bit_scan_forward(const size_t value) {
#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
		if (likely(value != 0))
			return unsafe_bit_scan_forward(value);
		return 0;
#endif  //Fallback already returns 0 when value == 0
		return unsafe_bit_scan_forward(value);
	}

	struct FastIntHash {
		//Use top bits
		size_t operator()(const size_t value) {
#if IS_64BIT
			return value * 0xff51afd7ed558ccd;
#else
			return value * 0x27d4eb2d;
#endif
		}
	};

	//These functions are used to obtain the data for the hash. If the number of bytes is higher than the word size,
	//they will be mixed. Also note that these might read more bytes than necessary.
	FORCE_INLINE size_t read_hash4(const uint8_t* const ptr) {
		uint32_t value;
		memcpy(&value, ptr, 4);
		return value;
	}
	FORCE_INLINE size_t read_hash8(const uint8_t* const ptr) {
#if IS_64BIT
		uint64_t value;
		memcpy(&value, ptr, 8);
#else
		uint32_t value = read_hash4(ptr) ^ read_hash4(ptr + 4);
#endif
		return value;
	}
	FORCE_INLINE size_t read_hash6(const uint8_t* const ptr) {
#if IS_64BIT
		if (is_little_endian())
			return read_hash8(ptr) << 16;
		return read_hash8(ptr) >> 16;
#else
		uint16_t b;
		memcpy(&b, ptr + 4, 2);
		return read_hash4(ptr) ^ b;
#endif
	}
	FORCE_INLINE size_t read_hash5(const uint8_t* const ptr) {
#if IS_64BIT
		if (is_little_endian())
			return read_hash8(ptr) << 24;
		return read_hash8(ptr) >> 24;
#else
		return read_hash4(ptr) ^ ptr[4];
#endif
	}
	FORCE_INLINE size_t read_hash3(const uint8_t* const ptr) {

		if (is_little_endian())
#if IS_64BIT
			return read_hash4(ptr) << 40;
#else
			return read_hash4(ptr) << 8;
#endif
		return read_hash4(ptr) >> 8;
	}

	FORCE_INLINE uint64_t read_uint64le(const uint8_t* const ptr) {
		if (is_little_endian()) {
			uint64_t value;
			memcpy(&value, ptr, 8);
			return value;
		}
		uint64_t value = 0;
		for (int i = 0; i < 8; i++)
			value |= (uint64_t)ptr[i] << i * 8;
		return value;
	}
	FORCE_INLINE void write_uint64le(uint8_t* const ptr, const uint64_t value) {
		if (is_little_endian())
			memcpy(ptr, &value, 8);
		else {
			for (int i = 0; i < 8; i++)
				ptr[i] = value >> i * 8;
		}
	}
	FORCE_INLINE uint32_t read_uint32le(const uint8_t* const ptr) {
		if (is_little_endian()) {
			uint32_t value;
			memcpy(&value, ptr, 4);
			return value;
		}
		uint32_t value = 0;
		for (int i = 0; i < 4; i++)
			value |= (uint32_t)ptr[i] << i * 8;
		return value;
	}
	FORCE_INLINE void write_uint32le(uint8_t* const ptr, const uint32_t value) {
		if (is_little_endian())
			memcpy(ptr, &value, 4);
		else {
			for (int i = 0; i < 4; i++)
				ptr[i] = value >> i * 8;
		}
	}
	FORCE_INLINE uint32_t read_uint24le(const uint8_t* const ptr) {
		uint32_t value = 0;
		for (int i = 0; i < 3; i++)
			value |= (uint32_t)ptr[i] << i * 8;
		return value;
	}
	FORCE_INLINE void write_uint24le(uint8_t* const ptr, const uint32_t value) {
		for (int i = 0; i < 3; i++)
			ptr[i] = value >> i * 8;
	}
	FORCE_INLINE uint16_t read_uint16le(const uint8_t* const ptr) {
		uint16_t value;
		if (is_little_endian())
			memcpy(&value, ptr, 2);
		else
			value = ptr[0] | (ptr[1] << 8);
		return value;
	}
	FORCE_INLINE void write_uint16le(uint8_t* const ptr, const uint16_t value) {
		if (is_little_endian())
			memcpy(ptr, &value, 2);
		else {
			for (int i = 0; i < 2; i++)
				ptr[i] = value >> i * 8;
		}
	}

	//https://github.com/romeric/fastapprox/blob/master/fastapprox/src/fastlog.h
	FORCE_INLINE float fast_log2(float x) {
		union { float f; uint32_t i; } vx = { x };
		float y = vx.i;
		y *= 1.1920928955078125e-7f;
		return y - 126.94269504f;
	}

	const int SKANDA_MIN_MATCH_LENGTH = 2;
	//These are written uncompressed
	const int SKANDA_LAST_BYTES = 63;
	const int SKANDA_MAX_WINDOW_LOG = 24;

	const int MAX_HUFFMAN_CODE_LENGTH = 11;
	const int HUFFMAN_CODE_SPACE = 1 << MAX_HUFFMAN_CODE_LENGTH;
	const size_t MAX_BLOCK_SIZE = 262144;

	enum {
		GREEDY1,
		GREEDY2,
		LAZY1,
		LAZY2,
		OPTIMAL1,
		OPTIMAL2,
		OPTIMAL3,
	};

	struct CompressorOptions {
		int parser;
		int maxHashLog;
		int maxElementsLog;
		int niceLength;
		int optimalBlockSize;        // (Optimal)
		int maxArrivals;             // (Optimal)
		int parserIterations;        // (Optimal)
	};

	//Tries to find a match between the two locations, and returns the length
	//Note that this function should be called with at least MIN_LENGTH + 8 bytes of buffer after limit
	FORCE_INLINE size_t test_match(const uint8_t* front, const uint8_t* back,
		const uint8_t* const limit, const size_t minLength, const int windowLog)
	{
		//Test first bytes
		//Usually compilers will optimize std::equal as 2 comparisons for minLength 5, 6, etc
		//We can only use one comparison to make it faster. For powers of 2, std::equal should be good
		if (minLength == 3) {
			uint32_t a, b;
			memcpy(&a, front, sizeof(uint32_t));
			memcpy(&b, back, sizeof(uint32_t));
			if ((is_little_endian() && (a << 8) != (b << 8)) || (!is_little_endian() && (a >> 8) != (b >> 8)))
				return 0;
		}
#if IS_64BIT
		else if (minLength == 5) {
			uint64_t a, b;
			memcpy(&a, front, sizeof(uint64_t));
			memcpy(&b, back, sizeof(uint64_t));
			if ((is_little_endian() && (a << 24) != (b << 24)) || (!is_little_endian() && (a >> 24) != (b >> 24)))
				return 0;
		}
		else if (minLength == 6) {
			uint64_t a, b;
			memcpy(&a, front, sizeof(uint64_t));
			memcpy(&b, back, sizeof(uint64_t));
			if ((is_little_endian() && (a << 16) != (b << 16)) || (!is_little_endian() && (a >> 16) != (b >> 16)))
				return 0;
		}
#endif
		else {
			if (!std::equal(back, back + minLength, front))
				return 0;
		}

		//Usually there is a higher chance that the first bytes are not equal
		// than being outside the window, so separating both cases is a speed win
		if ((size_t)(front - back) >> windowLog)
			return 0;

		const uint8_t* const matchOrigin = front;
		front += minLength;
		back += minLength;

		while (true) {
			if (unlikely(front + sizeof(size_t) > limit)) {
				if (front > limit)
					return 0;

				while (*front == *back && front < limit) {
					front++;
					back++;
				}
				return front - matchOrigin;
			}

			//Compare 4 or 8 bytes at a time using xor. It has the property of returning 0 if the two values are equal.
			//In case they differ, we can get the first byte that differs using a bit scan.
			size_t a, b;
			memcpy(&a, front, sizeof(size_t));
			memcpy(&b, back, sizeof(size_t));
			const size_t xorVal = a ^ b;

			if (xorVal) {
				if (is_little_endian())
					front += unsafe_bit_scan_forward(xorVal) >> 3;
				else
					front += unsafe_int_log2(xorVal) >> 3;
				return front - matchOrigin;
			}

			front += sizeof(size_t);
			back += sizeof(size_t);
		}
	}

	//A hash table which does not check for collisions
	template<class Value, class Hash>
	class HashTable {
		Value* arr = nullptr;
		int hashShift;
	public:
		//Use 2^x sizes to avoid the use of modulo operator
		HashTable() {}
		~HashTable() {
			delete[] arr;
		}
		void init(const int logSize) {
			arr = new Value[(size_t)1 << logSize]();
			hashShift = (IS_64BIT ? 64 : 32) - logSize;
		}
		Value& operator[](const size_t value) {
			return arr[Hash{}(value) >> hashShift];
		}
	};

	template<class Value>
	class LZ2WayCacheBucket {
		Value* data;
	public:
		LZ2WayCacheBucket() {}
		LZ2WayCacheBucket(Value* _data) {
			data = _data;
		}
		//Loads the first value, and at the same time pushes a value into that slot.
		void first(Value* value) {
			const Value tmp = data[0];
			data[0] = *value;
			*value = tmp;
		}
		//Loads the second value, and at the same time pushes a value into that slot.
		//Should be used after loading the first value.
		void second(Value* value) {
			const Value tmp = data[1];
			data[1] = *value;
			*value = tmp;
		}
		//Inserts a value into the first slot.
		//Used when skipping bytes.
		void push_in_first(const Value value) {
			const Value tmp = data[0];
			data[0] = value;
			data[1] = tmp;
		}
		//Inserts a value into the second slot.
		//Used when we have checked the first entry, but we wont check the second.
		void push_in_second(const size_t value) {
			data[1] = value;
		}
	};

	//Like a hash table, but stores 2 elements per entry
	template<class Value, class Hash>
	class LZ2WayCacheTable {
		Value* arr = nullptr;
		int hashShift;
	public:
		LZ2WayCacheTable() {}
		~LZ2WayCacheTable() {
			delete[] arr;
		}
		void init(const int logSize) {
			arr = new Value[(size_t)2 << logSize]();
			hashShift = (IS_64BIT ? 64 : 32) - logSize;
		}
		LZ2WayCacheBucket<Value> operator[](const size_t value) {
			return LZ2WayCacheBucket<Value>(arr + (Hash{}(value) >> hashShift) * 2);
		}
	};

	//Used for easier implementation
	template<class Value>
	class LZCacheBucket {
		Value* it;
		Value* last;
	public:
		LZCacheBucket() {}
		LZCacheBucket(Value* _begin, Value* _end) {
			it = _begin;
			last = _end;
		}
		//Pushes a new value into the bucket. Used when skipping bytes
		void push(const Value newValue) {
			//An std::move can be faster, but only for big buckets (8 elements or more)
			for (last--; last != it; last--)
				last[0] = last[-1];
			it[0] = newValue;
		}
		//Whether all positions in the bucket have been loaded
		bool ended() {
			return it == last;
		}
		//Loads the next position, and at the same time, stores the previous position in the same spot
		//The first time value must have the current position (so it basically pushes the new value into the bucket)
		void next(Value* value) {
			const Value tmp = *it;
			*it = *value;
			*value = tmp;
			it++;
		}
		//Loads the next position, but without updating the bucket as we go
		Value next() {
			return *it++;
		}
	};

	//This works like the basic hash table, except that it stores N positions per bucket
	template<class Value, class Hash>
	class LZCacheTable {
		Value* arr = nullptr;
		int hashShift;
		int elementsLog;
	public:
		//Use 2^x sizes to avoid the use of modulo and multiplication
		LZCacheTable() {}
		LZCacheTable(const int logSize, const int numElementsLog) {
			init(logSize, numElementsLog);
		}
		~LZCacheTable() {
			delete[] arr;
		}
		void init(const int logSize, const int numElementsLog) {
			arr = new Value[(size_t)1 << logSize << numElementsLog]();
			hashShift = (IS_64BIT ? 64 : 32) - logSize;
			elementsLog = numElementsLog;
		}
		LZCacheBucket<Value> operator[](size_t value) {
			value = Hash{}(value) >> hashShift;
			Value* bucket = arr + (value << elementsLog);
			return LZCacheBucket<Value>(bucket, bucket + ((size_t)1 << elementsLog));
		}
	};

	struct LZStructure {
		uint32_t matchLength;
		uint32_t matchDistance;
		uint32_t literalRunLength;
	};

	struct LZMatch {
		uint32_t length;
		uint32_t distance;
	};

	//Simple and fast
	class HashTableMatchFinder {
		HashTable<uint32_t, FastIntHash> lzdict3;
		LZCacheTable<uint32_t, FastIntHash> lzdict4;
		LZCacheTable<uint32_t, FastIntHash> lzdict8;

	public:
		void init(const int windowLog, const CompressorOptions& compressorOptions) {
			const int hashLog = std::min(compressorOptions.maxHashLog, windowLog - 3);
			lzdict3.init(std::min(hashLog, 14));
			lzdict4.init(hashLog, compressorOptions.maxElementsLog);
			lzdict8.init(hashLog, compressorOptions.maxElementsLog);
		}

		LZMatch* find_matches_and_update(const uint8_t* const input, const uint8_t* const inputStart,
			const uint8_t* const limit, LZMatch* matches, size_t minLength, const int windowLog, const CompressorOptions& compressorOptions)
		{
			uint32_t& chain3 = lzdict3[read_hash3(input)];
			size_t nextExpectedLength = minLength;

			if (nextExpectedLength <= 3) {
				const uint8_t* where = inputStart + chain3;
				size_t length = test_match(input, where, limit, 3, windowLog);

				if (length >= nextExpectedLength) {
					matches->distance = input - where;
					matches->length = length;
					matches++;

					if (length >= compressorOptions.niceLength) {
						chain3 = input - inputStart;
						return matches;

					}
					nextExpectedLength = length + 1;
				}
			}
			chain3 = input - inputStart;

			if (nextExpectedLength <= 7) {
				LZCacheBucket<uint32_t> chain4 = lzdict4[read_hash4(input)];
				uint32_t pos = input - inputStart;
				while (!chain4.ended()) {
					chain4.next(&pos);

					const uint8_t* where = inputStart + pos;

					if (*(input + nextExpectedLength - 1) != *(where + nextExpectedLength - 1))
						continue;

					const size_t length = test_match(input, where, limit, 4, windowLog);

					if (length >= nextExpectedLength) {
						matches->distance = input - where;

						matches->length = length;
						matches++;

						nextExpectedLength = length + 1;
						if (length >= 7) {
							while (!chain4.ended())
								chain4.next(&pos);
							break;
						}
					}
				}
			}

			if (nextExpectedLength >= 4 && nextExpectedLength <= compressorOptions.niceLength) {
				LZCacheBucket<uint32_t> chain8 = lzdict8[read_hash8(input)];
				uint32_t pos = input - inputStart;
				while (!chain8.ended()) {
					chain8.next(&pos);

					const uint8_t* where = inputStart + pos;
					if (*(input + nextExpectedLength - 1) != *(where + nextExpectedLength - 1))
						continue;

					const size_t length = test_match(input, where, limit, 8, windowLog);

					if (length >= nextExpectedLength) {
						matches->distance = input - where;
						matches->length = length;
						matches++;

						nextExpectedLength = length + 1;
						if (length >= compressorOptions.niceLength) {
							while (!chain8.ended())
								chain8.next(&pos);
							break;
						}
					}
				}
			}

			return matches;
		}

		void update_position(const uint8_t* const input, const uint8_t* const inputStart) {
			const size_t pos = input - inputStart;
			lzdict3[read_hash3(input)] = pos;
			lzdict4[read_hash4(input)].push(pos);
			lzdict8[read_hash8(input)].push(pos);
		}
	};

	const int NO_MATCH_POS = 0;
	//Original match finder implementation from BriefLZ
	class BinaryMatchFinder {

		HashTable<uint32_t, FastIntHash> lzdict3;
		HashTable<uint32_t, FastIntHash> nodeLookup;
		uint32_t* nodes = nullptr;
		size_t nodeListSize;
		size_t nodeListMask;

	public:

		~BinaryMatchFinder() {
			delete[] nodes;
		}
		BinaryMatchFinder() {}

		void init(const size_t inputSize, const int windowLog, const CompressorOptions& compressorOptions) {

			const size_t binaryTreeWindow = std::min(compressorOptions.maxHashLog, windowLog);

			//Input size is smaller than maximum binary tree size
			if (inputSize < ((size_t)1 << binaryTreeWindow)) {
				nodes = new uint32_t[(size_t)2 * inputSize];
				nodeListSize = inputSize;
				nodeListMask = -1;
			}
			else {
				nodes = new uint32_t[(size_t)2 << binaryTreeWindow];
				nodeListSize = (size_t)1 << binaryTreeWindow;
				nodeListMask = nodeListSize - 1;
			}
			if (compressorOptions.parser < OPTIMAL3)
				lzdict3.init(std::min(14, windowLog - 3));
			nodeLookup.init(std::min(20, windowLog - 3));
		}

		LZMatch* find_matches_and_update(const uint8_t* const input, const uint8_t* const inputStart,
			const uint8_t* const compressionLimit, const uint8_t* const blockLimit, LZMatch* matches,
			size_t minLength, const int windowLog, const CompressorOptions& compressorOptions)
		{
			const size_t inputPosition = input - inputStart;
			size_t nextExpectedLength = minLength;

			// First try to get a length 3 match
			if (compressorOptions.parser < OPTIMAL3) {
				uint32_t& chain3 = lzdict3[read_hash3(input)];
				if (nextExpectedLength <= 3) {
					const uint8_t* where = inputStart + chain3;
					const size_t length = test_match(input, where, blockLimit, 3, windowLog);

					if (length >= nextExpectedLength) {
						matches->distance = input - where;
						matches->length = length;
						matches++;

						if (length >= compressorOptions.niceLength) {
							update_position(input, inputStart, compressionLimit, windowLog, compressorOptions);
							return matches;
						}
						nextExpectedLength = length + 1;
					}
				}
				chain3 = inputPosition;
			}

			//If we reach this position stop the search
			const size_t btEnd = inputPosition < nodeListSize ? 0 : inputPosition - nodeListSize;

			uint32_t& lookupEntry = nodeLookup[compressorOptions.parser == OPTIMAL3 ? read_hash3(input) : read_hash4(input)];
			size_t backPosition = lookupEntry;
			lookupEntry = inputPosition;

			uint32_t* lesserNode = &nodes[2 * (lookupEntry & nodeListMask)];
			uint32_t* greaterNode = lesserNode + 1;
			const uint8_t* lesserFront = input;
			const uint8_t* greaterFront = input;

			size_t depth = (size_t)1 << compressorOptions.maxElementsLog;

			// Check matches
			while (true) {

				if (backPosition <= btEnd || depth-- == 0) {
					*lesserNode = NO_MATCH_POS;
					*greaterNode = NO_MATCH_POS;
					break;
				}

				const uint8_t* front = std::min(lesserFront, greaterFront);
				const uint8_t* back = front - (inputPosition - backPosition);

				const size_t extraLength = test_match(front, back, compressionLimit, 0, windowLog);
				front += extraLength;
				back += extraLength;

				size_t length = front - input;
				//Match cant go outside of block boundaries
				const size_t effectiveLength = std::min(length, (size_t)(blockLimit - input));
				uint32_t* const nextNode = &nodes[2 * (backPosition & nodeListMask)];
				if (effectiveLength >= nextExpectedLength) {
					nextExpectedLength = effectiveLength;
					if (compressorOptions.parser != OPTIMAL3)
						nextExpectedLength++;
					matches->distance = front - back;
					matches->length = effectiveLength;
					matches++;
				}

				if (length >= compressorOptions.niceLength) {
					*lesserNode = nextNode[0];
					*greaterNode = nextNode[1];
					return matches;
				}

				if (*back < *front) {
					*lesserNode = backPosition;
					lesserNode = &nextNode[1];
					backPosition = *lesserNode;
					lesserFront = front;
				}
				else {
					*greaterNode = backPosition;
					greaterNode = &nextNode[0];
					backPosition = *greaterNode;
					greaterFront = front;
				}
			}

			return matches;
		}

		void update_position(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const limit,
			const int windowLog, const CompressorOptions& compressorOptions) {

			const size_t inputPosition = input - inputStart;
			if (compressorOptions.parser < OPTIMAL3)
				lzdict3[read_hash3(input)] = inputPosition;

			//If we reach this position on the front stop the update
			const uint8_t* positionSkip = std::min(limit, input + compressorOptions.niceLength);
			//If we reach this position on the back stop the update
			const size_t btEnd = inputPosition < nodeListSize ? 0 : inputPosition - nodeListSize;
			uint32_t& lookupEntry = nodeLookup[compressorOptions.parser == OPTIMAL3 ? read_hash3(input) : read_hash4(input)];
			size_t backPosition = lookupEntry;
			lookupEntry = inputPosition;

			uint32_t* lesserNode = &nodes[2 * (inputPosition & nodeListMask)];
			uint32_t* greaterNode = &nodes[2 * (inputPosition & nodeListMask) + 1];

			const uint8_t* lesserFront = input;
			const uint8_t* greaterFront = input;
			size_t depth = (size_t)1 << compressorOptions.maxElementsLog;

			// Check matches
			while (true) {
				if (backPosition <= btEnd || depth-- == 0) {
					*lesserNode = NO_MATCH_POS;
					*greaterNode = NO_MATCH_POS;
					return;
				}

				const uint8_t* front = std::min(lesserFront, greaterFront);
				const uint8_t* back = front - (inputPosition - backPosition);

				const size_t length = test_match(front, back, positionSkip, 0, windowLog);
				front += length;
				back += length;

				uint32_t* const nextNode = &nodes[2 * (backPosition & nodeListMask)];
				if (front >= positionSkip) {
					*lesserNode = nextNode[0];
					*greaterNode = nextNode[1];
					return;
				}

				if (*back < *front) {
					*lesserNode = backPosition;
					lesserNode = &nextNode[1];
					backPosition = *lesserNode;
					lesserFront = front;
				}
				else {
					*greaterNode = backPosition;
					greaterNode = &nextNode[0];
					backPosition = *greaterNode;
					greaterFront = front;
				}
			}
		}
	};

	enum {
		ENTROPY_RAW = 0,
		ENTROPY_HUFFMAN,
	};

	struct HuffmanSymbol {
		uint32_t symbol;
		uint32_t count;
		uint32_t bits;
		uint32_t code;
	};

	struct {
		bool operator()(HuffmanSymbol a, HuffmanSymbol b) const { return a.count > b.count; }
	} HuffmanSortByCount;

	struct {
		bool operator()(HuffmanSymbol a, HuffmanSymbol b) const {
			if (a.bits != b.bits)
				return a.bits < b.bits;
			return a.symbol < b.symbol;
		}
	} HuffmanSortByBitsAndSymbol;

	struct {
		bool operator()(HuffmanSymbol a, HuffmanSymbol b) const { return a.symbol < b.symbol; }
	} HuffmanSortBySymbol;

	const uint32_t RANS_BIT_PRECISION = 16;
	const uint32_t RANS_NORMALIZATION_INTERVAL = 1 << (RANS_BIT_PRECISION - 8);
	const uint32_t RANS_MODEL_BIT_PRECISION = 8;

	struct alignas(4) RansSymbol {
		uint8_t symbol;
		uint8_t freq;
		uint8_t low;
	};

	struct PackageMergeNode {
		std::vector<uint8_t> symbols;
		size_t count;
	};

	struct {
		bool operator()(PackageMergeNode* a, PackageMergeNode* b) const { return a->count < b->count; }
	} SortPackageMergeNodePtr;

	//From https://experiencestack.co/length-limited-huffman-codes-21971f021d43
	void package_merge(HuffmanSymbol symbolData[256]) {

		PackageMergeNode originalList[256];
		size_t originalListSize = 0;
		for (size_t i = 0; i < 256; i++) {
			if (symbolData[i].count) {
				originalList[originalListSize].count = symbolData[i].count;
				originalList[originalListSize].symbols.push_back(i);
				originalListSize++;
				symbolData[i].bits = 0;
			}
			else
				symbolData[i].bits = MAX_HUFFMAN_CODE_LENGTH + 1;
		}

		//To avoid the continuous copy of vectors of symbols from the nodes, the merged list will
		// consist of pointers to the nodes from the original list and the new nodes from last iteration
		size_t mergedListSize = 0;
		PackageMergeNode* mergedList[256 * 2 - 1];
		for (mergedListSize = 0; mergedListSize < originalListSize; mergedListSize++)
			mergedList[mergedListSize] = &originalList[mergedListSize];

		//We will need 2 buffers for the nodes generated in each iteration: one for the previous iteration,
		// and one for the current. We will swap them to avoid copying the current into the previous.
		PackageMergeNode newNodesBufferA[255];
		PackageMergeNode newNodesBufferB[255];

		for (int i = 1; i < MAX_HUFFMAN_CODE_LENGTH; i++) {

			std::sort(mergedList, mergedList + mergedListSize, SortPackageMergeNodePtr);
			PackageMergeNode* newNodes = (i % 2) ? newNodesBufferA : newNodesBufferB;
			size_t numberNewNodes = 0;

			//If odd drop the last element
			for (size_t j = 0; j < (mergedListSize & ~1); j += 2) {
				newNodes[numberNewNodes].count = mergedList[j]->count + mergedList[j + 1]->count;
				newNodes[numberNewNodes].symbols = mergedList[j]->symbols;
				newNodes[numberNewNodes].symbols.insert(newNodes[numberNewNodes].symbols.end(), mergedList[j + 1]->symbols.begin(), mergedList[j + 1]->symbols.end());
				numberNewNodes++;
			}

			for (mergedListSize = 0; mergedListSize < originalListSize; mergedListSize++)
				mergedList[mergedListSize] = &originalList[mergedListSize];
			for (size_t j = 0; j < numberNewNodes; j++, mergedListSize++)
				mergedList[mergedListSize] = &newNodes[j];
			mergedListSize = originalListSize + numberNewNodes;
		}

		//We will only need this number of elements at the end
		size_t maxListLength = originalListSize * 2 - 2;
		for (int i = 0; i < maxListLength; i++) {
			for (int j = 0; j < mergedList[i]->symbols.size(); j++)
				symbolData[mergedList[i]->symbols[j]].bits++;
		}
	}

	void fast_huffman_codegen(HuffmanSymbol symbolData[256]) {
		//Dumb huffman code generation based on http://www.ezcodesample.com/prefixer/prefixer_article.html
		//counts[0] has the highest count
		std::sort(symbolData, symbolData + 256, HuffmanSortByCount);

		//Round frequencies to the closest power of 2 in log scale (works better than round down)
		int countSum = 0;
		for (int i = 0; i < 256; i++) {
			if (symbolData[i].count) {
				symbolData[i].count = 1 << (int)round(fast_log2(symbolData[i].count));
				countSum += symbolData[i].count;
			}
		}

		//Scale frequencies to fit into code space
		while (countSum > HUFFMAN_CODE_SPACE) {
			for (int i = 0; i < 256; i++) {
				if (symbolData[i].count > 1) {
					symbolData[i].count >>= 1;
					countSum -= symbolData[i].count;
				}
			}
		}

		int remainingCodeSpace = HUFFMAN_CODE_SPACE - countSum;
		//Distribute remaining code space
		while (remainingCodeSpace) {
			for (int i = 0; i < 256; i++) {
				if (symbolData[i].count <= remainingCodeSpace) {
					remainingCodeSpace -= symbolData[i].count;
					symbolData[i].count <<= 1;
				}
			}
		}

		//Give code lengths
		for (int i = 0; i < 256; i++) {
			if (symbolData[i].count)
				symbolData[i].bits = MAX_HUFFMAN_CODE_LENGTH - unsafe_int_log2(symbolData[i].count);
			else
				symbolData[i].bits = MAX_HUFFMAN_CODE_LENGTH + 1;
		}
	}

	class RansEncoder {

		int values[256];
		//rans stream has to be written backwards, store it first in a buffer
		uint8_t streamBuffer[256];

	public:
		RansEncoder() {}
		~RansEncoder() {}

		void add_symbol(int bits, int index) {
			values[index] = bits;
		}

		FORCE_INLINE void renormalize(const size_t interval, uint16_t& state, uint8_t*& streamBufferIt) {
			bool renormalize = state >= interval;
			streamBufferIt[-1] = state;
			streamBufferIt -= renormalize;
			state >>= renormalize << 3;
		}
		//Encodes all the symbols stored into a rans stream and returns the encoded size
		size_t encode_tree(uint8_t* output) {

			uint32_t counts[MAX_HUFFMAN_CODE_LENGTH + 2] = { 0 };
			for (size_t i = 0; i < 256; i++)
				counts[values[i]]++;

			RansSymbol symbols[MAX_HUFFMAN_CODE_LENGTH + 2];
			size_t accumulator = 0;
			for (size_t i = 1; i <= MAX_HUFFMAN_CODE_LENGTH; i++) {
				symbols[i].freq = counts[i];
				symbols[i].low = accumulator;
				accumulator += counts[i];
				//A byte can hold a maximum value of 255, while max freq is 256, but note
				// that can only happen if ALL symbols have 8 bits. In that case we would send it uncompressed.
				*output++ = counts[i];
			}

			//It is not necessary to send the count for "symbol not present",
			symbols[MAX_HUFFMAN_CODE_LENGTH + 1].freq = counts[MAX_HUFFMAN_CODE_LENGTH + 1];
			symbols[MAX_HUFFMAN_CODE_LENGTH + 1].low = accumulator;

			//Rans encoding
			uint16_t stateA = RANS_NORMALIZATION_INTERVAL;
			uint16_t stateB = RANS_NORMALIZATION_INTERVAL;
			uint8_t* streamBufferIt = streamBuffer + 256;
			for (size_t i = 256; i > 0; i -= 2) {
				size_t interval = (RANS_NORMALIZATION_INTERVAL << (8 - RANS_MODEL_BIT_PRECISION)) * symbols[values[i - 1]].freq;
				renormalize(interval, stateB, streamBufferIt);
				stateB = ((stateB / symbols[values[i - 1]].freq) << RANS_MODEL_BIT_PRECISION) + (stateB % symbols[values[i - 1]].freq) + symbols[values[i - 1]].low;

				interval = (RANS_NORMALIZATION_INTERVAL << (8 - RANS_MODEL_BIT_PRECISION)) * symbols[values[i - 2]].freq;
				renormalize(interval, stateA, streamBufferIt);
				stateA = ((stateA / symbols[values[i - 2]].freq) << RANS_MODEL_BIT_PRECISION) + (stateA % symbols[values[i - 2]].freq) + symbols[values[i - 2]].low;
			}
			streamBufferIt -= 4;
			write_uint16le(streamBufferIt + 0, stateA);
			write_uint16le(streamBufferIt + 2, stateB);
			size_t streamSize = streamBuffer + 256 - streamBufferIt;
			memcpy(output, streamBufferIt, streamSize);
			return streamSize + MAX_HUFFMAN_CODE_LENGTH;
		}
	};

	class HuffmanEncoder {

		//huffman streams will be written backwards, store them first in a buffer
		uint8_t* streamBuffer = nullptr;
		uint8_t* streamBufferBegin;
		//store huffman symbols
		uint8_t* symbolBuffer = nullptr;
		uint8_t* symbolBufferIt;
		HuffmanSymbol symbolData[256];

	public:
		HuffmanEncoder() {}
		~HuffmanEncoder() {
			delete[] streamBuffer;
			delete[] symbolBuffer;
		}

		//Returns whether it fails to initialize
		bool initialize_huffman_encoder(const size_t bufferSize, bool initializeStreamBuffer = true, bool initializeSymbolBuffer = true) {

			try {
				if (initializeStreamBuffer) {
					streamBuffer = new uint8_t[bufferSize];
					streamBufferBegin = streamBuffer + bufferSize;
				}
				if (initializeSymbolBuffer)
					symbolBuffer = new uint8_t[bufferSize];
			}
			catch (const std::bad_alloc& e) {
				delete[] streamBuffer;
				delete[] symbolBuffer;
				return true;
			}
			return false;
		}

		void start_huffman_block(uint8_t* outputBuffer = nullptr) {
			if (outputBuffer)
				symbolBufferIt = outputBuffer + 3;
			else
				symbolBufferIt = symbolBuffer;
		}
		FORCE_INLINE void add_distance(const uint32_t distance, size_t bytes) {
			write_uint32le(symbolBufferIt, distance);
			symbolBufferIt += bytes;
		}
		//Up to 8 elements
		FORCE_INLINE void add_literals_short(const uint8_t* literals, size_t count) {
			memcpy(symbolBufferIt, literals, 8);
			symbolBufferIt += count;
		}
		//Any number of elements
		FORCE_INLINE void add_literals_long(const uint8_t* literals, size_t count) {
			uint8_t* end = symbolBufferIt + count;
			do {
				memcpy(symbolBufferIt + 0, literals + 0, 16);
				memcpy(symbolBufferIt + 16, literals + 16, 16);
				literals += 32;
				symbolBufferIt += 32;
			} while (symbolBufferIt < end);
			symbolBufferIt = end;
		}
		FORCE_INLINE void add_byte(const uint8_t literal) {
			*symbolBufferIt++ = literal;
		}
		FORCE_INLINE size_t bytes_in_buffer() {
			return symbolBufferIt - symbolBuffer;
		}

		void symbol_histogram(const uint8_t* data, size_t count, uint32_t histogram[8][256]) {

			memset(histogram, 0, 8 * 256 * sizeof(uint32_t));
			const uint8_t* const fastLoopEnd = data + (count & ~0x7);
			const uint8_t* const end = data + count;
			for (; data < fastLoopEnd; data += 8) {
				if (IS_64BIT) {
					uint64_t w;
					memcpy(&w, data, 8);
					histogram[0][w >> 0 & 0xFF]++;
					histogram[1][w >> 8 & 0xFF]++;
					histogram[2][w >> 16 & 0xFF]++;
					histogram[3][w >> 24 & 0xFF]++;
					histogram[4][w >> 32 & 0xFF]++;
					histogram[5][w >> 40 & 0xFF]++;
					histogram[6][w >> 48 & 0xFF]++;
					histogram[7][w >> 56 & 0xFF]++;
				}
				else {
					uint32_t w;
					memcpy(&w, data, 4);
					histogram[0][w >> 0 & 0xFF]++;
					histogram[1][w >> 8 & 0xFF]++;
					histogram[2][w >> 16 & 0xFF]++;
					histogram[3][w >> 24 & 0xFF]++;
					memcpy(&w, data + 4, 4);
					histogram[4][w >> 0 & 0xFF]++;
					histogram[5][w >> 8 & 0xFF]++;
					histogram[6][w >> 16 & 0xFF]++;
					histogram[7][w >> 24 & 0xFF]++;
				}
			}
			for (; data < end; data++)
				histogram[0][*data]++;
		}
		float calculate_entropy(HuffmanSymbol counts[256], size_t symbolCount) {
			float entropy = 0;
			const float probDiv = 1 / float(symbolCount);
			for (int i = 0; i < 256; i++) {
				if (counts[i].count) {
					const float prob = counts[i].count * probDiv;
					entropy += -(prob * fast_log2(prob));
				}
			}
			return entropy;
		}
		FORCE_INLINE int sum_counts(HuffmanSymbol counts[256]) {
			int acc = 0;
			for (int i = 0; i < 256; i++)
				acc += counts[i].count;
			return acc;
		}
		void write_entropy_header(uint8_t*& output, uint32_t symbolCount, int mode) {
			uint32_t out;
			out = symbolCount << 5;
			out |= mode;
			write_uint24le(output, out);
			output += 3;
		}
		FORCE_INLINE void encode_op(HuffmanSymbol symbolData[256], const uint8_t symbol, size_t& state, size_t& bitCount) {
			state = (state << symbolData[symbol].bits) | symbolData[symbol].code;
			bitCount += symbolData[symbol].bits;
		}
		FORCE_INLINE void renormalize(size_t& state, size_t& bitCount, uint8_t*& streamBufferIt) {
			//Branchless write
			if (IS_64BIT)
				write_uint64le(streamBufferIt - 8, state << (64 - bitCount));
			else
				write_uint32le(streamBufferIt - 4, state << (32 - bitCount));
			streamBufferIt -= bitCount / 8;
			bitCount &= 7;
		}
		void end_stream(uint64_t state, size_t bitCount, uint8_t*& streamBufferIt) {
			while (bitCount >= 8) {
				bitCount -= 8;
				streamBufferIt--;
				*streamBufferIt = state >> bitCount;
			}
			if (bitCount > 0) {
				streamBufferIt--;
				*streamBufferIt = state << (8 - bitCount);
			}
		}
		size_t encode_stream(HuffmanSymbol symbolData[256], const uint8_t* symbolIt, const uint8_t* symbolEnd, uint8_t* output) {

			const uint8_t* const fastLoopEnd = symbolIt + (symbolEnd - symbolIt) % (IS_64BIT ? 30 : 12);
			uint8_t* streamBufferIt = streamBufferBegin;
			size_t state = 0;
			size_t bitCount = 0;

			//Fast loop: write 5 symbols per iteration
			for (; symbolIt < fastLoopEnd;) {
				encode_op(symbolData, symbolIt[0], state, bitCount);
				encode_op(symbolData, symbolIt[6], state, bitCount);
				if (IS_64BIT) {
					encode_op(symbolData, symbolIt[12], state, bitCount);
					encode_op(symbolData, symbolIt[18], state, bitCount);
					encode_op(symbolData, symbolIt[24], state, bitCount);
				}
				symbolIt += IS_64BIT ? 30 : 12;
				renormalize(state, bitCount, streamBufferIt);
			}
			for (; symbolIt < symbolEnd; ) {
				encode_op(symbolData, symbolIt[0], state, bitCount);
				symbolIt += 6;  //Number of streams
				renormalize(state, bitCount, streamBufferIt);
			}
			//Output remaining bits
			end_stream(state, bitCount, streamBufferIt);

			size_t streamSize = streamBufferBegin - streamBufferIt;
			memcpy(output, streamBufferIt, streamSize);
			return streamSize;
		}
		size_t store_block_raw(const uint8_t* symbols, uint8_t* output, const size_t symbolCount) {
			write_entropy_header(output, symbolCount, ENTROPY_RAW);
			memcpy(output, symbols, symbolCount);
			return symbolCount + 3;
		}
		HuffmanSymbol* get_huffman_codes(float decSpeedBias, bool* uncompressed) {

			const size_t symbolCount = symbolBufferIt - symbolBuffer;
			if (symbolCount < 512) {
				for (int i = 0; i < 256; i++)
					symbolData[i].bits = 8;
				*uncompressed = true;
				return symbolData;
			}
			float maxBitsPerByte = 7.9 - 4 * decSpeedBias;

			uint32_t histogram[8][256];
			symbol_histogram(symbolBuffer, symbolCount, histogram);

			for (int i = 0; i < 256; i++) {
				symbolData[i].symbol = i;
				symbolData[i].count = histogram[0][i] + histogram[1][i] + histogram[2][i] + histogram[3][i] +
					histogram[4][i] + histogram[5][i] + histogram[6][i] + histogram[7][i];
			}

			//Check if there is only one unique symbol. If that is the case, give some probability to
			// another symbol. Having a probability of 1 might break something
			for (int i = 0; i < 256; i++) {
				if (symbolData[i].count == symbolCount) {
					symbolData[i].count--;
					symbolData[(i + 1) % 256].count = 1;
					break;
				}
			}

			package_merge(symbolData);
			size_t compressedSize = 0;
			for (int i = 0; i < 256; i++)
				compressedSize += symbolData[i].bits * symbolData[i].count;

			//Not compressible
			if (compressedSize + 128 * 8 > symbolCount * maxBitsPerByte) {
				for (int i = 0; i < 256; i++)
					symbolData[i].bits = 8;
				*uncompressed = true;
			}
			else
				*uncompressed = false;
			
			return symbolData;
		}
		FORCE_INLINE size_t generate_codes_fast(uint8_t* output, const float decSpeedBias, bool useFastCodeGen) {

			//This means we are directly writing to output, so just write the header
			if (!symbolBuffer) {
				size_t symbolCount = symbolBufferIt - output - 3;
				write_entropy_header(output, symbolCount, ENTROPY_RAW);
				return symbolCount + 3;
			}

			const size_t symbolCount = symbolBufferIt - symbolBuffer;
			//Not enough symbols to justify compression or disabled huffman coding
			if (symbolCount < 512 || !streamBuffer)
				return store_block_raw(symbolBuffer, output, symbolCount);

			uint8_t* const outputStart = output;
			float maxBitsPerByte = 7.9 - 4 * decSpeedBias;

			uint32_t histogram[8][256];
			symbol_histogram(symbolBuffer, symbolCount, histogram);
			
			for (int i = 0; i < 256; i++) {
				symbolData[i].symbol = i;
				symbolData[i].count = histogram[0][i] + histogram[1][i] + histogram[2][i] + histogram[3][i] +
					histogram[4][i] + histogram[5][i] + histogram[6][i] + histogram[7][i];
			}

			//Check if there is only one unique symbol. If that is the case, give some probability to
			// another symbol. Having a probability of 1 might break something
			for (int i = 0; i < 256; i++) {
				if (symbolData[i].count == symbolCount) {
					symbolData[i].count--;
					symbolData[(i + 1) % 256].count = 1;
					break;
				}
			}

			//Not compressible
			float entropy = calculate_entropy(symbolData, symbolCount);
			if (symbolCount * entropy + 128 * 8 >= symbolCount * maxBitsPerByte)
				return store_block_raw(symbolBuffer, output, symbolCount);

			if (useFastCodeGen)
				fast_huffman_codegen(symbolData);
			else
				package_merge(symbolData);

			std::sort(symbolData, symbolData + 256, HuffmanSortByBitsAndSymbol);

			//Convert counts to bit lengths and codes, and sort by symbol
			HuffmanSymbol finalSymbols[256];
			int accumulator = 0;
			//Convert counts to codes
			for (int i = 0; i < 256; i++) {
				finalSymbols[symbolData[i].symbol].bits = symbolData[i].bits;
				if (!symbolData[i].bits)
					continue;
				finalSymbols[symbolData[i].symbol].bits = symbolData[i].bits;
				finalSymbols[symbolData[i].symbol].code = accumulator >> (MAX_HUFFMAN_CODE_LENGTH - symbolData[i].bits);
				accumulator += HUFFMAN_CODE_SPACE >> symbolData[i].bits;
			}

			write_entropy_header(output, symbolCount, ENTROPY_HUFFMAN);
			//Store the tree
			RansEncoder normalTreeEncoder;
			for (int i = 0; i < 256; i++)
				normalTreeEncoder.add_symbol(finalSymbols[i].bits, i);
			size_t treeSize = normalTreeEncoder.encode_tree(output);
			output += treeSize;

			uint8_t* jumpTable = output;
			output += 12;

			//Encode the 6 streams
			size_t streamSize;
			streamSize = encode_stream(finalSymbols, symbolBuffer + 0, symbolBuffer + symbolCount, output);
			write_uint16le(jumpTable + 0, streamSize);
			output += streamSize;
			streamSize = encode_stream(finalSymbols, symbolBuffer + 1, symbolBuffer + symbolCount, output);
			write_uint16le(jumpTable + 2, streamSize);
			output += streamSize;
			streamSize = encode_stream(finalSymbols, symbolBuffer + 2, symbolBuffer + symbolCount, output);
			write_uint16le(jumpTable + 4, streamSize);
			output += streamSize;
			streamSize = encode_stream(finalSymbols, symbolBuffer + 3, symbolBuffer + symbolCount, output);
			write_uint16le(jumpTable + 6, streamSize);
			output += streamSize;
			streamSize = encode_stream(finalSymbols, symbolBuffer + 4, symbolBuffer + symbolCount, output);
			write_uint16le(jumpTable + 8, streamSize);
			output += streamSize;
			streamSize = encode_stream(finalSymbols, symbolBuffer + 5, symbolBuffer + symbolCount, output);
			write_uint16le(jumpTable + 10, streamSize);
			output += streamSize;

			//If compressed size is very close to uncompressed size send it uncompressed
			if ((output - outputStart + 128) * 8 >= symbolCount * maxBitsPerByte) {
				output = outputStart;
				return store_block_raw(symbolBuffer, output, symbolCount);
			}

			return output - outputStart;
		}
	};

	FORCE_INLINE void encode_length(HuffmanEncoder* lengthEncoder, size_t var, const size_t overflow) {
		var -= overflow;
		if (likely(var <= 251))
			lengthEncoder->add_byte(var);
		else {
			var -= 252;
			lengthEncoder->add_byte(252 + (var & 3));
			lengthEncoder->add_distance(var >> 2, 2);
		}
	}

	FORCE_INLINE void encode_literal_run(HuffmanEncoder* literalEncoder, HuffmanEncoder* lengthEncoder, const uint8_t* const input,
		const uint8_t* const literalRunStart, uint8_t* const controlByte) {

		const size_t literalRunLength = input - literalRunStart;

		if (literalRunLength >= 7) {
			*controlByte = (7 << 5);
			encode_length(lengthEncoder, literalRunLength, 7);
			literalEncoder->add_literals_long(literalRunStart, literalRunLength);
		}
		else {
			*controlByte = (literalRunLength << 5);
			//It is faster to unconditionally copy 8 bytes
			literalEncoder->add_literals_short(literalRunStart, literalRunLength);
		}
	}

	//Only encodes matches with rep offset
	FORCE_INLINE void encode_rep_match(HuffmanEncoder* tokenEncoder, HuffmanEncoder* lengthEncoder, uint8_t* const controlByte, size_t* matchLength) {

		*matchLength -= SKANDA_MIN_MATCH_LENGTH;
		if (unlikely(*matchLength >= 7)) {
			*controlByte |= 7;
			encode_length(lengthEncoder, *matchLength, 7);
		}
		else
			*controlByte |= *matchLength;
		tokenEncoder->add_byte(*controlByte);
	}

	//Only encodes matches with no rep offset
	FORCE_INLINE void encode_normal_match(HuffmanEncoder* tokenEncoder, HuffmanEncoder* lengthEncoder, HuffmanEncoder* distanceEncoder, uint8_t* const controlByte,
		size_t* matchLength, const size_t& distance, size_t* repOffset) {

		*matchLength -= SKANDA_MIN_MATCH_LENGTH;
		*repOffset = distance;

		size_t bytes = unsafe_int_log2(distance) / 8 + 1;
		distanceEncoder->add_distance(distance, bytes);
		*controlByte |= bytes << 3;

		if (unlikely(*matchLength >= 7)) {
			*controlByte |= 7;
			encode_length(lengthEncoder, *matchLength, 7);
		}
		else
			*controlByte |= *matchLength;

		tokenEncoder->add_byte(*controlByte);
	}

	//Selects between encoding a normal or a rep match
	FORCE_INLINE void encode_match(HuffmanEncoder* tokenEncoder, HuffmanEncoder* lengthEncoder, HuffmanEncoder* distanceEncoder, uint8_t* const controlByte,
		size_t* matchLength, const size_t& distance, size_t* repOffset) {

		if (*repOffset == distance)
			encode_rep_match(tokenEncoder, lengthEncoder, controlByte, matchLength);
		else
			encode_normal_match(tokenEncoder, lengthEncoder, distanceEncoder, controlByte, matchLength, distance, repOffset);
	}

	FORCE_INLINE void write_block_header(uint8_t*& output, size_t blockSize) {
		write_uint24le(output, blockSize - 1);
		output += 3;
	}

	size_t compress_ultra_fast(const uint8_t* input, const size_t size, uint8_t* output, const int windowLog, 
		const float decSpeedBias, ProgressCallback* progress) 
	{
		//Constants for this encoder
		const size_t hashTableSize = 14;
		const size_t accelerationThreshold = 4;
		const size_t accelerationMax = 63;

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		//The last match or literal run must end before this limit
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;

		size_t repOffset = 1;
		//Keeps track of how many match searches we have made without success.
		//When we dont find matches, we will skip more or less depending on this variable.
		size_t acceleration = 1 << accelerationThreshold;
		//First byte is send uncompressed
		*output++ = *input++;

		//The match finder is a simple hash table with constant size
		HashTable<uint32_t, FastIntHash> lzdict;
		try {
			lzdict.init(hashTableSize);
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		HuffmanEncoder literalEncoder;
		HuffmanEncoder tokenEncoder;
		HuffmanEncoder lengthEncoder;
		HuffmanEncoder distanceEncoder;

		bool disableHuffman = decSpeedBias >= 0.99;
		size_t huffmanBufSize = std::min(MAX_BLOCK_SIZE, size);

		if (literalEncoder.initialize_huffman_encoder(huffmanBufSize + 128, !disableHuffman, !disableHuffman))
			return -1;
		if (tokenEncoder.initialize_huffman_encoder(huffmanBufSize / 2 + 128, !disableHuffman))
			return -1;
		if (lengthEncoder.initialize_huffman_encoder(huffmanBufSize / 8 + 128, !disableHuffman))
			return -1;
		if (distanceEncoder.initialize_huffman_encoder(huffmanBufSize + 128, !disableHuffman))
			return -1;

		//Main compression loop
		while (input < compressionLimit) {

			const size_t thisBlockSize = compressionLimit - input < MAX_BLOCK_SIZE ? compressionLimit - input : MAX_BLOCK_SIZE;
			const uint8_t* const thisBlockEnd = input + thisBlockSize;
			const uint8_t* const thisBlockStart = input;

			write_block_header(output, thisBlockSize);

			literalEncoder.start_huffman_block(disableHuffman ? output : nullptr);
			tokenEncoder.start_huffman_block();
			lengthEncoder.start_huffman_block();
			distanceEncoder.start_huffman_block();
			const uint8_t* literalRunStart = input;

			while (likely(input < thisBlockEnd)) {

				//Get possible match location and update the table
				uint32_t* const dictEntry = &lzdict[read_hash6(input)];
				const uint8_t* match = inputStart + *dictEntry;
				*dictEntry = input - inputStart;
				//Try to find a match
				size_t matchLength = test_match(input, match, thisBlockEnd, 6, windowLog);

				//Have we found a match?
				if (matchLength) {

					//Add the next position to the table as well
					lzdict[read_hash6(input + 1)] = input - inputStart + 1;

					//Try to extend the match to the left
					while (input > literalRunStart && match > inputStart && input[-1] == match[-1]) {
						matchLength++;
						input--;
						match--;
					}
					const size_t distance = input - match;

					//First output the literal run
					uint8_t controlByte;
					encode_literal_run(&literalEncoder, &lengthEncoder, input, literalRunStart, &controlByte);

					input += matchLength;
					literalRunStart = input;
					//Output the match
					encode_normal_match(&tokenEncoder, &lengthEncoder, &distanceEncoder, &controlByte, &matchLength, distance, &repOffset);

					//Try to find further rep matches
					while (true) {

						size_t off = input[1] == input[1 - repOffset] ? 1 : 2;
						matchLength = test_match(input + off, input + off - repOffset, thisBlockEnd, 3, windowLog);
						if (matchLength == 0)
							break;

						input += off;
						uint8_t controlByte;
						encode_literal_run(&literalEncoder, &lengthEncoder, input, literalRunStart, &controlByte);
						//Add the position of the match to the hash table
						lzdict[read_hash6(input)] = input - inputStart;

						input += matchLength;
						literalRunStart = input;
						encode_rep_match(&tokenEncoder, &lengthEncoder, &controlByte, &matchLength);
					}

					acceleration = 1 << accelerationThreshold;
				}
				//If there isnt advance the position
				else {
					input += acceleration >> accelerationThreshold;
					if (acceleration < (accelerationMax << accelerationThreshold))
						acceleration++;
				}
			}

			//Maybe we went beyond block end because of acceleration
			input = thisBlockEnd;
			//Send last literal run
			if (input != literalRunStart) {
				uint8_t controlByte;
				encode_literal_run(&literalEncoder, &lengthEncoder, input, literalRunStart, &controlByte);
				tokenEncoder.add_byte(controlByte);
			}

			size_t literalsSize = literalEncoder.generate_codes_fast(output, decSpeedBias, true);
			output += literalsSize;
			size_t distanceSize = distanceEncoder.generate_codes_fast(output, decSpeedBias, true);
			output += distanceSize;
			size_t tokensSize = tokenEncoder.generate_codes_fast(output, decSpeedBias, true);
			output += tokensSize;
			size_t lengthSize = lengthEncoder.generate_codes_fast(output, decSpeedBias, true);
			output += lengthSize;

			if (progress) {
				if (progress->progress(input - inputStart, output - outputStart))
					return 0;
			}
		}

		memcpy(output, input, SKANDA_LAST_BYTES);
		if (progress)
			progress->progress(size, output - outputStart + SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	size_t compress_greedy(const uint8_t* input, const size_t size, uint8_t* output,
		const int windowLog, const float decSpeedBias, const CompressorOptions& compressorOptions, ProgressCallback* progress)
	{
		const size_t accelerationThreshold = 5;
		const size_t accelerationMax = 63;

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;

		size_t repOffset = 1;
		size_t acceleration = 1 << accelerationThreshold;
		//First byte is send uncompressed
		*output++ = *input++;

		const size_t hashLog = std::min(compressorOptions.maxHashLog, windowLog - 3);
		HashTable<uint32_t, FastIntHash> lzdict;
		try {
			lzdict.init(hashLog);
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		HuffmanEncoder literalEncoder;
		HuffmanEncoder tokenEncoder;
		HuffmanEncoder lengthEncoder;
		HuffmanEncoder distanceEncoder;

		bool disableHuffman = decSpeedBias >= 0.99;
		size_t huffmanBufSize = std::min(MAX_BLOCK_SIZE, size);

		if (literalEncoder.initialize_huffman_encoder(huffmanBufSize + 128, !disableHuffman, !disableHuffman))
			return -1;
		if (tokenEncoder.initialize_huffman_encoder(huffmanBufSize / 2 + 128, !disableHuffman))
			return -1;
		if (lengthEncoder.initialize_huffman_encoder(huffmanBufSize / 8 + 128, !disableHuffman))
			return -1;
		if (distanceEncoder.initialize_huffman_encoder(huffmanBufSize + 128, !disableHuffman))
			return -1;

		//Main compression loop
		while (input < compressionLimit) {

			const size_t thisBlockSize = compressionLimit - input < MAX_BLOCK_SIZE ? compressionLimit - input : MAX_BLOCK_SIZE;
			const uint8_t* const thisBlockEnd = input + thisBlockSize;
			const uint8_t* const thisBlockStart = input;

			write_block_header(output, thisBlockSize);

			literalEncoder.start_huffman_block(disableHuffman ? output : nullptr);
			tokenEncoder.start_huffman_block();
			lengthEncoder.start_huffman_block();
			distanceEncoder.start_huffman_block();
			const uint8_t* literalRunStart = input;

			while (input < thisBlockEnd) {

				//First try to find a rep match. Doing it at position +1 gives better results.
				//If one is found simply take it and skip normal match finding.
				size_t matchLength = test_match(input + 1, input + 1 - repOffset, thisBlockEnd, 3, windowLog);
				if (matchLength) {

					input++;
					uint8_t controlByte;
					encode_literal_run(&literalEncoder, &lengthEncoder, input, literalRunStart, &controlByte);

					lzdict[read_hash5(input)] = input - inputStart;

					input += matchLength;
					literalRunStart = input;
					encode_rep_match(&tokenEncoder, &lengthEncoder, &controlByte, &matchLength);
					acceleration = 1 << accelerationThreshold;
					continue;
				}

				//If no rep, try a normal match
				uint32_t* dictEntry = &lzdict[read_hash5(input)];
				const uint8_t* match = inputStart + *dictEntry;
				*dictEntry = input - inputStart;
				matchLength = test_match(input, match, thisBlockEnd, 5, windowLog);

				if (matchLength) {
					//Add as many positions as minimum searched length
					lzdict[read_hash5(input + 1)] = input - inputStart + 1;
					lzdict[read_hash5(input + 2)] = input - inputStart + 2;
					lzdict[read_hash5(input + 3)] = input - inputStart + 3;
					lzdict[read_hash5(input + 4)] = input - inputStart + 4;

					while (input > literalRunStart && match > inputStart && input[-1] == match[-1]) {
						matchLength++;
						input--;
						match--;
					}

					uint8_t controlByte;
					encode_literal_run(&literalEncoder, &lengthEncoder, input, literalRunStart, &controlByte);

					size_t distance = input - match;
					input += matchLength;
					literalRunStart = input;
					encode_normal_match(&tokenEncoder, &lengthEncoder, &distanceEncoder, &controlByte, &matchLength, distance, &repOffset);
					acceleration = 1 << accelerationThreshold;
				}
				else {
					input += acceleration >> accelerationThreshold;
					if (acceleration < (accelerationMax << accelerationThreshold))
						acceleration++;
				}
			}

			input = thisBlockEnd;
			if (input != literalRunStart) {
				uint8_t controlByte;
				encode_literal_run(&literalEncoder, &lengthEncoder, input, literalRunStart, &controlByte);
				tokenEncoder.add_byte(controlByte);
			}

			size_t literalsSize = literalEncoder.generate_codes_fast(output, decSpeedBias, true);
			output += literalsSize;
			size_t distanceSize = distanceEncoder.generate_codes_fast(output, decSpeedBias, true);
			output += distanceSize;
			size_t tokensSize = tokenEncoder.generate_codes_fast(output, decSpeedBias, true);
			output += tokensSize;
			size_t lengthSize = lengthEncoder.generate_codes_fast(output, decSpeedBias, true);
			output += lengthSize;

			if (progress) {
				if (progress->progress(input - inputStart, output - outputStart))
					return 0;
			}
		}

		memcpy(output, input, SKANDA_LAST_BYTES);
		if (progress)
			progress->progress(size, output - outputStart + SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	//Try to find the longest match for a given position. Then try to find an even longer one in the next.
	//If it is found, code a literal, and the longer one found. If not, code the original.
	FORCE_INLINE void fast_lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit,
		LZ2WayCacheTable<uint32_t, FastIntHash>* lzdict, size_t* bestMatchLength, size_t* bestMatchDistance,
		int* lazySteps, int* testedPositions, const int niceLength, const int windowLog) {

		*lazySteps = 0;
		*testedPositions = 1;
		LZ2WayCacheBucket<uint32_t> dictEntry = (*lzdict)[read_hash5(input)];

		//Test first entry
		uint32_t pos = input - inputStart;
		dictEntry.first(&pos);
		const uint8_t* where = inputStart + pos;
		*bestMatchLength = test_match(input, where, limit, 5, windowLog);
		*bestMatchDistance = input - where;
		if (*bestMatchLength >= niceLength) {
			dictEntry.push_in_second(pos);
			return;
		}

		//Test second entry
		dictEntry.second(&pos);
		where = inputStart + pos;
		//Simple heuristic: as we are looking for a longer match, we can first
		//test the byte that would make this match longer. There is a high
		//chance it will differ, so the rest of the match wont need to be tested
		if (*(input + *bestMatchLength) == *(where + *bestMatchLength)) {
			const size_t length = test_match(input, where, limit, 5, windowLog);
			const size_t distance = input - where;
			if (length > *bestMatchLength) {
				*bestMatchDistance = input - where;
				*bestMatchLength = length;

				if (*bestMatchLength >= niceLength)
					return;
			}
		}

		//Nothing was found, code a literal and try again from the begining
		if (*bestMatchLength < 5)
			return;

		//Now try to find a longer match at next position
		input++;
		*testedPositions = 2;
		dictEntry = (*lzdict)[read_hash5(input)];

		//Test first entry
		pos = input - inputStart;
		dictEntry.first(&pos);
		where = inputStart + pos;
		if (*(input + *bestMatchLength) == *(where + *bestMatchLength)) {
			//Since these matches must be at least one byte longer,
			// we can pass as minimum length 6
			const size_t length = test_match(input, where, limit, 6, windowLog);
			if (length > *bestMatchLength) {
				*bestMatchDistance = input - where;
				*bestMatchLength = length;
				*lazySteps = 1;

				if (*bestMatchLength >= niceLength) {
					dictEntry.push_in_second(pos);
					return;
				}
			}
		}
		//Test second entry
		dictEntry.second(&pos);
		where = inputStart + pos;
		if (*(input + *bestMatchLength) == *(where + *bestMatchLength)) {
			const size_t length = test_match(input, where, limit, 6, windowLog);
			if (length > *bestMatchLength) {
				*bestMatchDistance = input - where;
				*bestMatchLength = length;
				*lazySteps = 1;
			}
		}
	}

	size_t compress_lazy_fast(const uint8_t* input, const size_t size, uint8_t* output,
		const int windowLog, const float decSpeedBias, const CompressorOptions& compressorOptions, ProgressCallback* progress)
	{
		const size_t accelerationThreshold = 6;
		const size_t accelerationMax = 63;

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;

		size_t repOffset = 1;
		size_t acceleration = 1 << accelerationThreshold;
		//First byte is send uncompressed
		*output++ = *input++;

		const size_t hashLog = std::min(compressorOptions.maxHashLog, windowLog - 3);
		LZ2WayCacheTable<uint32_t, FastIntHash> lzdict;
		try {
			lzdict.init(hashLog);
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		HuffmanEncoder literalEncoder;
		HuffmanEncoder tokenEncoder;
		HuffmanEncoder lengthEncoder;
		HuffmanEncoder distanceEncoder;

		bool disableHuffman = decSpeedBias >= 0.99;
		size_t huffmanBufSize = std::min(MAX_BLOCK_SIZE, size);

		if (literalEncoder.initialize_huffman_encoder(huffmanBufSize + 128, !disableHuffman, !disableHuffman))
			return -1;
		if (tokenEncoder.initialize_huffman_encoder(huffmanBufSize / 2 + 128, !disableHuffman))
			return -1;
		if (lengthEncoder.initialize_huffman_encoder(huffmanBufSize / 8 + 128, !disableHuffman))
			return -1;
		if (distanceEncoder.initialize_huffman_encoder(huffmanBufSize + 128, !disableHuffman))
			return -1;

		//Main compression loop
		while (input < compressionLimit) {

			const size_t thisBlockSize = compressionLimit - input < MAX_BLOCK_SIZE ? compressionLimit - input : MAX_BLOCK_SIZE;
			const uint8_t* const thisBlockEnd = input + thisBlockSize;
			const uint8_t* const thisBlockStart = input;

			write_block_header(output, thisBlockSize);

			literalEncoder.start_huffman_block(disableHuffman ? output : nullptr);
			tokenEncoder.start_huffman_block();
			lengthEncoder.start_huffman_block();
			distanceEncoder.start_huffman_block();
			const uint8_t* literalRunStart = input;

			while (input < thisBlockEnd) {

				//First try a rep match
				size_t matchLength = test_match(input + 1, input + 1 - repOffset, thisBlockEnd, 3, windowLog);
				if (matchLength) {

					input++;
					uint8_t controlByte;
					encode_literal_run(&literalEncoder, &lengthEncoder, input, literalRunStart, &controlByte);

					lzdict[read_hash5(input - 1)].push_in_first(input - 1 - inputStart);
					lzdict[read_hash5(input + 0)].push_in_first(input + 0 - inputStart);
					lzdict[read_hash5(input + 1)].push_in_first(input + 1 - inputStart);
					lzdict[read_hash5(input + 2)].push_in_first(input + 2 - inputStart);

					input += matchLength;
					literalRunStart = input;
					encode_rep_match(&tokenEncoder, &lengthEncoder, &controlByte, &matchLength);
					acceleration = 1 << accelerationThreshold;
					continue;
				}

				size_t distance;
				int lazySteps;   //bytes to skip because of lazy matching
				int testedPositions;   //number of positions that have been added to hash table

				fast_lazy_search(input, inputStart, thisBlockEnd, &lzdict,
					&matchLength, &distance, &lazySteps, &testedPositions, compressorOptions.niceLength, windowLog);

				if (matchLength) {

					input += lazySteps;
					//If distance < length, update the hash table only on the first "distance" bytes
					const uint8_t* const updateEnd = input + std::min(std::min(distance, matchLength), (size_t)compressorOptions.niceLength);
					const uint8_t* matchPos = input + testedPositions - lazySteps;
					for (; matchPos < updateEnd; matchPos++)
						lzdict[read_hash5(matchPos)].push_in_first(matchPos - inputStart);

					const uint8_t* match = input - distance;
					while (input > literalRunStart && match > inputStart && input[-1] == match[-1]) {
						matchLength++;
						input--;
						match--;
					}

					uint8_t controlByte;
					encode_literal_run(&literalEncoder, &lengthEncoder, input, literalRunStart, &controlByte);

					input += matchLength;
					literalRunStart = input;
					encode_match(&tokenEncoder, &lengthEncoder, &distanceEncoder, &controlByte, &matchLength, distance, &repOffset);
					acceleration = 1 << accelerationThreshold;
				}
				else {
					input += acceleration >> accelerationThreshold;
					if (acceleration < (accelerationMax << accelerationThreshold))
						acceleration++;
				}
			}

			input = thisBlockEnd;
			if (input != literalRunStart) {
				uint8_t controlByte;
				encode_literal_run(&literalEncoder, &lengthEncoder, input, literalRunStart, &controlByte);
				tokenEncoder.add_byte(controlByte);
			}

			size_t literalsSize = literalEncoder.generate_codes_fast(output, decSpeedBias, true);
			output += literalsSize;
			size_t distanceSize = distanceEncoder.generate_codes_fast(output, decSpeedBias, true);
			output += distanceSize;
			size_t tokensSize = tokenEncoder.generate_codes_fast(output, decSpeedBias, true);
			output += tokensSize;
			size_t lengthSize = lengthEncoder.generate_codes_fast(output, decSpeedBias, true);
			output += lengthSize;

			if (progress) {
				if (progress->progress(input - inputStart, output - outputStart))
					return 0;
			}
		}

		memcpy(output, input, SKANDA_LAST_BYTES);
		if (progress)
			progress->progress(size, output - outputStart + SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	//Same principle as the last function, but with additional heuristics to improve ratio
	FORCE_INLINE void lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit,
		LZCacheTable<uint32_t, FastIntHash>* lzdict4, LZCacheTable<uint32_t, FastIntHash>* lzdict6,
		size_t* bestLength, size_t* bestDistance, int* lazySteps, int* testedPositions, const int niceLength, const int windowLog) {

		LZCacheBucket<uint32_t> chain4 = (*lzdict4)[read_hash4(input)];
		LZCacheBucket<uint32_t> chain6 = (*lzdict6)[read_hash6(input)];
		*lazySteps = 0;
		*testedPositions = 1;
		size_t length;
		size_t bestMatchCost = 0;

		//First try to find a length 4 match
		uint32_t pos = input - inputStart;
		while (!chain4.ended()) {
			chain4.next(&pos);

			const uint8_t* where = inputStart + pos;

			if (*(input + *bestLength) != *(where + *bestLength))
				continue;

			length = test_match(input, where, limit, 4, windowLog);

			size_t distance = input - where;
			size_t matchCost = 2 + unsafe_int_log2(distance) / 8;
			if (length + bestMatchCost > matchCost + *bestLength) {
				*bestDistance = distance;
				*bestLength = length;
				bestMatchCost = matchCost;

				//The next length would be 6+, so go to the hash 6 chain
				if (*bestLength >= 5) {
					while (!chain4.ended())
						chain4.next(&pos);
					if (*bestLength >= niceLength) {
						chain6.push(input - inputStart);
						return;
					}
					break;
				}
			}
		}

		//Only test length 6 if at least one match was found
		if (*bestLength >= 4) {

			pos = input - inputStart;
			while (!chain6.ended()) {
				chain6.next(&pos);

				const uint8_t* where = inputStart + pos;

				if (*(input + *bestLength) != *(where + *bestLength))
					continue;

				length = test_match(input, where, limit, 6, windowLog);

				size_t distance = input - where;
				size_t matchCost = 2 + unsafe_int_log2(distance) / 8;
				if (length + bestMatchCost > matchCost + *bestLength) {
					*bestDistance = distance;
					*bestLength = length;
					bestMatchCost = matchCost;

					if (*bestLength >= niceLength) {
						while (!chain6.ended())
							chain6.next(&pos);
						return;
					}
				}
			}
		}
		else
			//No match found, code a literal and retry
			return;

		//Now try to get a better match at pos + 1
		input++;
		pos = input - inputStart;
		//We wont search for length < 6
		(*lzdict4)[read_hash4(input)].push(pos);
		chain6 = (*lzdict6)[read_hash6(input)];
		*testedPositions = 2;

		//Only try to find matches of length at least 6 at pos + 1
		while (!chain6.ended()) {
			chain6.next(&pos);

			const uint8_t* where = inputStart + pos;

			if (*(input + *bestLength) != *(where + *bestLength))
				continue;

			length = test_match(input, where, limit, 6, windowLog);

			size_t distance = input - where;
			size_t matchCost = 2 + unsafe_int_log2(distance) / 8;
			if (length + bestMatchCost > matchCost + *bestLength) {
				*bestDistance = distance;
				*bestLength = length;
				*lazySteps = 1;
				bestMatchCost = matchCost;

				if (*bestLength >= niceLength) {
					while (!chain6.ended())
						chain6.next(&pos);
					return;
				}
			}
		}

		//Now get an even better match at pos + 2
		input++;
		pos = input - inputStart;
		(*lzdict4)[read_hash4(input)].push(pos);
		chain6 = (*lzdict6)[read_hash6(input)];
		*testedPositions = 3;

		while (!chain6.ended()) {
			chain6.next(&pos);

			const uint8_t* where = inputStart + pos;

			if (*(input + *bestLength) != *(where + *bestLength))
				continue;

			length = test_match(input, where, limit, 6, windowLog);

			size_t distance = input - where;
			size_t matchCost = 2 + unsafe_int_log2(distance) / 8;
			if (length + bestMatchCost > matchCost + *bestLength) {
				*bestDistance = distance;
				*bestLength = length;
				*lazySteps = 2;
				bestMatchCost = matchCost;

				if (*bestLength >= niceLength) {
					while (!chain6.ended())
						chain6.next(&pos);
					return;
				}
			}
		}
	}

	size_t compress_lazy(const uint8_t* input, const size_t size, uint8_t* output,
		const int windowLog, const float decSpeedBias, const CompressorOptions& compressorOptions, ProgressCallback* progress)
	{
		const size_t accelerationThreshold = 6;
		const size_t accelerationMax = 63;

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;

		size_t repOffset = 1;
		size_t acceleration = 1 << accelerationThreshold;
		//First byte is send uncompressed
		*output++ = *input++;

		const int hashLog = std::min(compressorOptions.maxHashLog, windowLog - 3);
		LZCacheTable<uint32_t, FastIntHash> lzdict4;
		LZCacheTable<uint32_t, FastIntHash> lzdict6;

		try {
			lzdict4.init(hashLog, compressorOptions.maxElementsLog);
			lzdict6.init(hashLog, compressorOptions.maxElementsLog);
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		HuffmanEncoder literalEncoder;
		HuffmanEncoder tokenEncoder;
		HuffmanEncoder lengthEncoder;
		HuffmanEncoder distanceEncoder;

		bool disableHuffman = decSpeedBias >= 0.99;
		size_t huffmanBufSize = std::min(MAX_BLOCK_SIZE, size);

		if (literalEncoder.initialize_huffman_encoder(huffmanBufSize + 128, !disableHuffman, !disableHuffman))
			return -1;
		if (tokenEncoder.initialize_huffman_encoder(huffmanBufSize / 2 + 128, !disableHuffman))
			return -1;
		if (lengthEncoder.initialize_huffman_encoder(huffmanBufSize / 8 + 128, !disableHuffman))
			return -1;
		if (distanceEncoder.initialize_huffman_encoder(huffmanBufSize + 128, !disableHuffman))
			return -1;

		//Main compression loop
		while (input < compressionLimit) {

			const size_t thisBlockSize = compressionLimit - input < MAX_BLOCK_SIZE ? compressionLimit - input : MAX_BLOCK_SIZE;
			const uint8_t* const thisBlockEnd = input + thisBlockSize;
			const uint8_t* const thisBlockStart = input;

			write_block_header(output, thisBlockSize);

			literalEncoder.start_huffman_block(disableHuffman ? output : nullptr);
			tokenEncoder.start_huffman_block();
			lengthEncoder.start_huffman_block();
			distanceEncoder.start_huffman_block();
			const uint8_t* literalRunStart = input;

			while (input < thisBlockEnd) {

				//First try a rep match
				size_t matchLength = test_match(input + 1, input + 1 - repOffset, thisBlockEnd, 3, windowLog);
				if (matchLength) {

					input++;
					uint8_t controlByte;
					encode_literal_run(&literalEncoder, &lengthEncoder, input, literalRunStart, &controlByte);

					lzdict4[read_hash4(input - 1)].push(input - 1 - inputStart);
					lzdict6[read_hash6(input - 1)].push(input - 1 - inputStart);
					lzdict4[read_hash4(input + 0)].push(input + 0 - inputStart);
					lzdict6[read_hash6(input + 0)].push(input + 0 - inputStart);
					lzdict4[read_hash4(input + 1)].push(input + 1 - inputStart);
					lzdict6[read_hash6(input + 1)].push(input + 1 - inputStart);
					lzdict4[read_hash4(input + 2)].push(input + 2 - inputStart);
					lzdict6[read_hash6(input + 2)].push(input + 2 - inputStart);

					input += matchLength;
					literalRunStart = input;
					encode_rep_match(&tokenEncoder, &lengthEncoder, &controlByte, &matchLength);
					acceleration = 1 << accelerationThreshold;
					continue;
				}

				size_t distance;
				int lazySteps;
				int testedPositions;

				lazy_search(input, inputStart, thisBlockEnd, &lzdict4, &lzdict6, &matchLength,
					&distance, &lazySteps, &testedPositions, compressorOptions.niceLength, windowLog);

				if (matchLength) {

					input += lazySteps;
					//If distance < length, update the hash table only on the first "distance" bytes
					const uint8_t* const updateEnd = input + std::min(std::min(distance, matchLength), (size_t)compressorOptions.niceLength);
					const uint8_t* matchPos = input + testedPositions - lazySteps;
					for (; matchPos < updateEnd; matchPos++) {
						lzdict4[read_hash4(matchPos)].push(matchPos - inputStart);
						lzdict6[read_hash6(matchPos)].push(matchPos - inputStart);
					}

					const uint8_t* match = input - distance;
					while (input > literalRunStart && match > inputStart && input[-1] == match[-1]) {
						matchLength++;
						input--;
						match--;
					}

					uint8_t controlByte;
					encode_literal_run(&literalEncoder, &lengthEncoder, input, literalRunStart, &controlByte);

					input += matchLength;
					literalRunStart = input;
					encode_match(&tokenEncoder, &lengthEncoder, &distanceEncoder, &controlByte, &matchLength, distance, &repOffset);
					acceleration = 1 << accelerationThreshold;
				}
				else {
					input += acceleration >> accelerationThreshold;
					if (acceleration < (accelerationMax << accelerationThreshold))
						acceleration++;
				}
			}

			input = thisBlockEnd;
			if (input != literalRunStart) {
				uint8_t controlByte;
				encode_literal_run(&literalEncoder, &lengthEncoder, input, literalRunStart, &controlByte);
				tokenEncoder.add_byte(controlByte);
			}

			size_t literalsSize = literalEncoder.generate_codes_fast(output, decSpeedBias, true);
			output += literalsSize;
			size_t distanceSize = distanceEncoder.generate_codes_fast(output, decSpeedBias, true);
			output += distanceSize;
			size_t tokensSize = tokenEncoder.generate_codes_fast(output, decSpeedBias, true);
			output += tokensSize;
			size_t lengthSize = lengthEncoder.generate_codes_fast(output, decSpeedBias, true);
			output += lengthSize;

			if (progress) {
				if (progress->progress(input - inputStart, output - outputStart))
					return 0;
			}
		}

		memcpy(output, input, SKANDA_LAST_BYTES);
		if (progress)
			progress->progress(size, output - outputStart + SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	struct SkandaOptimalParserState {
		//16 high bits store size cost, 16 low bits the speed cost
		//This way we can compare costs, prioritising one over another in a single operation
		uint32_t cost;
		//Serves as both match distance and rep offset.
		uint32_t distance;
		uint32_t literalRunLength;
		uint16_t matchLength;
		uint16_t path;
	};

	LZStructure* forward_optimal_parse(const uint8_t* const input, const uint8_t* const inputStart,
		const uint8_t* const blockLimit, HashTableMatchFinder* matchFinder, SkandaOptimalParserState* parser,
		LZStructure* stream, const size_t startRepOffset, size_t* acceleration, const size_t accelerationThreshold,
		const CompressorOptions& compressorOptions, const int windowLog)
	{
		const size_t blockLength = std::min((size_t)(blockLimit - input), (size_t)compressorOptions.optimalBlockSize);
		//Initialize positions cost to maximum. We dont need to initialize ALL, only enough ahead
		// to cover the maximum match length we can write, which is niceLength - 1, otherwise we would simply skip.
		//This speeds up compression on data with a lot of long matches.
		for (size_t i = 1; i < compressorOptions.niceLength; i++)
			parser[i].cost = UINT32_MAX;
		parser[0].cost = 0;
		parser[0].distance = startRepOffset;
		parser[0].literalRunLength = 0;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;

		size_t position = 0;
		bool positionSkip = false;
		for (; position < blockLength; position++) {

			const uint8_t* inputPosition = input + position;
			SkandaOptimalParserState* const parserPosition = parser + position;
			//Make sure we have enough positions ahead initialized
			parserPosition[compressorOptions.niceLength].cost = UINT32_MAX;

			//From Zstd: skip unpromising position
			if (parserPosition[0].cost >= parserPosition[1].cost || positionSkip) {
				matchFinder->update_position(inputPosition, inputStart);
				positionSkip = false;
				continue;
			}

			//Size cost
			size_t literalCost = parserPosition->cost + (0x10000 << (parserPosition->literalRunLength == 6));
			literalCost += (parserPosition->literalRunLength == 6) * 2; //Speed cost
			SkandaOptimalParserState* nextPosition = parserPosition + 1;
			if (literalCost < nextPosition->cost) {
				nextPosition->cost = literalCost;
				nextPosition->distance = parserPosition->distance; //Carry forward the rep offset
				nextPosition->literalRunLength = parserPosition->literalRunLength + 1;
			}

			size_t repMatchLength = test_match(inputPosition, inputPosition - parserPosition->distance, blockLimit, 2, windowLog);
			if (repMatchLength) {
				//Rep matches can be unconditionally taken with lower lengths
				if (repMatchLength >= compressorOptions.niceLength / 2) {
					matchFinder->update_position(inputPosition, inputStart);
					lastMatchLength = repMatchLength;
					lastMatchDistance = parserPosition->distance;
					break;
				}

				if (repMatchLength > 8 && repMatchLength <= 16)
					repMatchLength = 8;

				size_t repMatchCost = parserPosition->cost;
				repMatchCost += 0x10000 << (repMatchLength > 8);  //size cost
				repMatchCost += 1 + (repMatchLength > 8) * 2 + (parserPosition->distance < (repMatchLength > 8 ? 16 : 8));;  //speed cost

				nextPosition = parserPosition + repMatchLength;
				if (repMatchCost < nextPosition->cost) {
					nextPosition->cost = repMatchCost;
					nextPosition->matchLength = repMatchLength;
					nextPosition->distance = parserPosition->distance;
					nextPosition->literalRunLength = 0;
				}

				//When finding a rep match skip the next position
				positionSkip = true;
				if (repMatchLength >= 3)
					*acceleration = 1 << accelerationThreshold;
			}

			LZMatch matches[17];
			const LZMatch* matchesEnd = matchFinder->find_matches_and_update(inputPosition, inputStart,
				blockLimit, matches, repMatchLength + 1, windowLog, compressorOptions);

			//At least one match was found
			if (matchesEnd != matches) {
				//The last match should be the longest
				const LZMatch* const longestMatch = matchesEnd - 1;
				if (longestMatch->length >= compressorOptions.niceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					break;
				}

				for (const LZMatch* matchIt = longestMatch; matchIt >= matches; matchIt--) {

					size_t matchCost = parserPosition->cost;
					//size cost
					matchCost += (2 + unsafe_int_log2(matchIt->distance) / 8 + (matchIt->length > 8)) << 16;
					matchCost += 1 + (matchIt->length > 8) * 2 + (matchIt->distance < (matchIt->length > 8 ? 16 : 8)); //speed cost

					nextPosition = parserPosition + matchIt->length;
					if (matchCost < nextPosition->cost) {
						nextPosition->cost = matchCost;
						nextPosition->matchLength = matchIt->length;
						nextPosition->distance = matchIt->distance;
						nextPosition->literalRunLength = 0;
					}
					//If this match does not help, the shorter ones probably wont
					else
						break;
				}

				if (longestMatch->length >= 4)
					*acceleration = 1 << accelerationThreshold;
			}
			else {
				(*acceleration)++;
				//If acceleration goes too high go to greedy parse
				if (*acceleration >= (2 << accelerationThreshold)) {
					position++;
					break;
				}
			}
		}

		// Backward pass, pick best option at each step.
		const SkandaOptimalParserState* backwardParse;
		const SkandaOptimalParserState* const parseEnd = parser;

		if (lastMatchLength) {
			stream->literalRunLength = 0;
			stream++;
			stream->matchDistance = lastMatchDistance;
			stream->matchLength = lastMatchLength;
			backwardParse = parser + position;
			stream->literalRunLength = backwardParse->literalRunLength;
			backwardParse -= backwardParse->literalRunLength;
			stream++;

			//Update only first positions if distance < length
			const uint8_t* const updateEnd = input + position + std::min(std::min(lastMatchDistance, lastMatchLength), (size_t)compressorOptions.niceLength);
			const uint8_t* inputPosition = input + position + 1;
			for (; inputPosition < updateEnd; inputPosition++)
				matchFinder->update_position(inputPosition, inputStart);
		}
		else {
			size_t bestPos = position;
			size_t bestSize = parser[position].cost;
			for (size_t i = 1; i < compressorOptions.niceLength; i++) {
				if (parser[position + i].cost <= bestSize) {
					bestPos = position + i;
					bestSize = parser[position + i].cost;
				}
			}
			backwardParse = parser + bestPos;
			stream->literalRunLength = backwardParse->literalRunLength;
			backwardParse -= backwardParse->literalRunLength;
			stream++;
		}
		while (backwardParse > parseEnd) {

			stream->matchDistance = backwardParse->distance;
			stream->matchLength = backwardParse->matchLength;
			backwardParse -= backwardParse->matchLength;
			stream->literalRunLength = backwardParse->literalRunLength;
			backwardParse -= backwardParse->literalRunLength;
			stream++;
		}

		return stream - 1;
	}

	class MatchBuffer {
		LZMatch* matches = nullptr;
		int* matchesCount = nullptr;
		size_t maxMatchesPerPosition;
	public:
		MatchBuffer() {}
		~MatchBuffer() {
			delete[] matches;
			delete[] matchesCount;
		}
		void init(size_t inSize, const CompressorOptions& compressorOptions) {
			maxMatchesPerPosition = (1 << compressorOptions.maxElementsLog) + 1;
			matches = new LZMatch[maxMatchesPerPosition * std::min(inSize, MAX_BLOCK_SIZE)];
			matchesCount = new int[std::min(inSize, MAX_BLOCK_SIZE)];
		}
		void find_matches(const uint8_t* in, const uint8_t* const inputStart, const uint8_t* const blockLimit, const uint8_t* const compressionLimit, 
			BinaryMatchFinder* matchFinder, const int windowLog, const CompressorOptions& compressorOptions) 
		{
			const size_t accelerationThreshold = 6;
			const size_t accelerationMax = 63;
			size_t acceleration = 1 << accelerationThreshold;

			size_t blockSize = blockLimit - in;
			for (size_t pos = 0; pos < blockSize; ) {

				LZMatch* matchesEnd = matchFinder->find_matches_and_update(in + pos, inputStart, compressionLimit, blockLimit, matches + pos * maxMatchesPerPosition,
					3, windowLog, compressorOptions);
				matchesCount[pos] = matchesEnd - (matches + pos * maxMatchesPerPosition);

				if (matchesCount[pos] != 0) {
					if (matchesEnd[-1].length >= compressorOptions.niceLength) {
						//Fill all the following positions with the match we have found
						size_t length = matchesEnd[-1].length - 1;
						for (pos++; length > 2; pos++, length--) {
							matches[pos * maxMatchesPerPosition].length = length;
							matches[pos * maxMatchesPerPosition].distance = matchesEnd[-1].distance;
							matchesCount[pos] = 1;
							matchFinder->update_position(in + pos, inputStart, compressionLimit, windowLog, compressorOptions);
						}
						continue;
					}
					if (matchesEnd[-1].length >= 4)
						acceleration = 1 << accelerationThreshold;
				}
				//Disable acceleration on level 9
				else if (compressorOptions.parser < OPTIMAL3) {
					if (acceleration < (accelerationMax << accelerationThreshold))
						acceleration++;
				}

				pos++;
				//Skip positions
				for (size_t skip = 1; skip < (acceleration >> accelerationThreshold) && pos < blockSize; skip++, pos++)
					matchesCount[pos] = 0;
			}
		}
		void get_position_matches(LZMatch** matchesBegin, LZMatch** matchesEnd, size_t pos) {
			*matchesBegin = matches + pos * maxMatchesPerPosition;
			*matchesEnd = *matchesBegin + matchesCount[pos];
		}
	};

	LZStructure* multi_arrivals_parse(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const blockStart,
		const uint8_t* const compressionLimit, const uint8_t* blockLimit, BinaryMatchFinder* matchFinder, MatchBuffer* matchBuffer, SkandaOptimalParserState* parser,
		LZStructure* stream, const size_t startRepOffset, size_t* acceleration, const size_t accelerationThreshold,
		const CompressorOptions& compressorOptions, const int windowLog)
	{
		const size_t blockLength = std::min((size_t)(blockLimit - input), (size_t)compressorOptions.optimalBlockSize - 1);
		for (size_t i = 0; i < compressorOptions.niceLength * compressorOptions.maxArrivals; i++)
			parser[i].cost = UINT32_MAX;
		//Fill only the first arrival of the first position
		parser[0].cost = 64 << 16;
		parser[0].distance = startRepOffset;
		parser[0].literalRunLength = 0;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;
		size_t lastMatchPath;

		size_t position = 0;
		for (; position < blockLength; position++) {

			const size_t blockPos = input + position - blockStart;
			const uint8_t* inputPosition = input + position;
			SkandaOptimalParserState* const parserPosition = parser + position * compressorOptions.maxArrivals;
			//Initialize position ahead
			for (size_t i = 0; i < compressorOptions.maxArrivals; i++)
				parserPosition[compressorOptions.niceLength * compressorOptions.maxArrivals + i].cost = UINT32_MAX;

			size_t nextExpectedLength = 2;  //Only take matches as long as this

			for (size_t i = 0; i < compressorOptions.maxArrivals; i++) {

				SkandaOptimalParserState* const currentArrival = parserPosition + i;
				//Break literal/rep parsing if current arrival has a very high cost
				//Will also break if there are no more arrivals for this position
				if (currentArrival->cost - 0x10000 >= parserPosition[compressorOptions.maxArrivals].cost)
					break;

				size_t literalCost = currentArrival->cost + ((1 + (currentArrival->literalRunLength == 6)) << 16);  //size cost
				literalCost += (currentArrival->literalRunLength == 6) * 2;  //speed cost
				SkandaOptimalParserState* arrivalIt = parserPosition + compressorOptions.maxArrivals;
				SkandaOptimalParserState* lastArrival = arrivalIt + compressorOptions.maxArrivals;

				for (; arrivalIt < lastArrival; arrivalIt++) {

					if (literalCost < arrivalIt->cost) {

						for (SkandaOptimalParserState* it = lastArrival - 1; it != arrivalIt; it--)
							memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState));

						arrivalIt->cost = literalCost;
						arrivalIt->matchLength = 1;
						arrivalIt->distance = currentArrival->distance;
						arrivalIt->literalRunLength = currentArrival->literalRunLength + 1;
						arrivalIt->path = i;
						break;
					}
				}

				size_t repMatchLength = test_match(inputPosition, inputPosition - currentArrival->distance, blockLimit, 2, windowLog);
				//Since we go from lowest cost arrival to highest, it makes sense that the rep match
				// should be at least as long as the best found so far
				if (repMatchLength >= nextExpectedLength) {
					//Rep matches can be unconditionally taken with lower lengths
					if (repMatchLength >= compressorOptions.niceLength / 2) {
						if (matchFinder)
							matchFinder->update_position(inputPosition, inputStart, compressionLimit, windowLog, compressorOptions);
						lastMatchLength = repMatchLength;
						lastMatchDistance = currentArrival->distance;
						lastMatchPath = i;
						goto doBackwardParse;
					}

					//Sending match + rep match has the same size cost as sending an additional length byte, but it avoids a branch in the decompressor
					if (repMatchLength > 8 && repMatchLength <= 16)
						repMatchLength = 8;

					nextExpectedLength = repMatchLength;
					//Small heuristic: instead of testing all positions, only test the maximum match length,
					// and if it overflows, just before the overflow
					//There is a notable speed increase for a negligible size penalty
					size_t repMatchCost = currentArrival->cost;
					repMatchCost += (1 + (repMatchLength > 8)) << 16;  //size cost
					repMatchCost += 1 + (repMatchLength > 8) * 2 + (currentArrival->distance < (repMatchLength > 8 ? 16 : 8));  //speed cost

					SkandaOptimalParserState* arrivalIt = parserPosition + repMatchLength * compressorOptions.maxArrivals;
					SkandaOptimalParserState* lastArrival = arrivalIt + compressorOptions.maxArrivals;

					for (; arrivalIt < lastArrival; arrivalIt++) {

						if (repMatchCost < arrivalIt->cost) {

							for (SkandaOptimalParserState* it = lastArrival - 1; it != arrivalIt; it--)
								memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState));

							arrivalIt->cost = repMatchCost;
							arrivalIt->matchLength = repMatchLength;
							arrivalIt->distance = currentArrival->distance;
							arrivalIt->literalRunLength = 0;
							arrivalIt->path = i;
							break;
						}
						//If there is already an arrival with this offset, and it has lower cost,
						// skip this match. This keeps more interesting offsets in the arrivals.
						if (arrivalIt->distance == currentArrival->distance)
							break;
					}

					if (repMatchLength >= 3)
						*acceleration = 1 << accelerationThreshold;
				}
			}

			//Unpromising position
			if (parserPosition[0].cost >= parserPosition[compressorOptions.maxArrivals].cost) {
				if (matchFinder)
					matchFinder->update_position(inputPosition, inputStart, compressionLimit, windowLog, compressorOptions);
				continue;
			}

			LZMatch matchFinderBuf[65];
			LZMatch* matches, *matchesEnd;
			if (matchFinder) {
				matchesEnd = matchFinder->find_matches_and_update(inputPosition, inputStart, compressionLimit, blockLimit, matchFinderBuf, 3, windowLog, compressorOptions);
				matches = matchFinderBuf;
			}
			else 
				matchBuffer->get_position_matches(&matches, &matchesEnd, blockPos);

			if (matchesEnd != matches) {

				//We have the guarantee that matches are in increasing order
				const LZMatch* const longestMatch = matchesEnd - 1;
				if (longestMatch->length >= compressorOptions.niceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					lastMatchPath = 0;
					break;
				}

				for (LZMatch* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					if (matchIt->distance == parserPosition->distance)
						continue;

					size_t length = matchIt->length;
					if (length > 8 && length <= 16)
						length = 8;

					size_t matchCost = parserPosition->cost;
					//size cost
					matchCost += (2 + unsafe_int_log2(matchIt->distance) / 8 + (length > 8)) << 16;
					matchCost += 1 + (length > 8) * 2 + (matchIt->distance < (length > 8 ? 16 : 8)) + (matchIt->distance > (1 << 16));  //speed cost

					SkandaOptimalParserState* arrivalIt = parserPosition + length * compressorOptions.maxArrivals;
					SkandaOptimalParserState* lastArrival = arrivalIt + compressorOptions.maxArrivals;

					for (; arrivalIt < lastArrival; arrivalIt++) {

						if (matchCost < arrivalIt->cost) {

							for (SkandaOptimalParserState* it = lastArrival - 1; it != arrivalIt; it--)
								memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState));

							arrivalIt->cost = matchCost;
							arrivalIt->matchLength = length;
							arrivalIt->distance = matchIt->distance;
							arrivalIt->literalRunLength = 0;
							arrivalIt->path = 0;
							break;
						}

						if (arrivalIt->distance == matchIt->distance)
							break;
					}
				}

				if (longestMatch->length >= 4)
					*acceleration = 1 << accelerationThreshold;
			}
			//Disable acceleration on level 9
			else if (compressorOptions.parser < OPTIMAL3) {
				(*acceleration)++;
				//If acceleration goes too high go to greedy parse
				if (*acceleration >= (2 << accelerationThreshold)) {
					position++;
					break;
				}
			}
		}

	doBackwardParse:

		// Backward pass, pick best option at each step.
		const SkandaOptimalParserState* backwardParse;
		const SkandaOptimalParserState* const parseEnd = parser;
		size_t path = 0;

		if (lastMatchLength) {
			stream->literalRunLength = 0;
			stream++;
			stream->matchDistance = lastMatchDistance;
			stream->matchLength = lastMatchLength;
			stream->literalRunLength = 0;
			stream++;
			backwardParse = parser + position * compressorOptions.maxArrivals;
			path = lastMatchPath;

			const uint8_t* const updateEnd = input + position + std::min(std::min(lastMatchDistance, lastMatchLength), (size_t)compressorOptions.niceLength);
			const uint8_t* updatePos = input + position + 1;
			if (matchFinder) {
				for (; updatePos < updateEnd; updatePos++)
					matchFinder->update_position(updatePos, inputStart, compressionLimit, windowLog, compressorOptions);
			}
		}
		else {
			size_t bestPos = position;
			size_t bestSize = parser[position * compressorOptions.maxArrivals].cost;
			for (size_t i = 1; i < compressorOptions.niceLength; i++) {
				if (parser[(position + i) * compressorOptions.maxArrivals].cost <= bestSize) {
					bestPos = position + i;
					bestSize = parser[(position + i) * compressorOptions.maxArrivals].cost;
				}
			}
			backwardParse = parser + bestPos * compressorOptions.maxArrivals;
			stream->literalRunLength = 0;
			stream++;
		}
		while (backwardParse > parseEnd) {
			if (backwardParse[path].matchLength > 1) {
				stream->matchDistance = backwardParse[path].distance;
				stream->matchLength = backwardParse[path].matchLength;
				stream->literalRunLength = 0;
				path = backwardParse[path].path;
				backwardParse -= stream->matchLength * compressorOptions.maxArrivals;
				stream++;
			}
			else {
				(stream - 1)->literalRunLength++;
				path = backwardParse[path].path;
				backwardParse -= compressorOptions.maxArrivals;
			}
		}

		return stream - 1;
	}

	size_t get_length_cost(size_t length, HuffmanSymbol* lengthSymbols) {
		if (length <= 251)
			return lengthSymbols[length].bits;

		length -= 252;
		size_t cost = lengthSymbols[252 + length % 4].bits;
		cost += lengthSymbols[(length >> 2) & 0xFF].bits;
		cost += lengthSymbols[(length >> 10) & 0xFF].bits;
		return cost;
	}

	size_t get_token_cost(size_t literalRunLength, size_t matchLength, size_t distanceBytes, HuffmanSymbol* tokenSymbols) {
		return tokenSymbols[(std::min(literalRunLength, (size_t)7) << 5) | (distanceBytes << 3) | (std::min(matchLength, (size_t)9) - 2)].bits;
	}

	LZStructure* multi_arrivals_parse_priced(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const blockStart,
		const uint8_t* const compressionLimit, const uint8_t* blockLimit, MatchBuffer* matchBuffer, SkandaOptimalParserState* parser,
		LZStructure* stream, const size_t startRepOffset, size_t* acceleration, const size_t accelerationThreshold, 
		HuffmanSymbol* literalSymbols, HuffmanSymbol* tokenSymbols, HuffmanSymbol* lengthSymbols, HuffmanSymbol* distanceSymbols,
		const CompressorOptions& compressorOptions, const int windowLog)
	{
		const size_t MAX_LENGTH_REDUCTION = 3;

		const size_t blockLength = std::min((size_t)(blockLimit - input), (size_t)compressorOptions.optimalBlockSize - 1);
		for (size_t i = 0; i < compressorOptions.niceLength * compressorOptions.maxArrivals; i++)
			parser[i].cost = UINT32_MAX;
		//Fill only the first arrival of the first position
		parser[0].cost = 64 << 16;
		parser[0].distance = startRepOffset;
		parser[0].literalRunLength = 0;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;
		size_t lastMatchPath;

		size_t position = 0;
		for (; position < blockLength; position++) {

			const size_t blockPos = input + position - blockStart;
			const uint8_t* inputPosition = input + position;
			SkandaOptimalParserState* const parserPosition = parser + position * compressorOptions.maxArrivals;
			//Initialize position ahead
			for (size_t i = 0; i < compressorOptions.maxArrivals; i++)
				parserPosition[compressorOptions.niceLength * compressorOptions.maxArrivals + i].cost = UINT32_MAX;

			size_t nextExpectedLength = 2;  //Only take matches as long as this

			for (size_t i = 0; i < compressorOptions.maxArrivals; i++) {

				SkandaOptimalParserState* const currentArrival = parserPosition + i;
				//Break literal/rep parsing if current arrival has a very high cost
				//Will also break if there are no more arrivals for this position
				if (currentArrival->cost - 0x10000 * compressorOptions.maxArrivals >= parserPosition[compressorOptions.maxArrivals].cost)
					break;

				//size cost
				size_t literalCost = currentArrival->cost;
				literalCost += literalSymbols[*inputPosition].bits << 16;
				if (currentArrival->literalRunLength >= 6)
					literalCost += get_length_cost(currentArrival->literalRunLength + 1 - 7, lengthSymbols) << 16;
				if (currentArrival->literalRunLength >= 7)
					literalCost -= get_length_cost(currentArrival->literalRunLength - 7, lengthSymbols) << 16;
				//speed cost
				literalCost += (currentArrival->literalRunLength == 6) * 2; 
				SkandaOptimalParserState* arrivalIt = parserPosition + compressorOptions.maxArrivals;
				SkandaOptimalParserState* lastArrival = arrivalIt + compressorOptions.maxArrivals;

				for (; arrivalIt < lastArrival; arrivalIt++) {

					if (literalCost < arrivalIt->cost) {

						for (SkandaOptimalParserState* it = lastArrival - 1; it != arrivalIt; it--)
							memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState));

						arrivalIt->cost = literalCost;
						arrivalIt->matchLength = 1;
						arrivalIt->distance = currentArrival->distance;
						arrivalIt->literalRunLength = currentArrival->literalRunLength + 1;
						arrivalIt->path = i;
						break;
					}
				}

				size_t repMatchLength = test_match(inputPosition, inputPosition - currentArrival->distance, blockLimit, 2, windowLog);
				//Since we go from lowest cost arrival to highest, it makes sense that the rep match
				// should be at least as long as the best found so far
				if (repMatchLength >= nextExpectedLength) {
					//Rep matches can be unconditionally taken with lower lengths
					if (repMatchLength >= compressorOptions.niceLength / 2) {
						lastMatchLength = repMatchLength;
						lastMatchDistance = currentArrival->distance;
						lastMatchPath = i;
						goto doBackwardParse;
					}

					const size_t maxLengthReduction = repMatchLength - nextExpectedLength > MAX_LENGTH_REDUCTION ?
						repMatchLength - MAX_LENGTH_REDUCTION : nextExpectedLength;

					for (size_t length = repMatchLength; length >= maxLengthReduction; length--) {

						size_t repMatchCost = currentArrival->cost;
						//size cost
						repMatchCost += get_token_cost(currentArrival->literalRunLength, length, 0, tokenSymbols) << 16;
						if (length > 8)
							repMatchCost += get_length_cost(length - 9, lengthSymbols) << 16;
						//speed cost
						repMatchCost += 1 + (length > 8) * 2 + (currentArrival->distance < (length > 8 ? 16 : 8));

						SkandaOptimalParserState* arrivalIt = parserPosition + length * compressorOptions.maxArrivals;
						SkandaOptimalParserState* lastArrival = arrivalIt + compressorOptions.maxArrivals;

						for (; arrivalIt < lastArrival; arrivalIt++) {

							if (repMatchCost < arrivalIt->cost) {

								for (SkandaOptimalParserState* it = lastArrival - 1; it != arrivalIt; it--)
									memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState));

								arrivalIt->cost = repMatchCost;
								arrivalIt->matchLength = length;
								arrivalIt->distance = currentArrival->distance;
								arrivalIt->literalRunLength = 0;
								arrivalIt->path = i;
								break;
							}
							//If there is already an arrival with this offset, and it has lower cost,
							// skip this match. This keeps more interesting offsets in the arrivals.
							if (arrivalIt->distance == currentArrival->distance)
								break;
						}
					}

					nextExpectedLength = repMatchLength;
					if (repMatchLength >= 3)
						*acceleration = 1 << accelerationThreshold;
				}
			}

			//Unpromising position
			if (parserPosition[0].cost >= parserPosition[compressorOptions.maxArrivals].cost) 
				continue;

			LZMatch* matches, *matchesEnd;
			matchBuffer->get_position_matches(&matches, &matchesEnd, blockPos);

			if (matchesEnd != matches) {

				//We have the guarantee that matches are in increasing order
				const LZMatch* const longestMatch = matchesEnd - 1;
				if (longestMatch->length >= compressorOptions.niceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					lastMatchPath = 0;
					break;
				}

				size_t previousMatchLength = 3;

				for (LZMatch* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					if (matchIt->distance == parserPosition->distance)
						continue;

					size_t matchLengthReductionLimit = matchIt->length == previousMatchLength ?
						previousMatchLength : previousMatchLength + 1;
					if (matchIt->length - matchLengthReductionLimit > MAX_LENGTH_REDUCTION)
						matchLengthReductionLimit = matchIt->length - MAX_LENGTH_REDUCTION;
					previousMatchLength = matchIt->length;

					//Start search at the highest length. Stop when we reach the limit,
					for (size_t length = matchIt->length; length >= matchLengthReductionLimit; length--) {

						size_t matchCost = parserPosition->cost;
						//size cost
						size_t distanceBytes = unsafe_int_log2(matchIt->distance) / 8 + 1;
						matchCost += get_token_cost(parserPosition->literalRunLength, length, distanceBytes, tokenSymbols) << 16;
						for (int i = 0; i < distanceBytes; i++)
							matchCost += distanceSymbols[(matchIt->distance >> i * 8) & 0xFF].bits << 16;
						if (length > 8)
							matchCost += get_length_cost(length - 9, lengthSymbols) << 16;
						//speed cost
						matchCost += 1 + (length > 8) * 2 + (matchIt->distance < (length > 8 ? 16 : 8)) + (matchIt->distance > (1 << 16));

						SkandaOptimalParserState* arrivalIt = parserPosition + length * compressorOptions.maxArrivals;
						SkandaOptimalParserState* lastArrival = arrivalIt + compressorOptions.maxArrivals;

						for (; arrivalIt < lastArrival; arrivalIt++) {

							if (matchCost < arrivalIt->cost) {

								for (SkandaOptimalParserState* it = lastArrival - 1; it != arrivalIt; it--)
									memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState));

								arrivalIt->cost = matchCost;
								arrivalIt->matchLength = length;
								arrivalIt->distance = matchIt->distance;
								arrivalIt->literalRunLength = 0;
								arrivalIt->path = 0;
								break;
							}

							if (arrivalIt->distance == matchIt->distance)
								break;
						}
					}
				}

				if (longestMatch->length >= 4)
					*acceleration = 1 << accelerationThreshold;
			}
			//Disable acceleration on level 9
			else if (compressorOptions.parser < OPTIMAL3) {
				(*acceleration)++;
				//If acceleration goes too high go to greedy parse
				if (*acceleration >= (2 << accelerationThreshold)) {
					position++;
					break;
				}
			}
		}

	doBackwardParse:

		// Backward pass, pick best option at each step.
		const SkandaOptimalParserState* backwardParse;
		const SkandaOptimalParserState* const parseEnd = parser;
		size_t path = 0;

		if (lastMatchLength) {
			stream->literalRunLength = 0;
			stream++;
			stream->matchDistance = lastMatchDistance;
			stream->matchLength = lastMatchLength;
			stream->literalRunLength = 0;
			stream++;
			backwardParse = parser + position * compressorOptions.maxArrivals;
			path = lastMatchPath;
		}
		else {
			size_t bestPos = position;
			size_t bestSize = parser[position * compressorOptions.maxArrivals].cost;
			for (size_t i = 1; i < compressorOptions.niceLength; i++) {
				if (parser[(position + i) * compressorOptions.maxArrivals].cost <= bestSize) {
					bestPos = position + i;
					bestSize = parser[(position + i) * compressorOptions.maxArrivals].cost;
				}
			}
			backwardParse = parser + bestPos * compressorOptions.maxArrivals;
			stream->literalRunLength = 0;
			stream++;
		}
		while (backwardParse > parseEnd) {
			if (backwardParse[path].matchLength > 1) {
				stream->matchDistance = backwardParse[path].distance;
				stream->matchLength = backwardParse[path].matchLength;
				stream->literalRunLength = 0;
				path = backwardParse[path].path;
				backwardParse -= stream->matchLength * compressorOptions.maxArrivals;
				stream++;
			}
			else {
				(stream - 1)->literalRunLength++;
				path = backwardParse[path].path;
				backwardParse -= compressorOptions.maxArrivals;
			}
		}

		return stream - 1;
	}

	size_t compress_optimal(const uint8_t* input, const size_t size, uint8_t* output,
		const int windowLog, const float decSpeedBias, const CompressorOptions& compressorOptions, ProgressCallback* progress)
	{
		const size_t accelerationThreshold = 6;
		const size_t accelerationMax = 63;

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;

		size_t repOffset = 1;
		size_t acceleration = 1 << accelerationThreshold;
		//First byte is send uncompressed
		*output++ = *input++;

		bool disableHuffman = decSpeedBias >= 0.99;

		HashTableMatchFinder hashMatchFinder;
		BinaryMatchFinder binaryMatchFinder;
		SkandaOptimalParserState* parser = nullptr;
		LZStructure* stream = nullptr;
		MatchBuffer matchBuffer;

		try {
			if (compressorOptions.parser == OPTIMAL1) {
				hashMatchFinder.init(windowLog, compressorOptions);
				parser = new SkandaOptimalParserState[compressorOptions.optimalBlockSize + compressorOptions.niceLength];
			}
			else {
				binaryMatchFinder.init(size, windowLog, compressorOptions);
				parser = new SkandaOptimalParserState[(compressorOptions.optimalBlockSize + compressorOptions.niceLength) * compressorOptions.maxArrivals];
			}
			stream = new LZStructure[compressorOptions.optimalBlockSize];
			if (!disableHuffman)
				matchBuffer.init(size, compressorOptions);
		}
		catch (const std::bad_alloc& e) {
			delete[] parser;
			delete[] stream;
			return -1;
		}

		HuffmanEncoder literalEncoder;
		HuffmanEncoder tokenEncoder;
		HuffmanEncoder lengthEncoder;
		HuffmanEncoder distanceEncoder;

		size_t huffmanBufSize = std::min(MAX_BLOCK_SIZE, size);

		if (literalEncoder.initialize_huffman_encoder(huffmanBufSize + 128, !disableHuffman, !disableHuffman))
			return -1;
		if (tokenEncoder.initialize_huffman_encoder(huffmanBufSize / 2 + 128, !disableHuffman))
			return -1;
		if (lengthEncoder.initialize_huffman_encoder(huffmanBufSize / 8 + 128, !disableHuffman))
			return -1;
		if (distanceEncoder.initialize_huffman_encoder(huffmanBufSize + 128, !disableHuffman))
			return -1;

		//Main compression loop
		while (input < compressionLimit) {

			const size_t thisBlockSize = compressionLimit - input < MAX_BLOCK_SIZE ? compressionLimit - input : MAX_BLOCK_SIZE;
			const uint8_t* const thisBlockEnd = input + thisBlockSize;
			const uint8_t* const thisBlockStart = input;
			write_block_header(output, thisBlockSize);
			const size_t startRepOffset = repOffset;

			if (!disableHuffman && compressorOptions.parser >= OPTIMAL2)
				matchBuffer.find_matches(thisBlockStart, inputStart, thisBlockEnd, compressionLimit, &binaryMatchFinder, windowLog, compressorOptions);

			for (int iteration = 0; iteration < (compressorOptions.parser < OPTIMAL2 || disableHuffman ? 1 : compressorOptions.parserIterations); iteration++) {

				HuffmanSymbol* literalSymbols = nullptr;
				HuffmanSymbol* tokenSymbols = nullptr;
				HuffmanSymbol* lengthSymbols = nullptr;
				HuffmanSymbol* distanceSymbols = nullptr;
				if (iteration != 0) {
					bool noHuffman = 1;
					bool uncompressedStream;
					literalSymbols = literalEncoder.get_huffman_codes(decSpeedBias, &uncompressedStream);
					noHuffman &= uncompressedStream;
					tokenSymbols = tokenEncoder.get_huffman_codes(decSpeedBias, &uncompressedStream);
					noHuffman &= uncompressedStream;
					lengthSymbols = lengthEncoder.get_huffman_codes(decSpeedBias, &uncompressedStream);
					noHuffman &= uncompressedStream;
					distanceSymbols = distanceEncoder.get_huffman_codes(decSpeedBias, &uncompressedStream);
					noHuffman &= uncompressedStream;

					//No stream will be compressed with huffman, no need for more iterations
					if (noHuffman) 
						break;
				}

				input = thisBlockStart;
				repOffset = startRepOffset;
				const uint8_t* literalRunStart = input;

				literalEncoder.start_huffman_block(disableHuffman ? output : nullptr);
				tokenEncoder.start_huffman_block();
				lengthEncoder.start_huffman_block();
				distanceEncoder.start_huffman_block();

				while (input < thisBlockEnd) {

					if (acceleration >= (2 << accelerationThreshold)) {

						LZMatch matchFinderBuf[65];
						LZMatch* matches, *matchesEnd;
						if (compressorOptions.parser >= OPTIMAL2) {
							if (disableHuffman) {
								matchesEnd = binaryMatchFinder.find_matches_and_update(input, inputStart, compressionLimit,
									thisBlockEnd, matchFinderBuf, 4, windowLog, compressorOptions);
								matches = matchFinderBuf;
							}
							else
								matchBuffer.get_position_matches(&matches, &matchesEnd, input - thisBlockStart);
						}
						else {
							matchesEnd = hashMatchFinder.find_matches_and_update(input, inputStart, thisBlockEnd, matchFinderBuf, 4, windowLog, compressorOptions);
							matches = matchFinderBuf;
						}

						if (matchesEnd != matches) {
							size_t distance = matchesEnd[-1].distance;
							size_t length = matchesEnd[-1].length;
							//Left match extension
							const uint8_t* match = input - distance;
							while (input > literalRunStart && match > inputStart && input[-1] == match[-1]) {
								length++;
								input--;
								match--;
							}

							uint8_t controlByte;
							encode_literal_run(&literalEncoder, &lengthEncoder, input, literalRunStart, &controlByte);

							input += length;
							literalRunStart = input;
							encode_match(&tokenEncoder, &lengthEncoder, &distanceEncoder, &controlByte, &length, distance, &repOffset);
							acceleration = 1 << accelerationThreshold;
						}
						else {
							input += acceleration >> accelerationThreshold;
							if (acceleration < (accelerationMax << accelerationThreshold))
								acceleration++;
						}
					}
					else {
						LZStructure* streamIt;
						if (compressorOptions.parser >= OPTIMAL2) {
							if (iteration == 0) {
								streamIt = multi_arrivals_parse(input, inputStart, thisBlockStart, compressionLimit, thisBlockEnd, 
									disableHuffman ? &binaryMatchFinder : nullptr, &matchBuffer, parser, stream, repOffset, 
									&acceleration, accelerationThreshold, compressorOptions, windowLog);
							}
							else {
								streamIt = multi_arrivals_parse_priced(input, inputStart, thisBlockStart, compressionLimit, thisBlockEnd, &matchBuffer,
									parser, stream, repOffset, &acceleration, accelerationThreshold, literalSymbols, tokenSymbols, lengthSymbols, distanceSymbols, 
									compressorOptions, windowLog);
							}
						}
						else {
							streamIt = forward_optimal_parse(input, inputStart, thisBlockEnd, &hashMatchFinder,
								parser, stream, repOffset, &acceleration, accelerationThreshold, compressorOptions, windowLog);
						}

						//Main compression loop
						while (true) {
							input += streamIt->literalRunLength;

							if (streamIt == stream)
								break;

							size_t matchLength = streamIt->matchLength;
							size_t distance = streamIt->matchDistance;

							if (compressorOptions.parser == OPTIMAL1 && distance != repOffset) {
								const uint8_t* match = input - distance;
								while (input > literalRunStart && match > inputStart && input[-1] == match[-1]) {
									matchLength++;
									input--;
									match--;
								}
							}

							uint8_t controlByte;
							encode_literal_run(&literalEncoder, &lengthEncoder, input, literalRunStart, &controlByte);

							input += matchLength;
							literalRunStart = input;
							encode_match(&tokenEncoder, &lengthEncoder, &distanceEncoder, &controlByte, &matchLength, distance, &repOffset);

							streamIt--;
						}
					}
				}

				input = thisBlockEnd;
				if (input != literalRunStart) {
					uint8_t controlByte;
					encode_literal_run(&literalEncoder, &lengthEncoder, input, literalRunStart, &controlByte);
					tokenEncoder.add_byte(controlByte);
				}
			}

			size_t literalsSize = literalEncoder.generate_codes_fast(output, decSpeedBias, false);
			output += literalsSize;
			size_t distanceSize = distanceEncoder.generate_codes_fast(output, decSpeedBias, false);
			output += distanceSize;
			size_t tokensSize = tokenEncoder.generate_codes_fast(output, decSpeedBias, false);
			output += tokensSize;
			size_t lengthSize = lengthEncoder.generate_codes_fast(output, decSpeedBias, false);
			output += lengthSize;

			if (progress) {
				if (progress->progress(input - inputStart, output - outputStart)) {
					delete[] parser;
					delete[] stream;
					return 0;
				}
			}
		}

		delete[] parser;
		delete[] stream;

		memcpy(output, input, SKANDA_LAST_BYTES);
		if (progress)
			progress->progress(size, output - outputStart + SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	const int NOT_USED = -1;
	const CompressorOptions skandaCompressorLevels[] = {
		//      Parser        Hash log     Elements per hash     Nice length      Block size      Max arrivals     Iterations
			{ GREEDY1      ,  NOT_USED  ,      NOT_USED       ,    NOT_USED   ,    NOT_USED    ,    NOT_USED    ,   NOT_USED   },
			{ GREEDY2      ,     16     ,      NOT_USED       ,    NOT_USED   ,    NOT_USED    ,    NOT_USED    ,   NOT_USED   },
			{ LAZY1        ,     16     ,      NOT_USED       ,       32      ,    NOT_USED    ,    NOT_USED    ,   NOT_USED   },
			{ LAZY2        ,     17     ,          1          ,       32      ,    NOT_USED    ,    NOT_USED    ,   NOT_USED   },
			{ LAZY2        ,     18     ,          2          ,       32      ,    NOT_USED    ,    NOT_USED    ,   NOT_USED   },
			{ OPTIMAL1     ,     18     ,          2          ,       32      ,      1024      ,    NOT_USED    ,   NOT_USED   },
			{ OPTIMAL1     ,     19     ,          3          ,       64      ,      1024      ,    NOT_USED    ,   NOT_USED   },
			{ OPTIMAL2     ,     24     ,          4          ,       128     ,      4096      ,       2        ,      2       },
			{ OPTIMAL2     ,     24     ,          5          ,       256     ,      4096      ,       4        ,      3       },
			{ OPTIMAL3     ,     24     ,          6          ,       512     ,      4096      ,       8        ,      5       },
			{ OPTIMAL3     ,     24     ,          6          ,       1024    ,      4096      ,       16       ,      10      },
	};

	size_t compress(const uint8_t* input, size_t size, uint8_t* output, int level, float decSpeedBias, ProgressCallback* progress) {

		if (size <= SKANDA_LAST_BYTES) {
			memcpy(output, input, size);
			if (progress)
				progress->progress(size, size);
			return size;
		}

		if (level > 10)
			level = 10;
		if (level < 0)
			level = 0;
		if (decSpeedBias < 0)
			decSpeedBias = 0;
		if (decSpeedBias > 1)
			decSpeedBias = 1;

		const int WINDOW_LOGS[] = { 24, 23, 22, 21, 20, 20 };
		int maxWindowLog = WINDOW_LOGS[(int)(decSpeedBias * 5)];
		int windowLog = int_log2(size - 1) + 1;
		if (windowLog > maxWindowLog)
			windowLog = maxWindowLog;
		if (windowLog < 6)
			windowLog = 6;

		if (skandaCompressorLevels[level].parser == GREEDY1)
			return compress_ultra_fast(input, size, output, windowLog, decSpeedBias, progress);
		else if (skandaCompressorLevels[level].parser == GREEDY2)
			return compress_greedy(input, size, output, windowLog, decSpeedBias, skandaCompressorLevels[level], progress);
		else if (skandaCompressorLevels[level].parser == LAZY1)
			return compress_lazy_fast(input, size, output, windowLog, decSpeedBias, skandaCompressorLevels[level], progress);
		else if (skandaCompressorLevels[level].parser == LAZY2)
			return compress_lazy(input, size, output, windowLog, decSpeedBias, skandaCompressorLevels[level], progress);
		return compress_optimal(input, size, output, windowLog, decSpeedBias, skandaCompressorLevels[level], progress);
	}

	size_t compress_bound(size_t size) {
		return size + size / 1024 + 8;
	}

	size_t lz_memory_usage(size_t size, int windowLog, int level, float decSpeedBias) {
		if (level < 0)
			return sizeof(uint32_t) << 12;
		if (skandaCompressorLevels[level].parser == GREEDY1)
			return sizeof(uint32_t) << 12;
		if (skandaCompressorLevels[level].parser == GREEDY2)
			return sizeof(uint32_t) << std::min(skandaCompressorLevels[level].maxHashLog, windowLog - 3);
		if (skandaCompressorLevels[level].parser == LAZY1)
			return sizeof(uint32_t) << std::min(skandaCompressorLevels[level].maxHashLog, windowLog - 3) << 1;
		if (skandaCompressorLevels[level].parser == LAZY2)
			//Lazy extra uses 2 tables
			return sizeof(uint32_t) << std::min(skandaCompressorLevels[level].maxHashLog, windowLog - 3)
			<< skandaCompressorLevels[level].maxElementsLog << 1;
		if (skandaCompressorLevels[level].parser == OPTIMAL1) {
			const int log2size = std::min(skandaCompressorLevels[level].maxHashLog, windowLog - 3);
			size_t memory = sizeof(uint32_t) << std::min(log2size, 14);  //hash 3 table
			//hash 4 and hash 8 tables
			memory += sizeof(uint32_t) << log2size << skandaCompressorLevels[level].maxElementsLog << 1;
			memory += sizeof(SkandaOptimalParserState) *
				(skandaCompressorLevels[level].optimalBlockSize + skandaCompressorLevels[level].niceLength);

			memory += sizeof(LZStructure) * skandaCompressorLevels[level].optimalBlockSize;
			return memory;
		}
		size_t memory = sizeof(uint32_t) << std::min(20, windowLog - 3);  //binary node lookup
		if (skandaCompressorLevels[level].parser < OPTIMAL3)
			memory += sizeof(uint32_t) << std::min(14, windowLog - 3);  //hash 3 table
		const size_t binaryTreeWindow = (size_t)1 << std::min(skandaCompressorLevels[level].maxHashLog, windowLog);
		if (size < binaryTreeWindow)
			memory += sizeof(uint32_t) * 2 * size;  //binary tree
		else
			memory += sizeof(uint32_t) * 2 * binaryTreeWindow;  //binary tree
		memory += sizeof(SkandaOptimalParserState) *
			(skandaCompressorLevels[level].optimalBlockSize + skandaCompressorLevels[level].niceLength) *
			skandaCompressorLevels[level].maxArrivals;
		memory += sizeof(LZStructure) * skandaCompressorLevels[level].optimalBlockSize;
		//match buffer
		if (decSpeedBias < 0.99) {
			size_t maxMatchesPerPosition = (1 << skandaCompressorLevels[level].maxElementsLog) + 1;
			memory += sizeof(LZMatch) * maxMatchesPerPosition * std::min(size, MAX_BLOCK_SIZE);
			memory += sizeof(int) * std::min(size, MAX_BLOCK_SIZE);
		}
		return memory;
	}

	size_t estimate_memory(size_t size, int level, float decSpeedBias) {

		if (size <= SKANDA_LAST_BYTES + 1)
			return 0;

		//Negative levels use the same memory as level 0
		if (level < 0)
			level = 0;
		if (level > 10)
			level = 10;
		const int WINDOW_LOGS[] = { 24, 23, 22, 21, 20, 20 };
		int maxWindowLog = WINDOW_LOGS[(int)(decSpeedBias * 5)];
		int windowLog = int_log2(size - 1) + 1;
		if (windowLog > maxWindowLog)
			windowLog = maxWindowLog;
		if (windowLog < 6)
			windowLog = 6;

		bool disableHuffman = decSpeedBias >= 0.99;
		size_t huffmanBufSize = std::min(MAX_BLOCK_SIZE, size);
		size_t huffmanMemoryUsage = disableHuffman ? 0 : (huffmanBufSize + 128) * 2; //Literal stream
		huffmanMemoryUsage += (huffmanBufSize / 2 + 128) * (disableHuffman ? 1 : 2); //Token stream
		huffmanMemoryUsage += (huffmanBufSize / 8 + 128) * (disableHuffman ? 1 : 2); //Length stream
		huffmanMemoryUsage += (huffmanBufSize + 128) * (disableHuffman ? 1 : 2); //Distance stream

		return huffmanMemoryUsage + lz_memory_usage(size, windowLog, level, decSpeedBias);
	}

	class HuffmanDecoder {
		size_t outBufferSize = 0;
		uint8_t* outBuffer = nullptr;
	public:
		HuffmanDecoder(size_t maxOutBufferSize) {
			outBufferSize = maxOutBufferSize;
		}
		~HuffmanDecoder() {
			delete[] outBuffer;
		}

		FORCE_INLINE void read_entropy_header(const uint8_t*& compressed, uint32_t* symbolCount, int* mode) {
			uint32_t in = read_uint24le(compressed);
			compressed += 3;
			*symbolCount = in >> 5;
			*mode = in & 0x1F;
		}

		struct HuffmanDecodeTableEntry {
			uint8_t symbol;
			uint8_t bits;
		};

		uint8_t* get_output_buffer() {
			return outBuffer;
		}

		int decode_huffman_tree(HuffmanDecodeTableEntry symbols[256], const uint8_t*& compressed, const uint8_t* const compressedEnd) {

			//From the frequencies we can infer how many symbols are in each code length, and directly write them sorted
			size_t codeLengthIterators[MAX_HUFFMAN_CODE_LENGTH + 2];
			RansSymbol decodeTable[256 + 2];

			size_t accumulator = 0;
			for (size_t i = 1; i <= MAX_HUFFMAN_CODE_LENGTH; i++) {

				size_t freq = *compressed++;
				RansSymbol symbol = { (uint8_t)i, (uint8_t)freq, (uint8_t)accumulator };
				RansSymbol* it = decodeTable + accumulator;
				RansSymbol* end = it + freq;

				codeLengthIterators[i] = accumulator;
				accumulator += freq;
				if (accumulator > 256)
					return -1;

				do {
					it[0] = symbol;
					it[1] = symbol;
					it += 2;
				} while (it < end);
			}

			//Infer freq of "symbol not present"
			size_t freq = 256 - accumulator;
			RansSymbol symbol = { (uint8_t)(MAX_HUFFMAN_CODE_LENGTH + 1), (uint8_t)freq, (uint8_t)accumulator };
			RansSymbol* it = decodeTable + accumulator;
			RansSymbol* end = it + freq;

			codeLengthIterators[MAX_HUFFMAN_CODE_LENGTH + 1] = accumulator;

			do {
				it[0] = symbol;
				it[1] = symbol;
				it += 2;
			} while (it < end);

			//Rans decoding
			size_t stateA = read_uint16le(compressed + 0);
			size_t stateB = read_uint16le(compressed + 2);
			compressed += 4;

			size_t accumulatedFreq = 0;
			for (size_t i = 0; i < 256; i += 2) {
				size_t stateLow = stateA & ((1 << RANS_MODEL_BIT_PRECISION) - 1);
				stateA = decodeTable[stateLow].freq * (stateA >> RANS_MODEL_BIT_PRECISION) + stateLow - decodeTable[stateLow].low;
				size_t bits = decodeTable[stateLow].symbol;
				accumulatedFreq += HUFFMAN_CODE_SPACE >> bits;

				if (unlikely(codeLengthIterators[bits] == 256))
					return -1;
				symbols[codeLengthIterators[bits]].bits = bits;
				symbols[codeLengthIterators[bits]].symbol = i;
				codeLengthIterators[bits]++;

				//Normalize
				bool renormalize = stateA < RANS_NORMALIZATION_INTERVAL;
				size_t newState = (stateA << 8) | *compressed;
				stateA = (0 - renormalize & (newState ^ stateA)) ^ stateA;  //Hope the compiler replaces this with a cmov or equivalent
				compressed += renormalize;

				stateLow = stateB & ((1 << RANS_MODEL_BIT_PRECISION) - 1);
				stateB = decodeTable[stateLow].freq * (stateB >> RANS_MODEL_BIT_PRECISION) + stateLow - decodeTable[stateLow].low;
				bits = decodeTable[stateLow].symbol;
				accumulatedFreq += HUFFMAN_CODE_SPACE >> bits;

				if (unlikely(codeLengthIterators[bits] == 256))
					return -1;
				symbols[codeLengthIterators[bits]].bits = bits;
				symbols[codeLengthIterators[bits]].symbol = i + 1;
				codeLengthIterators[bits]++;

				//Normalize
				renormalize = stateB < RANS_NORMALIZATION_INTERVAL;
				newState = (stateB << 8) | *compressed;
				stateB = (0 - renormalize & (newState ^ stateB)) ^ stateB;
				compressed += renormalize;

				if (unlikely(compressed > compressedEnd))
					return -1;
			}

			if (accumulatedFreq != HUFFMAN_CODE_SPACE)
				return -1;
			return 0;
		}

		FORCE_INLINE void renormalize(size_t& state, const uint8_t*& stream) {
			size_t consumedBits = unsafe_bit_scan_forward(state);
			stream -= consumedBits >> 3;
			state = (IS_64BIT ? read_uint64le(stream) : read_uint32le(stream)) | 1;
			state <<= consumedBits & 7;
		}

		FORCE_INLINE void decode_op(size_t& state, uint8_t* const out, const HuffmanDecodeTableEntry* const table) {
			const size_t value = state >> ((IS_64BIT ? 64 : 32) - MAX_HUFFMAN_CODE_LENGTH);
			state <<= table[value].bits;
			*out = table[value].symbol;
		}

		void generate_decode_table(const HuffmanDecodeTableEntry symbols[256], HuffmanDecodeTableEntry* table)
		{
			HuffmanDecodeTableEntry* it = table;
			for (int i = 0; i < 256; i++) {
				if (symbols[i].bits > MAX_HUFFMAN_CODE_LENGTH)
					break;
				HuffmanDecodeTableEntry* upperRange = it + (HUFFMAN_CODE_SPACE >> symbols[i].bits);

				uint16_t entry;
				memcpy(&entry, symbols + i, 2);
				const uint64_t filling = 0x0001000100010001 * entry;
				do {
					memcpy(it + 0, &filling, sizeof(uint64_t));
					memcpy(it + 4, &filling, sizeof(uint64_t));
					memcpy(it + 8, &filling, sizeof(uint64_t));
					memcpy(it + 12, &filling, sizeof(uint64_t));
					it += 16;
				} while (it < upperRange);
				it = upperRange;
			}
		}

		template<int CHECK_LEVEL>
		int decode_symbols(const HuffmanDecodeTableEntry symbols[256], const size_t symbolCount, const uint8_t* const compressed, uint8_t* out)
		{
			HuffmanDecodeTableEntry table[HUFFMAN_CODE_SPACE + 16]; //The +16 allows optimizations in table building
			generate_decode_table(symbols, table);

			const uint8_t* streamA = compressed + 12 + read_uint16le(compressed + 0);
			const uint8_t* streamB = streamA + read_uint16le(compressed + 2);
			const uint8_t* streamC = streamB + read_uint16le(compressed + 4);
			const uint8_t* streamD = streamC + read_uint16le(compressed + 6);
			const uint8_t* streamE = streamD + read_uint16le(compressed + 8);
			const uint8_t* streamF = streamE + read_uint16le(compressed + 10);
			const uint8_t* const streamBase = compressed; //Do not read beyond this point

#if IS_64BIT
			streamA -= 8;
			size_t stateA = read_uint64le(streamA) | 1;
			streamB -= 8;
			size_t stateB = read_uint64le(streamB) | 1;
			streamC -= 8;
			size_t stateC = read_uint64le(streamC) | 1;
			streamD -= 8;
			size_t stateD = read_uint64le(streamD) | 1;
			streamE -= 8;
			size_t stateE = read_uint64le(streamE) | 1;
			streamF -= 8;
			size_t stateF = read_uint64le(streamF) | 1;
			uint8_t* const fastLoopEnd = out + (symbolCount - symbolCount % 30);
#else
			streamA -= 4;
			size_t stateA = read_uint32le(streamA) | 1;
			streamB -= 4;
			size_t stateB = read_uint32le(streamB) | 1;
			streamC -= 4;
			size_t stateC = read_uint32le(streamC) | 1;
			streamD -= 4;
			size_t stateD = read_uint32le(streamD) | 1;
			streamE -= 4;
			size_t stateE = read_uint32le(streamE) | 1;
			streamF -= 4;
			size_t stateF = read_uint32le(streamF) | 1;
			uint8_t* const fastLoopEnd = out + (symbolCount - symbolCount % 12);
#endif
			uint8_t* const outEnd = out + symbolCount;

			while (out < fastLoopEnd)
			{
				//First decode
				decode_op(stateA, out + 0, table);
				decode_op(stateB, out + 1, table);
				decode_op(stateC, out + 2, table);
				decode_op(stateD, out + 3, table);
				decode_op(stateE, out + 4, table);
				decode_op(stateF, out + 5, table);
				//Second decode
				decode_op(stateA, out + 6, table);
				decode_op(stateB, out + 7, table);
				decode_op(stateC, out + 8, table);
				decode_op(stateD, out + 9, table);
				decode_op(stateE, out + 10, table);
				decode_op(stateF, out + 11, table);

				if (IS_64BIT) {
					//Third decode
					decode_op(stateA, out + 12, table);
					decode_op(stateB, out + 13, table);
					decode_op(stateC, out + 14, table);
					decode_op(stateD, out + 15, table);
					decode_op(stateE, out + 16, table);
					decode_op(stateF, out + 17, table);
					//Fourth decode
					decode_op(stateA, out + 18, table);
					decode_op(stateB, out + 19, table);
					decode_op(stateC, out + 20, table);
					decode_op(stateD, out + 21, table);
					decode_op(stateE, out + 22, table);
					decode_op(stateF, out + 23, table);
					//Fifth decode
					decode_op(stateA, out + 24, table);
					decode_op(stateB, out + 25, table);
					decode_op(stateC, out + 26, table);
					decode_op(stateD, out + 27, table);
					decode_op(stateE, out + 28, table);
					decode_op(stateF, out + 29, table);
				}

				renormalize(stateA, streamA);
				renormalize(stateB, streamB);
				renormalize(stateC, streamC);
				renormalize(stateD, streamD);
				renormalize(stateE, streamE);
				renormalize(stateF, streamF);
				out += IS_64BIT ? 30 : 12;

				if (CHECK_LEVEL == 0 && unlikely(streamA < streamBase || streamB < streamBase || streamC < streamBase ||
					streamD < streamBase || streamE < streamBase || streamF < streamBase))
					return -1;
			}

			//States are already normalized, thus we have enough bits to decode remaining symbols
			while (out < outEnd)
			{
				decode_op(stateA, out + 0, table);
				decode_op(stateB, out + 1, table);
				decode_op(stateC, out + 2, table);
				decode_op(stateD, out + 3, table);
				decode_op(stateE, out + 4, table);
				decode_op(stateF, out + 5, table);
				out += 6;
			}

			return 0;
		}

		//Returns a pointer to decompressed data, or nullptr if error. compressed will point
		// to end of this compressed stream and streamEnd to the end of decompressed data
		const uint8_t* decode(const uint8_t*& compressed, const uint8_t* const compressedStart, const uint8_t* const compressedEnd, const uint8_t*& streamEnd) {

			uint32_t symbolCount;
			int mode;
			read_entropy_header(compressed, &symbolCount, &mode);
			if (unlikely(compressed > compressedEnd || symbolCount > outBufferSize))
				return nullptr;

			if (mode == ENTROPY_RAW) {
				if (unlikely(compressedEnd - compressed < symbolCount))
					return nullptr;
				const uint8_t* outData = compressed;
				compressed += symbolCount;
				streamEnd = compressed;
				return outData;
			}

			if (mode == ENTROPY_HUFFMAN) {
				if (!outBuffer) {
					try {
						//The +6 is because huffman decoder can write a few more symbols than needed
						outBuffer = new uint8_t[outBufferSize + 6];
					}
					catch (const std::bad_alloc& e) {
						return nullptr;
					}
				}
				streamEnd = outBuffer + symbolCount;

				HuffmanDecodeTableEntry symbols[256];
				int err = decode_huffman_tree(symbols, compressed, compressedEnd);
				if (err)
					return nullptr;

				const uint32_t streamSizeA = read_uint16le(compressed + 0);
				const uint32_t streamSizeB = read_uint16le(compressed + 2);
				const uint32_t streamSizeC = read_uint16le(compressed + 4);
				const uint32_t streamSizeD = read_uint16le(compressed + 6);
				const uint32_t streamSizeE = read_uint16le(compressed + 8);
				const uint32_t streamSizeF = read_uint16le(compressed + 10);
				const size_t totalStreamSize = streamSizeA + streamSizeB + streamSizeC + streamSizeD + streamSizeE + streamSizeF;
				const uint8_t* const compressedStreamEnd = compressed + 12 + totalStreamSize;

				//Huffman decoder reads backwards. If we are far from the beginning of compressed data we can skip input checks.
				if (compressed - compressedStart > symbolCount * 11 / 8)
					err = decode_symbols<1>(symbols, symbolCount, compressed, outBuffer);
				else
					err = decode_symbols<0>(symbols, symbolCount, compressed, outBuffer);

				compressed = compressedStreamEnd;
				return err ? nullptr : outBuffer;
			}

			return nullptr;
		}
	};

	const int8_t inc32table[16] = { 0, 1, 2, 1, 0,  4,  4,  4, 4, 4, 4, 4, 4, 4, 4, 4 };
	const int8_t inc64table[16] = { 0, 0, 0, 1, 4, -1, -2, -3, 4, 4, 4, 4, 4, 4, 4, 4 };

	FORCE_INLINE size_t decode_length(const uint8_t*& compressed) {
		uint8_t byte = *compressed++;
		if (likely(byte <= 251))
			return byte;
		else {
			size_t length = byte - 252;
			length |= read_uint16le(compressed) << 2;
			length += 252;
			compressed += 2;
			return length;
		}
	}

	struct LZDecodeResults {
		int error;
		const uint8_t* tokenStreamIt;
		const uint8_t* literalStreamIt;
		const uint8_t* lengthStreamIt;
		const uint8_t* distanceStreamIt;
		size_t distance;
		uint8_t* decompressed;
	};

	//CHECK_LEVEL modifies the number of checks performed to guarantee input safety. The higher the level, the less checks are performed.
	//Returns the last used distance or -1 on error.
	template<int CHECK_LEVEL>
	LZDecodeResults decode_lz_block(const uint8_t* tokenStreamIt, const uint8_t* literalStreamIt, const uint8_t* lengthStreamIt, const uint8_t* distanceStreamIt,
		const uint8_t* const tokenStreamEnd, const uint8_t* const literalStreamEnd, const uint8_t* const lengthStreamEnd, const uint8_t* const distanceStreamEnd,
		size_t distance, uint8_t* decompressed, uint8_t* const decompressedStart, uint8_t* const thisBlockEnd, uint8_t* const subBlockEnd)
	{
		do {
			const size_t token = *tokenStreamIt++;
			if (CHECK_LEVEL == 0 && unlikely(tokenStreamIt > tokenStreamEnd))
				return LZDecodeResults{ -1 };
			size_t length = token >> 5;

			if (unlikely(length == 7)) {

				length += decode_length(lengthStreamIt);
				//Detect buffer overflows
				if (CHECK_LEVEL == 0 && unlikely(lengthStreamIt > lengthStreamEnd || literalStreamEnd - literalStreamIt < length))
					return LZDecodeResults{ -1 };
				memcpy(decompressed, literalStreamIt, 16);
				memcpy(decompressed + 16, literalStreamIt + 16, 16);

				if (unlikely(length > 32)) {

					//We have the guarantee that decompressed < thisBlockEnd, so there is no overflow
					if (unlikely(thisBlockEnd - decompressed < length))
						return LZDecodeResults{ -1 };

					uint8_t* dst = decompressed + 32;
					const uint8_t* src = literalStreamIt + 32;
					uint8_t* const end = decompressed + length;
					do {
						memcpy(dst, src, 16);
						memcpy(dst + 16, src + 16, 16);
						src += 32;
						dst += 32;
					} while (dst < end);
				}

				decompressed += length;
				literalStreamIt += length;
			}
			else {
				memcpy(decompressed, literalStreamIt, 8);
				decompressed += length;
				literalStreamIt += length;
				//Detect buffer overflows
				if (CHECK_LEVEL == 0 && unlikely(literalStreamIt > literalStreamEnd))
					return LZDecodeResults{ -1 };
			}

			size_t distanceBits = token & 0x18;
			size_t newDistance = read_uint32le(distanceStreamIt) & (1 << distanceBits) - 1;
			length = token & 0x7;
			distance ^= (0 - (distanceBits != 0)) & (distance ^ newDistance);  //branchless selection
			//distance ^= (0 - distanceBits >> 8) & (distance ^ newDistance);
			distanceStreamIt += distanceBits >> 3;
			const uint8_t* match = decompressed - distance;

			if (CHECK_LEVEL < 2 && unlikely(decompressed - decompressedStart < distance))
				return LZDecodeResults{ -1 };
			if (CHECK_LEVEL == 0 && unlikely(distanceStreamIt > distanceStreamEnd))
				return LZDecodeResults{ -1 };

			if (length == 7) {
				length = decode_length(lengthStreamIt) + SKANDA_MIN_MATCH_LENGTH + 7;
				//Detect buffer overflows
				if (CHECK_LEVEL == 0 && unlikely(lengthStreamIt > lengthStreamEnd))
					return LZDecodeResults{ -1 };
				uint8_t* const copyEnd = decompressed + length;

				//If the distance is big enough we can perform a faster copy
				if (likely(distance >= 16)) {
					//Common case length <= 32: omit some instructions
					memcpy(decompressed, match, 16);
					memcpy(decompressed + 16, match + 16, 16);

					if (length > 32) {

						//For long copies make sure we have space. For length < 32 this is not necessary as the end buffer is 63 bytes.
						//Here it is possible we went beyond the block end because of literal copy
						if (unlikely(decompressed > thisBlockEnd || thisBlockEnd - decompressed < length))
							return LZDecodeResults{ -1 };
						match += 32;
						decompressed += 32;

						do {
							memcpy(decompressed, match, 16);
							memcpy(decompressed + 16, match + 16, 16);
							match += 32;
							decompressed += 32;
						} while (decompressed < copyEnd);
					}
				}
				//Else it is a run-length type match
				else {
					decompressed[0] = match[0];
					decompressed[1] = match[1];
					decompressed[2] = match[2];
					decompressed[3] = match[3];
					match += inc32table[distance];
					memcpy(decompressed + 4, match, 4);
					match += inc64table[distance];
					memcpy(decompressed + 8, match, 8);

					if (length > 16) {

						if (unlikely(decompressed > thisBlockEnd || thisBlockEnd - decompressed < length))
							return LZDecodeResults{ -1 };

						decompressed += 16;
						match += 8;
						do {
							memcpy(decompressed, match, 8);
							memcpy(decompressed + 8, match + 8, 8);
							match += 16;
							decompressed += 16;
						} while (decompressed < copyEnd);
					}
				}

				decompressed = copyEnd;
			}
			else {
				//If the distance is big enough we can perform a faster copy
				if (likely(distance >= 8))
					memcpy(decompressed, match, 8);
				//Else it is a run-length type match
				else {
					decompressed[0] = match[0];
					decompressed[1] = match[1];
					decompressed[2] = match[2];
					decompressed[3] = match[3];
					match += inc32table[distance];
					memcpy(decompressed + 4, match, 4);
				}
				decompressed += length + SKANDA_MIN_MATCH_LENGTH;
			}

		} while (likely(decompressed < subBlockEnd));

		return LZDecodeResults{ 0, tokenStreamIt, literalStreamIt, lengthStreamIt, distanceStreamIt, distance, decompressed };
	}

	int decompress(const uint8_t* compressed, size_t compressedSize, uint8_t* decompressed,
		size_t decompressedSize, ProgressCallback* progress)
	{
		//Only last bytes are present
		if (decompressedSize <= SKANDA_LAST_BYTES) {
			if (compressedSize < decompressedSize)
				return -1;
			memcpy(decompressed, compressed, decompressedSize);
			return 0;
		}

		if (unlikely(compressedSize < SKANDA_LAST_BYTES + 1))
			return -1;

		uint8_t* const decompressedStart = decompressed;
		uint8_t* const decompressedEnd = decompressed + decompressedSize - SKANDA_LAST_BYTES;
		const uint8_t* const compressedStart = compressed;
		const uint8_t* const compressedEnd = compressed + compressedSize - SKANDA_LAST_BYTES;
		//First byte is uncompressed
		*decompressed++ = *compressed++;

		HuffmanDecoder literalDecoder(MAX_BLOCK_SIZE + 128);
		HuffmanDecoder tokenDecoder(MAX_BLOCK_SIZE / 2 + 128);
		HuffmanDecoder lengthDecoder(MAX_BLOCK_SIZE / 8 + 128);
		HuffmanDecoder distanceDecoder(MAX_BLOCK_SIZE * 1.5 + 128);
		size_t distance = 1;

		while (decompressed < decompressedEnd) {

			uint32_t blockHeader = read_uint24le(compressed);
			compressed += 3;
			const size_t thisBlockSize = (blockHeader & 0x3FFFF) + 1;
			uint8_t* const thisBlockEnd = decompressed + thisBlockSize;

			const uint8_t* literalStreamEnd;
			const uint8_t* tokenStreamEnd;
			const uint8_t* lengthStreamEnd;
			const uint8_t* distanceStreamEnd;
			const uint8_t* literalStreamIt = literalDecoder.decode(compressed, compressedStart, compressedEnd, literalStreamEnd);
			if (unlikely(literalStreamIt == nullptr))
				return -1;
			const uint8_t* distanceStreamIt = distanceDecoder.decode(compressed, compressedStart, compressedEnd, distanceStreamEnd);
			if (unlikely(distanceStreamIt == nullptr))
				return -1;
			const uint8_t* tokenStreamIt = tokenDecoder.decode(compressed, compressedStart, compressedEnd, tokenStreamEnd);
			if (unlikely(tokenStreamIt == nullptr))
				return -1;
			const uint8_t* lengthStreamIt = lengthDecoder.decode(compressed, compressedStart, compressedEnd, lengthStreamEnd);
			if (unlikely(lengthStreamIt == nullptr))
				return -1;

			while (decompressed < thisBlockEnd) {

				size_t subBlockSize = std::min(16384, int(thisBlockEnd - decompressed));
				uint8_t* subBlockEnd = decompressed + subBlockSize;

				// LZ decoding obviously needs checks for malicious input, but they take a 15% performance hit.
				// We will divide them into 3 categories:
				// - Output buffer overflow: we have a 64 byte buffer at the end, so we only have to check
				//   these on long literal runs or matches, which are rare, so this is not a problem.
				// - Long distances: the only way this check can be avoided is after having decompressed 16MB of data
				// - Compressed streams: we need to make sure we dont read beyond the end of any of the data 4 streams.
				//   Note however, that for a given block size you can only read so many bytes in any stream. Specificaly:
				//   - Distance stream: 3 bytes per match, minimum match length 2 -> 1.5x block size
				//   - Token stream: minimum 2 bytes decompressed per token -> 0.5x block size
				//   - Literal stream -> 1x block size
				//   - Length stream: 1 byte read for 7 bytes of literal run length and 1 for 9 match length -> 0.125x block size
				//   If the streams are huffman compressed we just need a big enough buffer to ensure we cant read outside of it.
				//   If they come directly from compressed buffer we need to make sure we are far away from the end.
				// With these we can minimize the performance penalty while keeping the decompressor safe.

				LZDecodeResults results;
				//Stream was huffman compressed, or is far from the end
				if ((literalStreamIt == literalDecoder.get_output_buffer() || compressedEnd - literalStreamIt >= subBlockSize) &&
					(distanceStreamIt == distanceDecoder.get_output_buffer() || compressedEnd - distanceStreamIt >= subBlockSize * 1.5) &&
					(tokenStreamIt == tokenDecoder.get_output_buffer() || compressedEnd - tokenStreamIt >= subBlockSize / 2) &&
					(lengthStreamIt == lengthDecoder.get_output_buffer() || compressedEnd - lengthStreamIt >= subBlockSize / 8))
				{
					if (decompressed - decompressedStart >= (1 << 24)) {
						results = decode_lz_block<2>(tokenStreamIt, literalStreamIt, lengthStreamIt, distanceStreamIt,
							tokenStreamEnd, literalStreamEnd, lengthStreamEnd, distanceStreamEnd,
							distance, decompressed, decompressedStart, thisBlockEnd, subBlockEnd);
					}
					else {
						results = decode_lz_block<1>(tokenStreamIt, literalStreamIt, lengthStreamIt, distanceStreamIt,
							tokenStreamEnd, literalStreamEnd, lengthStreamEnd, distanceStreamEnd,
							distance, decompressed, decompressedStart, thisBlockEnd, subBlockEnd);
					}
				}
				else {
					results = decode_lz_block<0>(tokenStreamIt, literalStreamIt, lengthStreamIt, distanceStreamIt,
						tokenStreamEnd, literalStreamEnd, lengthStreamEnd, distanceStreamEnd,
						distance, decompressed, decompressedStart, thisBlockEnd, subBlockEnd);
				}

				if (unlikely(results.error))
					return -1;
				tokenStreamIt = results.tokenStreamIt;
				literalStreamIt = results.literalStreamIt;
				lengthStreamIt = results.lengthStreamIt;
				distanceStreamIt = results.distanceStreamIt;
				distance = results.distance;
				decompressed = results.decompressed;
			}

			decompressed = thisBlockEnd;
			if (progress) {
				if (progress->progress(decompressed - decompressedStart, compressed - (compressedEnd - compressedSize)))
					return 0;
			}
		}

		memcpy(decompressed, compressed, SKANDA_LAST_BYTES);
		return 0;
	}
}

#endif  //SKANDA_IMPLEMENTATION

#endif  //__SKANDA__
