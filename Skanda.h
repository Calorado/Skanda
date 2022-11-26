/*
 * Skanda Compression Algorithm v0.5
 * Copyright (c) 2022 Carlos de Diego
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef __SKANDA__

#define __SKANDA__

#include <cstdint>
#include <stdio.h> //size_t
#include <iostream>

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
	//"Level" specifies a tradeoff between compressed size and speed, and must be <= 9.
	//Negative levels specify an acceleration value similar to that of LZ4, up to -63.
	//You may pass a pointer to an object with base class ProgressCallback, to track progress.
	//windowLog determines the maximum backwards distance the matches can have, as a power of 2.
	//Larger values can improve compression, but may hurt decode speed if they exceed CPU cache.
	//Returns the size of the compressed stream or -1 on failure.
	size_t compress(const uint8_t* input, size_t size, uint8_t* output, int level = 1,
		int windowLog = 20, ProgressCallback* progress = nullptr);
	//Decompresses contents in "compressed" to "decompressed".
	//You may also pass a pointer to an object with base class ProgressCallback, to track progress.
	//Returns 0 on success or -1 on failure or corrupted data.
	int decompress(const uint8_t* compressed, size_t compressedSize, uint8_t* decompressed,
		size_t decompressedSize, ProgressCallback* progress = nullptr);

	//For a given input size, returns a size for the output buffer that is big enough to
	// contain the compressed stream even if it expands.
	size_t compress_bound(size_t size);
	//Returns the amount of memory the algorithm will consume on compression.
	size_t estimate_memory(size_t size, int level = 1, int windowLog = 20);
}

#ifdef SKANDA_IMPLEMENTATION

#include <algorithm>
#include <cstring>

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
#if UINTPTR_MAX > UINT32_MAX
  #define IS_64BIT 1
#else
  #define IS_64BIT 0
#endif

#if defined(_MSC_VER)
  #if defined(_M_AMD64)
    #include <intrin.h>
    #define BMI_COMPILE
  #endif
#elif defined(__GNUC__) || defined(__clang__)
  #if defined(__amd64__)
    #if defined(__BMI__) && defined(__BMI2__) 
      #include <x86intrin.h>
      #define BMI_COMPILE
	#endif
  #endif
#endif

#define MIN3(a, b, c) (std::min(a, std::min(b, c)))

namespace skanda {

	FORCE_INLINE bool is_little_endian() {
		const union { uint16_t u; uint8_t c[2]; } LITTLE_ENDIAN_CHECK = { 1 };
		return LITTLE_ENDIAN_CHECK.c[0];
	}

	//Detect if cpu has both bmi1 and bmi2
	FORCE_INLINE bool has_bmi() {
#if defined(BMI_COMPILE) && defined(_MSC_VER)
		int reg[4];
		__cpuid(reg, 0x00000007);
		return ((reg[1] >> 8) & 1) && ((reg[1] >> 3) & 1);
#elif defined(BMI_COMPILE) && (defined(__GNUC__) || defined(__clang__))
		return __builtin_cpu_supports("bmi") && __builtin_cpu_supports("bmi2");
#else
		return false;
#endif 
	}
	const bool HAS_BMI = has_bmi();

#if IS_64BIT

	//Undefined behaviour if value == 0
	FORCE_INLINE uint64_t unsafe_int_log2(const uint64_t value) {
#if defined(_MSC_VER)
		unsigned long result;
		_BitScanReverse64(&result, value);
		return result;
#elif defined(__GNUC__) || defined(__clang__)
		return 63 - __builtin_clzll(value);
#else
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

		uint64_t index = value;
		index |= index >> 1;
		index |= index >> 2;
		index |= index >> 4;
		index |= index >> 8;
		index |= index >> 16;
		index |= index >> 32;
		return tab64[index * 0x03f6eaf2cd271461 >> 58];
#endif
	}

	//Way faster than using log2(double), also returns 0 for a value of 0
	FORCE_INLINE uint64_t int_log2(const uint64_t value) {
#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
		if (likely(value != 0))
			return unsafe_int_log2(value);
		return 0;
#endif  //Fallback already returns 0 when value == 0
		return unsafe_int_log2(value);
	}

	//Undefined behaviour when value == 0
	FORCE_INLINE uint64_t unsafe_bit_scan_forward(const uint64_t value) {
#if defined(_MSC_VER)
		unsigned long result;
		_BitScanForward64(&result, value);
		return result;
#elif defined(__GNUC__) || defined(__clang__)
		return __builtin_ctzll(value);
#else
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
#endif
	}

	//Returns the index of the first set bit, starting from the least significant, or 0 if the input is null
	FORCE_INLINE uint64_t bit_scan_forward(const uint64_t value) {
#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
		if (likely(value != 0))
			return unsafe_bit_scan_forward(value);
		return 0;
#endif  //Fallback already returns 0 when value == 0
		return unsafe_bit_scan_forward(value);
	}

	struct FastIntHash {
		//Use top bits
		uint64_t operator()(const uint64_t value) {
			return value * 0xff51afd7ed558ccd;
		}
	};

	//These functions are used to obtain the data for the hash. If the number of bytes is higher than the word size,
	//they will be mixed. Also note that these might read more bytes than necessary.
	FORCE_INLINE uint64_t read_hash8(const uint8_t* const ptr) {
		uint64_t value;
		memcpy(&value, ptr, 8);
		return value;
	}
	FORCE_INLINE uint64_t read_hash4(const uint8_t* const ptr) {
		uint32_t value;
		memcpy(&value, ptr, 4);
		return value;
	}
	FORCE_INLINE uint64_t read_hash16(const uint8_t* const ptr) {
		return read_hash8(ptr) ^ read_hash8(ptr + 8);
	}
	FORCE_INLINE uint64_t read_hash5(const uint8_t* const ptr) {
		if (is_little_endian())
			return read_hash8(ptr) << 24;
		return read_hash8(ptr) >> 24;
	}
	FORCE_INLINE uint64_t read_hash3(const uint8_t* const ptr) {
		if (is_little_endian())
			return read_hash4(ptr) << 40;
		return read_hash4(ptr) >> 8;
	}

#else

	FORCE_INLINE uint32_t unsafe_int_log2(uint32_t value) {
#if defined(_MSC_VER)
		unsigned long result;
		_BitScanReverse(&result, value);
		return result;
#elif defined(__GNUC__)
		return 31 - __builtin_clz(value);
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
	}

	FORCE_INLINE uint32_t int_log2(uint32_t value) {
#if defined(_MSC_VER) || defined(__GNUC__)
		if (likely(value != 0))
			return unsafe_int_log2(value);
		return 0;
#endif
		return unsafe_int_log2(value);
	}

	FORCE_INLINE uint32_t unsafe_bit_scan_forward(uint32_t value) {
#if defined(_MSC_VER)
		unsigned long result;
		_BitScanForward(&result, value);
		return result;
#elif defined(__GNUC__)
		return __builtin_ctz(value);
#else
		static uint8_t tab32[32] = {
			 0,  1, 28,  2, 29, 14, 24,  3,
			30, 22, 20, 15, 25, 17,  4,  8,
			31, 27, 13, 23, 21, 19, 16,  7,
			26, 12, 18,  6, 11,  5, 10,  9
		};
		return tab32[(value & (0 - value)) * 0x077CB531U >> 27];
#endif
	}

	FORCE_INLINE uint32_t bit_scan_forward(uint32_t value) {
#if defined(_MSC_VER) || defined(__GNUC__)
		if (likely(value != 0))
			return unsafe_bit_scan_forward(value);
		return 0;
#endif
		return unsafe_bit_scan_forward(value);
	}

	struct FastIntHash {
		uint32_t operator()(const uint32_t value) {
			return value * 0x27d4eb2d;
		}
	};

	FORCE_INLINE uint32_t read_hash4(const uint8_t* const ptr) {
		uint32_t value;
		memcpy(&value, ptr, 4);
		return value;
	}
	FORCE_INLINE uint32_t read_hash8(const uint8_t* const ptr) {
		return read_hash4(ptr) ^ read_hash4(ptr + 4);
	}
	FORCE_INLINE uint32_t read_hash16(const uint8_t* const ptr) {
		return read_hash8(ptr) ^ read_hash8(ptr + 8);
	}
	FORCE_INLINE uint32_t read_hash5(const uint8_t* const ptr) {
		return read_hash4(ptr) ^ ptr[4];
	}
	FORCE_INLINE uint32_t read_hash3(const uint8_t* const ptr) {
		if (is_little_endian())
			return read_hash4(ptr) << 8;
		return read_hash4(ptr) >> 8;
	}

#endif

	FORCE_INLINE uint32_t read_uint32le(const uint8_t* const ptr) {
		if (is_little_endian()) {
			uint32_t value;
			memcpy(&value, ptr, 4);
			return value;
		}
		uint32_t value = 0;
		for (int i = 0; i < 4; i++)
			value |= (uint64_t)ptr[i] << i * 8;
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

	//Tries to find a match between the two locations, and returns the length
	//Note that this function should be called with at least MIN_LENGTH + 8 bytes of buffer after limit
	FORCE_INLINE size_t test_match(const uint8_t* front, const uint8_t* back, const uint8_t* const limit,
		const size_t minLength, const int windowLog) {

		//Test first bytes
		//Usually compilers will optimize std::equal as 2 comparisons for minLength 5, 6, etc
		//We can only use one comparison to make it faster. For powers of 2, std::equal should be good
		if (minLength == 3) {
			uint32_t a, b;
			memcpy(&a, front, sizeof(uint32_t));
			memcpy(&b, back, sizeof(uint32_t));
			if (is_little_endian()) {
				if ((a << 8) != (b << 8) || ((size_t)(front - back) >> windowLog))
					return 0;
			}
			else {
				if ((a >> 8) != (b >> 8) || ((size_t)(front - back) >> windowLog))
					return 0;
			}
		}
		else if (minLength == 5 && IS_64BIT) {
			uint64_t a, b;
			memcpy(&a, front, sizeof(uint64_t));
			memcpy(&b, back, sizeof(uint64_t));
			if (is_little_endian()) {
				if ((a << 24) != (b << 24) || ((size_t)(front - back) >> windowLog))
					return 0;
			}
			else {
				if ((a >> 24) != (b >> 24) || ((size_t)(front - back) >> windowLog))
					return 0;
			}
		}
		else if (minLength == 6 && IS_64BIT) {
			uint64_t a, b;
			memcpy(&a, front, sizeof(uint64_t));
			memcpy(&b, back, sizeof(uint64_t));
			if (is_little_endian()) {
				if ((a << 16) != (b << 16) || ((size_t)(front - back) >> windowLog))
					return 0;
			}
			else {
				if ((a >> 16) != (b >> 16) || ((size_t)(front - back) >> windowLog))
					return 0;
			}
		}
		else {
			if (!std::equal(back, back + minLength, front) || ((size_t)(front - back) >> windowLog))
				return 0;
		}

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

	const int GREEDY1 = 0;
	const int GREEDY2 = 1;
	const int LAZY1 = 2;
	const int LAZY2 = 3;
	const int OPTIMAL1 = 4;
	const int OPTIMAL2 = 5;
	const int OPTIMAL3 = 6;

	struct CompressorOptions {
		int parser;
		int maxHashLog;
		int maxElementsLog;
		int niceLength;
		int optimalBlockSize;        // (Optimal)
		int maxArrivals;             // (Optimal)
	};

	template<class IntPtr>
	struct LZStructure {
		IntPtr matchLength;
		IntPtr matchDistance;
		IntPtr literalRunLength;
	};

	template<class IntPtr>
	struct LZMatch {
		IntPtr length;
		IntPtr distance;
	};

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

	template<class IntPtr>
	class LZ2WayCacheBucket {
		IntPtr* data;
	public:
		LZ2WayCacheBucket() {}
		LZ2WayCacheBucket(IntPtr* _data) {
			data = _data;
		}
		//Loads the first value, and at the same time pushes a value into that slot.
		void first(size_t* value) {
			const IntPtr tmp = data[0];
			data[0] = *value;
			*value = tmp;
		}
		//Loads the second value, and at the same time pushes a value into that slot.
		//Should be used after loading the first value.
		void second(size_t* value) {
			const IntPtr tmp = data[1];
			data[1] = *value;
			*value = tmp;
		}
		//Inserts a value into the first slot.
		//Used when skipping bytes.
		void push_in_first(const size_t value) {
			const IntPtr tmp = data[0];
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
	template<class IntPtr, class Hash>
	class LZ2WayCacheTable {
		IntPtr* arr = nullptr;
		size_t hashShift;
	public:
		LZ2WayCacheTable() {}
		~LZ2WayCacheTable() {
			delete[] arr;
		}
		void init(const size_t logSize) {
			arr = new IntPtr[(size_t)2 << logSize]();
			hashShift = (IS_64BIT ? 64 : 32) - logSize;
		}
		LZ2WayCacheBucket<IntPtr> operator[](const size_t value) {
			return LZ2WayCacheBucket<IntPtr>(arr + (Hash{}(value) >> hashShift) * 2);
		}
	};

	//Used for easier implementation
	//Please do not use push() and next() on the same bucket
	template<class IntPtr>
	class LZCacheBucket {
		IntPtr* it;
		IntPtr* last;
	public:
		LZCacheBucket() {}
		LZCacheBucket(IntPtr* _begin, IntPtr* _end) {
			it = _begin;
			last = _end;
		}
		//Pushes a new value into the bucket. Used when skipping bytes
		void push(const size_t newValue) {
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
		void next(size_t* value) {
			const IntPtr tmp = *it;
			*it = *value;
			*value = tmp;
			it++;
		}
		//Loads the next position, but without updating the bucket as we go
		IntPtr next() {
			return *it++;
		}
	};

	//This works like the basic hash table, except that it stores N positions per bucket
	template<class IntPtr, class Hash>
	class LZCacheTable {
		IntPtr* arr = nullptr;
		size_t hashShift;
		size_t elementsLog;  //log2
	public:
		//Use 2^x sizes to avoid the use of modulo and multiplication
		LZCacheTable() {}
		LZCacheTable(const size_t logSize, const size_t numElementsLog) {
			init(logSize, numElementsLog);
		}
		~LZCacheTable() {
			delete[] arr;
		}
		void init(const size_t logSize, const size_t numElementsLog) {
			arr = new IntPtr[(size_t)1 << logSize << numElementsLog]();
			hashShift = (IS_64BIT ? 64 : 32) - logSize;
			elementsLog = numElementsLog;
		}
		LZCacheBucket<IntPtr> operator[](size_t value) {
			value = Hash{}(value) >> hashShift;
			IntPtr* bucket = arr + (value << elementsLog);
			return LZCacheBucket<IntPtr>(bucket, bucket + ((size_t)1 << elementsLog));
		}
	};

	//Simple and fast
	template<class IntPtr>
	class HashTableMatchFinder {
		HashTable<IntPtr, FastIntHash> lzdict3;
		LZCacheTable<IntPtr, FastIntHash> lzdict4;
		LZCacheTable<IntPtr, FastIntHash> lzdict8;

	public:
		void init(const size_t inputSize, const CompressorOptions& compressorOptions, const int windowLog) {
			const int hashLog = MIN3((int)int_log2(inputSize) - 3, compressorOptions.maxHashLog, windowLog - 3);
			lzdict3.init(std::min(hashLog, 14));
			lzdict4.init(hashLog, compressorOptions.maxElementsLog);
			lzdict8.init(hashLog, compressorOptions.maxElementsLog);
		}

		LZMatch<IntPtr>* find_matches_and_update(const uint8_t* const input, const uint8_t* const inputStart,
			const uint8_t* const limit, LZMatch<IntPtr>* matches, size_t minLength,
			const CompressorOptions& compressorOptions, const int windowLog)
		{
			IntPtr& chain3 = lzdict3[read_hash3(input)];
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
				LZCacheBucket<IntPtr> chain4 = lzdict4[read_hash4(input)];
				size_t pos = input - inputStart;
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

			if (nextExpectedLength > 4 && nextExpectedLength <= compressorOptions.niceLength) {
				LZCacheBucket<IntPtr> chain8 = lzdict8[read_hash8(input)];
				size_t pos = input - inputStart;
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
	template<class IntPtr>
	class BinaryMatchFinder {

		HashTable<IntPtr, FastIntHash> lzdict3;
		HashTable<IntPtr, FastIntHash> nodeLookup;
		IntPtr* nodes = nullptr;
		size_t nodeListSize;
		size_t nodeListMask;

	public:

		~BinaryMatchFinder() {
			delete[] nodes;
		}
		BinaryMatchFinder() {}

		void init(const size_t inputSize, const CompressorOptions& compressorOptions, const int windowLog) {

			const size_t binaryTreeWindow = std::min(compressorOptions.maxHashLog, windowLog);

			//Input size is smaller than maximum binary tree size
			if (inputSize < ((size_t)1 << binaryTreeWindow)) {
				nodes = new IntPtr[(size_t)2 * inputSize];
				nodeListSize = inputSize;
				nodeListMask = -1;
			}
			else {
				nodes = new IntPtr[(size_t)2 << binaryTreeWindow];
				nodeListSize = (size_t)1 << binaryTreeWindow;
				nodeListMask = nodeListSize - 1;
			}
			if (compressorOptions.parser < OPTIMAL3)
				lzdict3.init(MIN3((int)int_log2(inputSize) - 3, 14, windowLog - 3));
			nodeLookup.init(MIN3((int)int_log2(inputSize) - 3, 20, windowLog - 3));
		}

		LZMatch<IntPtr>* find_matches_and_update(const uint8_t* const input, const uint8_t* const inputStart,
			const uint8_t* const limit, LZMatch<IntPtr>* matches, size_t minLength,
			const CompressorOptions& compressorOptions, const int windowLog)
		{
			const size_t inputPosition = input - inputStart;
			size_t nextExpectedLength = minLength;

			// First try to get a length 3 match
			if (compressorOptions.parser < OPTIMAL3) {
				IntPtr& chain3 = lzdict3[read_hash3(input)];
				if (nextExpectedLength <= 3) {
					const uint8_t* where = inputStart + chain3;
					const size_t length = test_match(input, where, limit, 3, windowLog);

					if (length >= nextExpectedLength) {
						matches->distance = input - where;
						matches->length = length;
						matches++;

						if (length >= compressorOptions.niceLength) {
							update_position(input, inputStart, limit, compressorOptions, windowLog);
							return matches;
						}
						nextExpectedLength = length + 1;
					}
				}
				chain3 = inputPosition;
			}

			//If we reach this position stop the search
			const size_t btEnd = inputPosition < nodeListSize ? 0 : inputPosition - nodeListSize;

			IntPtr& lookupEntry = nodeLookup[compressorOptions.parser >= OPTIMAL3 ? read_hash3(input) : read_hash4(input)];
			size_t backPosition = lookupEntry;
			lookupEntry = inputPosition;

			IntPtr* lesserNode = &nodes[2 * (lookupEntry & nodeListMask)];
			IntPtr* greaterNode = &nodes[2 * (lookupEntry & nodeListMask) + 1];
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

				const size_t extraLength = test_match(front, back, limit, 0, windowLog);
				front += extraLength;
				back += extraLength;

				size_t length = front - input;
				IntPtr* const nextNode = &nodes[2 * (backPosition & nodeListMask)];
				if (length >= nextExpectedLength) {
					matches->distance = front - back;
					matches->length = length;
					matches++;

					if (length >= compressorOptions.niceLength) {
						*lesserNode = nextNode[0];
						*greaterNode = nextNode[1];
						return matches;
					}
					nextExpectedLength = length;
					if (compressorOptions.parser < OPTIMAL3)
						nextExpectedLength++;
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
			const CompressorOptions& compressorOptions, const int windowLog) {

			const size_t inputPosition = input - inputStart;
			if (compressorOptions.parser < OPTIMAL3)
				lzdict3[read_hash3(input)] = inputPosition;

			//If we reach this position on the front stop the update
			const uint8_t* positionSkip = std::min(limit, input + compressorOptions.niceLength);
			//If we reach this position on the back stop the update
			const size_t btEnd = inputPosition < nodeListSize ? 0 : inputPosition - nodeListSize;
			IntPtr& lookupEntry = nodeLookup[compressorOptions.parser >= OPTIMAL3 ? read_hash3(input) : read_hash4(input)];
			size_t backPosition = lookupEntry;
			lookupEntry = inputPosition;

			IntPtr* lesserNode = &nodes[2 * (inputPosition & nodeListMask)];
			IntPtr* greaterNode = &nodes[2 * (inputPosition & nodeListMask) + 1];

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

				IntPtr* const nextNode = &nodes[2 * (backPosition & nodeListMask)];
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

	//How many bytes to process before reporting progress or checking for abort flag
	const int SKANDA_PROGRESS_REPORT_PERIOD = 512 * 1024;

	FORCE_INLINE void copy_literal_run(const uint8_t* src, uint8_t* dst, const size_t length) {

		uint8_t* const end = dst + length;
		do {
			memcpy(dst, src, 16);
			memcpy(dst + 16, src + 16, 16);
			src += 32;
			dst += 32;
		} while (dst < end);
	}

	FORCE_INLINE void encode_length(uint8_t*& output, size_t var, const size_t overflow) {
		var -= overflow;
		if (likely(var <= 127))
			*output++ = var;
		else {
			int iterations = unsafe_int_log2(var) / 7;
			do {
				*output++ = ((var >> iterations * 7) & 0x7F) | ((iterations != 0) << 7);
				iterations--;
			} while (iterations >= 0);
		}
	}

	const int SKANDA_MIN_MATCH_LENGTH = 2;
	//These are written uncompressed
	const int SKANDA_LAST_BYTES = 63;

	FORCE_INLINE void encode_literal_run(const uint8_t* const input,
		const uint8_t* const literalRunStart, uint8_t* const controlByte, uint8_t*& output) {

		const size_t literalRunLength = input - literalRunStart;

		if (literalRunLength >= 7) {
			size_t literalsToCopy = literalRunLength;
			*controlByte = (7 << 5);
			encode_length(output, literalsToCopy, 7);
			copy_literal_run(literalRunStart, output, literalRunLength);
		}
		else {
			*controlByte = (literalRunLength << 5);
			//It is faster to unconditionally copy 8 bytes
			memcpy(output, literalRunStart, 8);
		}
		output += literalRunLength;
	}

	//Only encodes matches with rep offset
	FORCE_INLINE void encode_rep_match(uint8_t*& output, uint8_t* const controlByte, size_t* matchLength) {

		*matchLength -= SKANDA_MIN_MATCH_LENGTH;
		if (unlikely(*matchLength >= 7)) {
			*controlByte |= 7;
			encode_length(output, *matchLength, 7);
		}
		else
			*controlByte |= *matchLength;
	}

	//Only encodes matches with no rep offset
	FORCE_INLINE void encode_normal_match(uint8_t*& output, uint8_t* const controlByte,
		size_t* matchLength, const size_t& distance, size_t* repOffset) {

		*matchLength -= SKANDA_MIN_MATCH_LENGTH;
		*repOffset = distance;

		size_t bytes = unsafe_int_log2(distance) / 8;
		write_uint32le(output, distance);
		output += bytes + 1;

		*controlByte |= (1 + bytes) << 3;
		if (unlikely(*matchLength >= 7)) {
			*controlByte |= 7;
			encode_length(output, *matchLength, 7);
		}
		else
			*controlByte |= *matchLength;
	}

	//Selects between encoding a normal or a rep match
	FORCE_INLINE void encode_match(uint8_t*& output, uint8_t* const controlByte,
		size_t* matchLength, const size_t& distance, size_t* repOffset) {

		if (*repOffset == distance)
			encode_rep_match(output, controlByte, matchLength);
		else
			encode_normal_match(output, controlByte, matchLength, distance, repOffset);
	}

	template<class IntPtr>
	size_t compress_accelerated(const uint8_t* input, const size_t size, uint8_t* output,
		ProgressCallback* progress, const int windowLog, size_t accelerationBase) {

		//Constants for this encoder
		const size_t hashTableSize = 12;
		const size_t accelerationThreshold = 3;
		const size_t accelerationMax = 63;

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		//The last match or literal run must end before this limit
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;

		size_t repOffset = 1;
		//Keeps track of how many match searches we have made without success. 
		//When we dont find matches, we will skip more or less depending on this variable.
		size_t acceleration = accelerationBase << accelerationThreshold;
		const uint8_t* literalRunStart = input;
		//Skip first byte
		input++;

		//The match finder is a simple hash table with constant size
		HashTable<IntPtr, FastIntHash> lzdict;
		try {
			lzdict.init(hashTableSize);
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		//Main compression loop
		while (input < compressionLimit) {

			const uint8_t* const nextProgressReport = (compressionLimit - input < SKANDA_PROGRESS_REPORT_PERIOD) ?
				compressionLimit : input + SKANDA_PROGRESS_REPORT_PERIOD;

			while (likely(input < nextProgressReport)) {

				//Get possible match location and update the table
				IntPtr* const dictEntry = &lzdict[read_hash8(input)];
				const uint8_t* match = inputStart + *dictEntry;
				*dictEntry = input - inputStart;
				//Try to find a match
				size_t matchLength = test_match(input, match, compressionLimit, 8, windowLog);

				//Have we found a match?
				if (matchLength) {

					//Add the next position to the table as well
					lzdict[read_hash8(input + 1)] = input - inputStart + 1;

					//Try to extend the match to the left
					while (input > literalRunStart && match > inputStart && input[-1] == match[-1]) {
						matchLength++;
						input--;
						match--;
					}
					size_t distance = input - match;

					//First output the literal run
					uint8_t* const controlByte = output++;
					encode_literal_run(input, literalRunStart, controlByte, output);

					input += matchLength;
					literalRunStart = input;
					//Output the match
					encode_match(output, controlByte, &matchLength, distance, &repOffset);

					//Try to find further rep matches
					while (true) {

						matchLength = test_match(input + 1, input + 1 - repOffset, compressionLimit, 3, windowLog);
						if (matchLength == 0)
							break;

						input++;
						uint8_t* const controlByte = output++;
						encode_literal_run(input, literalRunStart, controlByte, output);
						//Add the position of the match to the hash table
						lzdict[read_hash8(input)] = input - inputStart;

						input += matchLength;
						literalRunStart = input;
						encode_rep_match(output, controlByte, &matchLength);
					}

					acceleration = accelerationBase << accelerationThreshold;
				}
				//If there isnt advance the position
				else {
					input += acceleration >> accelerationThreshold;
					if (acceleration < (accelerationMax << accelerationThreshold))
						acceleration++;
				}
			}

			if (progress->progress(input - inputStart, output - outputStart))
				return 0;
		}

		//Due to the acceleration we might have gone beyond the end
		if (input > compressionLimit)
			input = compressionLimit;
		//Output the last literal run. This is necessary even when no more 
		// literals remain, as the decoder expects to end with this
		uint8_t* const controlByte = output++;
		encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		progress->progress(input - inputStart + SKANDA_LAST_BYTES, output - outputStart + SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	template<class IntPtr>
	size_t compress_ultra_fast(const uint8_t* input, const size_t size, uint8_t* output,
		ProgressCallback* progress, const int windowLog) {

		//Constants for this encoder
		const size_t hashTableSize = 12;
		const size_t accelerationThreshold = 3;
		const size_t accelerationMax = 63;

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		//The last match or literal run must end before this limit
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;

		size_t repOffset = 1;
		//Keeps track of how many match searches we have made without success. 
		//When we dont find matches, we will skip more or less depending on this variable.
		size_t acceleration = 1 << accelerationThreshold;
		const uint8_t* literalRunStart = input;
		//Skip first byte
		input++;

		//The match finder is a simple hash table with constant size
		HashTable<IntPtr, FastIntHash> lzdict;
		try {
			lzdict.init(hashTableSize);
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		//Main compression loop
		while (input < compressionLimit) {

			const uint8_t* const nextProgressReport = (compressionLimit - input < SKANDA_PROGRESS_REPORT_PERIOD) ?
				compressionLimit : input + SKANDA_PROGRESS_REPORT_PERIOD;

			while (likely(input < nextProgressReport)) {

				//First try to find a rep match. Doing it at position +1 gives better results.
				//If one is found simply take it and skip normal match finding.
				size_t matchLength = test_match(input + 1, input + 1 - repOffset, compressionLimit, 3, windowLog);
				if (matchLength) {

					input++;
					uint8_t* const controlByte = output++;
					encode_literal_run(input, literalRunStart, controlByte, output);
					//Add the position of the match to the hash table
					lzdict[read_hash5(input)] = input - inputStart;

					input += matchLength;
					literalRunStart = input;
					encode_rep_match(output, controlByte, &matchLength);
					acceleration = 1 << accelerationThreshold;
					continue;
				}

				//Get possible match location and update the table
				IntPtr* const dictEntry = &lzdict[read_hash5(input)];
				const uint8_t* match = inputStart + *dictEntry;
				*dictEntry = input - inputStart;
				//Try to find a match
				matchLength = test_match(input, match, compressionLimit, 5, windowLog);

				//Have we found a match?
				if (matchLength) {

					//Add the next position to the table as well
					lzdict[read_hash5(input + 1)] = input - inputStart + 1;

					//Try to extend the match to the left
					while (input > literalRunStart && match > inputStart && input[-1] == match[-1]) {
						matchLength++;
						input--;
						match--;
					}
					size_t distance = input - match;

					//First output the literal run
					uint8_t* const controlByte = output++;
					encode_literal_run(input, literalRunStart, controlByte, output);

					input += matchLength;
					literalRunStart = input;
					//Output the match
					encode_normal_match(output, controlByte, &matchLength, distance, &repOffset);

					acceleration = 1 << accelerationThreshold;
				}
				//If there isnt advance the position
				else {
					input += acceleration >> accelerationThreshold;
					if (acceleration < (accelerationMax << accelerationThreshold))
						acceleration++;
				}
			}

			if (progress->progress(input - inputStart, output - outputStart))
				return 0;
		}

		//Due to the acceleration we might have gone beyond the end
		if (input > compressionLimit)
			input = compressionLimit;
		//Output the last literal run. This is necessary even when no more 
		// literals remain, as the decoder expects to end with this
		uint8_t* const controlByte = output++;
		encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		progress->progress(input - inputStart + SKANDA_LAST_BYTES, output - outputStart + SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	template<class IntPtr>
	size_t compress_greedy(const uint8_t* input, const size_t size, uint8_t* output,
		ProgressCallback* progress, const int windowLog, const CompressorOptions compressorOptions) {

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;

		size_t repOffset = 1;
		const uint8_t* literalRunStart = input;
		input++;

		const size_t hashLog = MIN3((int)int_log2(size) - 3, compressorOptions.maxHashLog, windowLog - 3);
		HashTable<IntPtr, FastIntHash> lzdict;
		try {
			lzdict.init(hashLog);
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		while (input < compressionLimit) {

			const uint8_t* const nextProgressReport = (compressionLimit - input < SKANDA_PROGRESS_REPORT_PERIOD) ?
				compressionLimit : input + SKANDA_PROGRESS_REPORT_PERIOD;

			while (input < nextProgressReport) {

				//First try a rep match
				size_t matchLength = test_match(input + 1, input + 1 - repOffset, compressionLimit, 3, windowLog);

				if (matchLength) {

					input++;
					uint8_t* const controlByte = output++;
					encode_literal_run(input, literalRunStart, controlByte, output);

					lzdict[read_hash5(input)] = input - inputStart;

					input += matchLength;
					literalRunStart = input;
					encode_rep_match(output, controlByte, &matchLength);
					continue;
				}

				//If no rep, try a normal match
				IntPtr* dictEntry = &lzdict[read_hash5(input)];
				const uint8_t* match = inputStart + *dictEntry;
				*dictEntry = input - inputStart;
				matchLength = test_match(input, match, compressionLimit, 5, windowLog);

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

					uint8_t* const controlByte = output++;
					encode_literal_run(input, literalRunStart, controlByte, output);

					size_t distance = input - match;
					input += matchLength;
					literalRunStart = input;
					encode_normal_match(output, controlByte, &matchLength, distance, &repOffset);
					continue;
				}
				else {
					input++;
				}
			}

			if (progress->progress(input - inputStart, output - outputStart))
				return 0;
		}

		uint8_t* const controlByte = output++;
		encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		progress->progress(input - inputStart + SKANDA_LAST_BYTES, output - outputStart + SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	//Try to find the longest match for a given position. Then try to find an even longer one in the next.
	//If it is found, code a literal, and the longer one found. If not, code the original.
	template<class IntPtr>
	FORCE_INLINE void fast_lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit,
		LZ2WayCacheTable<IntPtr, FastIntHash>* lzdict, size_t* bestMatchLength, size_t* bestMatchDistance,
		int* lazySteps, int* testedPositions, const int windowLog, const CompressorOptions& compressorOptions) {

		*testedPositions = 1;
		LZ2WayCacheBucket<IntPtr> dictEntry = (*lzdict)[read_hash5(input)];

		//Test first entry
		size_t pos = input - inputStart;
		dictEntry.first(&pos);
		const uint8_t* where = inputStart + pos;
		*bestMatchLength = test_match(input, where, limit, 5, windowLog);
		*bestMatchDistance = input - where;
		if (*bestMatchLength >= compressorOptions.niceLength) {
			dictEntry.push_in_second(pos);
			*lazySteps = 0;
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

				if (*bestMatchLength >= compressorOptions.niceLength) {
					*lazySteps = 0;
					return;
				}
			}
		}

		//Nothing was found, code a literal and try again from the begining
		if (*bestMatchLength < 5) {
			*lazySteps = 1;
			return;
		}

		//Now try to find a longer match at next position
		input++;
		*lazySteps = 0;
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

				if (*bestMatchLength >= compressorOptions.niceLength) {
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

	template<class IntPtr>
	size_t compress_lazy_fast(const uint8_t* input, const size_t size, uint8_t* output,
		ProgressCallback* progress, const int windowLog, const CompressorOptions& compressorOptions) {

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;

		size_t repOffset = 1;
		const uint8_t* literalRunStart = input;
		input++;

		const size_t hashLog = MIN3((int)int_log2(size) - 3, compressorOptions.maxHashLog, windowLog - 3);
		LZ2WayCacheTable<IntPtr, FastIntHash> lzdict;
		try {
			lzdict.init(hashLog);
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		while (input < compressionLimit) {

			const uint8_t* const nextProgressReport = (compressionLimit - input < SKANDA_PROGRESS_REPORT_PERIOD) ?
				compressionLimit : input + SKANDA_PROGRESS_REPORT_PERIOD;

			while (input < nextProgressReport) {

				//First try a rep match
				size_t matchLength = test_match(input + 1, input + 1 - repOffset, compressionLimit, 3, windowLog);
				if (matchLength) {

					input++;
					uint8_t* const controlByte = output++;
					encode_literal_run(input, literalRunStart, controlByte, output);

					lzdict[read_hash5(input - 1)].push_in_first(input - 1 - inputStart);
					lzdict[read_hash5(input + 0)].push_in_first(input + 0 - inputStart);
					lzdict[read_hash5(input + 1)].push_in_first(input + 1 - inputStart);
					lzdict[read_hash5(input + 2)].push_in_first(input + 2 - inputStart);

					input += matchLength;
					literalRunStart = input;
					encode_rep_match(output, controlByte, &matchLength);
					continue;
				}

				size_t distance;
				int lazySteps;   //bytes to skip because of lazy matching
				int testedPositions;   //number of positions that have been added to hash table

				fast_lazy_search<IntPtr>(input, inputStart, compressionLimit, &lzdict,
					&matchLength, &distance, &lazySteps, &testedPositions, windowLog, compressorOptions);

				input += lazySteps;

				if (matchLength) {

					const uint8_t* const matchEnd = input + matchLength;
					const uint8_t* matchPos = input + testedPositions - lazySteps;
					for (; matchPos < matchEnd; matchPos++)
						lzdict[read_hash5(matchPos)].push_in_first(matchPos - inputStart);

					const uint8_t* match = input - distance;
					while (input > literalRunStart && match > inputStart && input[-1] == match[-1]) {
						matchLength++;
						input--;
						match--;
					}

					uint8_t* const controlByte = output++;
					encode_literal_run(input, literalRunStart, controlByte, output);

					input = matchEnd;
					literalRunStart = input;
					encode_normal_match(output, controlByte, &matchLength, distance, &repOffset);
				}
			}

			if (progress->progress(input - inputStart, output - outputStart))
				return 0;
		}

		uint8_t* const controlByte = output++;
		encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		progress->progress(input - inputStart + SKANDA_LAST_BYTES, output - outputStart + SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	//Same principle as the last function, but with additional heuristics to improve ratio
	template<class IntPtr>
	FORCE_INLINE void lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit,
		LZCacheTable<IntPtr, FastIntHash>* lzdict4, LZCacheTable<IntPtr, FastIntHash>* lzdict8,
		size_t* bestLength, size_t* bestDistance, int* lazySteps, int* testedPositions,
		const CompressorOptions& compressorOptions, const int windowLog) {

		LZCacheBucket<IntPtr> chain4 = (*lzdict4)[read_hash4(input)];
		LZCacheBucket<IntPtr> chain8 = (*lzdict8)[read_hash8(input)];
		*lazySteps = 0;
		*testedPositions = 1;
		size_t length;
		size_t bestMatchCost = 0;

		//First try to find a length 4 match
		size_t pos = input - inputStart;
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

				//The next length would be 8+, so go to the hash 8 chain
				if (*bestLength >= 7) {
					while (!chain4.ended())
						chain4.next(&pos);
					if (*bestLength >= compressorOptions.niceLength) {
						chain8.push(input - inputStart);
						return;
					}
					break;
				}
			}
		}

		//Only test length 8 if at least one match was found
		if (*bestLength >= 4) {

			pos = input - inputStart;
			while (!chain8.ended()) {
				chain8.next(&pos);

				const uint8_t* where = inputStart + pos;

				if (*(input + *bestLength) != *(where + *bestLength))
					continue;

				length = test_match(input, where, limit, 8, windowLog);

				size_t distance = input - where;
				size_t matchCost = 2 + unsafe_int_log2(distance) / 8;
				if (length + bestMatchCost > matchCost + *bestLength) {
					*bestDistance = distance;
					*bestLength = length;
					bestMatchCost = matchCost;

					if (*bestLength >= compressorOptions.niceLength) {
						while (!chain8.ended())
							chain8.next(&pos);
						return;
					}
				}
			}
		}
		else {
			//No match found, code a literal and retry
			*lazySteps = 1;
			return;
		}

		//Now try to get a better match at pos + 1
		input++;
		pos = input - inputStart;
		//We wont search for length < 8
		(*lzdict4)[read_hash4(input)].push(pos);
		chain8 = (*lzdict8)[read_hash8(input)];
		*testedPositions = 2;

		//Only try to find matches of length at least 8 at pos + 1
		while (!chain8.ended()) {
			chain8.next(&pos);

			const uint8_t* where = inputStart + pos;

			if (*(input + *bestLength) != *(where + *bestLength))
				continue;

			length = test_match(input, where, limit, 8, windowLog);

			size_t distance = input - where;
			size_t matchCost = 2 + unsafe_int_log2(distance) / 8;
			if (length + bestMatchCost > matchCost + *bestLength) {
				*bestDistance = distance;
				*bestLength = length;
				*lazySteps = 1;
				bestMatchCost = matchCost;

				if (*bestLength >= compressorOptions.niceLength) {
					while (!chain8.ended())
						chain8.next(&pos);
					return;
				}
			}
		}

		//Now get an even better match at pos + 2
		input++;
		pos = input - inputStart;
		(*lzdict4)[read_hash4(input)].push(pos);
		chain8 = (*lzdict8)[read_hash8(input)];
		*testedPositions = 3;

		while (!chain8.ended()) {
			chain8.next(&pos);

			const uint8_t* where = inputStart + pos;

			if (*(input + *bestLength) != *(where + *bestLength))
				continue;

			length = test_match(input, where, limit, 8, windowLog);

			size_t distance = input - where;
			size_t matchCost = 2 + unsafe_int_log2(distance) / 8;
			if (length + bestMatchCost > matchCost + *bestLength) {
				*bestDistance = distance;
				*bestLength = length;
				*lazySteps = 2;
				bestMatchCost = matchCost;

				if (*bestLength >= compressorOptions.niceLength) {
					while (!chain8.ended())
						chain8.next(&pos);
					return;
				}
			}
		}
	}

	template<class IntPtr>
	size_t compress_lazy(const uint8_t* input, const size_t size, uint8_t* output,
		ProgressCallback* progress, const int windowLog, const CompressorOptions& compressorOptions) {

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;

		size_t repOffset = 1;
		const uint8_t* literalRunStart = input;
		input++;

		const int hashLog = MIN3((int)int_log2(size) - 3, compressorOptions.maxHashLog, windowLog - 3);
		LZCacheTable<IntPtr, FastIntHash> lzdict4;
		LZCacheTable<IntPtr, FastIntHash> lzdict8;

		try {
			lzdict4.init(hashLog, compressorOptions.maxElementsLog);
			lzdict8.init(hashLog, compressorOptions.maxElementsLog);
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		while (input < compressionLimit) {

			const uint8_t* const nextProgressReport = (compressionLimit - input < SKANDA_PROGRESS_REPORT_PERIOD) ?
				compressionLimit : input + SKANDA_PROGRESS_REPORT_PERIOD;

			while (input < nextProgressReport) {

				//First try a rep match
				size_t matchLength = test_match(input + 1, input + 1 - repOffset, compressionLimit, 3, windowLog);
				if (matchLength) {

					input++;
					uint8_t* const controlByte = output++;
					encode_literal_run(input, literalRunStart, controlByte, output);

					lzdict4[read_hash4(input - 1)].push(input - 1 - inputStart);
					lzdict8[read_hash8(input - 1)].push(input - 1 - inputStart);
					lzdict4[read_hash4(input + 0)].push(input + 0 - inputStart);
					lzdict8[read_hash8(input + 0)].push(input + 0 - inputStart);
					lzdict4[read_hash4(input + 1)].push(input + 1 - inputStart);
					lzdict8[read_hash8(input + 1)].push(input + 1 - inputStart);
					lzdict4[read_hash4(input + 2)].push(input + 2 - inputStart);
					lzdict8[read_hash8(input + 2)].push(input + 2 - inputStart);

					input += matchLength;
					literalRunStart = input;
					encode_rep_match(output, controlByte, &matchLength);
					continue;
				}

				size_t distance;
				int lazySteps;
				int testedPositions;

				lazy_search(input, inputStart, compressionLimit, &lzdict4, &lzdict8, &matchLength,
					&distance, &lazySteps, &testedPositions, compressorOptions, windowLog);

				input += lazySteps;

				if (matchLength) {

					const uint8_t* const matchEnd = input + matchLength;
					const uint8_t* matchPos = input + testedPositions - lazySteps;
					for (; matchPos != matchEnd; matchPos++) {
						lzdict4[read_hash4(matchPos)].push(matchPos - inputStart);
						lzdict8[read_hash8(matchPos)].push(matchPos - inputStart);
					}

					const uint8_t* match = input - distance;
					while (input > literalRunStart && match > inputStart && input[-1] == match[-1]) {
						matchLength++;
						input--;
						match--;
					}

					uint8_t* const controlByte = output++;
					encode_literal_run(input, literalRunStart, controlByte, output);

					input = matchEnd;
					literalRunStart = input;
					encode_match(output, controlByte, &matchLength, distance, &repOffset);
				}
			}

			if (progress->progress(input - inputStart, output - outputStart))
				return 0;
		}

		uint8_t* const controlByte = output++;
		encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		progress->progress(input - inputStart + SKANDA_LAST_BYTES, output - outputStart + SKANDA_LAST_BYTES);

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

	template<class IntPtr>
	LZStructure<IntPtr>* forward_optimal_parse(const uint8_t* const input, const uint8_t* const inputStart,
		const uint8_t* const limit, HashTableMatchFinder<IntPtr>* matchFinder, SkandaOptimalParserState* parser,
		LZStructure<IntPtr>* stream, const size_t startRepOffset, const CompressorOptions& compressorOptions,
		const int windowLog)
	{
		const size_t blockLength = std::min((size_t)(limit - input), (size_t)compressorOptions.optimalBlockSize);
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
		for (; position < blockLength; position++) {

			const uint8_t* inputPosition = input + position;
			SkandaOptimalParserState* const parserPosition = parser + position;
			//Make sure we have enough positions ahead initialized
			parserPosition[compressorOptions.niceLength].cost = UINT32_MAX;

			//From Zstd: skip unpromising position
			if (parserPosition[1].cost <= parserPosition[0].cost) {
				matchFinder->update_position(inputPosition, inputStart);
				continue;
			}

			//Size cost
			size_t literalCost = parserPosition->cost + (0x10000 << (parserPosition->literalRunLength == 6));
			literalCost += parserPosition->literalRunLength == 6; //Speed cost
			SkandaOptimalParserState* nextPosition = parserPosition + 1;
			if (literalCost < nextPosition->cost) {
				nextPosition->cost = literalCost;
				nextPosition->distance = parserPosition->distance; //Carry forward the rep offset
				nextPosition->literalRunLength = parserPosition->literalRunLength + 1;
			}

			size_t repMatchLength = test_match(inputPosition, inputPosition - parserPosition->distance, limit, 2, windowLog);
			if (repMatchLength) {
				//Rep matches can be unconditionally taken with lower lengths
				if (repMatchLength >= compressorOptions.niceLength / 2) {
					lastMatchLength = repMatchLength;
					lastMatchDistance = parserPosition->distance;
					break;
				}

				size_t repMatchCost = parserPosition->cost;
				repMatchCost += 0x10000 << (repMatchLength > 8);  //size cost
				repMatchCost += 1 + (repMatchLength > 8);  //speed cost

				nextPosition = parserPosition + repMatchLength;
				if (repMatchCost < nextPosition->cost) {
					nextPosition->cost = repMatchCost;
					nextPosition->matchLength = repMatchLength;
					nextPosition->distance = parserPosition->distance;
					nextPosition->literalRunLength = 0;
				}
			}

			LZMatch<IntPtr> matches[17];
			const LZMatch<IntPtr>* matchesEnd = matchFinder->find_matches_and_update(inputPosition, inputStart,
				limit, matches, repMatchLength + 2, compressorOptions, windowLog);

			//At least one match was found
			if (matchesEnd != matches) {
				//The last match should be the longest
				const LZMatch<IntPtr>* const longestMatch = matchesEnd - 1;
				if (longestMatch->length >= compressorOptions.niceLength) {

					//Perform left match extension
					size_t matchLength = longestMatch->length;
					const uint8_t* leftExtension = inputPosition;
					const uint8_t* literalRunStart = inputPosition - parserPosition->literalRunLength;
					const uint8_t* match = inputPosition - longestMatch->distance;
					while (leftExtension > literalRunStart && match > inputStart && leftExtension[-1] == match[-1]) {
						matchLength++;
						leftExtension--;
						match--;
						position--;
					}

					lastMatchLength = matchLength;
					lastMatchDistance = longestMatch->distance;
					break;
				}

				//The parsing cycle always starts with a literal run length of 0 -> this wont go beyond the beginning of this cycle
				const uint8_t* const literalRunStart = inputPosition - parserPosition->literalRunLength;
				for (const LZMatch<IntPtr>* matchIt = longestMatch; matchIt >= matches; matchIt--) {

					//Perform left match extension
					size_t matchLength = matchIt->length;
					nextPosition = parserPosition + matchLength;

					const uint8_t* leftExtension = inputPosition;
					const uint8_t* match = inputPosition - matchIt->distance;
					while (leftExtension > literalRunStart && match > inputStart && leftExtension[-1] == match[-1]) {
						matchLength++;
						leftExtension--;
						match--;
					}

					SkandaOptimalParserState* prevState = nextPosition - matchLength;
					size_t matchCost = prevState->cost;
					//size cost
					matchCost += (2 + unsafe_int_log2(matchIt->distance) / 8 + (matchLength > 8)) << 16;
					matchCost += 1 + (matchLength > 8); //speed cost

					if (matchCost < nextPosition->cost) {
						nextPosition->cost = matchCost;
						nextPosition->matchLength = matchLength;
						nextPosition->distance = matchIt->distance;
						nextPosition->literalRunLength = 0;
					}
					//If this match does not help, the shorter ones probably wont
					else
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

			const uint8_t* const matchEnd = input + position + lastMatchLength;
			const uint8_t* inputPosition = input + position + 1;
			for (; inputPosition < matchEnd; inputPosition++)
				matchFinder->update_position(inputPosition, inputStart);
		}
		else {
			size_t bestPos = blockLength;
			size_t bestSize = parser[blockLength].cost;
			for (size_t i = 1; i < compressorOptions.niceLength; i++) {
				if (parser[blockLength + i].cost <= bestSize) {
					bestPos = blockLength + i;
					bestSize = parser[blockLength + i].cost;
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

	template<class IntPtr>
	LZStructure<IntPtr>* multi_arrivals_parse(const uint8_t* input, const uint8_t* const inputStart,
		const uint8_t* const limit, BinaryMatchFinder<IntPtr>* matchFinder, SkandaOptimalParserState* parser,
		LZStructure<IntPtr>* stream, const size_t startRepOffset, const CompressorOptions& compressorOptions,
		const int windowLog)
	{
		const size_t blockLength = std::min((size_t)(limit - input), (size_t)compressorOptions.optimalBlockSize - 1);
		for (size_t i = 0; i < (blockLength + compressorOptions.niceLength) * compressorOptions.maxArrivals; i++)
			parser[i].cost = UINT32_MAX;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;
		size_t lastMatchPath;

		//Fill only the first arrival of the first position
		parser[0].cost = 0;
		parser[0].distance = startRepOffset;
		parser[0].literalRunLength = 0;

		size_t position = 0;
		for (; position < blockLength; position++) {

			const uint8_t* inputPosition = input + position;
			SkandaOptimalParserState* const parserPosition = parser + position * compressorOptions.maxArrivals;

			//Unpromising position
			if (parserPosition[compressorOptions.maxArrivals].cost < parserPosition[0].cost) {
				matchFinder->update_position(inputPosition, inputStart, limit, compressorOptions, windowLog);
				continue;
			}

			size_t expectedNextLength = 2;  //Only take rep matches as long as this

			//Literal and rep match parsing can be done at the same time
			for (size_t i = 0; i < compressorOptions.maxArrivals; i++) {

				SkandaOptimalParserState* const currentArrival = parserPosition + i;

				//There might be less than max arrivals for this position. Note that there will always be at least one
				if (currentArrival->cost == UINT32_MAX)
					break;

				//only size cost
				size_t literalCost = currentArrival->cost + (0x10000 << (currentArrival->literalRunLength == 6));
				literalCost += currentArrival->literalRunLength == 6;
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

				size_t repMatchLength = test_match(inputPosition, inputPosition - currentArrival->distance, limit, 2, windowLog);
				//Since we go from lowest cost arrival to highest, it makes sense that the rep match
				// should be at least as long as the best found so far
				if (repMatchLength >= expectedNextLength) {
					//Rep matches can be unconditionally taken with lower lengths
					if (repMatchLength >= compressorOptions.niceLength / 2) {
						lastMatchLength = repMatchLength;
						lastMatchDistance = currentArrival->distance;
						lastMatchPath = i;
						goto doBackwardParse;
					}

					expectedNextLength = repMatchLength;
					//Small heuristic: instead of testing all positions, only test the maximum match length,
					// and if it overflows, just before the overflow
					//There is a notable speed increase for a negligible size penalty
					size_t repMatchCost = currentArrival->cost;
					repMatchCost += 0x10000 << (repMatchLength > 8);  //size cost
					repMatchCost += 1 + (repMatchLength > 8);  //speed cost

					arrivalIt = parserPosition + repMatchLength * compressorOptions.maxArrivals;
					lastArrival = arrivalIt + compressorOptions.maxArrivals;

					for (; arrivalIt < lastArrival; arrivalIt++) {

						if (repMatchCost < arrivalIt->cost) {

							for (SkandaOptimalParserState* it = lastArrival - 1; it != arrivalIt; it--)
								memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState));

							arrivalIt->cost = repMatchCost;
							arrivalIt->matchLength = repMatchLength;
							arrivalIt->distance = currentArrival->distance;
							arrivalIt->literalRunLength = 0;
							arrivalIt->path = i;

							//Only try a length of 8 if the full length actually resulted in a better arrival
							if (repMatchLength > 8) {

								repMatchCost -= 0x10001;  //Remove the cost of the additional length byte

								arrivalIt = parserPosition + 8 * compressorOptions.maxArrivals;
								lastArrival = arrivalIt + compressorOptions.maxArrivals;

								for (; arrivalIt < lastArrival; arrivalIt++) {

									if (repMatchCost < arrivalIt->cost) {

										for (SkandaOptimalParserState* it = lastArrival - 1; it != arrivalIt; it--)
											memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState));

										arrivalIt->cost = repMatchCost;
										arrivalIt->matchLength = 8;
										arrivalIt->distance = currentArrival->distance;
										arrivalIt->literalRunLength = 0;
										arrivalIt->path = i;
										break;
									}
								}
							}
							break;
						}
					}
				}
			}

			LZMatch<IntPtr> matches[66];
			LZMatch<IntPtr>* matchesEnd = matchFinder->find_matches_and_update(inputPosition, inputStart, limit, matches,
				std::max(expectedNextLength, (size_t)3), compressorOptions, windowLog);

			if (matchesEnd != matches) {

				//We have the guarantee that matches are in increasing order
				const LZMatch<IntPtr>* const longestMatch = matchesEnd - 1;
				if (longestMatch->length >= compressorOptions.niceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					lastMatchPath = 0;
					break;
				}

				bool lengthOverflowTested = false;
				const size_t lengthOverflow = 8;

				for (LZMatch<IntPtr>* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					if (matchIt->distance == parserPosition->distance)
						continue;

					size_t matchCost = parserPosition->cost;
					//size cost
					matchCost += (2 + unsafe_int_log2(matchIt->distance) / 8 + (matchIt->length > lengthOverflow)) << 16;
					matchCost += 1 + (matchIt->length > lengthOverflow);  //speed cost

					SkandaOptimalParserState* arrivalIt = parserPosition + matchIt->length * compressorOptions.maxArrivals;
					SkandaOptimalParserState* lastArrival = arrivalIt + compressorOptions.maxArrivals;

					for (; arrivalIt < lastArrival; arrivalIt++) {

						if (matchCost < arrivalIt->cost) {

							for (SkandaOptimalParserState* it = lastArrival - 1; it != arrivalIt; it--)
								memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState));

							arrivalIt->cost = matchCost;
							arrivalIt->matchLength = matchIt->length;
							arrivalIt->distance = matchIt->distance;
							arrivalIt->literalRunLength = 0;
							arrivalIt->path = 0;

							//If the current match has a length that overflows, and we have not tried any
							// length just below that overflow, try it
							if (!lengthOverflowTested && matchIt->length > lengthOverflow) {
								matchCost -= 0x10001; //Remove length overflow cost
								arrivalIt = parserPosition + lengthOverflow * compressorOptions.maxArrivals;
								lastArrival = arrivalIt + compressorOptions.maxArrivals;

								for (; arrivalIt < lastArrival; arrivalIt++) {

									if (matchCost < arrivalIt->cost) {

										for (SkandaOptimalParserState* it = lastArrival - 1; it != arrivalIt; it--)
											memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState));

										arrivalIt->cost = matchCost;
										arrivalIt->matchLength = lengthOverflow;
										arrivalIt->distance = matchIt->distance;
										arrivalIt->literalRunLength = 0;
										arrivalIt->path = 0;
										break;
									}
								}
							}
							break;
						}
					}
					lengthOverflowTested = matchIt->length >= lengthOverflow;
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

			const uint8_t* const matchEnd = input + position + lastMatchLength;
			const uint8_t* inputPosition = input + position + 1;
			for (; inputPosition < matchEnd; inputPosition++)
				matchFinder->update_position(inputPosition, inputStart, limit, compressorOptions, windowLog);
		}
		else {
			size_t bestPos = blockLength;
			size_t bestSize = parser[blockLength * compressorOptions.maxArrivals].cost;
			for (size_t i = 1; i < compressorOptions.niceLength; i++) {
				if (parser[(blockLength + i) * compressorOptions.maxArrivals].cost <= bestSize) {
					bestPos = blockLength + i;
					bestSize = parser[(blockLength + i) * compressorOptions.maxArrivals].cost;
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

	template<class IntPtr>
	size_t compress_optimal(const uint8_t* input, const size_t size, uint8_t* output,
		ProgressCallback* progress, const int windowLog, const CompressorOptions& compressorOptions) {

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;

		//Data for the format
		size_t repOffset = 1;
		//Skip first byte
		const uint8_t* literalRunStart = input;
		input++;

		HashTableMatchFinder<IntPtr> hashMatchFinder;
		BinaryMatchFinder<IntPtr> binaryMatchFinder;
		SkandaOptimalParserState* parser = nullptr;
		LZStructure<IntPtr>* stream = nullptr;

		try {
			if (compressorOptions.parser == OPTIMAL1) {
				hashMatchFinder.init(size, compressorOptions, windowLog);
				parser = new SkandaOptimalParserState[compressorOptions.optimalBlockSize + compressorOptions.niceLength];
			}
			else {
				binaryMatchFinder.init(size, compressorOptions, windowLog);
				parser = new SkandaOptimalParserState[(compressorOptions.optimalBlockSize + compressorOptions.niceLength) * compressorOptions.maxArrivals];
			}
			stream = new LZStructure<IntPtr>[compressorOptions.optimalBlockSize];
		}
		catch (const std::bad_alloc& e) {
			delete[] parser;
			delete[] stream;
			return -1;
		}

		while (input < compressionLimit) {

			const uint8_t* const nextProgressReport = (compressionLimit - input < SKANDA_PROGRESS_REPORT_PERIOD) ?
				compressionLimit : input + SKANDA_PROGRESS_REPORT_PERIOD;

			while (input < nextProgressReport) {

				LZStructure<IntPtr>* streamIt;
				if (compressorOptions.parser >= OPTIMAL2) {
					streamIt = multi_arrivals_parse(input, inputStart, compressionLimit, &binaryMatchFinder,
						parser, stream, repOffset, compressorOptions, windowLog);
				}
				else {
					streamIt = forward_optimal_parse(input, inputStart, compressionLimit, &hashMatchFinder,
						parser, stream, repOffset, compressorOptions, windowLog);
				}

				//Main compression loop
				while (true) {
					input += streamIt->literalRunLength;

					if (streamIt == stream)
						break;

					size_t matchLength = streamIt->matchLength;
					size_t distance = streamIt->matchDistance;

					uint8_t* const controlByte = output++;
					encode_literal_run(input, literalRunStart, controlByte, output);

					input += matchLength;
					encode_match(output, controlByte, &matchLength, distance, &repOffset);
					literalRunStart = input;

					streamIt--;
				}
			}

			if (progress->progress(input - inputStart, output - outputStart)) {
				delete[] parser;
				delete[] stream;
				return 0;
			}
		}

		delete[] parser;
		delete[] stream;

		uint8_t* const controlByte = output++;
		encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		progress->progress(input - inputStart + SKANDA_LAST_BYTES, output - outputStart + SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	const int NOT_USED = -1;
	const CompressorOptions skandaCompressorLevels[] = {
		//      Parser        Hash log     Elements per hash     Nice length      Block size      Max arrivals
			{ GREEDY1      ,  NOT_USED  ,      NOT_USED       ,    NOT_USED   ,    NOT_USED     ,   NOT_USED   },
			{ GREEDY2      ,     16     ,      NOT_USED       ,    NOT_USED   ,    NOT_USED     ,   NOT_USED   },
			{ LAZY1        ,     16     ,      NOT_USED       ,       32      ,    NOT_USED     ,   NOT_USED   },
			{ LAZY2        ,     17     ,          1          ,       32      ,    NOT_USED     ,   NOT_USED   },
			{ LAZY2        ,     18     ,          2          ,       32      ,    NOT_USED     ,   NOT_USED   },
			{ OPTIMAL1     ,     18     ,          2          ,       32      ,      1024       ,   NOT_USED   },
			{ OPTIMAL1     ,     19     ,          3          ,       64      ,      1024       ,   NOT_USED   },
			{ OPTIMAL2     ,     22     ,          4          ,       128     ,      4096       ,       2      },
			{ OPTIMAL2     ,     24     ,          5          ,       320     ,      4096       ,       4      },
			{ OPTIMAL3     ,     24     ,          6          ,       1024    ,      8192       ,       8      },
			{ OPTIMAL3     ,     24     ,          6          ,       4096    ,      8192       ,       16     },
	};

	size_t compress(const uint8_t* input, size_t size, uint8_t* output,
		int level, int windowLog, ProgressCallback* progress) {

		ProgressCallback defaultProgress;
		if (!progress)
			progress = &defaultProgress;

		if (size <= SKANDA_LAST_BYTES) {
			memcpy(output, input, size);
			progress->progress(size, size);
			return size;
		}

		if (level < -63)
			level = -63;
		if (level > 10)
			level = 10;
		if (windowLog > 24)
			windowLog = 24;
		if (windowLog < 6)
			windowLog = 6;

#if defined(IS_64BIT) && defined(SKANDA_LARGE_INPUT_SUPPORT)
		if (size > ((uint64_t)1 << 32)) {
			if (level < 0)
				return compress_accelerated<uint64_t>(input, size, output, progress, windowLog, -level);
			if (skandaCompressorLevels[level].parser == GREEDY1)
				return compress_ultra_fast<uint64_t>(input, size, output, progress, windowLog);
			if (skandaCompressorLevels[level].parser == GREEDY2)
				return compress_greedy<uint64_t>(input, size, output, progress, windowLog, skandaCompressorLevels[level]);
			if (skandaCompressorLevels[level].parser == LAZY1)
				return compress_lazy_fast<uint64_t>(input, size, output, progress, windowLog, skandaCompressorLevels[level]);
			if (skandaCompressorLevels[level].parser == LAZY2)
				return compress_lazy<uint64_t>(input, size, output, progress, windowLog, skandaCompressorLevels[level]);
			return compress_optimal<uint64_t>(input, size, output, progress, windowLog, skandaCompressorLevels[level]);
		}
#endif
		if (level < 0)
			return compress_accelerated<uint32_t>(input, size, output, progress, windowLog, -level);
		if (skandaCompressorLevels[level].parser == GREEDY1)
			return compress_ultra_fast<uint32_t>(input, size, output, progress, windowLog);
		if (skandaCompressorLevels[level].parser == GREEDY2)
			return compress_greedy<uint32_t>(input, size, output, progress, windowLog, skandaCompressorLevels[level]);
		if (skandaCompressorLevels[level].parser == LAZY1)
			return compress_lazy_fast<uint32_t>(input, size, output, progress, windowLog, skandaCompressorLevels[level]);
		if (skandaCompressorLevels[level].parser == LAZY2)
			return compress_lazy<uint32_t>(input, size, output, progress, windowLog, skandaCompressorLevels[level]);
		return compress_optimal<uint32_t>(input, size, output, progress, windowLog, skandaCompressorLevels[level]);
	}

	size_t compress_bound(size_t size) {
		return size + size / 512 + 8;
	}

	template<class IntPtr>
	size_t memory_estimator(size_t size, int windowLog, int level) {
		if (level < 0)
			return sizeof(IntPtr) << 12;
		if (skandaCompressorLevels[level].parser == GREEDY1)
			return sizeof(IntPtr) << 12;
		if (skandaCompressorLevels[level].parser == GREEDY2)
			return sizeof(IntPtr) << MIN3((int)int_log2(size) - 3, skandaCompressorLevels[level].maxHashLog, windowLog - 3);
		if (skandaCompressorLevels[level].parser == LAZY1)
			return sizeof(IntPtr) <<
			MIN3((int)int_log2(size) - 3, skandaCompressorLevels[level].maxHashLog, windowLog - 3) << 1;
		if (skandaCompressorLevels[level].parser == LAZY2)
			//Lazy extra uses 2 tables
			return sizeof(IntPtr) << MIN3((int)int_log2(size) - 3, skandaCompressorLevels[level].maxHashLog, windowLog - 3)
			<< skandaCompressorLevels[level].maxElementsLog << 1;
		if (skandaCompressorLevels[level].parser == OPTIMAL1) {
			const int log2size = MIN3((int)int_log2(size) - 3, skandaCompressorLevels[level].maxHashLog, windowLog - 3);
			size_t memory = sizeof(IntPtr) << std::min(log2size, 14);  //hash 3 table
			//hash 4 and hash 8 tables
			memory += sizeof(IntPtr) << log2size << skandaCompressorLevels[level].maxElementsLog << 1;
			memory += sizeof(SkandaOptimalParserState) *
				(skandaCompressorLevels[level].optimalBlockSize + skandaCompressorLevels[level].niceLength);
			memory += sizeof(LZStructure<IntPtr>) * skandaCompressorLevels[level].optimalBlockSize;
			return memory;
		}
		size_t memory = sizeof(IntPtr) << MIN3((int)int_log2(size) - 3, 20, windowLog - 3);  //binary node lookup
		if (skandaCompressorLevels[level].parser < OPTIMAL3)
			memory += sizeof(IntPtr) << MIN3((int)int_log2(size) - 3, 14, windowLog - 3);  //hash 3 table
		const size_t binaryTreeWindow = (size_t)1 << std::min(skandaCompressorLevels[level].maxHashLog, windowLog);
		if (size < binaryTreeWindow)
			memory += sizeof(IntPtr) * 2 * size;  //binary tree
		else
			memory += sizeof(IntPtr) * 2 * binaryTreeWindow;  //binary tree
		memory += sizeof(SkandaOptimalParserState) *
			(skandaCompressorLevels[level].optimalBlockSize + skandaCompressorLevels[level].niceLength) *
			skandaCompressorLevels[level].maxArrivals;
		memory += sizeof(LZStructure<IntPtr>) * skandaCompressorLevels[level].optimalBlockSize;
		return memory;
	}

	size_t estimate_memory(size_t size, int level, int windowLog) {

		//Negative levels use the same memory as level 0
		if (level < 0)
			level = 0;
		if (level > 10)
			level = 10;
		if (windowLog > 24)
			windowLog = 24;
		if (windowLog < 6)
			windowLog = 6;

		if (size <= SKANDA_LAST_BYTES + 1)
			return 0;

#ifdef IS_64BIT
		if (size > ((uint64_t)1 << 32))
			return memory_estimator<uint64_t>(size, windowLog, level);
#endif
		return memory_estimator<uint32_t>(size, windowLog, level);
	}

	const int8_t inc32table[16] = { 0, 1, 2, 1, 0,  4,  4,  4, 4, 4, 4, 4, 4, 4, 4, 4 };
	const int8_t inc64table[16] = { 0, 0, 0, 1, 4, -1, -2, -3, 4, 4, 4, 4, 4, 4, 4, 4 };

	//Optimized for long matches
	FORCE_INLINE void copy_match(uint8_t* dst, const size_t offset, const size_t length) {

		uint8_t* const end = dst + length;
		const uint8_t* src = dst - offset;

		//If the offset is big enough we can perform a faster copy
		if (offset >= 16) {
			do {
				memcpy(dst, src, 16);
				memcpy(dst + 16, src + 16, 16);
				src += 32;
				dst += 32;
			} while (dst < end);
		}
		//Else it is a run-length type match
		else {

			dst[0] = src[0];
			dst[1] = src[1];
			dst[2] = src[2];
			dst[3] = src[3];
			src += inc32table[offset];
			memcpy(dst + 4, src, 4);
			src += inc64table[offset];
			memcpy(dst + 8, src, 8);

			dst += 16;
			src += 8;
			do {
				memcpy(dst, src, 8);
				memcpy(dst + 8, src + 8, 8);
				src += 16;
				dst += 16;
			} while (dst < end);
		}
	}

	//For 64bit this wont read more than 10 bytes
	FORCE_INLINE size_t decode_length(const uint8_t*& compressed) {
		uint8_t byte = *compressed++;
		if (likely(byte <= 127))
			return byte;
		else {
			size_t length = byte & 0x7F;
			size_t iterations = 1;
			do {
				length <<= 7;
				byte = *compressed++;
				length |= byte & 0x7F;
				iterations++;
			} while (byte >= 128 && iterations <= (IS_64BIT ? 9 : 4));
			return length;
		}
	}

	int decompress(const uint8_t* compressed, size_t compressedSize, uint8_t* decompressed,
		size_t decompressedSize, ProgressCallback* progress) {

		ProgressCallback defaultProgress;
		if (!progress)
			progress = &defaultProgress;

		//Only last bytes are present
		if (decompressedSize <= SKANDA_LAST_BYTES) {
			if (compressedSize < decompressedSize)
				return -1;
			memcpy(decompressed, compressed, decompressedSize);
			progress->progress(decompressedSize, decompressedSize);
			return 0;
		}

		if (compressedSize < SKANDA_LAST_BYTES + 1)
			return -1;

		const uint8_t* const decompressedStart = decompressed;
		const uint8_t* const decompressedEnd = decompressed + decompressedSize - SKANDA_LAST_BYTES;
		const uint8_t* const compressedEnd = compressed + compressedSize - SKANDA_LAST_BYTES;
		const uint8_t* nextProgressReport = (decompressedEnd - decompressed < SKANDA_PROGRESS_REPORT_PERIOD) ?
			decompressedEnd : decompressed + SKANDA_PROGRESS_REPORT_PERIOD;

		size_t distance = 1;
		size_t length;  //Stores the literal run length and match length

		while (true) {

			const size_t token = *compressed++;
			length = token >> 5;

			if (length == 7) {

				length += decode_length(compressed);
				memcpy(decompressed, compressed, 16);
				memcpy(decompressed + 16, compressed + 16, 16);

				if (unlikely(length > 32)) {

					//We dont have the guarantee that compressedEnd >= compressed or decompressed < decompressedEnd
					if (unlikely(compressed > compressedEnd || decompressed > decompressedEnd ||
						compressedEnd - compressed < length || decompressedEnd - decompressed < length)) {
						return -1;
					}

					uint8_t* dst = decompressed + 32;
					const uint8_t* src = compressed + 32;
					uint8_t* const end = decompressed + length;
					do {
						memcpy(dst, src, 16);
						memcpy(dst + 16, src + 16, 16);
						src += 32;
						dst += 32;
					} while (dst < end);
				}
			}
			else {
				//If the cpu supports fast unaligned access, it is faster to copy a 
				// fixed amount of bytes instead of the needed amount
				memcpy(decompressed, compressed, 8);
			}
			decompressed += length;
			compressed += length;

			if (unlikely(decompressed >= nextProgressReport)) {
				if (progress->progress(decompressed - decompressedStart, compressed - (compressedEnd - compressedSize)))
					return 0;

				nextProgressReport = (decompressedEnd - decompressed < SKANDA_PROGRESS_REPORT_PERIOD) ?
					decompressedEnd : decompressed + SKANDA_PROGRESS_REPORT_PERIOD;

				if (decompressed >= decompressedEnd) {
					//The maximum number of bytes that can be written in a single iteration
					// (excluding long match lengths or long literal runs) 
					// is 32 from match and 32 from literal run.
					//There is just enough end buffer to not write out of bounds.
					if (decompressed > decompressedEnd || compressed > compressedEnd)
						return -1;
					break;
				}
			}

			//The maximum number of bytes that can be read in a single iteration
			// (excluding long literal runs) is 1 from token, 32 from literals,
			// 3 from distance, and 10 from match length. This is less than the end buffer,
			// so this one check is enough.
			if (unlikely(compressed > compressedEnd))
				return -1;

			if (HAS_BMI) {
#ifdef BMI_COMPILE
				const size_t distanceBytes = _bextr_u32(token, 3, 2);
				length = (token & 0x7) + SKANDA_MIN_MATCH_LENGTH;
				const size_t newDistance = _bzhi_u32(read_uint32le(compressed), distanceBytes * 8);
				distance = distanceBytes ? newDistance : distance;
				compressed += distanceBytes;
#endif
			}
			else {
				const size_t distanceBytes = (token >> 3) & 0x3;
				length = (token & 0x7) + SKANDA_MIN_MATCH_LENGTH;
				size_t newDistance = read_uint32le(compressed);
				newDistance &= ~(-1 << distanceBytes * 8);
				distance = distanceBytes ? newDistance : distance;
				compressed += distanceBytes;
			}

			if (unlikely(decompressed - decompressedStart < distance))
				return -1;

			if (length == 7 + SKANDA_MIN_MATCH_LENGTH) {
				length += decode_length(compressed);
				//We have the guarantee that decompressed < decompressedEnd, so there is no overflow
				if (unlikely(decompressedEnd - decompressed < length))
					return -1;
				copy_match(decompressed, distance, length);
			}
			else {
				const uint8_t* match = decompressed - distance;
				//If the distance is big enough we can perform a faster copy
				if (distance >= 8)
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
			}
			decompressed += length;
		}

		memcpy(decompressed, compressed, SKANDA_LAST_BYTES);
		progress->progress(decompressedSize, compressed - (compressedEnd - compressedSize));
		return 0;
	}
}

#endif  //SKANDA_IMPLEMENTATION

#endif  //__SKANDA__
