/*
 * Skanda Compression Algorithm v1.4.0
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
	// - progress(bytes): the algorithm will pass the number of bytes that have been compressed/decompressed
	// - abort(): if this returns true, the compression or decompression will stop with a return code of 0
	class ProgressCallback {
	public:
		virtual void progress(size_t bytes) {
			return;
		}
		virtual int abort() {
			return false;
		}
	};

	//Compresses "size" bytes of data present in "input", and stores it in "output".
	//"Level" specifies a tradeoff between compressed size and speed, and must be <= 9.
	//Negative levels specify an acceleration value similar to that of LZ4, up to -64.
	//You may pass a pointer to an object with base class ProgressCallback, to track progress.
	//window determines the maximum backwards distance the matches can have, as a power of 2.
	//Larger values can improve compression, but may hurt decode speed if they exceed CPU cache.
	//Returns the size of the compressed stream or -1 on failure.
	size_t compress(const uint8_t* input, size_t size, uint8_t* output, int level = 1,
		int window = 20, ProgressCallback* progress = nullptr);
	//Decompresses contents in "compressed" to "decompressed".
	//You may also pass a pointer to an object with base class ProgressCallback, to track progress.
	//Returns 0 on success or -1 on failure or corrupted data.
	int decompress(const uint8_t* compressed, size_t compressedSize, uint8_t* decompressed,
		size_t decompressedSize, ProgressCallback* progress = nullptr);

	//For a given input size, returns a size for the output buffer that is big enough to
	// contain the compressed stream even if it expands.
	size_t compress_bound(size_t size);
	//Returns the amount of memory the algorithm will consume on compression.
	size_t estimate_memory(size_t size, int level = 1, int window = 20);
}

#ifdef SKANDA_IMPLEMENTATION

#include <algorithm>
#include <cstring>

#if defined(_MSC_VER)
#define FORCE_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
#define FORCE_INLINE inline __attribute__((always_inline))
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

#define MIN3(a, b, c) (std::min(a, std::min(b, c)))

namespace skanda {

	bool is_little_endian() {
		const union { uint16_t u; uint8_t c[2]; } LITTLE_ENDIAN_CHECK = { 1 };
		return LITTLE_ENDIAN_CHECK.c[0];
	}

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

	//Tries to find a match between the two locations, and returns the length
	//Note that this function should be called with at least MIN_LENGTH + 8 bytes of buffer after limit
	FORCE_INLINE size_t test_match(const uint8_t* front, const uint8_t* back, const uint8_t* const limit,
		const size_t minLength, const int window) {

		//Test first bytes
		//Usually compilers will optimize std::equal as 2 comparisons for minLength 5, 6, etc
		//We can only use one comparison to make it faster. For powers of 2, std::equal should be good
		if (IS_64BIT && is_little_endian()) {
			switch (minLength) {
			case 5:
				if ((read_uint64le(front) << 24) != (read_uint64le(back) << 24) || ((size_t)(front - back) >> window))
					return 0;
				break;
			case 3:
				if ((read_uint32le(front) << 8) != (read_uint32le(back) << 8) || ((size_t)(front - back) >> window))
					return 0;
				break;
			default:
				if (!std::equal(back, back + minLength, front) || ((size_t)(front - back) >> window))
					return 0;
				break;
			}
		}
		else {
			if (!std::equal(back, back + minLength, front) || ((size_t)(front - back) >> window))
				return 0;
		}

		const uint8_t* const matchOrigin = front;
		front += minLength;
		back += minLength;

		if (IS_64BIT && is_little_endian()) {
			while (true) {
				if (unlikely(front + 8 > limit)) {
					if (front > limit)
						return 0;

					while (*front == *back && front < limit) {
						front++;
						back++;
					}
					return front - matchOrigin;
				}

				//Compare 8 bytes at a time using xor. It has the property of returning 0 if the two values are equal.
				//In case they differ, we can get the first byte that differs using a bit scan.
				const uint64_t xorVal = read_uint64le(front) ^ read_uint64le(back);

				if (xorVal) {
					front += unsafe_bit_scan_forward(xorVal) >> 3;
					return front - matchOrigin;
				}

				front += 8;
				back += 8;
			}
		}
		else {
			if (front > limit)
				return 0;
			while (*front == *back && front < limit) {
				front++;
				back++;
			}
			return front - matchOrigin;
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
		int parserFunction;
		int maxHashTableSize;
		int maxElementsPerHash;
		int niceLength;
		int optimalBlockSize;        // (Optimal)
		int maxArrivals;             // (Optimal)
	};

	template<class IntType>
	struct LZStructure {
		IntType matchLength;
		IntType matchDistance;
		IntType literalRunLength;
	};

	template<class IntType>
	struct LZMatch {
		IntType length;
		IntType distance;
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

	template<class IntType>
	class LZ2WayCacheBucket {
		IntType* data;
	public:
		LZ2WayCacheBucket() {}
		LZ2WayCacheBucket(IntType* _data) {
			data = _data;
		}
		//Loads the first value, and at the same time pushes a value into that slot.
		void first(size_t* value) {
			const IntType tmp = data[0];
			data[0] = *value;
			*value = tmp;
		}
		//Loads the second value, and at the same time pushes a value into that slot.
		//Should be used after loading the first value.
		void second(size_t* value) {
			const IntType tmp = data[1];
			data[1] = *value;
			*value = tmp;
		}
		//Inserts a value into the first slot.
		//Used when skipping bytes.
		void push_in_first(const size_t value) {
			const IntType tmp = data[0];
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
	template<class IntType, class Hash>
	class LZ2WayCacheTable {
		IntType* arr = nullptr;
		size_t hashShift;
	public:
		LZ2WayCacheTable() {}
		~LZ2WayCacheTable() {
			delete[] arr;
		}
		void init(const size_t logSize) {
			arr = new IntType[(size_t)2 << logSize]();
			hashShift = (IS_64BIT ? 64 : 32) - logSize;
		}
		LZ2WayCacheBucket<IntType> operator[](const size_t value) {
			return LZ2WayCacheBucket<IntType>(arr + (Hash{}(value) >> hashShift) * 2);
		}
	};

	//Used for easier implementation
	//Please do not use push() and next() on the same bucket
	template<class IntType>
	class LZCacheBucket {
		IntType* it;
		IntType* last;
	public:
		LZCacheBucket() {}
		LZCacheBucket(IntType* _begin, IntType* _end) {
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
			const IntType tmp = *it;
			*it = *value;
			*value = tmp;
			it++;
		}
		//Loads the next position, but without updating the bucket as we go
		IntType next() {
			return *it++;
		}
	};

	//This works like the basic hash table, except that it stores N positions per bucket
	template<class IntType, class Hash>
	class LZCacheTable {
		IntType* arr = nullptr;
		size_t hashShift;
		size_t elementsPerBucket;  //log2
	public:
		//Use 2^x sizes to avoid the use of modulo and multiplication
		LZCacheTable() {}
		LZCacheTable(const size_t logSize, const size_t numElements) {
			init(logSize, numElements);
		}
		~LZCacheTable() {
			delete[] arr;
		}
		void init(const size_t logSize, const size_t numElements) {
			arr = new IntType[(size_t)1 << logSize << numElements]();
			hashShift = (IS_64BIT ? 64 : 32) - logSize;
			elementsPerBucket = numElements;
		}
		LZCacheBucket<IntType> operator[](size_t value) {
			value = Hash{}(value) >> hashShift;
			IntType* bucket = arr + (value << elementsPerBucket);
			return LZCacheBucket<IntType>(bucket, bucket + ((size_t)1 << elementsPerBucket));
		}
	};

	//Simple and fast
	template<class IntType>
	class HashTableMatchFinder {
		HashTable<IntType, FastIntHash> lzdict3;
		LZCacheTable<IntType, FastIntHash> lzdict4;
		LZCacheTable<IntType, FastIntHash> lzdict8;

	public:
		void init(const size_t size, const CompressorOptions& compressorOptions, const int window) {
			const int hashSize = MIN3((int)int_log2(size) - 3, compressorOptions.maxHashTableSize, window - 3);
			lzdict3.init(std::min(hashSize, 14));
			lzdict4.init(hashSize, compressorOptions.maxElementsPerHash);
			lzdict8.init(hashSize, compressorOptions.maxElementsPerHash);
		}

		LZMatch<IntType>* find_matches_and_update(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const limit,
			LZMatch<IntType>* matches, size_t highestLength, const CompressorOptions& compressorOptions, const int window) {

			IntType& chain3 = lzdict3[read_hash3(input)];

			if (highestLength < 3) {
				const uint8_t* where = inputStart + chain3;
				size_t length = test_match(input, where, limit, 3, window);

				if (length > highestLength) {
					matches->distance = input - where;
					matches->length = length;
					matches++;

					highestLength = length;
					if (highestLength >= compressorOptions.niceLength) {
						chain3 = input - inputStart;
						return matches;
					}
				}
			}
			chain3 = input - inputStart;

			if (highestLength < 7) {
				LZCacheBucket<IntType> chain4 = lzdict4[read_hash4(input)];
				size_t pos = input - inputStart;
				while (!chain4.ended()) {
					chain4.next(&pos);

					const uint8_t* where = inputStart + pos;

					if (*(input + highestLength) != *(where + highestLength))
						continue;

					const size_t length = test_match(input, where, limit, 4, window);

					if (length > highestLength) {
						matches->distance = input - where;
						matches->length = length;
						matches++;

						highestLength = length;
						if (highestLength >= 7) {
							while (!chain4.ended())
								chain4.next(&pos);
							break;
						}
					}
				}
			}

			if (highestLength >= 4 && highestLength < compressorOptions.niceLength) {
				LZCacheBucket<IntType> chain8 = lzdict8[read_hash8(input)];
				size_t pos = input - inputStart;
				while (!chain8.ended()) {
					chain8.next(&pos);

					const uint8_t* where = inputStart + pos;
					if (*(input + highestLength) != *(where + highestLength))
						continue;

					const size_t length = test_match(input, where, limit, 8, window);

					if (length > highestLength) {
						matches->distance = input - where;
						matches->length = length;
						matches++;

						highestLength = length;
						if (highestLength >= compressorOptions.niceLength) {
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
	template<class IntType>
	class BinaryMatchFinder {

		HashTable<IntType, FastIntHash> lzdict16;
		HashTable<IntType, FastIntHash> lzdict3;
		HashTable<IntType, FastIntHash> nodeLookup;
		IntType* nodes = nullptr;
		size_t nodeListSize;
		size_t nodeListMask;

	public:

		~BinaryMatchFinder() {
			delete[] nodes;
		}
		BinaryMatchFinder() {}

		void init(const size_t inputSize, const CompressorOptions& compressorOptions, const int window) {

			const size_t binaryTreeWindow = std::min(compressorOptions.maxHashTableSize, window);

			//Input size is smaller than maximum binary tree size
			if (inputSize < ((size_t)1 << binaryTreeWindow)) {
				nodes = new IntType[(size_t)2 * inputSize];
				nodeListSize = inputSize;
				nodeListMask = -1;
			}
			else {
				nodes = new IntType[(size_t)2 << binaryTreeWindow];
				nodeListSize = (size_t)1 << binaryTreeWindow;
				nodeListMask = nodeListSize - 1;

				if (window > compressorOptions.maxHashTableSize)
					lzdict16.init(std::max(1, (int)int_log2(std::min(inputSize, (size_t)1 << window) - nodeListSize) - 3));
			}
			lzdict3.init(MIN3((int)int_log2(inputSize) - 3, 14, window - 3));
			nodeLookup.init(MIN3((int)int_log2(inputSize) - 3, 20, window - 3));
		}

		LZMatch<IntType>* find_matches_and_update(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const limit,
			LZMatch<IntType>* matches, size_t highestLength, const CompressorOptions& compressorOptions, const int window) {

			const size_t inputPosition = input - inputStart;

			// First try to get a length 3 match
			IntType& chain3 = lzdict3[read_hash3(input)];
			if (highestLength < 3) {
				const uint8_t* where = inputStart + chain3;
				const size_t length = test_match(input, where, limit, 3, window);

				if (length >= 3) {
					matches->distance = input - where;
					matches->length = length;
					highestLength = length;
					matches++;

					if (highestLength >= compressorOptions.niceLength) {
						update_position(input, inputStart, limit, compressorOptions, window);
						return matches;
					}
				}
			}
			chain3 = inputPosition;

			//If we reach this position stop the search
			const size_t btEnd = inputPosition < nodeListSize ? 0 : inputPosition - nodeListSize;

			IntType& lookupEntry = nodeLookup[read_hash4(input)];
			size_t backPosition = lookupEntry;
			lookupEntry = inputPosition;

			IntType* lesserNode = &nodes[2 * (lookupEntry & nodeListMask)];
			IntType* greaterNode = &nodes[2 * (lookupEntry & nodeListMask) + 1];
			const uint8_t* lesserFront = input;
			const uint8_t* greaterFront = input;

			size_t depth = (size_t)1 << compressorOptions.maxElementsPerHash;
			highestLength = std::max(highestLength, (size_t)3); //Avoid reading lengths < 4 in the binary tree
			//optimal brute reads lengths >= highest length, but we want 
			// lengths strictly longer than what highest length is right now
			highestLength += compressorOptions.parserFunction == OPTIMAL3;

			// Check matches
			while (true) {

				if (backPosition <= btEnd || depth-- == 0) {
					*lesserNode = NO_MATCH_POS;
					*greaterNode = NO_MATCH_POS;
					break;
				}

				const uint8_t* front = std::min(lesserFront, greaterFront);
				const uint8_t* back = front - (inputPosition - backPosition);

				const size_t extraLength = test_match(front, back, limit, 0, window);
				front += extraLength;
				back += extraLength;

				size_t length = front - input;
				IntType* const nextNode = &nodes[2 * (backPosition & nodeListMask)];
				if (length > highestLength || (compressorOptions.parserFunction == OPTIMAL3 && length >= highestLength)) {
					highestLength = length;
					matches->distance = front - back;
					matches->length = length;
					matches++;

					if (length >= compressorOptions.niceLength) {
						*lesserNode = nextNode[0];
						*greaterNode = nextNode[1];
						if (inputPosition > nodeListSize && ((size_t)1 << window) > nodeListSize)
							lzdict16[read_hash16(input - nodeListSize)] = (inputPosition - nodeListSize);
						return matches;
					}
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

			if (inputPosition > nodeListSize && ((size_t)1 << window) > nodeListSize) {

				const uint8_t* const where = inputStart + lzdict16[read_hash16(input)];
				const size_t length = test_match(input, where, limit, 16, window);

				if (length > highestLength) {
					matches->distance = input - where;
					matches->length = length;
					matches++;
				}

				lzdict16[read_hash16(input - nodeListSize)] = (inputPosition - nodeListSize);
			}

			return matches;
		}

		void update_position(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const limit,
			const CompressorOptions& compressorOptions, const int window) {

			const size_t inputPosition = input - inputStart;
			lzdict3[read_hash3(input)] = inputPosition;
			if (inputPosition > nodeListSize && ((size_t)1 << window) > nodeListSize)
				lzdict16[read_hash16(input - nodeListSize)] = (inputPosition - nodeListSize);

			//If we reach this position on the front stop the update
			const uint8_t* positionSkip = std::min(limit, input + compressorOptions.niceLength);
			//If we reach this position on the back stop the update
			const size_t btEnd = inputPosition < nodeListSize ? 0 : inputPosition - nodeListSize;
			IntType& lookupEntry = nodeLookup[read_hash4(input)];
			size_t backPosition = lookupEntry;
			lookupEntry = inputPosition;

			IntType* lesserNode = &nodes[2 * (inputPosition & nodeListMask)];
			IntType* greaterNode = &nodes[2 * (inputPosition & nodeListMask) + 1];

			const uint8_t* lesserFront = input;
			const uint8_t* greaterFront = input;
			size_t depth = (size_t)1 << compressorOptions.maxElementsPerHash;

			// Check matches
			while (true) {
				if (backPosition <= btEnd || depth-- == 0) {
					*lesserNode = NO_MATCH_POS;
					*greaterNode = NO_MATCH_POS;
					return;
				}

				const uint8_t* front = std::min(lesserFront, greaterFront);
				const uint8_t* back = front - (inputPosition - backPosition);

				const size_t length = test_match(front, back, positionSkip, 0, window);
				front += length;
				back += length;

				IntType* const nextNode = &nodes[2 * (backPosition & nodeListMask)];
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

	const int SKANDA_NORMAL_MIN_MATCH_LENGTH = 3;
	const int SKANDA_REP_MIN_MATCH_LENGTH = 2;
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
	FORCE_INLINE void encode_rep_match(uint8_t*& output, uint8_t* const controlByte, size_t matchLength) {

		matchLength -= SKANDA_REP_MIN_MATCH_LENGTH;
		if (unlikely(matchLength >= 15)) {
			*controlByte |= 31;
			encode_length(output, matchLength, 15);
		}
		else
			*controlByte |= 16 | matchLength;
	}

	//Only encodes matches with no rep offset
	FORCE_INLINE void encode_normal_match(uint8_t*& output, uint8_t* const controlByte,
		size_t matchLength, size_t distance, size_t* repOffset) {

		matchLength -= SKANDA_NORMAL_MIN_MATCH_LENGTH;
		*repOffset = distance;

		size_t bytes = (unsafe_int_log2(distance) + 2) / 8;
		write_uint32le(output, (distance << 2) | bytes);
		output += bytes + 1;

		const size_t lengthOverflow = (*controlByte) ? 15 : 31;
		if (unlikely(matchLength >= lengthOverflow)) {
			*controlByte |= lengthOverflow;
			encode_length(output, matchLength, lengthOverflow);
		}
		else
			*controlByte |= matchLength;
	}

	//Selects between encoding a normal or a rep match
	FORCE_INLINE void encode_match(uint8_t*& output, uint8_t* const controlByte,
		size_t matchLength, size_t distance, size_t* repOffset) {

		if (*repOffset == distance)
			encode_rep_match(output, controlByte, matchLength);
		else
			encode_normal_match(output, controlByte, matchLength, distance, repOffset);
	}

	template<class IntType>
	size_t compress_accelerated(const uint8_t* input, const size_t size, uint8_t* output,
		ProgressCallback* progress, const int window, size_t accelerationBase) {

		//Constants for this encoder
		const size_t hashTableSize = 12;
		const size_t accelerationThreshold = 3;
		const size_t accelerationMax = 64;

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
		HashTable<IntType, FastIntHash> lzdict;
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
				IntType* const dictEntry = &lzdict[read_hash8(input)];
				const uint8_t* match = inputStart + *dictEntry;
				*dictEntry = input - inputStart;
				//Try to find a match
				size_t matchLength = test_match(input, match, compressionLimit, 8, window);

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
					encode_normal_match(output, controlByte, matchLength, distance, &repOffset);

					while (true) {

						//Try to find first matching byte for rep offset
						size_t gap = int_log2(read_uint32le(input) ^ read_uint32le(input - repOffset)) / 8 + 1;
						matchLength = test_match(input + gap, input + gap - repOffset, compressionLimit, 3, window);
						if (matchLength == 0)
							break;

						input += gap;
						uint8_t* const controlByte = output++;
						encode_literal_run(input, literalRunStart, controlByte, output);
						//Add the position of the match to the hash table
						lzdict[read_hash8(input)] = input - inputStart;

						input += matchLength;
						literalRunStart = input;
						encode_rep_match(output, controlByte, matchLength);
					}

					acceleration = accelerationBase << accelerationThreshold;
				}
				//If there isnt advance the position
				else {
					input += acceleration >> accelerationThreshold;
					acceleration += acceleration < (accelerationMax << accelerationThreshold);
				}
			}

			if (progress->abort())
				return 0;
			progress->progress(input - inputStart);
		}

		//Due to the acceleration we might have gone beyond the end
		if (input > compressionLimit)
			input = compressionLimit;
		//Output the last literal run. This is necessary even when no more literals remain, as the decoder expects to end with this
		uint8_t* const controlByte = output++;
		encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		progress->progress(input - inputStart + SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	template<class IntType>
	size_t compress_ultra_fast(const uint8_t* input, const size_t size, uint8_t* output,
		ProgressCallback* progress, const int window) {

		//Constants for this encoder
		const size_t hashTableSize = 12;
		const size_t accelerationThreshold = 3;
		const size_t accelerationMax = 64;

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
		HashTable<IntType, FastIntHash> lzdict;
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
				size_t matchLength = test_match(input + 1, input + 1 - repOffset, compressionLimit, 3, window);
				if (matchLength) {

					input++;
					uint8_t* const controlByte = output++;
					encode_literal_run(input, literalRunStart, controlByte, output);
					//Add the position of the match to the hash table
					lzdict[read_hash5(input)] = input - inputStart;

					input += matchLength;
					literalRunStart = input;
					encode_rep_match(output, controlByte, matchLength);
					acceleration = 1 << accelerationThreshold;
					continue;
				}

				//Get possible match location and update the table
				IntType* const dictEntry = &lzdict[read_hash5(input)];
				const uint8_t* match = inputStart + *dictEntry;
				*dictEntry = input - inputStart;
				//Try to find a match
				matchLength = test_match(input, match, compressionLimit, 5, window);

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
					encode_normal_match(output, controlByte, matchLength, distance, &repOffset);

					acceleration = 1 << accelerationThreshold;
				}
				//If there isnt advance the position
				else {
					input += acceleration >> accelerationThreshold;
					acceleration += acceleration < (accelerationMax << accelerationThreshold);
				}
			}

			if (progress->abort())
				return 0;
			progress->progress(input - inputStart);
		}

		//Due to the acceleration we might have gone beyond the end
		if (input > compressionLimit)
			input = compressionLimit;
		//Output the last literal run. This is necessary even when no more literals remain, as the decoder expects to end with this
		uint8_t* const controlByte = output++;
		encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		progress->progress(input - inputStart + SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	template<class IntType>
	size_t compress_greedy(const uint8_t* input, const size_t size, uint8_t* output,
		ProgressCallback* progress, const int window, const CompressorOptions compressorOptions) {

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;

		size_t repOffset = 1;
		const uint8_t* literalRunStart = input;
		input++;

		const size_t hashSize = MIN3((int)int_log2(size) - 3, compressorOptions.maxHashTableSize, window - 3);
		HashTable<IntType, FastIntHash> lzdict;
		try {
			lzdict.init(hashSize);
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		while (input < compressionLimit) {

			const uint8_t* const nextProgressReport = (compressionLimit - input < SKANDA_PROGRESS_REPORT_PERIOD) ?
				compressionLimit : input + SKANDA_PROGRESS_REPORT_PERIOD;

			while (input < nextProgressReport) {

				//First try a rep match
				size_t matchLength = test_match(input + 1, input + 1 - repOffset, compressionLimit, 3, window);

				if (matchLength) {

					input++;
					uint8_t* const controlByte = output++;
					encode_literal_run(input, literalRunStart, controlByte, output);

					lzdict[read_hash5(input)] = input - inputStart;

					input += matchLength;
					literalRunStart = input;
					encode_match(output, controlByte, matchLength, repOffset, &repOffset);
					continue;
				}

				//If no rep, try a normal match
				IntType* dictEntry = &lzdict[read_hash5(input)];
				const uint8_t* match = inputStart + *dictEntry;
				*dictEntry = input - inputStart;
				matchLength = test_match(input, match, compressionLimit, 5, window);

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
					encode_match(output, controlByte, matchLength, distance, &repOffset);
					continue;
				}
				else {
					input++;
				}
			}

			if (progress->abort())
				return 0;
			progress->progress(input - inputStart);
		}

		uint8_t* const controlByte = output++;
		encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		progress->progress(input - inputStart + SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	//Try to find the longest match for a given position. Then try to find an even longer one in the next.
	//If it is found, code a literal, and the longer one found. If not, code the original.
	template<class IntType>
	FORCE_INLINE void fast_lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit,
		LZ2WayCacheTable<IntType, FastIntHash>* lzdict, size_t* bestMatchLength, size_t* bestMatchDistance,
		int* lazySteps, int* testedPositions, const int window, const CompressorOptions& compressorOptions) {

		*testedPositions = 1;
		LZ2WayCacheBucket<IntType> dictEntry = (*lzdict)[read_hash5(input)];

		//Test first entry
		size_t pos = input - inputStart;
		dictEntry.first(&pos);
		const uint8_t* where = inputStart + pos;
		*bestMatchLength = test_match(input, where, limit, 5, window);
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
			const size_t length = test_match(input, where, limit, 5, window);
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
			const size_t length = test_match(input, where, limit, 6, window);
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
			const size_t length = test_match(input, where, limit, 6, window);
			if (length > *bestMatchLength) {
				*bestMatchDistance = input - where;
				*bestMatchLength = length;
				*lazySteps = 1;
			}
		}
	}

	template<class IntType>
	size_t compress_lazy_fast(const uint8_t* input, const size_t size, uint8_t* output,
		ProgressCallback* progress, const int window, const CompressorOptions& compressorOptions) {

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;

		size_t repOffset = 1;
		const uint8_t* literalRunStart = input;
		input++;

		const size_t hashSize = MIN3((int)int_log2(size) - 3, compressorOptions.maxHashTableSize, window - 3);
		LZ2WayCacheTable<IntType, FastIntHash> lzdict;
		try {
			lzdict.init(hashSize);
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		while (input < compressionLimit) {

			const uint8_t* const nextProgressReport = (compressionLimit - input < SKANDA_PROGRESS_REPORT_PERIOD) ?
				compressionLimit : input + SKANDA_PROGRESS_REPORT_PERIOD;

			while (input < nextProgressReport) {

				//First try a rep match
				size_t matchLength = test_match(input + 1, input + 1 - repOffset, compressionLimit, 3, window);
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
					encode_rep_match(output, controlByte, matchLength);
					continue;
				}

				size_t distance;
				int lazySteps;   //bytes to skip because of lazy matching
				int testedPositions;   //number of positions that have been added to hash table

				fast_lazy_search<IntType>(input, inputStart, compressionLimit, &lzdict,
					&matchLength, &distance, &lazySteps, &testedPositions, window, compressorOptions);

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
					encode_normal_match(output, controlByte, matchLength, distance, &repOffset);
				}
			}

			if (progress->abort())
				return 0;
			progress->progress(input - inputStart);
		}

		uint8_t* const controlByte = output++;
		encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		progress->progress(input - inputStart + SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	//Same principle as the last function, but with additional heuristics to improve ratio
	template<class IntType>
	FORCE_INLINE void lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit,
		LZCacheTable<IntType, FastIntHash>* lzdict4, LZCacheTable<IntType, FastIntHash>* lzdict8,
		size_t* bestLength, size_t* bestDistance, int* lazySteps, int* testedPositions,
		const CompressorOptions& compressorOptions, const int window) {

		LZCacheBucket<IntType> chain4 = (*lzdict4)[read_hash4(input)];
		LZCacheBucket<IntType> chain8 = (*lzdict8)[read_hash8(input)];
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

			length = test_match(input, where, limit, 4, window);

			size_t distance = input - where;
			size_t matchCost = 2 + (unsafe_int_log2(distance) + 2) / 8;
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

				length = test_match(input, where, limit, 8, window);

				size_t distance = input - where;
				size_t matchCost = 2 + (unsafe_int_log2(distance) + 2) / 8;
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

			length = test_match(input, where, limit, 8, window);

			size_t distance = input - where;
			size_t matchCost = 2 + (unsafe_int_log2(distance) + 2) / 8;
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

			length = test_match(input, where, limit, 8, window);

			size_t distance = input - where;
			size_t matchCost = 2 + (unsafe_int_log2(distance) + 2) / 8;
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

	template<class IntType>
	size_t compress_lazy(const uint8_t* input, const size_t size, uint8_t* output,
		ProgressCallback* progress, const int window, const CompressorOptions& compressorOptions) {

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;

		size_t repOffset = 1;
		const uint8_t* literalRunStart = input;
		input++;

		const int hashSize = MIN3((int)int_log2(size) - 3, compressorOptions.maxHashTableSize, window - 3);
		LZCacheTable<IntType, FastIntHash> lzdict4;
		LZCacheTable<IntType, FastIntHash> lzdict8;

		try {
			lzdict4.init(hashSize, compressorOptions.maxElementsPerHash);
			lzdict8.init(hashSize, compressorOptions.maxElementsPerHash);
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		while (input < compressionLimit) {

			const uint8_t* const nextProgressReport = (compressionLimit - input < SKANDA_PROGRESS_REPORT_PERIOD) ?
				compressionLimit : input + SKANDA_PROGRESS_REPORT_PERIOD;

			while (input < nextProgressReport) {

				//First try a rep match
				size_t matchLength = test_match(input + 1, input + 1 - repOffset, compressionLimit, 3, window);
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
					encode_rep_match(output, controlByte, matchLength);
					continue;
				}

				size_t distance;
				int lazySteps;
				int testedPositions;

				lazy_search(input, inputStart, compressionLimit, &lzdict4, &lzdict8, &matchLength,
					&distance, &lazySteps, &testedPositions, compressorOptions, window);

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
					encode_match(output, controlByte, matchLength, distance, &repOffset);
				}
			}

			if (progress->abort())
				return 0;
			progress->progress(input - inputStart);
		}

		uint8_t* const controlByte = output++;
		encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		progress->progress(input - inputStart + SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	struct SkandaOptimalParserState {
		//16 high bits store size cost, 16 low bits the speed cost
		//This way we can also compare both costs prioritising size cost with a single operation
		uint32_t cost;
		//Serves as both match distance and rep offset.
		uint32_t distance;
		uint32_t literalRunLength;
		uint16_t matchLength;
		uint16_t path;
	};

	template<class IntType>
	LZStructure<IntType>* forward_optimal_parse(const uint8_t* const input, const uint8_t* const inputStart,
		const uint8_t* const limit, HashTableMatchFinder<IntType>* matchFinder, SkandaOptimalParserState* parser,
		LZStructure<IntType>* stream, const size_t startRepOffset, const CompressorOptions& compressorOptions, const int window) {

		const size_t blockLength = std::min((size_t)(limit - input), (size_t)compressorOptions.optimalBlockSize);
		for (size_t i = 1; i < blockLength + compressorOptions.niceLength + 1; i++)
			parser[i].cost = UINT32_MAX;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;

		parser[0].cost = 0;
		parser[0].distance = startRepOffset;
		parser[0].literalRunLength = 0;

		size_t position = 0;
		for (; position < blockLength; position++) {

			const uint8_t* inputPosition = input + position;
			SkandaOptimalParserState* const parserPosition = parser + position;

			//From Zstd: skip unpromising position
			if (parserPosition[1].cost <= parserPosition[0].cost) {
				matchFinder->update_position(inputPosition, inputStart);
				continue;
			}

			const size_t literalCost = parserPosition->cost + (0x10000 << (parserPosition->literalRunLength == 6));  //only size cost
			SkandaOptimalParserState* nextPosition = parserPosition + 1;
			if (literalCost < nextPosition->cost) {
				nextPosition->cost = literalCost;
				nextPosition->distance = parserPosition->distance; //Carry forward the rep offset
				nextPosition->literalRunLength = parserPosition->literalRunLength + 1;
			}

			size_t repMatchLength = 0;
			if (parserPosition->literalRunLength) {
				repMatchLength = test_match(inputPosition, inputPosition - parserPosition->distance, limit, 2, window);

				if (repMatchLength) {
					//Rep matches can be unconditionally taken with lower lengths
					if (repMatchLength >= compressorOptions.niceLength / 2) {
						lastMatchLength = repMatchLength;
						lastMatchDistance = parserPosition->distance;
						break;
					}

					size_t repMatchCost = parserPosition->cost;
					repMatchCost += 0x10000 << (repMatchLength > 16);  //size cost
					repMatchCost += 1;  //speed cost

					nextPosition = parserPosition + repMatchLength;
					if (repMatchCost < nextPosition->cost) {
						nextPosition->cost = repMatchCost;
						nextPosition->matchLength = repMatchLength;
						nextPosition->distance = parserPosition->distance;
						nextPosition->literalRunLength = 0;
					}
				}
			}

			LZMatch<IntType> matches[17];
			const LZMatch<IntType>* matchesEnd = matchFinder->find_matches_and_update(inputPosition, inputStart,
				limit, matches, repMatchLength + 1, compressorOptions, window);

			//At least one match was found
			if (matchesEnd != matches) {
				//The last match should be the longest
				const LZMatch<IntType>* const longestMatch = matchesEnd - 1;
				if (longestMatch->length >= compressorOptions.niceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					break;
				}
				//For left match extension: do not go beyond the beginning of this parser cycle
				const uint8_t* const literalRunStart = inputPosition - std::min((size_t)parserPosition->literalRunLength, position);
				const size_t lengthOverflow = parserPosition->literalRunLength ? 17 : 33;
				for (const LZMatch<IntType>* matchIt = longestMatch; matchIt >= matches; matchIt--) {

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
					matchCost += (2 + (unsafe_int_log2(matchIt->distance) + 2) / 8 + (matchLength > lengthOverflow)) << 16; //size cost
					matchCost += 1 + (matchIt->distance > (1 << 20)); //speed cost

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

	template<class IntType>
	LZStructure<IntType>* multi_arrivals_parse(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit,
		BinaryMatchFinder<IntType>* matchFinder, SkandaOptimalParserState* parser, LZStructure<IntType>* stream,
		const size_t startRepOffset, const CompressorOptions& compressorOptions, const int window) {

		const size_t blockLength = std::min((size_t)(limit - input), (size_t)compressorOptions.optimalBlockSize - 1);
		for (size_t i = 0; i < (blockLength + compressorOptions.niceLength + 1) * compressorOptions.maxArrivals; i++)
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
				matchFinder->update_position(inputPosition, inputStart, limit, compressorOptions, window);
				continue;
			}

			size_t acceptableRepMatchLength = 2;  //Only take rep matches as long as this

			//Literal and rep match parsing can be done at the same time
			for (size_t i = 0; i < compressorOptions.maxArrivals; i++) {

				SkandaOptimalParserState* const currentArrival = parserPosition + i;

				//There might be less than max arrivals for this position. Note that there will always be at least one
				if (currentArrival->cost == UINT32_MAX)
					break;

				const size_t literalCost = currentArrival->cost + (0x10000 << (currentArrival->literalRunLength == 6)); //only size cost
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

				if (currentArrival->literalRunLength > 0) {
					size_t repMatchLength = test_match(inputPosition, inputPosition - currentArrival->distance, limit, 2, window);
					//Since we go from lowest cost arrival to highest, it makes sense that the rep match
					// should be at least as long as the best found so far
					if (repMatchLength >= acceptableRepMatchLength) {
						//Rep matches can be unconditionally taken with lower lengths
						if (repMatchLength >= compressorOptions.niceLength / 2) {
							lastMatchLength = repMatchLength;
							lastMatchDistance = currentArrival->distance;
							lastMatchPath = i;
							goto doBackwardParse;
						}

						acceptableRepMatchLength = repMatchLength;
						//Small heuristic: instead of testing all positions, only test the maximum match length,
						// and if it overflows, just before the overflow
						//There is a notable speed increase for a negligible size penalty
						size_t repMatchCost = currentArrival->cost;
						repMatchCost += 0x10000 << (repMatchLength > 16);  //size cost
						repMatchCost += 1;  //speed cost

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

								//Only try a length of 16 if the full length actually resulted in a better arrival
								if (repMatchLength > 16) {

									repMatchCost -= 0x10000;  //Remove the cost of the additional length byte

									arrivalIt = parserPosition + 16 * compressorOptions.maxArrivals;
									lastArrival = arrivalIt + compressorOptions.maxArrivals;

									for (; arrivalIt < lastArrival; arrivalIt++) {

										if (repMatchCost < arrivalIt->cost) {

											for (SkandaOptimalParserState* it = lastArrival - 1; it != arrivalIt; it--)
												memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState));

											arrivalIt->cost = repMatchCost;
											arrivalIt->matchLength = 16;
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
			}

			LZMatch<IntType> matches[130];
			LZMatch<IntType>* matchesEnd = matchFinder->find_matches_and_update(inputPosition, inputStart, limit, matches,
				std::max(acceptableRepMatchLength - 1, (size_t)2), compressorOptions, window);

			if (matchesEnd != matches) {

				//We have the guarantee that matches are in increasing order
				const LZMatch<IntType>* const longestMatch = matchesEnd - 1;
				if (longestMatch->length >= compressorOptions.niceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					lastMatchPath = 0;
					break;
				}

				bool lengthOverflowTested = false;
				const size_t lengthOverflow = parserPosition->literalRunLength ? 17 : 33;

				for (LZMatch<IntType>* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					if (matchIt->distance == parserPosition->distance)
						continue;

					size_t matchCost = parserPosition->cost;
					matchCost += (2 + (unsafe_int_log2(matchIt->distance) + 2) / 8 + (matchIt->length > lengthOverflow)) << 16;  //size cost
					matchCost += 1 + (matchIt->distance > (1 << 20)) * 2;  //speed cost

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
								matchCost -= 0x10000; //Remove length overflow cost
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
				matchFinder->update_position(inputPosition, inputStart, limit, compressorOptions, window);
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

	template<class IntType>
	size_t compress_optimal(const uint8_t* input, const size_t size, uint8_t* output,
		ProgressCallback* progress, const int window, const CompressorOptions& compressorOptions) {

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;

		//Data for the format
		size_t repOffset = 1;
		//Skip first byte
		const uint8_t* literalRunStart = input;
		input++;

		HashTableMatchFinder<IntType> hashMatchFinder;
		BinaryMatchFinder<IntType> binaryMatchFinder;
		SkandaOptimalParserState* parser = nullptr;
		LZStructure<IntType>* stream = nullptr;

		try {
			if (compressorOptions.parserFunction == OPTIMAL1) {
				hashMatchFinder.init(size, compressorOptions, window);
				parser = new SkandaOptimalParserState[compressorOptions.optimalBlockSize + compressorOptions.niceLength + 1];
			}
			else {
				binaryMatchFinder.init(size, compressorOptions, window);
				parser = new SkandaOptimalParserState[(compressorOptions.optimalBlockSize + compressorOptions.niceLength + 1) * compressorOptions.maxArrivals];
			}
			stream = new LZStructure<IntType>[compressorOptions.optimalBlockSize];
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

				LZStructure<IntType>* streamIt;
				if (compressorOptions.parserFunction >= OPTIMAL2) {
					streamIt = multi_arrivals_parse(input, inputStart, compressionLimit, &binaryMatchFinder,
						parser, stream, repOffset, compressorOptions, window);
				}
				else {
					streamIt = forward_optimal_parse(input, inputStart, compressionLimit, &hashMatchFinder,
						parser, stream, repOffset, compressorOptions, window);
				}

				//Main compression loop
				while (true) {
					input += streamIt->literalRunLength;

					if (streamIt == stream)
						break;

					size_t matchLength = streamIt->matchLength;
					size_t distance = streamIt->matchDistance;

					//Perform left match expansion. This helps even with the binary tree match finder.
					//Reps should not need left extension. This also avoids having a rep match 
					// after a literal run length of 0, which would make the match encoder 
					// output invalid data.
					if (distance != repOffset) {
						const uint8_t* match = input - distance;
						while (input > literalRunStart && match > inputStart && input[-1] == match[-1]) {
							matchLength++;
							input--;
							match--;
						}
					}

					uint8_t* const controlByte = output++;
					encode_literal_run(input, literalRunStart, controlByte, output);

					input += matchLength;
					encode_match(output, controlByte, matchLength, distance, &repOffset);
					literalRunStart = input;

					streamIt--;
				}
			}

			if (progress->abort()) {
				delete[] parser;
				delete[] stream;
				return 0;
			}
			progress->progress(input - inputStart);
		}

		delete[] parser;
		delete[] stream;

		//Output the last literal run. This is necessary even when no more literals remain, as the decoder expects to end with this
		uint8_t* const controlByte = output++;
		encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		progress->progress(input - inputStart + SKANDA_LAST_BYTES);

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
			{ OPTIMAL1     ,     19     ,          3          ,       48      ,      1024       ,   NOT_USED   },
			{ OPTIMAL2     ,     22     ,          4          ,       48      ,      4096       ,       2      },
			{ OPTIMAL2     ,     24     ,          5          ,       80      ,      4096       ,       4      },
			{ OPTIMAL3     ,     26     ,          6          ,       256     ,      4096       ,       6      },
			{ OPTIMAL3     ,     30     ,          7          ,       1024    ,      4096       ,       16     },
	};

	size_t compress(const uint8_t* input, size_t size, uint8_t* output,
		int level, int window, ProgressCallback* progress) {

		ProgressCallback defaultProgress;
		if (!progress)
			progress = &defaultProgress;

		if (size <= SKANDA_LAST_BYTES) {
			memcpy(output, input, size);
			progress->progress(size);
			return size;
		}

		if (level < -64)
			level = -64;
		if (level > 10)
			level = 10;
		if (window > 30)
			window = 30;
		if (window < 6)
			window = 6;

#if defined(IS_64BIT) && defined(SKANDA_LARGE_INPUT_SUPPORT)
		if (size > ((uint64_t)1 << 32)) {
			if (level < 0)
				return compress_accelerated<uint64_t>(input, size, output, progress, window, -level);
			if (skandaCompressorLevels[level].parserFunction == GREEDY1)
				return compress_ultra_fast<uint64_t>(input, size, output, progress, window);
			if (skandaCompressorLevels[level].parserFunction == GREEDY2)
				return compress_greedy<uint64_t>(input, size, output, progress, window, skandaCompressorLevels[level]);
			if (skandaCompressorLevels[level].parserFunction == LAZY1)
				return compress_lazy_fast<uint64_t>(input, size, output, progress, window, skandaCompressorLevels[level]);
			if (skandaCompressorLevels[level].parserFunction == LAZY2)
				return compress_lazy<uint64_t>(input, size, output, progress, window, skandaCompressorLevels[level]);
			return compress_optimal<uint64_t>(input, size, output, progress, window, skandaCompressorLevels[level]);  //optimal levels
		}
#endif
		if (level < 0)
			return compress_accelerated<uint32_t>(input, size, output, progress, window, -level);
		if (skandaCompressorLevels[level].parserFunction == GREEDY1)
			return compress_ultra_fast<uint32_t>(input, size, output, progress, window);
		if (skandaCompressorLevels[level].parserFunction == GREEDY2)
			return compress_greedy<uint32_t>(input, size, output, progress, window, skandaCompressorLevels[level]);
		if (skandaCompressorLevels[level].parserFunction == LAZY1)
			return compress_lazy_fast<uint32_t>(input, size, output, progress, window, skandaCompressorLevels[level]);
		if (skandaCompressorLevels[level].parserFunction == LAZY2)
			return compress_lazy<uint32_t>(input, size, output, progress, window, skandaCompressorLevels[level]);
		return compress_optimal<uint32_t>(input, size, output, progress, window, skandaCompressorLevels[level]);  //optimal levels
	}

	size_t compress_bound(size_t size) {
		return size + size / 512 + 8;
	}

	template<class IntType>
	size_t memory_estimator(size_t size, int window, int level) {
		if (level < 0)
			return sizeof(IntType) << 12;
		if (skandaCompressorLevels[level].parserFunction == GREEDY1)
			return sizeof(IntType) << 12;
		if (skandaCompressorLevels[level].parserFunction == GREEDY2)
			return sizeof(IntType) << MIN3((int)int_log2(size) - 3, skandaCompressorLevels[level].maxHashTableSize, window - 3);
		if (skandaCompressorLevels[level].parserFunction == LAZY1)
			return sizeof(IntType) << MIN3((int)int_log2(size) - 3, skandaCompressorLevels[level].maxHashTableSize, window - 3) << 1;
		if (skandaCompressorLevels[level].parserFunction == LAZY2)
			//Lazy extra uses 2 tables
			return sizeof(IntType) << MIN3((int)int_log2(size) - 3, skandaCompressorLevels[level].maxHashTableSize, window - 3)
			<< skandaCompressorLevels[level].maxElementsPerHash << 1;
		if (skandaCompressorLevels[level].parserFunction == OPTIMAL1) {
			const int log2size = MIN3((int)int_log2(size) - 3, skandaCompressorLevels[level].maxHashTableSize, window - 3);
			size_t memory = sizeof(IntType) << std::min(log2size, 14);  //hash 3 table
			memory += sizeof(IntType) << log2size << skandaCompressorLevels[level].maxElementsPerHash << 1;  //hash 4 and hash 8 tables
			memory += sizeof(SkandaOptimalParserState) * (skandaCompressorLevels[level].optimalBlockSize + skandaCompressorLevels[level].niceLength + 1);
			memory += sizeof(LZStructure<IntType>) * skandaCompressorLevels[level].optimalBlockSize;
			return memory;
		}
		size_t memory = sizeof(IntType) << MIN3((int)int_log2(size) - 3, 20, window - 3);  //binary node lookup
		memory += sizeof(IntType) << MIN3((int)int_log2(size) - 3, 14, window - 3);  //hash 3 table
		const size_t binaryTreeWindow = (size_t)1 << std::min(skandaCompressorLevels[level].maxHashTableSize, window);
		if (size < binaryTreeWindow)
			memory += sizeof(IntType) * 2 * size;  //binary tree
		else {
			memory += sizeof(IntType) * 2 * binaryTreeWindow;  //binary tree
			if (window > skandaCompressorLevels[level].maxHashTableSize)
				memory += sizeof(IntType) << std::max(1, (int)int_log2(std::min(size, (size_t)1 << window) - binaryTreeWindow) - 3);  //hash 16 table
		}
		memory += sizeof(SkandaOptimalParserState) * (skandaCompressorLevels[level].optimalBlockSize + skandaCompressorLevels[level].niceLength + 1) * skandaCompressorLevels[level].maxArrivals;
		memory += sizeof(LZStructure<IntType>) * skandaCompressorLevels[level].optimalBlockSize;
		return memory;
	}

	size_t estimate_memory(size_t size, int level, int window) {

		if (level < 0)
			level = 0;
		if (level > 10)
			level = 10;
		if (window > 30)
			window = 30;
		if (window < 6)
			window = 6;

		if (size <= SKANDA_LAST_BYTES + 1)
			return 0;

#ifdef IS_64BIT
		if (size > ((uint64_t)1 << 32))
			return memory_estimator<uint64_t>(size, window, level);
#endif
		return memory_estimator<uint32_t>(size, window, level);
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
			progress->progress(decompressedSize);
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

			const uint8_t token = *compressed++;
			length = token >> 5;

			if (length == 7) {

				length += decode_length(compressed);

				//The maximum number of bytes we could have written without checks for decompression buffer
				// overflow is 33 (from match length). At the same time, we could have read
				// 4 (distance) + 11 (match length) + 1 (token) + 11 (literal run length) = 27 bytes

				memcpy(decompressed, compressed, 16);
				memcpy(decompressed + 16, compressed + 16, 16);

				if (unlikely(length > 32)) {

					//We dont have the guarantee that compressedEnd >= compressed
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
				//If the cpu supports fast unaligned access, it is faster to copy a fixed amount of bytes instead of the needed amount
				memcpy(decompressed, compressed, 8);
			}
			decompressed += length;
			compressed += length;

			if (unlikely(decompressed >= nextProgressReport)) {
				if (progress->abort())
					return 0;
				progress->progress(decompressed - decompressedStart);

				nextProgressReport = (decompressedEnd - decompressed < SKANDA_PROGRESS_REPORT_PERIOD) ?
					decompressedEnd : decompressed + SKANDA_PROGRESS_REPORT_PERIOD;

				if (decompressed >= decompressedEnd) {
					//The maximum number of bytes that can be written in a single iteration
					//(assuming no overflows) is 33 from match and 8(because we write more than needed) from literal run.
					//As it is less than the end buffer, this check should be enough.
					if (decompressed > decompressedEnd || compressed > compressedEnd)
						return -1;
					break;
				}
			}

			//The maximum number of bytes that can be advanced with a match is 4(distance) + 11(length)
			//For a literal run, 1 (token) + 8(literals)(I put 8 because we always read more than needed,
			// and this is the last read). That makes a total of 30, which is less than the
			//end buffer, and so only a check is necessary here. The exception is an overflow in the
			//literal run, so that needs its own check.
			if (unlikely(compressed > compressedEnd))
				return -1;

			//Reuse the last offset
			if ((token & 0x10) && length) {
				length = (token & 0xF) + SKANDA_REP_MIN_MATCH_LENGTH;

				if (length == 15 + SKANDA_REP_MIN_MATCH_LENGTH) {
					length += decode_length(compressed);
					//We have the guarantee that decompressed < decompressedEnd, so there is no overflow
					if (unlikely(decompressedEnd - decompressed < length))
						return -1;

					copy_match(decompressed, distance, length);
				}
				else {
					//Match length is at most 16 bytes
					const uint8_t* src = decompressed - distance;

					//If the offset is big enough we can perform a faster copy
					if (distance >= 8) {

						memcpy(decompressed, src, 8);
						memcpy(decompressed + 8, src + 8, 8);
					}
					//Else it is a run-length type match
					else {
						decompressed[0] = src[0];
						decompressed[1] = src[1];
						decompressed[2] = src[2];
						decompressed[3] = src[3];
						src += inc32table[distance];
						memcpy(decompressed + 4, src, 4);
						src += inc64table[distance];
						memcpy(decompressed + 8, src, 8);
					}
				}
			}
			else {
				const size_t lengthOverflow = length ? 18 : 34;
				length = (token & 0x1F) + SKANDA_NORMAL_MIN_MATCH_LENGTH;

				distance = read_uint32le(compressed);
				size_t bytes = distance & 0x3;
				distance = (distance >> 2) & (0x40 << bytes * 8) - 1;
				compressed += bytes + 1;

				//This can only happen with a new distance. decompressed > decompressedStart always, so no overflow
				if (unlikely(decompressed - decompressedStart < distance))
					return -1;

				if (length == lengthOverflow) {
					length += decode_length(compressed);
					//We have the guarantee that decompressed < decompressedEnd, so there is no overflow
					if (unlikely(decompressedEnd - decompressed < length))
						return -1;

					copy_match(decompressed, distance, length);
				}
				else {
					const uint8_t* src = decompressed - distance;

					//If the offset is big enough we can perform a faster copy
					if (distance >= 8) {

						memcpy(decompressed, src, 8);
						memcpy(decompressed + 8, src + 8, 8);
						if (length > 16) {
							memcpy(decompressed + 16, src + 16, 8);
							memcpy(decompressed + 24, src + 24, 8);
							decompressed[32] = src[32];
						}
					}
					//Else it is a run-length type match
					else {

						decompressed[0] = src[0];
						decompressed[1] = src[1];
						decompressed[2] = src[2];
						decompressed[3] = src[3];
						src += inc32table[distance];
						memcpy(decompressed + 4, src, 4);
						src += inc64table[distance];
						memcpy(decompressed + 8, src, 8);

						if (length > 16) {
							memcpy(decompressed + 16, src + 8, 8);
							memcpy(decompressed + 24, src + 16, 8);
							decompressed[32] = src[24];
						}
					}
				}
			}

			decompressed += length;
		}

		memcpy(decompressed, compressed, SKANDA_LAST_BYTES);
		progress->progress(SKANDA_LAST_BYTES);
		return 0;
	}
}

#endif  //SKANDA_IMPLEMENTATION

#endif  //__SKANDA__
