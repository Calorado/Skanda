/*
 * Skanda Compression Algorithm v1.1.1
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

namespace skanda {
	//Base class that allows to track progress of compression and decompression of
	// a Skanda stream. You will have to create a child class which implements the functions.
	// - progress(bytes): the algorithm will pass the number of bytes that have been compressed since last call
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
	//"Level" specifies a tradeoff between compressed size and speed, and must be in range [0, 9].
	//You may pass a pointer to an object with base class ProgressCallback, to track progress.
	//window determines the maximum backwards distance the matches can have, as a power of 2.
	//Larger values can improve compression, but may hurt decode speed if they exceed CPU cache.
	//Returns the size of the compressed stream or -1 on failure.
	size_t skanda_compress(const uint8_t* input, const size_t size, uint8_t* output, const uint8_t level,
		ProgressCallback* progress = nullptr, const size_t window = 20);
	//Decompresses contents in "compressed" to "decompressed".
	//You may also pass a pointer to an object with base class ProgressCallback, to track progress.
	//Returns 0 on success or -1 on failure or corrupted data.
	int skanda_decompress(const uint8_t* compressed, const size_t compressedSize, uint8_t* decompressed,
		const size_t uncompressedSize, ProgressCallback* progress = nullptr);

	//For a given input size, returns a size for the output buffer that is big enough to
	// contain the compressed stream even if it expands.
	size_t skanda_compress_bound(const size_t size);
	//Returns the amount of memory the algorithm will consume on compression.
	size_t skanda_estimate_memory(const size_t size, const uint8_t level, const size_t window = 20);
}

#ifdef SKANDA_IMPLEMENTATION

#include <algorithm>
#include <cstring>

using namespace std;

namespace skanda {

#if defined(_MSC_VER)
#define FORCE_INLINE __forceinline
#elif defined(__GNUC__)
#define FORCE_INLINE inline __attribute__((always_inline))
#else
#define FORCE_INLINE inline
#endif

#if (defined(__GNUC__) && (__GNUC__ >= 3)) || (defined(__INTEL_COMPILER) && (__INTEL_COMPILER >= 800)) || defined(__clang__)
#define expect(expr,value)    (__builtin_expect ((expr),(value)) )
#define likely(expr)     expect((expr) != 0, 1)
#define unlikely(expr)   expect((expr) != 0, 0)
#else
#define likely(expr)     (expr)
#define unlikely(expr)   (expr)
#endif

#if defined(_MSC_VER)
#if defined(_M_AMD64)
#include <intrin.h>
#define x64
#endif
#elif defined(__GNUC__)
#if defined(__amd64__)
#include <x86intrin.h>
#define x64
#endif
#endif

	//Probably not the correct way to do it but bleh
#if UINTPTR_MAX > UINT32_MAX
#define IS_64BIT 1
#else
#define IS_64BIT 0
#endif

	const union { uint16_t u; uint8_t c[2]; } LITTLE_ENDIAN_CHECK = { 1 };
#define IS_LITTLE_ENDIAN LITTLE_ENDIAN_CHECK.c[0]

#define MIN3(a, b, c) (std::min(a, std::min(b, c)))

#if IS_64BIT

	//Undefined behaviour if value == 0
	FORCE_INLINE uint64_t unsafe_int_log2(const uint64_t value) {
#if defined(_MSC_VER)
		unsigned long result;
		_BitScanReverse64(&result, value);
		return result;
#elif defined(__GNUC__)
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
#if defined(_MSC_VER) || defined(__GNUC__)
		if (likely(value != 0))
			return unsafe_int_log2(value);
		return 0;
#endif  //Fallback already returns 0 when value == 0
		return unsafe_int_log2(value);
	}

	//Undefined behaviour when value == 0
	FORCE_INLINE uint64_t unsafeBitScanForward(const uint64_t value) {
#if defined(_MSC_VER)
		unsigned long result;
		_BitScanForward64(&result, value);
		return result;
#elif defined(__GNUC__)
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
	FORCE_INLINE uint64_t bitScanForward(const uint64_t value) {
#if defined(_MSC_VER) || defined(__GNUC__)
		if (likely(value != 0))
			return unsafeBitScanForward(value);
		return 0;
#endif  //Fallback already returns 0 when value == 0
		return unsafeBitScanForward(value);
	}

	struct FastIntHash {
		//Use top bits
		uint64_t operator()(const uint64_t value) {
			return value * 0xff51afd7ed558ccd;
		}
	};

	//These functions are used to obtain the data for the hash. If the number of bytes is higher than the word size,
	//they will be mixed. Also note that these might read more bytes than necessary.
	FORCE_INLINE uint64_t readHash8(const uint8_t* const ptr) {
		uint64_t value;
		memcpy(&value, ptr, 8);
		return value;
	}
	FORCE_INLINE uint64_t readHash4(const uint8_t* const ptr) {
		uint32_t value;
		memcpy(&value, ptr, 4);
		return value;
	}
	FORCE_INLINE uint64_t readHash12(const uint8_t* const ptr) {
		return readHash8(ptr) ^ readHash4(ptr + 8);
	}
	FORCE_INLINE uint64_t readHash5(const uint8_t* const ptr) {
		if (IS_LITTLE_ENDIAN)
			return readHash8(ptr) << 24;
		return readHash8(ptr) >> 24;  //Assumes big endian
	}
	FORCE_INLINE uint64_t readHash3(const uint8_t* const ptr) {
		if (IS_LITTLE_ENDIAN)
			return readHash4(ptr) << 40;
		return readHash4(ptr) >> 8;  //Assumes big endian
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

	FORCE_INLINE uint32_t unsafeBitScanForward(uint32_t value) {
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

	FORCE_INLINE uint32_t bitScanForward(uint32_t value) {
#if defined(_MSC_VER) || defined(__GNUC__)
		if (likely(value != 0))
			return unsafeBitScanForward(value);
		return 0;
#endif
		return unsafeBitScanForward(value);
	}

	struct FastIntHash {
		uint32_t operator()(const uint32_t value) {
			return value * 0x27d4eb2d;
		}
	};

	FORCE_INLINE uint32_t readHash4(const uint8_t* const ptr) {
		uint32_t value;
		memcpy(&value, ptr, 4);
		return value;
	}
	FORCE_INLINE uint32_t readHash12(const uint8_t* const ptr) {
		return readHash4(ptr) ^ readHash4(ptr + 4) ^ readHash4(ptr + 8);
	}
	FORCE_INLINE uint32_t readHash8(const uint8_t* const ptr) {
		return readHash4(ptr) ^ readHash4(ptr + 4);
	}
	FORCE_INLINE uint32_t readHash5(const uint8_t* const ptr) {
		return readHash4(ptr) ^ ptr[4];
	}
	FORCE_INLINE uint32_t readHash3(const uint8_t* const ptr) {
		if (IS_LITTLE_ENDIAN)
			return readHash4(ptr) << 8;
		return readHash4(ptr) >> 8;  //Assumes big endian
	}

#endif

	FORCE_INLINE uint64_t readUint64LE(const uint8_t* const ptr) {
		if (IS_LITTLE_ENDIAN) {
			uint64_t value;
			memcpy(&value, ptr, 8);
			return value;
		}
		uint64_t value = 0;
		for (int i = 0; i < 8; i++)
			value |= (uint64_t)ptr[i] << i * 8;
		return value;
	}
	FORCE_INLINE void writeUint64LE(uint8_t* const ptr, const uint64_t value) {
		if (IS_LITTLE_ENDIAN)
			memcpy(ptr, &value, 8);
		else {
			for (int i = 0; i < 8; i++)
				ptr[i] = value >> i * 8;
		}
	}
	FORCE_INLINE uint32_t readUint32LE(const uint8_t* const ptr) {
		if (IS_LITTLE_ENDIAN) {
			uint32_t value;
			memcpy(&value, ptr, 4);
			return value;
		}
		uint32_t value = 0;
		for (int i = 0; i < 4; i++)
			value |= (uint64_t)ptr[i] << i * 8;
		return value;
	}
	FORCE_INLINE void writeUint32LE(uint8_t* const ptr, const uint32_t value) {
		if (IS_LITTLE_ENDIAN)
			memcpy(ptr, &value, 4);
		else {
			for (int i = 0; i < 4; i++)
				ptr[i] = value >> i * 8;
		}
	}
	FORCE_INLINE uint16_t readUint16LE(const uint8_t* const ptr) {
		uint16_t value;
		if (IS_LITTLE_ENDIAN)
			memcpy(&value, ptr, 2);
		else
			value = ptr[0] | (ptr[1] << 8);
		return value;
	}
	FORCE_INLINE void writeUint16LE(uint8_t* const ptr, const uint16_t value) {
		if (IS_LITTLE_ENDIAN)
			memcpy(ptr, &value, 2);
		else {
			for (int i = 0; i < 2; i++)
				ptr[i] = value >> i * 8;
		}
	}

	//Tries to find a match between the two locations, and returns the length
	//Note that this function should be called with at least MIN_LENGTH + 8 bytes of buffer after limit
	FORCE_INLINE size_t test_match(const uint8_t* front, const uint8_t* back, const uint8_t* const limit,
		const size_t minLength, const size_t window) {

		//Test first bytes
		//Usually compilers will optimize std::equal as 2 comparisons for minLength 5, 6, etc
		//We can only use one comparison to make it faster. For powers of 2, std::equal should be good
		if (IS_64BIT && IS_LITTLE_ENDIAN) {
			switch (minLength) {
			case 6:
				if ((readUint64LE(front) << 16) != (readUint64LE(back) << 16) || ((size_t)(front - back) >> window))
					return 0;
				break;
			case 5:
				if ((readUint64LE(front) << 24) != (readUint64LE(back) << 24) || ((size_t)(front - back) >> window))
					return 0;
				break;
			case 3:
				if ((readUint32LE(front) << 8) != (readUint32LE(back) << 8) || ((size_t)(front - back) >> window))
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

		if (IS_64BIT && IS_LITTLE_ENDIAN) {
			while (true) {
				if (front + 8 > limit) {
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
				const uint64_t xorVal = readUint64LE(front) ^ readUint64LE(back);

				if (xorVal) {
					front += unsafeBitScanForward(xorVal) >> 3;
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

	const int GREEDY_FAST = 0;
	const int GREEDY_NORMAL = 1;
	const int LAZY_NORMAL = 2;
	const int LAZY_EXTRA = 3;
	const int OPTIMAL_FAST = 4;
	const int OPTIMAL = 5;
	const int OPTIMAL_BRUTE = 6;

	struct CompressorOptions {
		size_t parserFunction;
		size_t maxHashTableSize;
		size_t maxElementsPerHash;
		size_t standardNiceLength;
		size_t repNiceLength;
		size_t optimalBlockSize;        // (Optimal)
		size_t maxArrivals;             // (Optimal)
	};

	template<class IntPtr>
	struct LZ_Structure {
		IntPtr matchLength;
		IntPtr matchDistance;
		IntPtr literalRunLength;
	};

	template<class IntPtr>
	struct LZ_Match {
		IntPtr length;
		IntPtr distance;
	};

	//A hash table which does not check for collisions, and has a constant size, known at compile time
	template<class Value, class Hash, size_t hashSize>
	class StaticHashTable {
		Value* arr = nullptr;
	public:
		//Use 2^x sizes to avoid the use of modulo operator
		StaticHashTable() {}
		~StaticHashTable() {
			delete[] arr;
		}
		void init() {
			arr = new Value[(size_t)1 << hashSize]();
		}
		Value& operator[](const size_t value) {
			return arr[Hash{}(value) >> (IS_64BIT ? 64 : 32) - hashSize];
		}
	};

	//A hash table which does not check for collisions
	template<class Value, class Hash>
	class HashTable {
		Value* arr = nullptr;
		size_t hashShift;
	public:
		//Use 2^x sizes to avoid the use of modulo operator
		HashTable() {}
		~HashTable() {
			delete[] arr;
		}
		void init(const size_t logSize) {
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
		void first(size_t& value) {
			const IntPtr tmp = data[0];
			data[0] = value;
			value = tmp;
		}
		//Loads the second value, and at the same time pushes a value into that slot.
		//Should be used after loading the first value.
		void second(size_t& value) {
			const IntPtr tmp = data[1];
			data[1] = value;
			value = tmp;
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
		void init(const size_t hashSize) {
			arr = new IntPtr[(size_t)2 << hashSize]();
			hashShift = (IS_64BIT ? 64 : 32) - hashSize;
		}
		LZ2WayCacheBucket<IntPtr> operator[](size_t value) {
			return LZ2WayCacheBucket<IntPtr>(arr + (Hash{}(value) >> hashShift) * 2);
		}
	};

	//Used for easier implementation
	//Please do not use push() and next() on the same bucket
	template<class IntPtr>
	class LzCacheBucket {
		IntPtr* it;
		IntPtr* last;
	public:
		LzCacheBucket() {}
		LzCacheBucket(IntPtr* _begin, IntPtr* _end) {
			it = _begin;
			last = _end;
		}
		//Pushes a new value into the bucket. Used when skipping bytes
		void push(size_t newValue) {
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
		void next(size_t& value) {
			const IntPtr tmp = *it;
			*it = value;
			value = tmp;
			it++;
		}
		//Loads the next position, but without updating the bucket as we go
		IntPtr next() {
			return *it++;
		}
	};

	//This works like the basic hash table, except that it stores N positions per bucket
	template<class IntPtr, class Hash>
	class LzCacheTable {
		IntPtr* arr = nullptr;
		size_t hashShift;
		size_t elementsPerBucket;  //log2
	public:
		//Use 2^x sizes to avoid the use of modulo and multiplication
		LzCacheTable() {}
		LzCacheTable(const size_t logSize, const size_t numElements) {
			init(logSize, numElements);
		}
		~LzCacheTable() {
			delete[] arr;
		}
		void init(const size_t logSize, const size_t numElements) {
			arr = new IntPtr[(size_t)1 << logSize << numElements]();
			hashShift = (IS_64BIT ? 64 : 32) - logSize;
			elementsPerBucket = numElements;
		}
		LzCacheBucket<IntPtr> operator[](size_t value) {
			value = Hash{}(value) >> hashShift;
			IntPtr* bucket = arr + (value << elementsPerBucket);
			return LzCacheBucket<IntPtr>(bucket, bucket + ((size_t)1 << elementsPerBucket));
		}
	};

	//Simple and fast
	template<class IntPtr>
	class HashTableMatchFinder {
		HashTable<IntPtr, FastIntHash> lzdict3;
		LzCacheTable<IntPtr, FastIntHash> lzdict4;
		LzCacheTable<IntPtr, FastIntHash> lzdict8;

	public:
		void init(const size_t size, const CompressorOptions compressorOptions, const size_t window) {
			const size_t log2size = MIN3(int_log2(size) - 3, compressorOptions.maxHashTableSize, window - 3);
			lzdict3.init(std::min(log2size, (size_t)14));
			lzdict4.init(log2size, compressorOptions.maxElementsPerHash);
			lzdict8.init(log2size, compressorOptions.maxElementsPerHash);
		}

		LZ_Match<IntPtr>* find_matches_and_update(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const limit,
			LZ_Match<IntPtr>* matches, size_t highestLength, const CompressorOptions compressorOptions, const size_t window) {

			IntPtr& chain3 = lzdict3[readHash3(input)];
			LzCacheBucket<IntPtr> chain4 = lzdict4[readHash4(input)];
			LzCacheBucket<IntPtr> chain8 = lzdict8[readHash8(input)];

			if (highestLength < 3) {
				const uint8_t* where = inputStart + chain3;
				size_t length = test_match(input, where, limit, 3, window);

				if (length > highestLength) {
					matches->distance = input - where;
					matches->length = length;
					matches++;

					highestLength = length;
					if (highestLength > compressorOptions.standardNiceLength) {
						chain3 = input - inputStart;
						chain4.push(input - inputStart);
						chain8.push(input - inputStart);
						return matches;
					}
				}
			}
			chain3 = input - inputStart;

			if (highestLength < 7) {

				size_t pos = input - inputStart;
				while (!chain4.ended()) {
					chain4.next(pos);

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
								chain4.next(pos);
							break;
						}
					}
				}
			}
			else {
				chain4.push(input - inputStart);
			}

			if (highestLength >= 4 && highestLength < compressorOptions.standardNiceLength) {

				size_t pos = input - inputStart;
				while (!chain8.ended()) {
					chain8.next(pos);

					const uint8_t* where = inputStart + pos;
					if (*(input + highestLength) != *(where + highestLength))
						continue;

					const size_t length = test_match(input, where, limit, 8, window);

					if (length > highestLength) {
						matches->distance = input - where;
						matches->length = length;
						matches++;

						highestLength = length;
						if (highestLength > compressorOptions.standardNiceLength) {
							while (!chain8.ended())
								chain8.next(pos);
							break;
						}
					}
				}
			}
			else {
				chain8.push(input - inputStart);
			}

			return matches;
		}

		void update_position(const uint8_t* const input, const uint8_t* const inputStart) {
			const size_t pos = input - inputStart;
			lzdict3[readHash3(input)] = pos;
			lzdict4[readHash4(input)].push(pos);
			lzdict8[readHash8(input)].push(pos);
		}
	};

	const int NO_MATCH_POS = 0;

	//Original match finder implementation from BriefLZ
	template<class IntPtr>
	class BinaryMatchFinder {
	public:

		IntPtr* nodes = nullptr;
		size_t nodeListMask;
		size_t nodeListSize;

		HashTable<IntPtr, FastIntHash> nodeLookup;
		HashTable<IntPtr, FastIntHash> lzdict3;
		LzCacheTable<IntPtr, FastIntHash> lzdict12;

		~BinaryMatchFinder() {
			delete[] nodes;
		}
		BinaryMatchFinder() {}

		void init(const size_t size, const CompressorOptions compressorOptions, const size_t window) {

			const size_t binaryTreeSize = (size_t)1 << std::min(compressorOptions.maxHashTableSize, window);
			const size_t totalWindowSize = std::min(size, (size_t)1 << window);
			nodeListSize = std::min(binaryTreeSize, size);
			nodeListMask = binaryTreeSize - 1;
			//It is not necessary to initialize to 0
			nodes = new IntPtr[2 * nodeListSize];

			nodeLookup.init(MIN3(int_log2(size) - 3, (size_t)20, window - 3));

			lzdict3.init(MIN3(int_log2(size) - 3, (size_t)16, window - 3));
			if (totalWindowSize > nodeListSize)
				lzdict12.init(std::max(4, (int)int_log2(totalWindowSize - nodeListSize) - 4), compressorOptions.maxElementsPerHash - 4);
		}

		LZ_Match<IntPtr>* find_matches_and_update(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const limit,
			LZ_Match<IntPtr>* matches, size_t highestLength, const CompressorOptions compressorOptions, const size_t window) {

			const size_t inputPosition = input - inputStart;

			// First try to get a length 3 match
			IntPtr& chain3 = lzdict3[readHash3(input)];
			const uint8_t* where = inputStart + chain3;
			size_t length = test_match(input, where, limit, 3, window);

			if (length > highestLength) {
				matches->distance = input - where;
				matches->length = length;
				highestLength = length;
				matches++;

				if (highestLength >= compressorOptions.standardNiceLength) {
					update_position(input, inputStart, limit, compressorOptions, window);
					return matches;
				}
			}
			chain3 = inputPosition;

			//If we reach this position on the back stop the search
			const size_t btEnd = inputPosition < nodeListSize ? 0 : inputPosition - nodeListSize;

			IntPtr& lookupEntry = nodeLookup[readHash4(input)];
			size_t backPosition = lookupEntry;
			lookupEntry = inputPosition;

			IntPtr* lesserNode = &nodes[2 * (lookupEntry & nodeListMask)];
			IntPtr* greaterNode = &nodes[2 * (lookupEntry & nodeListMask) + 1];
			const uint8_t* lesserFront = input;
			const uint8_t* greaterFront = input;

			size_t depth = (size_t)1 << compressorOptions.maxElementsPerHash;

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

				length = front - input;
				IntPtr* const nextNode = &nodes[2 * (backPosition & nodeListMask)];
				if (length > highestLength) {
					matches->distance = front - back;
					matches->length = length;
					matches++;

					if (length >= compressorOptions.standardNiceLength) {
						*lesserNode = nextNode[0];
						*greaterNode = nextNode[1];
						if (inputPosition > nodeListSize && ((size_t)1 << window) > nodeListSize)
							lzdict12[readHash12(input - nodeListSize)].push(inputPosition - nodeListSize);
						return matches;
					}
					highestLength = length;
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

				LzCacheBucket<IntPtr> cacheBucket = lzdict12[readHash12(input)];

				while (!cacheBucket.ended()) {
					const IntPtr pos = cacheBucket.next();

					const uint8_t* const where = inputStart + pos;
					if (*(input + highestLength) != *(where + highestLength))
						continue;

					const size_t length = test_match(input, where, limit, 12, window);

					if (length > highestLength) {
						matches->distance = input - where;
						matches->length = length;
						matches++;

						highestLength = length;
						if (highestLength >= compressorOptions.standardNiceLength)
							break;
					}
				}

				lzdict12[readHash12(input - nodeListSize)].push(inputPosition - nodeListSize);
			}

			return matches;
		}

		void update_position(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const limit,
			const CompressorOptions& compressorOptions, const size_t window) {

			const size_t inputPosition = input - inputStart;
			lzdict3[readHash3(input)] = inputPosition;
			if (inputPosition > nodeListSize && ((size_t)1 << window) > nodeListSize)
				lzdict12[readHash12(input - nodeListSize)].push(inputPosition - nodeListSize);

			//If we reach this position on the front stop the update
			const uint8_t* positionSkip = std::min(limit, input + compressorOptions.standardNiceLength);
			//If we reach this position on the back stop the update
			const size_t btEnd = inputPosition < nodeListSize ? 0 : inputPosition - nodeListSize;
			IntPtr& lookupEntry = nodeLookup[readHash4(input)];
			size_t backPosition = lookupEntry;
			lookupEntry = inputPosition;

			IntPtr* lesserNode = &nodes[2 * (inputPosition & nodeListMask)];
			IntPtr* greaterNode = &nodes[2 * (inputPosition & nodeListMask) + 1];

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

	FORCE_INLINE void copy_literal_run(const uint8_t* src, uint8_t*& dst, const size_t length) {

		uint8_t* const end = dst + length;
		memcpy(dst, src, 16);
		memcpy(dst + 16, src + 16, 16);
		if (length > 32) {
			src += 32;
			dst += 32;
			do {
				memcpy(dst, src, 16);
				memcpy(dst + 16, src + 16, 16);
				src += 32;
				dst += 32;
			} while (dst < end);
		}

		dst = end;
	}

	FORCE_INLINE void encode_prefixVarInt(uint8_t*& output, size_t& var, const bool nonZero) {

		size_t index = (nonZero ? unsafe_int_log2(var) : int_log2(var)) / 7;
		if (IS_64BIT && IS_LITTLE_ENDIAN) {

			if (index >= 8) {
				size_t base = var << (index + 1);
				base |= (size_t)1 << index;
				writeUint64LE(output, base);
				output += 8;
				const size_t extra = var >> (63 - index);
				writeUint16LE(output, extra);
				output += 1 + (index == 9);
			}
			else {
				//Branchless encoding
				var <<= (index + 1);
				var |= (size_t)1 << index;
				writeUint64LE(output, var);
				output += index + 1;
			}
		}
		else {
			while (index >= 8) {
				*output++ = 0;
				index -= 8;
			}

			*output++ = (var << (index + 1)) | ((size_t)1 << index);
			var >>= 7 - index;
			for (; var; ) {
				*output++ = var;
				var >>= 8;
			}
		}
	}

	FORCE_INLINE void encode_length(uint8_t*& output, size_t& var, const size_t overflow) {
		var -= overflow;
		if (var >= 255) {
			*output++ = 255;
			var -= 255;
			encode_prefixVarInt(output, var, 0);
		}
		else
			*output++ = var;
	}

	const int SKANDA_NORMAL_MIN_MATCH_LENGTH = 3;
	const int SKANDA_REP_MIN_MATCH_LENGTH = 2;
	//These are written uncompressed
	const int SKANDA_LAST_BYTES = 63;

	FORCE_INLINE void skanda_encode_literal_run(const uint8_t* const input,
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
			output += literalRunLength;
		}
	}

	FORCE_INLINE void skanda_encode_match(uint8_t*& output, uint8_t* const controlByte,
		size_t& matchLength, size_t& distance, size_t& repOffset) {

		//Repetition match found
		if (distance == repOffset) {
			matchLength -= SKANDA_REP_MIN_MATCH_LENGTH;
			if (matchLength >= 15) {
				*controlByte |= 31;
				encode_length(output, matchLength, 15);
			}
			else
				*controlByte |= 16 | matchLength;
		}
		//Standard match encoding
		else {
			matchLength -= SKANDA_NORMAL_MIN_MATCH_LENGTH;
			repOffset = distance;

			encode_prefixVarInt(output, distance, 1);

			const size_t lengthOverflow = (*controlByte) ? 15 : 31;
			if (matchLength >= lengthOverflow) {
				*controlByte |= lengthOverflow;
				encode_length(output, matchLength, lengthOverflow);
			}
			else
				*controlByte |= matchLength;
		}
	}

	//It is important to have all these variables as compile-time constants to get maximum performance
	template<class IntPtr>
	size_t skanda_compress_ultra_fast(const uint8_t* input, const size_t size, uint8_t* output,
		ProgressCallback* progress, const size_t window) {

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		//The last match must end before this limit
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;
		const uint8_t* lastProgressReport;

		//Data for the format
		size_t repOffset = 1;
		//Skip first byte
		const uint8_t* literalRunStart = input;
		input++;

		//Hash table for match finder
		StaticHashTable<IntPtr, FastIntHash, 12> lzdict;
		try {
			lzdict.init();
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		//Main compression loop
		while (input < compressionLimit) {

			const uint8_t* const nextProgressReport = (compressionLimit - input < SKANDA_PROGRESS_REPORT_PERIOD) ?
				compressionLimit : input + SKANDA_PROGRESS_REPORT_PERIOD;
			lastProgressReport = input;

			while (input < nextProgressReport) {

				//Try to find a match
				IntPtr* dictEntry = &lzdict[readHash5(input)];
				size_t matchLength = test_match(input, inputStart + *dictEntry, compressionLimit, 5, window);

				//Have we found a match?
				if (matchLength) {
					//First output the literal run
					uint8_t* const controlByte = output++;
					skanda_encode_literal_run(input, literalRunStart, controlByte, output);

					const size_t pos = input - inputStart;
					size_t distance = pos - *dictEntry;

					//Update hash table. Since we already have the hash bucket for the first byte, reuse it
					*dictEntry = pos;
					lzdict[readHash5(input + 1)] = pos + 1;

					input += matchLength;
					literalRunStart = input;
					//Output the match
					skanda_encode_match(output, controlByte, matchLength, distance, repOffset);
				}
				//If there isnt advance one byte and try again
				else {
					*dictEntry = input - inputStart;
					input++;
				}
			}

			if (progress->abort())
				return 0;
			progress->progress(input - lastProgressReport);
		}

		//Output the last literal run. This is necessary even when no more literals remain, as the decoder expects to end with this
		uint8_t* const controlByte = output++;
		skanda_encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		progress->progress(SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	template<class IntPtr>
	size_t skanda_compress_greedy(const uint8_t* input, const size_t size, uint8_t* output,
		ProgressCallback* progress, const size_t window, const CompressorOptions compressorOptions) {

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;
		const uint8_t* lastProgressReport;

		size_t repOffset = 1;
		const uint8_t* literalRunStart = input;
		input++;

		const size_t hashSize = MIN3(int_log2(size) - 3, compressorOptions.maxHashTableSize, window - 3);
		HashTable<IntPtr, FastIntHash> lzdict;
		try {
			lzdict.init(hashSize);
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		while (input < compressionLimit) {

			const uint8_t* const nextProgressReport = (compressionLimit - input < SKANDA_PROGRESS_REPORT_PERIOD) ?
				compressionLimit : input + SKANDA_PROGRESS_REPORT_PERIOD;
			lastProgressReport = input;

			while (input < nextProgressReport) {

				IntPtr* dictEntry = &lzdict[readHash5(input)];
				size_t matchLength = test_match(input, inputStart + *dictEntry, compressionLimit, 5, window);

				if (matchLength) {
					uint8_t* const controlByte = output++;
					skanda_encode_literal_run(input, literalRunStart, controlByte, output);

					const size_t pos = input - inputStart;
					size_t distance = pos - *dictEntry;

					*dictEntry = pos;
					lzdict[readHash5(input + 1)] = pos + 1;
					lzdict[readHash5(input + 2)] = pos + 2;
					lzdict[readHash5(input + 3)] = pos + 3;
					lzdict[readHash5(input + 4)] = pos + 4;

					input += matchLength;
					literalRunStart = input;
					skanda_encode_match(output, controlByte, matchLength, distance, repOffset);
				}
				else {
					*dictEntry = input - inputStart;
					input++;
				}
			}

			if (progress->abort())
				return 0;
			progress->progress(input - lastProgressReport);
		}

		uint8_t* const controlByte = output++;
		skanda_encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		progress->progress(SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	//Try to find the longest match for a given position. Then try to find an even longer one in the next.
	//If it is found, code a literal, and the longer one found. If not, code the original.
	template<class IntPtr>
	FORCE_INLINE void skanda_fast_lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit,
		LZ2WayCacheTable<IntPtr, FastIntHash>& lzdict, size_t& bestMatchLength, size_t& bestMatchDistance,
		size_t& lazySteps, size_t& testedPositions, const size_t window, const CompressorOptions compressorOptions) {

		testedPositions = 1;
		LZ2WayCacheBucket<IntPtr> dictEntry = lzdict[readHash5(input)];

		//Test first entry
		size_t pos = input - inputStart;
		dictEntry.first(pos);
		const uint8_t* where = inputStart + pos;
		bestMatchLength = test_match(input, where, limit, 5, window);
		bestMatchDistance = input - where;
		if (bestMatchLength >= compressorOptions.standardNiceLength) {
			dictEntry.push_in_second(pos);
			lazySteps = 0;
			return;
		}

		//Test second entry
		dictEntry.second(pos);
		where = inputStart + pos;
		//Simple heuristic: as we are looking for a longer match, we can first
		//test the byte that would make this match longer. There is a high
		//chance it will differ, so the rest of the match wont need to be tested
		if (*(input + bestMatchLength) == *(where + bestMatchLength)) {
			const size_t length = test_match(input, where, limit, 5, window);
			if (length > bestMatchLength) {
				bestMatchDistance = input - where;
				bestMatchLength = length;

				if (bestMatchLength >= compressorOptions.standardNiceLength) {
					lazySteps = 0;
					return;
				}
			}
		}

		//Nothing was found, code a literal and try again from the begining
		if (bestMatchLength < 5) {
			lazySteps = 1;
			return;
		}

		//Now try to find a longer match at next position
		input++;
		lazySteps = 0;
		testedPositions = 2;
		dictEntry = lzdict[readHash5(input)];

		//Test first entry
		pos = input - inputStart;
		dictEntry.first(pos);
		where = inputStart + pos;
		if (*(input + bestMatchLength) == *(where + bestMatchLength)) {
			const size_t length = test_match(input, where, limit, 5, window);
			if (length > bestMatchLength) {
				bestMatchDistance = input - where;
				bestMatchLength = length;
				lazySteps = 1;

				if (bestMatchLength >= compressorOptions.standardNiceLength) {
					dictEntry.push_in_second(pos);
					return;
				}
			}
		}
		//Test second entry
		dictEntry.second(pos);
		where = inputStart + pos;
		if (*(input + bestMatchLength) == *(where + bestMatchLength)) {
			const size_t length = test_match(input, where, limit, 5, window);
			if (length > bestMatchLength) {
				bestMatchDistance = input - where;
				bestMatchLength = length;
				lazySteps = 1;
			}
		}
	}

	template<class IntPtr>
	size_t skanda_compress_lazy_fast(const uint8_t* input, const size_t size, uint8_t* output,
		ProgressCallback* progress, const size_t window, const CompressorOptions compressorOptions) {

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;
		const uint8_t* lastProgressReport;

		size_t repOffset = 1;
		const uint8_t* literalRunStart = input;
		input++;

		const size_t hashSize = MIN3(int_log2(size) - 3, compressorOptions.maxHashTableSize, window - 3);
		LZ2WayCacheTable<IntPtr, FastIntHash> lzdict;
		try {
			lzdict.init(hashSize);
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		while (input < compressionLimit) {

			const uint8_t* const nextProgressReport = (compressionLimit - input < SKANDA_PROGRESS_REPORT_PERIOD) ?
				compressionLimit : input + SKANDA_PROGRESS_REPORT_PERIOD;
			lastProgressReport = input;

			while (input < nextProgressReport) {

				size_t matchLength;
				size_t distance;
				size_t lazySteps;   //bytes to skip because of lazy matching
				size_t testedPositions;   //number of positions that have been added to hash table

				skanda_fast_lazy_search<IntPtr>(input, inputStart, compressionLimit, lzdict,
					matchLength, distance, lazySteps, testedPositions, window, compressorOptions);

				input += lazySteps;

				if (matchLength) {

					uint8_t* const controlByte = output++;
					skanda_encode_literal_run(input, literalRunStart, controlByte, output);

					const uint8_t* const matchEnd = input + matchLength;
					input += testedPositions - lazySteps;

					//If the match is RLE-like, add only the last repetition of the run
					if (distance < matchLength) 
						input = matchEnd - distance;
					for (; input != matchEnd; input++)
						lzdict[readHash5(input)].push_in_first(input - inputStart);

					literalRunStart = input;
					skanda_encode_match(output, controlByte, matchLength, distance, repOffset);
				}
			}

			if (progress->abort())
				return 0;
			progress->progress(input - lastProgressReport);
		}

		uint8_t* const controlByte = output++;
		skanda_encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		progress->progress(SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	//Same principle as the last function, but with additional heuristics to improve ratio
	template<class IntPtr>
	FORCE_INLINE void skanda_lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit,
		LzCacheTable<IntPtr, FastIntHash>& lzdict4, LzCacheTable<IntPtr, FastIntHash>& lzdict8, size_t& bestLength,
		size_t& bestDistance, size_t& repOffset, size_t& lazySteps, size_t& testedPositions,
		const CompressorOptions compressorOptions, const size_t window) {

		LzCacheBucket<IntPtr> chain4 = lzdict4[readHash4(input)];
		LzCacheBucket<IntPtr> chain8 = lzdict8[readHash8(input)];
		size_t pos = input - inputStart;

		lazySteps = 0;
		testedPositions = 1;

		//First try to find a match in the rep offset. If it is found simply take it
		size_t length = test_match(input, input - repOffset, limit, 3, window);
		if (length) {
			bestDistance = repOffset;
			bestLength = length;
			chain4.push(pos);
			chain8.push(pos);
			return;
		}

		bestLength = 0;
		size_t bestMatchCost = 0;

		//If no rep offset was found try to get a length 8 match
		while (!chain8.ended()) {
			chain8.next(pos);

			const uint8_t* where = inputStart + pos;

			if (*(input + bestLength) != *(where + bestLength))
				continue;

			length = test_match(input, where, limit, 8, window);

			size_t distance = input - where;
			size_t matchCost = 2 + unsafe_int_log2(distance) / 7;
			if (length + bestMatchCost > matchCost + bestLength) {
				bestDistance = distance;
				bestLength = length;
				bestMatchCost = matchCost;

				if (bestLength >= compressorOptions.standardNiceLength) {
					while (!chain8.ended())
						chain8.next(pos);
					chain4.push(input - inputStart);
					return;
				}
			}
		}

		//If still nothing was found, try a length 4
		if (bestLength < 8) {

			pos = input - inputStart;
			while (!chain4.ended()) {
				chain4.next(pos);

				const uint8_t* where = inputStart + pos;

				if (*(input + bestLength) != *(where + bestLength))
					continue;

				length = test_match(input, where, limit, 4, window);

				size_t distance = input - where;
				size_t matchCost = 2 + unsafe_int_log2(distance) / 7;
				if (length + bestMatchCost > matchCost + bestLength) {
					bestDistance = distance;
					bestLength = length;
					bestMatchCost = matchCost;
					//We did not found any length 8 earlier, so we wont find anything better than 7 now
					if (bestLength >= 7) {
						if (bestLength >= compressorOptions.standardNiceLength) {
							while (!chain4.ended())
								chain4.next(pos);
							return;
						}
						break;
					}
				}
			}

			//No match found, code a literal and retry
			if (bestLength == 0) {
				lazySteps = 1;
				return;
			}
		}
		else {
			chain4.push(input - inputStart);
		}

		//Now try to get a better match at pos + 1
		input++;
		pos = input - inputStart;
		lzdict4[readHash4(input)].push(pos);  //We wont search for length < 8
		chain8 = lzdict8[readHash8(input)];
		testedPositions = 2;

		//Increase rep offset minimum length to 4
		length = test_match(input, input - repOffset, limit, 4, window);
		if (length) {
			bestDistance = repOffset;
			bestLength = length;
			chain8.push(pos);
			lazySteps = 1;
			return;
		}

		//Only try to find matches of length at least 8 at pos + 1
		while (!chain8.ended()) {
			chain8.next(pos);

			const uint8_t* where = inputStart + pos;

			if (*(input + bestLength) != *(where + bestLength))
				continue;

			length = test_match(input, where, limit, 8, window);

			size_t distance = input - where;
			size_t matchCost = 2 + unsafe_int_log2(distance) / 7;
			if (length + bestMatchCost > matchCost + bestLength) {
				bestDistance = distance;
				bestLength = length;
				bestMatchCost = matchCost;
				lazySteps = 1;

				if (bestLength >= compressorOptions.standardNiceLength) {
					while (!chain8.ended())
						chain8.next(pos);
					return;
				}
			}
		}
	}

	template<class IntPtr>
	size_t skanda_compress_lazy(const uint8_t* input, const size_t size, uint8_t* output,
		ProgressCallback* progress, const size_t window, const CompressorOptions compressorOptions) {

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;
		const uint8_t* lastProgressReport;

		size_t repOffset = 1;
		const uint8_t* literalRunStart = input;
		input++;

		const size_t log2size = MIN3(int_log2(size) - 3, compressorOptions.maxHashTableSize, window - 3);
		LzCacheTable<IntPtr, FastIntHash> lzdict4;
		LzCacheTable<IntPtr, FastIntHash> lzdict8;

		try {
			lzdict4.init(log2size, compressorOptions.maxElementsPerHash);
			lzdict8.init(log2size, compressorOptions.maxElementsPerHash);
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		while (input < compressionLimit) {

			const uint8_t* const nextProgressReport = (compressionLimit - input < SKANDA_PROGRESS_REPORT_PERIOD) ?
				compressionLimit : input + SKANDA_PROGRESS_REPORT_PERIOD;
			lastProgressReport = input;

			while (input < nextProgressReport) {

				size_t matchLength;
				size_t distance;
				size_t lazySteps;
				size_t testedPositions;

				skanda_lazy_search(input, inputStart, compressionLimit, lzdict4, lzdict8, matchLength,
					distance, repOffset, lazySteps, testedPositions, compressorOptions, window);

				input += lazySteps;

				if (matchLength) {

					uint8_t* const controlByte = output++;
					skanda_encode_literal_run(input, literalRunStart, controlByte, output);

					const uint8_t* const matchEnd = input + matchLength;
					input += testedPositions - lazySteps;

					//If the match is RLE-like, add only the last repetition of the run
					if (distance < matchLength)
						input = matchEnd - distance;
					for (; input != matchEnd; input++) {
						const size_t ptr = input - inputStart;
						lzdict4[readHash4(input)].push(ptr);
						lzdict8[readHash8(input)].push(ptr);
					}

					literalRunStart = input;
					skanda_encode_match(output, controlByte, matchLength, distance, repOffset);
				}
			}

			if (progress->abort())
				return 0;
			progress->progress(input - lastProgressReport);
		}

		uint8_t* const controlByte = output++;
		skanda_encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		progress->progress(SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}


	template<class IntPtr>
	struct SkandaOptimalParserState {
		//16 high bits store size cost, 16 low bits the speed cost
		//This way we can also compare both costs prioritising size cost with a single operation
		uint32_t cost;
		//Serves as both match distance and rep offset.
		IntPtr distance;
		//Literal run length in a certain state can be unbounded, while
		// match length is limited by "nice length"
		IntPtr literalRunLength;
		uint16_t matchLength;
		uint16_t path;
	};

	template<class IntPtr>
	LZ_Structure<IntPtr>* skanda_forward_optimal_parse(const uint8_t* const input, const uint8_t* const inputStart,
		const uint8_t* const limit, HashTableMatchFinder<IntPtr>& matchFinder, SkandaOptimalParserState<IntPtr>* parser,
		LZ_Structure<IntPtr>* stream, const size_t startLiteralRunLength, const size_t startRepOffset,
		const CompressorOptions compressorOptions, const size_t window) {

		const size_t blockLength = std::min((size_t)(limit - input), compressorOptions.optimalBlockSize);
		for (size_t i = 1; i <= blockLength; i++)
			parser[i].cost = UINT32_MAX;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;

		parser[0].cost = 0;
		parser[0].distance = startRepOffset;
		parser[0].literalRunLength = startLiteralRunLength;

		size_t position = 0;
		for (; position < blockLength; position++) {

			const uint8_t* const inputPosition = input + position;
			SkandaOptimalParserState<IntPtr>* const parserPosition = parser + position;

			const size_t literalCost = parserPosition->cost + (0x10000 << (parserPosition->literalRunLength == 6));  //only size cost
			SkandaOptimalParserState<IntPtr>* nextPosition = parserPosition + 1;
			if (literalCost < nextPosition->cost) {
				nextPosition->cost = literalCost;
				nextPosition->distance = parserPosition->distance;
				nextPosition->literalRunLength = parserPosition->literalRunLength + 1;
			}

			size_t repMatchLength = 0;
			if (parserPosition->literalRunLength) {
				size_t repMatchLength = test_match(inputPosition, inputPosition - parserPosition->distance, limit, 2, window);

				if (repMatchLength) {
					if (position + repMatchLength >= blockLength || repMatchLength >= compressorOptions.repNiceLength) {
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

			LZ_Match<IntPtr> matches[16];
			const LZ_Match<IntPtr>* matchesEnd = matchFinder.find_matches_and_update(inputPosition, inputStart,
				limit, matches, repMatchLength, compressorOptions, window);

			//At least one match was found
			if (matchesEnd != matches) {
				//The last match should be the longest
				const LZ_Match<IntPtr>* const longestMatch = matchesEnd - 1;
				//Match goes outside the buffer or is very long
				if (position + longestMatch->length >= blockLength || longestMatch->length >= compressorOptions.standardNiceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					break;
				}

				const size_t lengthOverflow = parserPosition->literalRunLength ? 17 : 33;
				for (const LZ_Match<IntPtr>* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					if (matchIt->distance == parserPosition->distance)
						continue;

					size_t matchCost = parserPosition->cost;
					matchCost += (2 + unsafe_int_log2(matchIt->distance) / 7 + (matchIt->length > lengthOverflow)) << 16; //size cost
					matchCost += 1 + (matchIt->distance > (1 << 20)); //speed cost

					nextPosition = parserPosition + matchIt->length;
					if (matchCost < nextPosition->cost) {
						nextPosition->cost = matchCost;
						nextPosition->matchLength = matchIt->length;
						nextPosition->distance = matchIt->distance;
						nextPosition->literalRunLength = 0;
					}
				}
			}
		}

		// Backward pass, pick best option at each step.
		const SkandaOptimalParserState<IntPtr>* backwardParse = parser + position;
		const SkandaOptimalParserState<IntPtr>* const parseEnd = parser;

		if (lastMatchLength) {
			stream->literalRunLength = 0;
			stream++;
			stream->matchDistance = lastMatchDistance;
			stream->matchLength = lastMatchLength;
			stream->literalRunLength = backwardParse->literalRunLength;
			backwardParse -= backwardParse->literalRunLength;
			stream++;

			const uint8_t* inputPosition = input + position;
			const uint8_t* const matchEnd = inputPosition + lastMatchLength;
			for (inputPosition++; inputPosition < matchEnd; inputPosition++)
				matchFinder.update_position(inputPosition, inputStart);
		}
		else {
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
		(stream - 1)->literalRunLength -= startLiteralRunLength;  //Remove, or it will produce invalid data

		return stream - 1;
	}

	template<class IntPtr>
	LZ_Structure<IntPtr>* skanda_multi_arrivals_parse(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit,
		BinaryMatchFinder<IntPtr>& matchFinder, SkandaOptimalParserState<IntPtr>* parser, LZ_Structure<IntPtr>* stream,
		const size_t startLiteralRunLength, const size_t startRepOffset,
		const CompressorOptions compressorOptions, const size_t window) {

		const size_t blockLength = std::min((size_t)(limit - input), compressorOptions.optimalBlockSize - 1);
		for (size_t i = 1; i <= blockLength * compressorOptions.maxArrivals; i++)
			parser[i].cost = UINT32_MAX;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;
		size_t lastPath;

		for (size_t i = 0; i < compressorOptions.maxArrivals; i++) {
			parser[i].cost = 0;
			parser[i].distance = startRepOffset;
			parser[i].literalRunLength = startLiteralRunLength;
		}

		size_t position = 0;
		for (; position < blockLength; position++) {

			const uint8_t* inputPosition = input + position;
			SkandaOptimalParserState<IntPtr>* const parserPosition = parser + position * compressorOptions.maxArrivals;

			size_t acceptableRepMatchLength = 2;  //Only take rep matches as long as this

			//Literal and rep match parsing can be done at the same time
			for (size_t i = 0; i < compressorOptions.maxArrivals; i++) {

				SkandaOptimalParserState<IntPtr>* const currentArrival = parserPosition + i;

				const size_t literalCost = currentArrival->cost + (0x10000 << (currentArrival->literalRunLength == 6)); //only size cost
				SkandaOptimalParserState<IntPtr>* arrivalIt = parserPosition + compressorOptions.maxArrivals;
				SkandaOptimalParserState<IntPtr>* lastArrival = arrivalIt + compressorOptions.maxArrivals;

				for (; arrivalIt < lastArrival; arrivalIt++) {

					if (literalCost < arrivalIt->cost) {

						for (SkandaOptimalParserState<IntPtr>* it = lastArrival - 1; it != arrivalIt; it--)
							memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState<IntPtr>));

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
						if (position + repMatchLength >= blockLength || repMatchLength >= compressorOptions.repNiceLength) {
							lastMatchLength = repMatchLength;
							lastMatchDistance = currentArrival->distance;
							lastPath = i;
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

								for (SkandaOptimalParserState<IntPtr>* it = lastArrival - 1; it != arrivalIt; it--)
									memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState<IntPtr>));

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

											for (SkandaOptimalParserState<IntPtr>* it = lastArrival - 1; it != arrivalIt; it--)
												memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState<IntPtr>));

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

			LZ_Match<IntPtr> matches[144];
			LZ_Match<IntPtr>* matchesEnd = matchFinder.find_matches_and_update(inputPosition, inputStart, limit, matches,
				std::max((size_t)2, acceptableRepMatchLength - 1), compressorOptions, window);

			if (matchesEnd != matches) {
				const LZ_Match<IntPtr>* const longestMatch = matchesEnd - 1;
				if (position + longestMatch->length >= blockLength || longestMatch->length >= compressorOptions.standardNiceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					lastPath = 0;
					break;
				}

				size_t length = 0;
				const size_t lengthOverflow = parserPosition->literalRunLength ? 17 : 33;

				for (LZ_Match<IntPtr>* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					if (matchIt->distance == parserPosition->distance)
						continue;

					size_t matchCost = parserPosition->cost;
					matchCost += (2 + unsafe_int_log2(matchIt->distance) / 7) << 16;  //size cost
					matchCost += 1 + (matchIt->distance > (1 << 20)) * 2;  //speed cost

					do {
						//If the current match has a length that overflows, and we have not tried any
						// length just below that overflow, try it
						if (length < lengthOverflow && matchIt->length > lengthOverflow)
							length = lengthOverflow;
						else
							length = matchIt->length;

						matchCost += (length > lengthOverflow) << 16;

						SkandaOptimalParserState<IntPtr>* arrivalIt = parserPosition + length * compressorOptions.maxArrivals;
						SkandaOptimalParserState<IntPtr>* const lastArrival = arrivalIt + compressorOptions.maxArrivals;

						for (; arrivalIt < lastArrival; arrivalIt++) {

							if (matchCost < arrivalIt->cost) {

								for (SkandaOptimalParserState<IntPtr>* it = lastArrival - 1; it != arrivalIt; it--)
									memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState<IntPtr>));

								arrivalIt->cost = matchCost;
								arrivalIt->matchLength = length;
								arrivalIt->distance = matchIt->distance;
								arrivalIt->literalRunLength = 0;
								arrivalIt->path = 0;
								break;
							}
						}

					} while (length != matchIt->length);
				}
			}
		}

	doBackwardParse:

		// Backward pass, pick best option at each step.
		const SkandaOptimalParserState<IntPtr>* backwardParse = parser + position * compressorOptions.maxArrivals;
		const SkandaOptimalParserState<IntPtr>* const parseEnd = parser;
		size_t path = 0;

		if (lastMatchLength) {
			stream->literalRunLength = 0;
			stream++;
			stream->matchDistance = lastMatchDistance;
			stream->matchLength = lastMatchLength;
			stream->literalRunLength = 0;
			stream++;
			path = lastPath;

			const uint8_t* inputPosition = input + position;
			const uint8_t* const matchEnd = inputPosition + lastMatchLength;
			for (inputPosition++; inputPosition < matchEnd; inputPosition++)
				matchFinder.update_position(inputPosition, inputStart, limit, compressorOptions, window);
		}
		else {
			stream->literalRunLength = 0;
			stream++;
			path = backwardParse[path].path;
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
	size_t skanda_compress_optimal(const uint8_t* input, const size_t size, uint8_t* output,
		ProgressCallback* progress, const size_t window, const CompressorOptions compressorOptions) {

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;
		const uint8_t* lastProgressReport;

		//Data for the format
		size_t repOffset = 1;
		//Skip first byte
		const uint8_t* literalRunStart = input;
		input++;

		HashTableMatchFinder<IntPtr> hashMatchFinder;
		BinaryMatchFinder<IntPtr> binaryMatchFinder;
		SkandaOptimalParserState<IntPtr>* parser = nullptr;
		LZ_Structure<IntPtr>* stream = nullptr;

		try {
			if (compressorOptions.parserFunction == OPTIMAL_FAST) {
				hashMatchFinder.init(size, compressorOptions, window);
				parser = new SkandaOptimalParserState<IntPtr>[(compressorOptions.optimalBlockSize + 1)];
			}
			else {
				binaryMatchFinder.init(size, compressorOptions, window);
				parser = new SkandaOptimalParserState<IntPtr>[(compressorOptions.optimalBlockSize + 1) * compressorOptions.maxArrivals];
			}
			stream = new LZ_Structure<IntPtr>[compressorOptions.optimalBlockSize];
		}
		catch (const std::bad_alloc& e) {
			delete[] parser;
			delete[] stream;
			return -1;
		}

		while (input < compressionLimit) {

			const uint8_t* const nextProgressReport = (compressionLimit - input < SKANDA_PROGRESS_REPORT_PERIOD) ?
				compressionLimit : input + SKANDA_PROGRESS_REPORT_PERIOD;
			lastProgressReport = input;

			while (input < nextProgressReport) {

				LZ_Structure<IntPtr>* streamIt;
				if (compressorOptions.parserFunction == OPTIMAL) {
					streamIt = skanda_multi_arrivals_parse(input, inputStart, compressionLimit, binaryMatchFinder,
						parser, stream, input - literalRunStart, repOffset, compressorOptions, window);
				}
				else {
					streamIt = skanda_forward_optimal_parse(input, inputStart, compressionLimit, hashMatchFinder,
						parser, stream, input - literalRunStart, repOffset, compressorOptions, window);
				}

				//Main compression loop
				while (true) {
					input += streamIt->literalRunLength;

					if (streamIt == stream)
						break;

					uint8_t* const controlByte = output++;
					skanda_encode_literal_run(input, literalRunStart, controlByte, output);

					size_t matchLength = streamIt->matchLength;
					size_t distance = streamIt->matchDistance;
					input += matchLength;
					skanda_encode_match(output, controlByte, matchLength, distance, repOffset);
					literalRunStart = input;

					streamIt--;
				}
			}

			if (progress->abort()) {
				delete[] parser;
				delete[] stream;
				return 0;
			}
			progress->progress(input - lastProgressReport);
		}

		delete[] parser;
		delete[] stream;

		//Output the last literal run. This is necessary even when no more literals remain, as the decoder expects to end with this
		uint8_t* const controlByte = output++;
		skanda_encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		progress->progress(SKANDA_LAST_BYTES);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	const size_t NOT_USED = -1;
	const CompressorOptions skandaCompressorLevels[] = {
		//      Parser        Hash log     Elements per hash     Nice length     Rep nice length      Block size      Max arrivals
			{ GREEDY_FAST  ,  NOT_USED  ,      NOT_USED       ,    NOT_USED   ,      NOT_USED     ,    NOT_USED     ,   NOT_USED   },
			{ GREEDY_NORMAL,     17     ,      NOT_USED       ,    NOT_USED   ,      NOT_USED     ,    NOT_USED     ,   NOT_USED   },
			{ LAZY_NORMAL  ,     17     ,      NOT_USED       ,       16      ,      NOT_USED     ,    NOT_USED     ,   NOT_USED   },
			{ LAZY_EXTRA   ,     17     ,          1          ,       16      ,      NOT_USED     ,    NOT_USED     ,   NOT_USED   },
			{ LAZY_EXTRA   ,     18     ,          2          ,       24      ,      NOT_USED     ,    NOT_USED     ,   NOT_USED   },
			{ OPTIMAL_FAST ,     18     ,          2          ,       24      ,         8         ,      1024       ,   NOT_USED   },
			{ OPTIMAL_FAST ,     19     ,          3          ,       32      ,         12        ,      1024       ,   NOT_USED   },
			{ OPTIMAL      ,     22     ,          4          ,       32      ,         16        ,      2048       ,       2      },
			{ OPTIMAL      ,     24     ,          5          ,       80      ,         40        ,      2048       ,       4      },
			{ OPTIMAL      ,     26     ,          6          ,       256     ,         128       ,      4096       ,       8      },
	};

	size_t skanda_compress(const uint8_t* input, const size_t size, uint8_t* output,
		const uint8_t level, ProgressCallback* progress, const size_t window) {

		if (level > 9)
			return 0;

		ProgressCallback defaultProgress;
		if (!progress)
			progress = &defaultProgress;

		if (size <= SKANDA_LAST_BYTES) {
			memcpy(output, input, size);
			progress->progress(size);
			return size;
		}

#ifdef COMPRESS_LARGE_BLOCK
		if (size > ((uint64_t)1 << 32)) {
			if (skandaCompressorLevels[level].parserFunction == GREEDY_FAST)
				return skanda_compress_ultra_fast<uint64_t>(input, size, output, progress, window);
			if (skandaCompressorLevels[level].parserFunction == GREEDY_NORMAL)
				return skanda_compress_greedy<uint64_t>(input, size, output, progress, window, skandaCompressorLevels[level]);
			if (skandaCompressorLevels[level].parserFunction == LAZY_NORMAL)
				return skanda_compress_lazy_fast<uint64_t>(input, size, output, progress, window, skandaCompressorLevels[level]);
			if (skandaCompressorLevels[level].parserFunction == LAZY_EXTRA)
				return skanda_compress_lazy<uint64_t>(input, size, output, progress, window, skandaCompressorLevels[level]);
			return skanda_compress_optimal<uint64_t>(input, size, output, progress, window, skandaCompressorLevels[level]);  //optimal levels
		}
#endif
		if (skandaCompressorLevels[level].parserFunction == GREEDY_FAST)
			return skanda_compress_ultra_fast<uint32_t>(input, size, output, progress, window);
		if (skandaCompressorLevels[level].parserFunction == GREEDY_NORMAL)
			return skanda_compress_greedy<uint32_t>(input, size, output, progress, window, skandaCompressorLevels[level]);
		if (skandaCompressorLevels[level].parserFunction == LAZY_NORMAL)
			return skanda_compress_lazy_fast<uint32_t>(input, size, output, progress, window, skandaCompressorLevels[level]);
		if (skandaCompressorLevels[level].parserFunction == LAZY_EXTRA)
			return skanda_compress_lazy<uint32_t>(input, size, output, progress, window, skandaCompressorLevels[level]);
		return skanda_compress_optimal<uint32_t>(input, size, output, progress, window, skandaCompressorLevels[level]);  //optimal levels
	}

	size_t skanda_compress_bound(const size_t size) {
		return size + size / 128 + 4;
	}

	template<class IntPtr>
	size_t skanda_memory_estimator(const size_t size, const size_t window, const uint8_t level) {
		if (skandaCompressorLevels[level].parserFunction == GREEDY_FAST)
			return sizeof(IntPtr) << 12;
		if (skandaCompressorLevels[level].parserFunction == GREEDY_NORMAL)
			return sizeof(IntPtr) << MIN3(int_log2(size) - 3, skandaCompressorLevels[level].maxHashTableSize, window - 3);
		if (skandaCompressorLevels[level].parserFunction == LAZY_NORMAL)
			return sizeof(IntPtr) << MIN3(int_log2(size) - 3, skandaCompressorLevels[level].maxHashTableSize, window - 3) << 1;
		if (skandaCompressorLevels[level].parserFunction == LAZY_EXTRA)
			//Lazy extra uses 2 tables
			return sizeof(IntPtr) << MIN3(int_log2(size) - 3, skandaCompressorLevels[level].maxHashTableSize, window - 3)
			<< skandaCompressorLevels[level].maxElementsPerHash << 1;
		if (skandaCompressorLevels[level].parserFunction == OPTIMAL_FAST) {
			const size_t log2size = MIN3(int_log2(size) - 3, skandaCompressorLevels[level].maxHashTableSize, window - 3);
			size_t memory = sizeof(IntPtr) << std::min(log2size, (size_t)14);  //hash 3 table
			memory += sizeof(IntPtr) << log2size << skandaCompressorLevels[level].maxElementsPerHash << 1;  //hash 4 and hash 8 tables
			memory += sizeof(SkandaOptimalParserState<IntPtr>) * skandaCompressorLevels[level].optimalBlockSize + 1;
			memory += sizeof(LZ_Structure<IntPtr>) * skandaCompressorLevels[level].optimalBlockSize;
			return memory;
		}
		const size_t binaryTreeSize = (size_t)1 << std::min(skandaCompressorLevels[level].maxHashTableSize, window);
		const size_t totalWindowSize = std::min(size, (size_t)1 << window);
		const size_t nodeListSize = std::min(binaryTreeSize, size);
		size_t memory = sizeof(IntPtr) * 2 * nodeListSize;  //binary tree
		memory += sizeof(IntPtr) << MIN3(int_log2(size) - 3, (size_t)20, window - 3);  //binary node lookup
		memory += sizeof(IntPtr) << MIN3(int_log2(size) - 3, (size_t)16, window - 3);  //hash 3 table
		if (totalWindowSize > nodeListSize)
			memory += sizeof(IntPtr) << std::max(4, (int)int_log2(totalWindowSize - nodeListSize) - 4) <<
			(skandaCompressorLevels[level].maxElementsPerHash - 4);  //extra hash 12 table
		memory += sizeof(SkandaOptimalParserState<IntPtr>) * (skandaCompressorLevels[level].optimalBlockSize + 1) * skandaCompressorLevels[level].maxArrivals;
		memory += sizeof(LZ_Structure<IntPtr>) * skandaCompressorLevels[level].optimalBlockSize;
		return memory;
	}

	size_t skanda_estimate_memory(const size_t size, const uint8_t level, const size_t window) {
		if (size <= SKANDA_LAST_BYTES + 1)
			return 0;

#ifdef COMPRESS_LARGE_BLOCK
		if (size > ((uint64_t)1 << 32))
			return skanda_memory_estimator<uint64_t>(size, window, level);
#endif
		return skanda_memory_estimator<uint32_t>(size, window, level);
	}

	const int8_t inc32table[16] = { 0, 1, 2, 1, 0,  4,  4,  4, 4, 4, 4, 4, 4, 4, 4, 4 };
	const int8_t inc64table[16] = { 0, 0, 0, 1, 4, -1, -2, -3, 4, 4, 4, 4, 4, 4, 4, 4 };

	//Optimized for long matches
	FORCE_INLINE void copy_match(uint8_t*& dst, const size_t offset, const size_t length) {

		uint8_t* const end = dst + length;
		const uint8_t* src = dst - offset;

		//If the offset is big enough we can perform a faster copy
		if (offset >= 16) {
			//Should be translated to some sse/neon or whatever-that-loads-stores-16-bytes instructions
			memcpy(dst, src, 16);
			src += 16;
			dst += 16;
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
			memcpy(dst + 16, src + 8, 8);

			dst += 24;
			src += 16;
			do {
				memcpy(dst, src, 8);
				memcpy(dst + 8, src + 8, 8);
				src += 16;
				dst += 16;
			} while (dst < end);
		}

		dst = end;
	}

	//For 64bit this wont read more than 10 bytes
	FORCE_INLINE size_t decode_prefixVarInt(const uint8_t*& compressed) {

		size_t result;
		if (IS_64BIT && IS_LITTLE_ENDIAN) {
			//Just take 8 bytes, and check the position of the first 1, that will tell how many bytes the distance occupies
			result = readUint64LE(compressed);
			size_t index = bitScanForward(result);
			//We will need additional bytes
			if (index >= 8) {
				compressed += 8;
				result >>= index + 1;
				const size_t extra = readUint16LE(compressed) & (index == 9 ? 0xFFFF : 0xFF);
				compressed += 1 + (index == 9);
				result |= extra << (63 - index);
			}
			else {
				//Throw away those leading 0 and the 1, and advance the compressed data stream
				result &= UINT64_MAX >> (7 - index) * 8;
				index++;
				result >>= index;
				compressed += index;
			}
		}
		else {
			result = *compressed++;
			bool bit = result & 1;
			result >>= 1;
			for (size_t i = 1; !bit && i <= 9; i++) {
				bit = result & 1;
				result >>= 1;
				result |= (size_t)(*compressed++) << (i * 7 - 1);
			}
		}
		return result;
	}

	FORCE_INLINE size_t decode_length(const uint8_t*& compressed) {
		size_t length = *compressed++;
		if (length == 255)
			length += decode_prefixVarInt(compressed);
		return length;
	}

	uint64_t iterations = 0;

	int skanda_decompress(const uint8_t* compressed, const size_t compressedSize, uint8_t* decompressed,
		const size_t uncompressedSize, ProgressCallback* progress) {

		ProgressCallback defaultProgress;
		if (!progress)
			progress = &defaultProgress;

		//Only last bytes are present
		if (uncompressedSize <= SKANDA_LAST_BYTES) {
			if (compressedSize < uncompressedSize)
				return -1;
			memcpy(decompressed, compressed, uncompressedSize);
			progress->progress(uncompressedSize);
			return 0;
		}

		if (compressedSize < SKANDA_LAST_BYTES + 1)
			return -1;

		const uint8_t* const decompressedStart = decompressed;
		const uint8_t* const decompressedEnd = decompressed + uncompressedSize - SKANDA_LAST_BYTES;
		const uint8_t* const compressedEnd = compressed + compressedSize - SKANDA_LAST_BYTES;
		const uint8_t* nextProgressReport = (decompressedEnd - decompressed < SKANDA_PROGRESS_REPORT_PERIOD) ?
			decompressedEnd : decompressed + SKANDA_PROGRESS_REPORT_PERIOD;
		const uint8_t* lastProgressReport = decompressed;

		size_t distance = 1;
		size_t length;  //Stores the literal run length and match length

		while (true) {

			const uint8_t token = *compressed++;

			if (token >= (7 << 5)) {

				length = decode_length(compressed) + 7;

				//The maximum number of bytes we could have written without checks for decompression buffer
				// overflow is 33 (from match length). At the same time, we could have read
				// 10 (distance) + 11 (match length) + 1 (token) + 11 (literal run length) = 33 bytes.
				//This limits the number of bytes we can safely copy without checks.

				memcpy(decompressed, compressed, 8);
				memcpy(decompressed + 8, compressed + 8, 8);
				memcpy(decompressed + 16, compressed + 16, 8);

				if (length > 24) {

					//We dont have the guarantee that compressedEnd >= compressed
					if (compressed > compressedEnd || decompressed > decompressedEnd ||
						compressedEnd - compressed < length ||
						decompressedEnd - decompressed < length) {
						return -1;
					}

					uint8_t* dst = decompressed + 24;
					const uint8_t* src = compressed + 24;
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
				length = token >> 5;
				//If the cpu supports fast unaligned access, it is faster to copy a fixed amount of bytes instead of the needed amount
				memcpy(decompressed, compressed, 8);
			}
			decompressed += length;
			compressed += length;

			if (decompressed >= nextProgressReport) {
				if (progress->abort())
					return 0;
				progress->progress(decompressed - lastProgressReport);

				nextProgressReport = (decompressedEnd - decompressed < SKANDA_PROGRESS_REPORT_PERIOD) ?
					decompressedEnd : decompressed + SKANDA_PROGRESS_REPORT_PERIOD;
				lastProgressReport = decompressed;

				if (decompressed >= decompressedEnd) {
					//The maximum number of bytes that can be written in a single iteration
					//(assuming no overflows) is 33 from match and 8(because we write more than needed) from literal run.
					//As it is less than the end buffer, this check should be enough.
					if (decompressed > decompressedEnd || compressed > compressedEnd) {
						return -1;
					}
					break;
				}
			}

			//The maximum number of bytes that can be advanced with a match is 10(distance) + 11(length)
			//For a literal run, 1 (token) + 8(literals)(I put 8 because we always read more than needed,
			// and this is the last read). That makes a total of 30, which is less than the
			//end buffer, and so only a check is necessary here. The exception is an overflow in the
			//literal run, so that needs its own check.
			if (compressed > compressedEnd) {
				return -1;
			}

			//Reuse the last offset
			if ((token & 0x10) && length) {
				length = (token & 0xF) + SKANDA_REP_MIN_MATCH_LENGTH;

				if (length == 15 + SKANDA_REP_MIN_MATCH_LENGTH) {
					length += decode_length(compressed);
					//We have the guarantee that decompressed < decompressedEnd, so there is no overflow
					if (decompressedEnd - decompressed < length) {
						return -1;
					}

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
					decompressed += length;
				}
			}
			else {
				const size_t lengthOverflow = length ? 18 : 34;
				length = (token & 0x1F) + SKANDA_NORMAL_MIN_MATCH_LENGTH;
				distance = decode_prefixVarInt(compressed);

				//This can only happen with a new distance. decompressed > decompressedStart always, so no overflow
				if (decompressed - decompressedStart < distance) {
					return -1;
				}

				if (length == lengthOverflow) {
					length += decode_length(compressed);
					//We have the guarantee that decompressed < decompressedEnd, so there is no overflow
					if (decompressedEnd - decompressed < length)
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
					decompressed += length;
				}
			}
		}

		memcpy(decompressed, compressed, SKANDA_LAST_BYTES);
		progress->progress(SKANDA_LAST_BYTES);
		return 0;
	}
}

#endif  //SKANDA_IMPLEMENTATION

#endif  //__SKANDA__
