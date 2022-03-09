/*
 * Skanda Compression Algorithm
 * Copyright (c) 2022 Calorado
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef __SKANDA__

#define __SKANDA__

#include <cstdint>

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
	//You may also pass a pointer to an object with base class ProgressCallback, to track progress.
	//Returns the size of the compressed stream or -1 on failure.
	size_t skanda_compress(const uint8_t* input, const size_t size, uint8_t* output, const uint8_t level,
		ProgressCallback* progress = nullptr);
	//Decompresses contents in "compressed" to "decompressed".
	//You may also pass a pointer to an object with base class ProgressCallback, to track progress.
	//Returns 0 on success or -1 on failure or corrupted data.
	int skanda_decompress(const uint8_t* compressed, const size_t compressedSize, uint8_t* decompressed,
		const size_t uncompressedSize, ProgressCallback* progress = nullptr);

	//For a given input size, returns a size for the output buffer that is big enough to
	// contain the compressed stream even if it expands.
	size_t skanda_compress_bound(const size_t size);
	//Returns the amount of memory the algorithm will consume on compression.
	size_t skanda_estimate_memory(const size_t size, const uint8_t level);
}

#ifdef SKANDA_IMPLEMENTATION

#include <algorithm>
#include <cstring>
#include <cmath>

using namespace std;

namespace skanda {

#if defined(_MSC_VER)
#define FORCE_INLINE __forceinline
#elif defined(__GNUC__)
#define FORCE_INLINE inline __attribute__((always_inline))
#else
#define FORCE_INLINE inline
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

#if IS_64BIT

	//Way faster than using log2(double), also returns 0 for a value of 0
	FORCE_INLINE uint64_t int_log2(const uint64_t value) {
#if defined(_MSC_VER)
		if (value) {
			unsigned long result;
			_BitScanReverse64(&result, value);
			return result;
		}
		return 0;
#elif defined(__GNUC__)
		if (value)
			return 63 - __builtin_clzll(value);
		return 0;
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

	//Returns the index of the first set bit, starting from the least significant, or 0 if the input is null
	FORCE_INLINE uint64_t bitScanForward(const uint64_t value) {
#if defined(_MSC_VER)
		if (value) {
			unsigned long result;
			_BitScanForward64(&result, value);
			return result;
		}
		return 0;
#elif defined(__GNUC__)
		if (value)
			return __builtin_ctzll(value);
		return 0;
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
	FORCE_INLINE uint64_t readHash2(const uint8_t* const ptr) {
		uint16_t value;
		memcpy(&value, ptr, 2);
		return value;
	}
	FORCE_INLINE uint64_t readHash12(const uint8_t* const ptr) {
		return readHash8(ptr) ^ readHash4(ptr + 8);
	}
	FORCE_INLINE uint64_t readHash6(const uint8_t* const ptr) {
		if (IS_LITTLE_ENDIAN)
			return readHash8(ptr) << 16;
		return readHash8(ptr) >> 16;  //Assumes big endian
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

	FORCE_INLINE uint32_t int_log2(uint32_t value) {
#if defined(_MSC_VER)
		if (value) {
			unsigned long result;
			_BitScanReverse(&result, value);
			return result;
		}
		return 0;
#elif defined(__GNUC__)
		if (value)
			return 31 - __builtin_clz(value);
		return 0;
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

	FORCE_INLINE uint32_t bitScanForward(uint32_t value) {
#if defined(_MSC_VER)
		if (value) {
			unsigned long result;
			_BitScanForward(&result, value);
			return result;
		}
		return 0;
#elif defined(__GNUC__)
		if (value)
			return __builtin_ctz(value);
		return 0;
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
	FORCE_INLINE uint32_t readHash2(const uint8_t* const ptr) {
		uint16_t value;
		memcpy(&value, ptr, 2);
		return value;
	}
	FORCE_INLINE uint32_t readHash12(const uint8_t* const ptr) {
		return readHash4(ptr) ^ readHash4(ptr + 4) ^ readHash4(ptr + 8);
	}
	FORCE_INLINE uint32_t readHash8(const uint8_t* const ptr) {
		return readHash4(ptr) ^ readHash4(ptr + 4);
	}
	FORCE_INLINE uint32_t readHash6(const uint8_t* const ptr) {
		uint16_t a;
		memcpy(&a, ptr + 0, 2);
		return readHash4(ptr + 2) ^ a;
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
	FORCE_INLINE size_t test_match(const uint8_t* front, const uint8_t* back, const uint8_t* const limit, const size_t minLength) {

		//Test first bytes
		//Usually compilers will optimize std::equal as 2 comparisons for minLength 5, 6, etc
		//We can only use one comparison to make it faster. For powers of 2, std::equal should be good
		if (IS_64BIT && IS_LITTLE_ENDIAN) {
			switch (minLength) {
			case 6:
				if ((readUint64LE(front) << 16) != (readUint64LE(back) << 16))
					return 0;
				break;
			case 5:
				if ((readUint64LE(front) << 24) != (readUint64LE(back) << 24))
					return 0;
				break;
			case 3:
				if ((readUint32LE(front) << 8) != (readUint32LE(back) << 8))
					return 0;
				break;
			default:
				if (!std::equal(back, back + minLength, front))
					return 0;
				break;
			}
		}
		else {
			if (!std::equal(back, back + minLength, front))
				return 0;
		}

		const uint8_t* const matchOrigin = back;
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
					return back - matchOrigin;
				}

				//Compare 8 bytes at a time using xor. It has the property of returning 0 if the two values are equal.
				//In case they differ, we can get the first byte that differs using a bit scan.
				const uint64_t xorVal = readUint64LE(front) ^ readUint64LE(back);

				if (xorVal) {
					back += bitScanForward(xorVal) >> 3;
					return back - matchOrigin;
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
			return back - matchOrigin;
		}
	}

	//Note that this function will write beyond the end of the match. It is important to keep
	//a buffer of at least 32 bytes after the end of match. But it is much faster than a memcpy()
	FORCE_INLINE void copy_match(uint8_t*& dst, const size_t offset, const size_t length) {

		uint8_t* const end = dst + length;
		const uint8_t* src = dst - offset;

		//If the offset is big enough we can perform a faster copy
		if (offset >= 16) {
			//Should be translated to some sse/neon or whatever-that-loads-stores-16-bytes instructions
			do {
				memcpy(dst, src, 16);
				memcpy(dst + 16, src + 16, 16);
				src += 32;
				dst += 32;
			} while (dst < end);
		}
		//Else it is a run-length type match
		else {
			//Change the offset to at least 2, so that we can copy 2 bytes at a time
			*dst++ = *src;
			src += offset > 1;

			//Now the offset is at least 4
			memcpy(dst, src, 2);
			dst += 2;
			src += ((offset >= 4) << 1) - (offset == 3);

			do {
				memcpy(dst, src, 4);
				memcpy(dst + 4, src + 4, 4);
				memcpy(dst + 8, src + 8, 4);
				memcpy(dst + 12, src + 12, 4);
				dst += 16;
				src += 16;
			} while (dst < end);
		}

		dst = end;
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
		size_t maxArrivals;             // (Optimal)
		size_t optimalBlockSize;        // (Optimal)
	};

	template<class Pointer>
	struct LZ_Structure {
		Pointer matchLength;
		Pointer matchDistance;
		Pointer literalRunLength;
	};

	template<class Pointer>
	struct LZ_Match {
		Pointer length;
		Pointer distance;
	};

	//A hash table which does not check for collisions
	template<class Value, class Hash>
	class HashTable {
		Value* arr = nullptr;
		size_t hashShift;
	public:
		//Use 2^x sizes to avoid the use of modulo operator
		HashTable() {}
		HashTable(const size_t logSize) {
			init(logSize);
		}
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

	//Used for easier implementation
	//Please do not use push() and next() on the same bucket
	template<class Pointer>
	class LzCacheBucket {
		Pointer* it;
		Pointer* last;
	public:
		LzCacheBucket() {}
		LzCacheBucket(Pointer* _begin, Pointer* _end) {
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
			const Pointer tmp = *it;
			*it = value;
			value = tmp;
			it++;
		}
		//Loads the next position, but without updating the bucket as we go
		Pointer next() {
			return *it++;
		}
	};

	//This works like the basic hash table, except that it stores N positions per bucket
	template<class Pointer, class Hash>
	class LzCacheTable {
		Pointer* arr = nullptr;
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
			arr = new Pointer[(size_t)1 << logSize << numElements]();
			hashShift = (IS_64BIT ? 64 : 32) - logSize;
			elementsPerBucket = numElements;
		}
		LzCacheBucket<Pointer> operator[](size_t value) {
			value = Hash{}(value) >> hashShift;
			Pointer* bucket = arr + (value << elementsPerBucket);
			return LzCacheBucket<Pointer>(bucket, bucket + ((size_t)1 << elementsPerBucket));
		}
	};

	//Simple and fast
	template<class Pointer>
	class HashTableMatchFinder {
		HashTable<Pointer, FastIntHash> lzdict3;
		LzCacheTable<Pointer, FastIntHash> lzdict4;
		LzCacheTable<Pointer, FastIntHash> lzdict8;

	public:
		void init(const size_t size, const CompressorOptions compressorOptions) {
			const size_t log2size = std::min(int_log2(size) - 3, compressorOptions.maxHashTableSize);
			lzdict3.init(std::min(log2size, (size_t)14));
			lzdict4.init(log2size, compressorOptions.maxElementsPerHash);
			lzdict8.init(log2size, compressorOptions.maxElementsPerHash);
		}

		LZ_Match<Pointer>* find_matches_and_update(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const limit,
			LZ_Match<Pointer>* matches, size_t highestLength, const CompressorOptions compressorOptions) {

			Pointer& chain3 = lzdict3[readHash3(input)];
			LzCacheBucket<Pointer> chain4 = lzdict4[readHash4(input)];
			LzCacheBucket<Pointer> chain8 = lzdict8[readHash8(input)];

			if (highestLength < 3) {
				const uint8_t* where = inputStart + chain3;
				size_t length = test_match(input, where, limit, 3);

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

					const size_t length = test_match(input, where, limit, 4);

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

					const size_t length = test_match(input, where, limit, 8);

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
	template<class Pointer>
	class BinaryMatchFinder {
	public:

		Pointer* nodes = nullptr;
		Pointer nodeListMask;
		Pointer nodeListSize;

		HashTable<Pointer, FastIntHash> nodeLookup;
		HashTable<Pointer, FastIntHash> lzdict3;
		HashTable<Pointer, FastIntHash> lzdict2;
		LzCacheTable<Pointer, FastIntHash> lzdict12;

		bool useHash2;

		~BinaryMatchFinder() {
			delete[] nodes;
		}
		BinaryMatchFinder() {}

		void init(const size_t size, const CompressorOptions compressorOptions, const bool _useHash2) {

			const size_t binaryTreeSize = (size_t)1 << compressorOptions.maxHashTableSize;

			//It is not necessary to initialize to 0
			nodes = new Pointer[2 * std::min(binaryTreeSize, size)];
			nodeListMask = binaryTreeSize - 1;
			nodeListSize = binaryTreeSize;
			nodeLookup.init(std::min(int_log2(size) - 3, compressorOptions.maxHashTableSize - 3));

			lzdict3.init(std::min(int_log2(size) - 3, (size_t)16));
			if (size >= binaryTreeSize)
				lzdict12.init(std::max(4, (int)int_log2(size - binaryTreeSize) - 4), compressorOptions.maxElementsPerHash - 4);

			if (_useHash2)
				lzdict2.init(std::min(int_log2(size) - 3, (size_t)12));
			useHash2 = _useHash2;
		}

		LZ_Match<Pointer>* find_matches_and_update(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const limit,
			LZ_Match<Pointer>* matches, size_t highestLength, const CompressorOptions compressorOptions) {

			const size_t inputPosition = input - inputStart;

			if (useHash2) {
				Pointer& chain2 = lzdict2[readHash2(input)];
				const uint8_t* where = inputStart + chain2;
				size_t length = test_match(input, where, limit, 2);

				if (length > highestLength) {
					matches->distance = input - where;
					matches->length = length;
					highestLength = length;
					matches++;

					if (highestLength >= compressorOptions.standardNiceLength) {
						update_position(input, inputStart, limit, compressorOptions);
						return matches;
					}
				}
				chain2 = inputPosition;
			}

			// First try to get a length 3 match
			Pointer& chain3 = lzdict3[readHash3(input)];
			const uint8_t* where = inputStart + chain3;
			size_t length = test_match(input, where, limit, 3);

			if (length > highestLength) {
				matches->distance = input - where;
				matches->length = length;
				highestLength = length;
				matches++;

				if (highestLength >= compressorOptions.standardNiceLength) {
					update_position(input, inputStart, limit, compressorOptions);
					return matches;
				}
			}
			chain3 = inputPosition;

			// Look up first match for current position
			//
			// backPosition is the current root of the tree of strings with this
			// hash. We are going to re-root the tree so inputPosition becomes the
			// new root.
			//
			//If we reach this position on the back stop the search
			const size_t btEnd = inputPosition < nodeListSize ? 0 : inputPosition - nodeListSize;

			Pointer& lookupEntry = nodeLookup[readHash4(input)];
			size_t backPosition = lookupEntry;
			lookupEntry = inputPosition;

			Pointer* lesserNode = &nodes[2 * (lookupEntry & nodeListMask)];
			Pointer* greaterNode = &nodes[2 * (lookupEntry & nodeListMask) + 1];
			const uint8_t* lesserFront = input;
			const uint8_t* greaterFront = input;

			size_t depth = (size_t)1 << compressorOptions.maxElementsPerHash;

			// Check matches
			while (true) {
				// If at bottom of tree, mark leaf nodes
				//
				// In case we reached max_depth, this also prunes the
				// subtree we have not searched yet and do not know
				// where belongs.
				//
				if (backPosition <= btEnd || depth-- == 0) {
					*lesserNode = NO_MATCH_POS;
					*greaterNode = NO_MATCH_POS;
					break;
				}

				// The string at backPosition is lexicographically greater than
				// a string that matched in the first lesserLength positions,
				// and less than a string that matched in the first
				// greaterLength positions, so it must match up to at least
				// the minimum of these.
				const uint8_t* front = std::min(lesserFront, greaterFront);
				const uint8_t* back = front - (inputPosition - backPosition);

				// Extend current match if possible
				//
				// Note that we are checking matches in order from the
				// closest and back. This means for a match further
				// away, the encoding of all lengths up to the current
				// max length will always be longer or equal, so we need
				// only consider the extension.
				//
				const size_t extraLength = test_match(front, back, limit, 0);
				front += extraLength;
				back += extraLength;

				length = front - input;
				Pointer* const nextNode = &nodes[2 * (backPosition & nodeListMask)];
				if (length > highestLength) {
					matches->distance = front - back;
					matches->length = length;
					matches++;

					// If we reach maximum match length, the string at backPosition
					// is equal to inputPosition, so we can assign the left and right
					// subtrees.
					//
					// This removes backPosition from the tree, but we added inputPosition
					// which is equal and closer for future matches.
					//
					if (length >= compressorOptions.standardNiceLength) {
						*lesserNode = nextNode[0];
						*greaterNode = nextNode[1];
						if (inputPosition >= nodeListSize)
							lzdict12[readHash12(input - nodeListSize)].push(inputPosition - nodeListSize);
						return matches;
					}
					highestLength = length;
				}

				// Go to previous match and restructure tree
				//
				// lesserNode points to a node that is going to contain
				// elements lexicographically less than inputPosition (the search
				// string).
				//
				// If the string at backPosition is less than inputPosition, we set that
				// lesserNode to backPosition. We know that all elements in the
				// left subtree are less than backPosition, and thus less than
				// inputPosition, so we point lesserNode at the right subtree of
				// backPosition and continue our search there.
				//
				// The equivalent applies to greaterNode when the string at
				// backPosition is greater than inputPosition.
				//
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

			if (inputPosition >= nodeListSize) {

				LzCacheBucket<Pointer> cacheBucket = lzdict12[readHash12(input)];

				while (!cacheBucket.ended()) {
					const Pointer pos = cacheBucket.next();

					const uint8_t* const where = inputStart + pos;
					if (*(input + highestLength) != *(where + highestLength))
						continue;

					const size_t length = test_match(input, where, limit, 12);

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
			const CompressorOptions& compressorOptions) {

			const size_t inputPosition = input - inputStart;
			if (useHash2)
				lzdict2[readHash2(input)] = inputPosition;
			lzdict3[readHash3(input)] = inputPosition;
			if (inputPosition >= nodeListSize)
				lzdict12[readHash12(input - nodeListSize)].push(inputPosition - nodeListSize);

			//If we reach this position on the front stop the update
			const uint8_t* positionSkip = std::min(limit, input + compressorOptions.standardNiceLength);
			//If we reach this position on the back stop the update
			const size_t btEnd = inputPosition < nodeListSize ? 0 : inputPosition - nodeListSize;
			Pointer& lookupEntry = nodeLookup[readHash4(input)];
			size_t backPosition = lookupEntry;
			lookupEntry = inputPosition;

			Pointer* lesserNode = &nodes[2 * (inputPosition & nodeListMask)];
			Pointer* greaterNode = &nodes[2 * (inputPosition & nodeListMask) + 1];

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

				const size_t length = test_match(front, back, positionSkip, 0);
				front += length;
				back += length;

				Pointer* const nextNode = &nodes[2 * (backPosition & nodeListMask)];
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

	// * / * // * / * // * / * // * / * // * / * // * / * //
	//                      SKANDA                        //
	// * / * // * / * // * / * // * / * // * / * // * / * //

	//How many bytes to process before reporting progress or checking for abort flag
	const int SKANDA_PROGRESS_REPORT_PERIOD = 512 * 1024;

	FORCE_INLINE void copy_literal_run(const uint8_t* src, uint8_t*& dst, const size_t length) {

		uint8_t* const end = dst + length;
		do {
			memcpy(dst, src, 16);
			memcpy(dst + 16, src + 16, 16);
			src += 32;
			dst += 32;
		} while (dst < end);
		dst = end;
	}

	FORCE_INLINE void encode_prefixVarInt(uint8_t*& output, size_t& var) {

		if (IS_64BIT && IS_LITTLE_ENDIAN) {
			const size_t index = int_log2(var) / 7;
			if (index >= 8) {
				size_t base = var << (index + 1);
				base |= (size_t)1 << index;
				writeUint64LE(output, base);
				output += 8;
				const size_t extra = var >> 63 - index;
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
			size_t index = int_log2(var) / 7;
			while (index >= 8) {
				*output++ = 0;
				index -= 8;
			}

			*output++ = (var << index + 1) | ((size_t)1 << index);
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
			encode_prefixVarInt(output, var);
		}
		else
			*output++ = var;
	}

	//For 64bit this wont read more than 10 bytes
	FORCE_INLINE void decode_prefixVarInt(const uint8_t*& compressed, size_t& var) {

		if (IS_64BIT && IS_LITTLE_ENDIAN) {
			//Just take 8 bytes, and check the position of the first 1, that will tell how many bytes the distance occupies
			var = readUint64LE(compressed);
			size_t index = bitScanForward(var);
			//We will need additional bytes
			if (index >= 8) {
				compressed += 8;
				var >>= index + 1;
				const size_t extra = readUint16LE(compressed) & (index == 9 ? 0xFFFF : 0xFF);
				compressed += 1 + (index == 9);
				var |= extra << 63 - index;
			}
			else {
				//Throw away those leading 0 and the 1, and advance the compressed data stream
				var &= UINT64_MAX >> (7 - index) * 8;
				index++;
				var >>= index;
				compressed += index;
			}
		}
		else {
			var = *compressed++;
			bool bit = var & 1;
			var >>= 1;
			for (size_t i = 1; !bit && i <= 9; i++) {
				bit = var & 1;
				var >>= 1;
				var |= (size_t)(*compressed++) << (i * 7) - 1;
			}
		}
	}

	FORCE_INLINE void decode_length(const uint8_t*& compressed, size_t& length, const size_t baseOverflow) {
		length += *compressed++;
		if (length == baseOverflow + 255) {
			decode_prefixVarInt(compressed, length);
			length += baseOverflow + 255;
		}
	}

	const int SKANDA_NORMAL_MIN_MATCH_LENGTH = 3;
	const int SKANDA_REP_MIN_MATCH_LENGTH = 2;
	//These are written uncompressed
	const int SKANDA_LAST_BYTES = 31;

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
		size_t& matchLength, size_t& distance, size_t& repOffsetA, size_t& repOffsetB) {

		//Repetition match found
		if (distance == repOffsetB || distance == repOffsetA) {
			matchLength -= SKANDA_REP_MIN_MATCH_LENGTH;
			if (matchLength >= 7) {
				*controlByte |= 23 | (distance == repOffsetB) * 8;
				encode_length(output, matchLength, 7);
			}
			else
				*controlByte |= (16 | matchLength | (distance == repOffsetB) * 8);
		}
		//Standard match encoding
		else {
			matchLength -= SKANDA_NORMAL_MIN_MATCH_LENGTH;
			repOffsetA = repOffsetB;
			repOffsetB = distance;

			distance--;
			encode_prefixVarInt(output, distance);

			if (matchLength >= 15) {
				*controlByte |= 15;
				encode_length(output, matchLength, 15);
			}
			else
				*controlByte |= matchLength;
		}
	}

	//Try to find the longest match for a given position. Then try to find an even longer one in the next.
	//If it is found, code a literal, and the longer one found. If not, code the original.
	template<class Pointer>
	FORCE_INLINE void skanda_fast_lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit,
		LzCacheTable<Pointer, FastIntHash>& lzdict5, size_t& bestLength, size_t& bestDistance,
		size_t& lazySteps, size_t& testedPositions, const CompressorOptions compressorOptions) {

		bestLength = 0;

		LzCacheBucket<Pointer> dictEntry5 = lzdict5[readHash5(input)];

		size_t pos = input - inputStart;
		while (!dictEntry5.ended()) {
			dictEntry5.next(pos);

			const uint8_t* where = inputStart + pos;

			//Simple heuristic: as we are looking for a longer match, we can first
			//test the byte that would make this match longer. There is a high
			//chance it will differ, so the rest of the match wont need to be tested
			if (*(input + bestLength) != *(where + bestLength))
				continue;

			const size_t length = test_match(input, where, limit, 5);

			if (length > bestLength) {
				bestDistance = input - where;
				bestLength = length;

				//If we reach a certain length, simply code that match
				if (length >= compressorOptions.standardNiceLength) {
					while (!dictEntry5.ended())
						dictEntry5.next(pos);
					lazySteps = 0;
					testedPositions = 1;
					return;
				}
			}
		}

		//Nothing was found, code a literal and try again from the begining
		if (bestLength < 5) {
			lazySteps = 1;
			return;
		}

		//Now try to find a longer match
		input++;
		lazySteps = 0;
		testedPositions = 2;
		dictEntry5 = lzdict5[readHash5(input)];
		pos = input - inputStart;
		while (!dictEntry5.ended()) {
			dictEntry5.next(pos);

			const uint8_t* where = inputStart + pos;

			if (*(input + bestLength) != *(where + bestLength))
				continue;

			const size_t length = test_match(input, where, limit, 5);

			if (length > bestLength) {
				lazySteps = 1;
				bestDistance = input - where;
				bestLength = length;

				if (length >= compressorOptions.standardNiceLength) {
					while (!dictEntry5.ended())
						dictEntry5.next(pos);
					break;
				}
			}
		}
		return;
	}

	//Same principle as the last function, but with additional heuristics to improve ratio
	template<class Pointer>
	FORCE_INLINE void skanda_lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit,
		LzCacheTable<Pointer, FastIntHash>& lzdict4, LzCacheTable<Pointer, FastIntHash>& lzdict6, size_t& bestLength,
		size_t& bestDistance, size_t& repOffsetA, size_t& repOffsetB, size_t& lazySteps, size_t& testedPositions,
		const CompressorOptions compressorOptions) {

		bestLength = 0;
		size_t bestMatchCost = 0;

		for (testedPositions = 0; testedPositions < 2; testedPositions++, input++) {
			LzCacheBucket<Pointer> chain4 = lzdict4[readHash4(input)];
			LzCacheBucket<Pointer> chain6 = lzdict6[readHash6(input)];

			//First try to find a match in any of the rep offsets. If it is found simply take it
			size_t length = test_match(input, input - repOffsetB, limit, 3);
			size_t matchCost = 1 + testedPositions;
			if (length) {
				bestDistance = repOffsetB;
				bestLength = length;
				chain4.push(input - inputStart);
				chain6.push(input - inputStart);
				lazySteps = testedPositions;
				testedPositions++;
				return;
			}

			length = test_match(input, input - repOffsetA, limit, 3);
			if (length) {
				bestDistance = repOffsetA;
				bestLength = length;
				chain4.push(input - inputStart);
				chain6.push(input - inputStart);
				lazySteps = testedPositions;
				testedPositions++;
				return;
			}

			//If no rep offset was found try to get a length 6 match
			size_t pos = input - inputStart;
			while (!chain6.ended()) {
				chain6.next(pos);

				const uint8_t* where = inputStart + pos;

				if (*(input + bestLength) != *(where + bestLength))
					continue;

				length = test_match(input, where, limit, 6);

				size_t distance = input - where;
				matchCost = 2 + testedPositions + int_log2(distance) / 7;
				if (length + bestMatchCost > matchCost + bestLength) {
					bestDistance = distance;
					bestLength = length;
					bestMatchCost = matchCost;
					lazySteps = testedPositions;

					if (bestLength >= compressorOptions.standardNiceLength) {
						while (!chain6.ended())
							chain6.next(pos);
						chain4.push(input - inputStart);
						testedPositions++;
						return;
					}
				}
			}

			//If still nothing was found, try a length 4
			if (bestLength < 6) {

				pos = input - inputStart;
				while (!chain4.ended()) {
					chain4.next(pos);

					const uint8_t* where = inputStart + pos;

					if (*(input + bestLength) != *(where + bestLength))
						continue;

					size_t length = test_match(input, where, limit, 4);

					size_t distance = input - where;
					matchCost = 2 + testedPositions + int_log2(distance) / 7;
					if (length + bestMatchCost > matchCost + bestLength) {
						bestDistance = distance;
						bestLength = length;
						bestMatchCost = matchCost;
						lazySteps = testedPositions;

						if (bestLength >= 5) {
							if (bestLength >= compressorOptions.standardNiceLength) {
								while (!chain4.ended())
									chain4.next(pos);
								//chain6 has already been updated
								testedPositions++;
								return;
							}
							break;
						}
					}
				}

				//No match found, code a literal and retry
				if (bestLength < 3) {
					testedPositions++;
					lazySteps = 1;
					return;
				}
			}
			else {
				chain4.push(input - inputStart);
			}
		}
	}

	//A position in the optimal parse. The struct is optimized to take only 16 bytes on 32 bit mode (24 on 64 bit)
	template<class Pointer>
	struct SkandaOptimalParserState {
		//16 high bits store size cost, 16 low bits the speed cost
		//This way we can also compare both costs prioritising size cost with a single operation
		uint32_t cost;
		//Two rep distances. A match distance can also be retrieved from here:
		//if the match is normal, or last rep it will be [0], if it is second
		//rep it will be [1]. We can store the decision with 1 bit.
		Pointer distances[2];
		uint16_t literalRunLength;
		uint8_t matchLength;  //As long as we put nice length below 256, an 8 bit number is enough
		uint8_t other;  //0byxxxxxxx, where y is which distance to take, x is the last path

		void set_distance_path(const bool isSecondRep, const size_t path) {
			other = (isSecondRep << 7) | path;
		}
		size_t get_path() const {
			return other & 0x7F;
		}
		size_t get_distance() const {
			return distances[other >> 7];
		}
	};

	template<class Pointer>
	LZ_Structure<Pointer>* skanda_forward_optimal_parse(const uint8_t* const input, const uint8_t* const inputStart,
		const uint8_t* const limit, HashTableMatchFinder<Pointer>& matchFinder, SkandaOptimalParserState<Pointer>* parser,
		LZ_Structure<Pointer>* stream, const size_t repOffsetA, const size_t repOffsetB, const CompressorOptions compressorOptions) {

		const size_t blockLength = std::min((size_t)(limit - input), compressorOptions.optimalBlockSize);
		for (size_t i = 1; i <= blockLength; i++)
			parser[i].cost = UINT32_MAX;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;

		parser[0].cost = 0;
		parser[0].distances[1] = repOffsetA;
		parser[0].distances[0] = repOffsetB;
		parser[0].literalRunLength = 0;

		size_t position = 0;
		for (; position < blockLength; position++) {

			const uint8_t* const inputPosition = input + position;
			SkandaOptimalParserState<Pointer>* const parserPosition = parser + position;

			const size_t literalCost = parserPosition->cost + (0x10000 << (parserPosition->literalRunLength == 6));  //only size cost
			SkandaOptimalParserState<Pointer>* nextPosition = parserPosition + 1;
			if (literalCost < nextPosition->cost) {
				nextPosition->cost = literalCost;
				memcpy(nextPosition->distances, parserPosition->distances, sizeof(Pointer) * 2);
				nextPosition->literalRunLength = parserPosition->literalRunLength + 1;
			}

			//First try to find a match at the last offset
			size_t repMatchLength = test_match(inputPosition, inputPosition - parserPosition->distances[0], limit, 2);
			size_t repDistance = 0;
			//Only try to find a match at the second last offset if there was not at the first
			if (!repMatchLength) {
				repMatchLength = test_match(inputPosition, inputPosition - parserPosition->distances[1], limit, 2);
				repDistance = 1;
			}

			if (repMatchLength) {
				if (position + repMatchLength >= blockLength || repMatchLength >= compressorOptions.repNiceLength) {
					lastMatchLength = repMatchLength;
					lastMatchDistance = parserPosition->distances[repDistance];
					break;
				}

				size_t repMatchCost = parserPosition->cost;
				repMatchCost += 0x10000 << (repMatchLength > 8);  //size cost
				repMatchCost += 1;  //speed cost

				nextPosition = parserPosition + repMatchLength;
				if (repMatchCost < nextPosition->cost) {
					nextPosition->cost = repMatchCost;
					nextPosition->matchLength = repMatchLength;
					nextPosition->set_distance_path(repDistance, 0);
					memcpy(nextPosition->distances, parserPosition->distances, sizeof(Pointer) * 2);
					nextPosition->literalRunLength = 0;
				}
			}

			LZ_Match<Pointer> matches[16];
			const LZ_Match<Pointer>* matchesEnd = matchFinder.find_matches_and_update(inputPosition, inputStart,
				limit, matches, repMatchLength, compressorOptions);

			//At least one match was found
			if (matchesEnd != matches) {
				//The last match should be the longest
				const LZ_Match<Pointer>* const longestMatch = matchesEnd - 1;
				//Match goes outside the buffer or is very long
				if (position + longestMatch->length >= blockLength || longestMatch->length >= compressorOptions.standardNiceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					break;
				}

				for (const LZ_Match<Pointer>* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					if (matchIt->distance == parserPosition->distances[0] || matchIt->distance == parserPosition->distances[1])
						continue;

					size_t matchCost = parserPosition->cost;
					matchCost += (2 + int_log2(matchIt->distance - 1) / 7 + (matchIt->length > 17)) << 16; //size cost
					matchCost += 1; //speed cost

					nextPosition = parserPosition + matchIt->length;
					if (matchCost < nextPosition->cost) {
						nextPosition->cost = matchCost;
						nextPosition->matchLength = matchIt->length;
						nextPosition->set_distance_path(0, 0);
						nextPosition->distances[0] = matchIt->distance;
						nextPosition->distances[1] = parserPosition->distances[0];
						nextPosition->literalRunLength = 0;
					}
				}
			}
		}

		// Backward pass, pick best option at each step.
		const SkandaOptimalParserState<Pointer>* backwardParse = parser + position;
		const SkandaOptimalParserState<Pointer>* const parseEnd = parser;

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

			stream->matchDistance = backwardParse->get_distance();
			stream->matchLength = backwardParse->matchLength;
			backwardParse -= backwardParse->matchLength;
			stream->literalRunLength = backwardParse->literalRunLength;
			backwardParse -= backwardParse->literalRunLength;
			stream++;
		}

		return stream - 1;
	}

	template<class Pointer>
	LZ_Structure<Pointer>* skanda_multi_arrivals_parse(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit,
		BinaryMatchFinder<Pointer>& matchFinder, SkandaOptimalParserState<Pointer>* parser, LZ_Structure<Pointer>* stream,
		const size_t repOffsetA, const size_t repOffsetB, const CompressorOptions compressorOptions) {

		const size_t blockLength = std::min((size_t)(limit - input), compressorOptions.optimalBlockSize - 1);
		for (size_t i = 1; i <= blockLength * compressorOptions.maxArrivals; i++)
			parser[i].cost = UINT32_MAX;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;
		size_t lastPath;

		for (size_t i = 0; i < compressorOptions.maxArrivals; i++) {
			parser[i].cost = 0;
			parser[i].distances[1] = repOffsetA;
			parser[i].distances[0] = repOffsetB;
			parser[i].literalRunLength = 0;
		}

		size_t position = 0;
		for (; position < blockLength; position++) {

			const uint8_t* inputPosition = input + position;
			SkandaOptimalParserState<Pointer>* const parserPosition = parser + position * compressorOptions.maxArrivals;

			size_t acceptableRepMatchLength = 2;  //Only take rep matches as long as this

			//Literal and rep match parsing can be done at the same time
			for (size_t i = 0; i < compressorOptions.maxArrivals; i++) {

				SkandaOptimalParserState<Pointer>* const currentArrival = parserPosition + i;

				const size_t literalCost = currentArrival->cost + (0x10000 << (currentArrival->literalRunLength == 6)); //only size cost

				SkandaOptimalParserState<Pointer>* arrivalIt = parserPosition + compressorOptions.maxArrivals;
				SkandaOptimalParserState<Pointer>* lastArrival = arrivalIt + compressorOptions.maxArrivals;

				for (; arrivalIt < lastArrival; arrivalIt++) {

					if (literalCost < arrivalIt->cost) {

						for (SkandaOptimalParserState<Pointer>* it = lastArrival - 1; it != arrivalIt; it--)
							memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState<Pointer>));

						arrivalIt->cost = literalCost;
						arrivalIt->matchLength = 1;
						memcpy(arrivalIt->distances, currentArrival->distances, sizeof(Pointer) * 2);
						arrivalIt->literalRunLength = currentArrival->literalRunLength + 1;
						arrivalIt->set_distance_path(0, i);
						break;
					}
				}

				//First try finding a match at last offset
				size_t repMatchLength = test_match(inputPosition, inputPosition - currentArrival->distances[0], limit, 2);
				//Then try second last offset
				size_t secondRepMatchLength = test_match(inputPosition, inputPosition - currentArrival->distances[1], limit, 2);
				//Take the longest of the two
				const size_t repDistance = secondRepMatchLength > repMatchLength;
				repMatchLength = std::max(repMatchLength, secondRepMatchLength);

				//Since we go from lowest cost arrival to highest, it makes sense that the rep match 
				// should be at least as long as the best found so far
				if (repMatchLength >= acceptableRepMatchLength) {
					if (position + repMatchLength >= blockLength || repMatchLength >= compressorOptions.repNiceLength) {
						lastMatchLength = repMatchLength;
						lastMatchDistance = currentArrival->distances[repDistance];
						lastPath = i;
						goto doBackwardParse;
					}

					acceptableRepMatchLength = repMatchLength;
					//Small heuristic: instead of testing all positions, only test the maximum match length, 
					// and if it overflows, just before the overflow
					//There is a notable speed increase for a negligible size penalty
					size_t repMatchCost = currentArrival->cost;
					repMatchCost += 0x10000 << (repMatchLength > 8);  //size cost
					repMatchCost += 1;  //speed cost

					arrivalIt = parserPosition + repMatchLength * compressorOptions.maxArrivals;
					lastArrival = arrivalIt + compressorOptions.maxArrivals;

					for (; arrivalIt < lastArrival; arrivalIt++) {

						if (repMatchCost < arrivalIt->cost) {

							for (SkandaOptimalParserState<Pointer>* it = lastArrival - 1; it != arrivalIt; it--)
								memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState<Pointer>));

							arrivalIt->cost = repMatchCost;
							arrivalIt->matchLength = repMatchLength;
							arrivalIt->set_distance_path(repDistance, i);
							memcpy(arrivalIt->distances, currentArrival->distances, sizeof(Pointer) * 2);
							arrivalIt->literalRunLength = 0;

							//Only try a length of 8 if the full length actually resulted in a better arrival
							if (repMatchLength > 8) {

								repMatchCost -= 0x10000;  //Remove the cost of the additional length byte

								arrivalIt = parserPosition + 8 * compressorOptions.maxArrivals;
								lastArrival = arrivalIt + compressorOptions.maxArrivals;

								for (; arrivalIt < lastArrival; arrivalIt++) {

									if (repMatchCost < arrivalIt->cost) {

										for (SkandaOptimalParserState<Pointer>* it = lastArrival - 1; it != arrivalIt; it--)
											memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState<Pointer>));

										arrivalIt->cost = repMatchCost;
										arrivalIt->matchLength = 8;
										arrivalIt->set_distance_path(repDistance, i);
										memcpy(arrivalIt->distances, currentArrival->distances, sizeof(Pointer) * 2);
										arrivalIt->literalRunLength = 0;
										break;
									}
								}
							}
							break;
						}
					}
				}
			}

			LZ_Match<Pointer> matches[144];
			LZ_Match<Pointer>* matchesEnd = matchFinder.find_matches_and_update(inputPosition, inputStart, limit, matches,
				std::max((size_t)2, acceptableRepMatchLength - 1), compressorOptions);

			if (matchesEnd != matches) {
				const LZ_Match<Pointer>* const longestMatch = matchesEnd - 1;
				if (position + longestMatch->length >= blockLength || longestMatch->length >= compressorOptions.standardNiceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					lastPath = 0;
					break;
				}

				size_t length = 0;

				for (LZ_Match<Pointer>* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					if (matchIt->distance == parserPosition->distances[0] || matchIt->distance == parserPosition->distances[1])
						continue;

					size_t matchCost = parserPosition->cost;
					matchCost += (2 + int_log2(matchIt->distance - 1) / 7) << 16;  //size cost
					matchCost += 1;  //speed cost

					do {
						//If the current match has a length over 17 (it overflows length coding), and we have not
						//tried a match length 17 in any match before, try it.
						if (length < 17 && matchIt->length > 17)
							length = 17;
						else
							length = matchIt->length;

						//Remember we can check the same match with 2 different lengths, and most of
						// the cost calculations only need to be done once
						matchCost += (length > 17) << 16;

						SkandaOptimalParserState<Pointer>* arrivalIt = parserPosition + length * compressorOptions.maxArrivals;
						SkandaOptimalParserState<Pointer>* const lastArrival = arrivalIt + compressorOptions.maxArrivals;

						for (; arrivalIt < lastArrival; arrivalIt++) {

							if (matchCost < arrivalIt->cost) {

								for (SkandaOptimalParserState<Pointer>* it = lastArrival - 1; it != arrivalIt; it--)
									memcpy(&it[0], &it[-1], sizeof(SkandaOptimalParserState<Pointer>));

								arrivalIt->cost = matchCost;
								arrivalIt->matchLength = length;
								arrivalIt->set_distance_path(0, 0);
								arrivalIt->distances[0] = matchIt->distance;
								arrivalIt->distances[1] = parserPosition->distances[0];
								arrivalIt->literalRunLength = 0;
								break;
							}
						}

					} while (length != matchIt->length);
				}
			}
		}

	doBackwardParse:

		// Backward pass, pick best option at each step.
		const SkandaOptimalParserState<Pointer>* backwardParse = parser + position * compressorOptions.maxArrivals;
		const SkandaOptimalParserState<Pointer>* const parseEnd = parser;
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
				matchFinder.update_position(inputPosition, inputStart, limit, compressorOptions);
		}
		else {
			stream->literalRunLength = 0;
			stream++;
			path = backwardParse[path].get_path();
		}
		while (backwardParse > parseEnd) {
			if (backwardParse[path].matchLength > 1) {
				stream->matchDistance = backwardParse[path].get_distance();
				stream->matchLength = backwardParse[path].matchLength;
				stream->literalRunLength = 0;
				path = backwardParse[path].get_path();
				backwardParse -= stream->matchLength * compressorOptions.maxArrivals;
				stream++;
			}
			else {
				(stream - 1)->literalRunLength++;
				path = backwardParse[path].get_path();
				backwardParse -= compressorOptions.maxArrivals;
			}
		}

		return stream - 1;
	}

	template<class Pointer>  //use uint32_t when size < 4gb, and uint64_t for the rest
	size_t skanda_compress_generic(const uint8_t* input, const size_t size, uint8_t* output,
		const CompressorOptions compressorOptions, ProgressCallback* progress) {

		//Last 31 bytes are always uncompressed
		if (size <= SKANDA_LAST_BYTES + 1) {
			memcpy(output, input, size);
			progress->progress(size);
			return size;
		}

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		//The last match must end before this limit
		const uint8_t* const compressionLimit = input + size - SKANDA_LAST_BYTES;
		*output++ = *input++;

		const uint8_t* lastProgressReport;

		//Data for the format
		size_t repOffsetA = 1;
		size_t repOffsetB = 1;
		//Skip first byte
		const uint8_t* literalRunStart = input;

		// * / * // * / * // * / * // * / * // * / * // * / * //
		//                 GREEDY PARSER                      //
		// * / * // * / * // * / * // * / * // * / * // * / * //
		if (compressorOptions.parserFunction == GREEDY_FAST || compressorOptions.parserFunction == GREEDY_NORMAL) {
			//Hash table for match finder. If the input is not very big we can reduce its size
			const size_t log2size = std::min(int_log2(size) - 3, compressorOptions.maxHashTableSize);
			HashTable<Pointer, FastIntHash> lzdict;
			try {
				lzdict.init(log2size);
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
					Pointer* dictEntry = &lzdict[readHash5(input)];
					size_t matchLength = test_match(input, inputStart + *dictEntry, compressionLimit, 5);

					//Have we found a match?
					if (matchLength) {
						//First output the literal run
						uint8_t* const controlByte = output++;
						skanda_encode_literal_run(input, literalRunStart, controlByte, output);

						//Update hash table. Since we already have the hash bucket for the first byte, reuse it
						const size_t pos = input - inputStart;
						size_t distance = pos - *dictEntry;
						*dictEntry = pos;

						//The greedy fast only updates the table in the first and last byte of a match
						if (compressorOptions.parserFunction == GREEDY_FAST) {
							input += matchLength;
							const uint8_t* const lastMatchByte = input - 1;
							lzdict[readHash5(lastMatchByte)] = lastMatchByte - inputStart;
						}
						else {
							const uint8_t* const matchEnd = input + matchLength;
							for (input++; input != matchEnd; input++)
								lzdict[readHash5(input)] = input - inputStart;
						}

						literalRunStart = input;
						//Output the match
						skanda_encode_match(output, controlByte, matchLength, distance, repOffsetA, repOffsetB);
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
		}
		// * / * // * / * // * / * // * / * // * / * // * / * //
		//                   LAZY PARSER                      //
		// * / * // * / * // * / * // * / * // * / * // * / * //
		else if (compressorOptions.parserFunction == LAZY_NORMAL || compressorOptions.parserFunction == LAZY_EXTRA)
		{
			const size_t log2size = std::min(int_log2(size) - 3, compressorOptions.maxHashTableSize);
			LzCacheTable<Pointer, FastIntHash> lzdict5;
			LzCacheTable<Pointer, FastIntHash> lzdict4;
			LzCacheTable<Pointer, FastIntHash> lzdict6;

			try {
				if (compressorOptions.parserFunction == LAZY_EXTRA) {
					lzdict4.init(log2size, compressorOptions.maxElementsPerHash);
					lzdict6.init(log2size, compressorOptions.maxElementsPerHash);
				}
				else
					lzdict5.init(log2size, compressorOptions.maxElementsPerHash);
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

					if (compressorOptions.parserFunction == LAZY_EXTRA)
						skanda_lazy_search(input, inputStart, compressionLimit, lzdict4, lzdict6, matchLength,
							distance, repOffsetA, repOffsetB, lazySteps, testedPositions, compressorOptions);
					else
						skanda_fast_lazy_search(input, inputStart, compressionLimit, lzdict5, matchLength,
							distance, lazySteps, testedPositions, compressorOptions);

					input += lazySteps;

					if (matchLength) {

						uint8_t* const controlByte = output++;
						skanda_encode_literal_run(input, literalRunStart, controlByte, output);

						const uint8_t* const matchEnd = input + matchLength;
						input += testedPositions - lazySteps;
						if (compressorOptions.parserFunction == LAZY_EXTRA) {
							for (; input != matchEnd; input++) {
								const size_t ptr = input - inputStart;
								lzdict4[readHash4(input)].push(ptr);
								lzdict6[readHash6(input)].push(ptr);
							}
						}
						else {
							for (; input != matchEnd; input++)
								lzdict5[readHash5(input)].push(input - inputStart);
						}

						literalRunStart = input;
						skanda_encode_match(output, controlByte, matchLength, distance, repOffsetA, repOffsetB);
					}
				}

				if (progress->abort())
					return 0;
				progress->progress(input - lastProgressReport);
			}
		}
		// * / * // * / * // * / * // * / * // * / * // * / * //
		//                  OPTIMAL PARSER                    //
		// * / * // * / * // * / * // * / * // * / * // * / * //
		else if (compressorOptions.parserFunction == OPTIMAL_FAST || compressorOptions.parserFunction == OPTIMAL)
		{
			HashTableMatchFinder<Pointer> hashMatchFinder;
			BinaryMatchFinder<Pointer> binaryMatchFinder;
			SkandaOptimalParserState<Pointer>* parser = nullptr;
			LZ_Structure<Pointer>* stream = nullptr;

			try {
				if (compressorOptions.parserFunction == OPTIMAL_FAST) {
					hashMatchFinder.init(size, compressorOptions);
				}
				else {
					binaryMatchFinder.init(size, compressorOptions, false);
				}
				parser = new SkandaOptimalParserState<Pointer>[(compressorOptions.optimalBlockSize + 1) * compressorOptions.maxArrivals];
				stream = new LZ_Structure<Pointer>[compressorOptions.optimalBlockSize];
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

					LZ_Structure<Pointer>* streamIt;
					if (compressorOptions.parserFunction == OPTIMAL) {
						streamIt = skanda_multi_arrivals_parse(input, inputStart, compressionLimit, binaryMatchFinder,
							parser, stream, repOffsetA, repOffsetB, compressorOptions);
					}
					else {
						streamIt = skanda_forward_optimal_parse(input, inputStart, compressionLimit, hashMatchFinder,
							parser, stream, repOffsetA, repOffsetB, compressorOptions);
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
						skanda_encode_match(output, controlByte, matchLength, distance, repOffsetA, repOffsetB);
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
		}

		//Output the last literal run. This is necessary even when no more literals remain, as the decoder expects to end with this
		uint8_t* const controlByte = output++;
		skanda_encode_literal_run(input, literalRunStart, controlByte, output);

		memcpy(output, input, SKANDA_LAST_BYTES);
		//Account here for the first byte of input
		progress->progress(SKANDA_LAST_BYTES + 1);

		return output - outputStart + SKANDA_LAST_BYTES;
	}

	const CompressorOptions skandaCompressorLevels[] = {
		//      Parser        Hash log     Elements per hash     Nice length     Rep nice length     Max arrivals     Block size
			{ GREEDY_FAST  ,     14     ,          0          ,       0       ,         0         ,        0       ,      0       },
			{ GREEDY_NORMAL,     17     ,          0          ,       0       ,         0         ,        0       ,      0       },
			{ LAZY_NORMAL  ,     17     ,          1          ,       16      ,         0         ,        0       ,      0       },
			{ LAZY_EXTRA   ,     17     ,          1          ,       16      ,         0         ,        0       ,      0       },
			{ LAZY_EXTRA   ,     18     ,          2          ,       24      ,         0         ,        0       ,      0       },
			{ OPTIMAL_FAST ,     18     ,          2          ,       24      ,         8         ,        1       ,      1024    },
			{ OPTIMAL_FAST ,     19     ,          3          ,       32      ,         12        ,        1       ,      1024    },
			{ OPTIMAL      ,     22     ,          4          ,       32      ,         16        ,        2       ,      2048    },
			{ OPTIMAL      ,     24     ,          5          ,       80      ,         40        ,        4       ,      2048    },
			{ OPTIMAL      ,     26     ,          6          ,       256     ,         128       ,        8       ,      4096    },
	};

	size_t skanda_compress(const uint8_t* input, const size_t size, uint8_t* output, const uint8_t level, ProgressCallback* progress) {

		if (level > 9)
			return 0;

		ProgressCallback defaultProgress;
		if (!progress)
			progress = &defaultProgress;

#ifdef COMPRESS_LARGE_BLOCK
		if (size > ((uint64_t)1 << 32))
			return skanda_compress_generic<uint64_t>(input, size, output, skandaCompressorLevels[level], progress);
#endif
		return skanda_compress_generic<uint32_t>(input, size, output, skandaCompressorLevels[level], progress);
	}

	size_t skanda_compress_bound(const size_t size) {
		return size + size / 128 + 4;
	}

	template<class Pointer>
	size_t skanda_memory_estimator(const size_t size, const uint8_t level) {
		if (skandaCompressorLevels[level].parserFunction < LAZY_NORMAL)
			return sizeof(Pointer) << std::min(int_log2(size) - 3, skandaCompressorLevels[level].maxHashTableSize);
		if (skandaCompressorLevels[level].parserFunction < OPTIMAL_FAST)
			//Lazy extra uses 2 tables
			return sizeof(Pointer) << std::min(int_log2(size) - 3, skandaCompressorLevels[level].maxHashTableSize)
			<< skandaCompressorLevels[level].maxElementsPerHash << (skandaCompressorLevels[level].parserFunction == LAZY_EXTRA);
		if (skandaCompressorLevels[level].parserFunction < OPTIMAL) {
			const size_t log2size = std::min(int_log2(size) - 3, skandaCompressorLevels[level].maxHashTableSize);
			size_t memory = sizeof(Pointer) << std::min(log2size, (size_t)14);  //hash 3 table
			memory += sizeof(Pointer) << log2size << skandaCompressorLevels[level].maxElementsPerHash << 1;  //hash 4 and hash8 tables
			memory += sizeof(SkandaOptimalParserState<Pointer>) * skandaCompressorLevels[level].optimalBlockSize + 1;
			memory += sizeof(LZ_Structure<Pointer>) * skandaCompressorLevels[level].optimalBlockSize;
			return memory;
		}
		const size_t binaryTreeSize = std::min(size, (size_t)1 << skandaCompressorLevels[level].maxHashTableSize);
		size_t memory = sizeof(Pointer) * 2 * binaryTreeSize; //binary tree
		memory += sizeof(Pointer) << std::min(int_log2(size) - 3, skandaCompressorLevels[level].maxHashTableSize - 3);  //binary node lookup
		memory += sizeof(Pointer) << std::min(int_log2(size) - 3, (size_t)16);  //hash 3 table
		if (size >= binaryTreeSize)
			memory += sizeof(Pointer) << std::max(4, (int)int_log2(size - binaryTreeSize) - 4) <<
			(skandaCompressorLevels[level].maxElementsPerHash - 4);  //extra hash 12 table
		memory += sizeof(SkandaOptimalParserState<Pointer>) * (skandaCompressorLevels[level].optimalBlockSize + 1) * skandaCompressorLevels[level].maxArrivals;
		memory += sizeof(LZ_Structure<Pointer>) * skandaCompressorLevels[level].optimalBlockSize;
		return memory;
	}

	size_t skanda_estimate_memory(const size_t size, const uint8_t level) {
		if (size <= SKANDA_LAST_BYTES + 1)
			return 0;

#ifdef COMPRESS_LARGE_BLOCK
		if (size > ((uint64_t)1 << 32))
			return skanda_memory_estimator<uint64_t>(size, level);
#endif
		return skanda_memory_estimator<uint32_t>(size, level);
	}

	int skanda_decompress(const uint8_t* compressed, const size_t compressedSize, uint8_t* decompressed,
		const size_t uncompressedSize, ProgressCallback* progress) {

		ProgressCallback defaultProgress;
		if (!progress)
			progress = &defaultProgress;

		//Only last bytes are present
		if (uncompressedSize <= SKANDA_LAST_BYTES + 1) {
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
		*decompressed++ = *compressed++;
		const uint8_t* nextProgressReport = (decompressedEnd - decompressed < SKANDA_PROGRESS_REPORT_PERIOD) ?
			decompressedEnd : decompressed + SKANDA_PROGRESS_REPORT_PERIOD;
		const uint8_t* lastProgressReport = decompressed;

		size_t distance;
		size_t repOffsetA = 1;
		size_t repOffsetB = 1;
		size_t matchLength;
		size_t literalRunLength;

		while (true) {

			const uint8_t token = *compressed++;
			literalRunLength = token >> 5;

			if (literalRunLength == 7) {
				//Worst case: we read 10 bytes for distance, 11 for match length,
				// 1 for token and 11 for literal run length here, total 33 bytes
				//An additional check is necessary
				if (compressed > compressedEnd)
					return -1;

				decode_length(compressed, literalRunLength, 7);
				//We dont have the guarantee that compressedEnd >= compressed
				if (compressed > compressedEnd || decompressed > decompressedEnd ||
					compressedEnd - compressed < literalRunLength ||
					decompressedEnd - decompressed < literalRunLength)
					return -1;

				copy_literal_run(compressed, decompressed, literalRunLength);
			}
			else {
				//If the cpu supports fast unaligned access, it is faster to copy a fixed amount of bytes instead of the needed amount
				memcpy(decompressed, compressed, 8);
				decompressed += literalRunLength;
			}

			compressed += literalRunLength;

			if (decompressed >= nextProgressReport) {

				if (progress->abort())
					return 0;
				progress->progress(decompressed - lastProgressReport);

				nextProgressReport = (decompressedEnd - decompressed < SKANDA_PROGRESS_REPORT_PERIOD) ?
					decompressedEnd : decompressed + SKANDA_PROGRESS_REPORT_PERIOD;
				lastProgressReport = decompressed;

				if (decompressed >= decompressedEnd) {
					//The maximum number of bytes that can be written in a single iteration
					//(assuming no overflows) is 17 from match and 8(because we write more than needed) from literal run.
					//As it is less than the end buffer, this check should be enough.
					if (decompressed > decompressedEnd || compressed > compressedEnd)
						return -1;
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

			//The first bit tells whether to use a last offset
			if (token & 0x10) {
				//The second bit tells which last offset to use
				distance = token & 0x8 ? repOffsetB : repOffsetA;
				//The remaining 3 bits tell the match length
				matchLength = token & 0x7;

				if (matchLength == 7) {
					decode_length(compressed, matchLength, 7);
					//We have the guarantee that decompressed < decompressedEnd, so there is no overflow
					if (decompressedEnd - decompressed < matchLength)
						return -1;

					matchLength += SKANDA_REP_MIN_MATCH_LENGTH;
					copy_match(decompressed, distance, matchLength);
				}
				else {
					//Match length is at most 8 bytes
					matchLength += SKANDA_REP_MIN_MATCH_LENGTH;
					const uint8_t* src = decompressed - distance;

					//If the offset is big enough we can perform a faster copy
					if (distance >= 8)
						memcpy(decompressed, src, 8);
					//Else it is a run-length type match
					else {
						//Change the offset to at least 2, so that we can copy 2 bytes at a time
						decompressed[0] = src[0];
						src += distance > 1;

						//Now the offset is at least 4
						memcpy(decompressed + 1, src, 2);
						src += ((size_t)(distance >= 4) << 1) - (distance == 3);

						memcpy(decompressed + 3, src, 4);
						decompressed[7] = src[4];
					}
					decompressed += matchLength;
				}
			}
			else {
				matchLength = token & 0xF;
				decode_prefixVarInt(compressed, distance);
				distance++;

				//This can only happen with a new distance. decompressed > decompressedStart always, so no overflow
				if (decompressed - decompressedStart < distance)
					return -1;

				repOffsetA = repOffsetB;
				repOffsetB = distance;

				if (matchLength == 15) {
					decode_length(compressed, matchLength, 15);
					//We have the guarantee that decompressed < decompressedEnd, so there is no overflow
					if (decompressedEnd - decompressed < matchLength)
						return -1;

					matchLength += SKANDA_NORMAL_MIN_MATCH_LENGTH;
					copy_match(decompressed, distance, matchLength);
				}
				else {
					//Match length is at most 17 bytes
					matchLength += SKANDA_NORMAL_MIN_MATCH_LENGTH;
					const uint8_t* src = decompressed - distance;

					//If the offset is big enough we can perform a faster copy
					if (distance >= 8) {

						memcpy(decompressed, src, 8);
						memcpy(decompressed + 8, src + 8, 8);
						decompressed[16] = src[16];
					}
					//Else it is a run-length type match
					else {
						//Change the offset to at least 2, so that we can copy 2 bytes at a time
						decompressed[0] = src[0];
						src += distance > 1;

						//Now the offset is at least 4
						memcpy(decompressed + 1, src, 2);
						src += ((size_t)(distance >= 4) << 1) - (distance == 3);

						memcpy(decompressed + 3, src, 4);
						memcpy(decompressed + 7, src + 4, 4);
						memcpy(decompressed + 11, src + 8, 4);
						memcpy(decompressed + 15, src + 12, 2);
					}
					decompressed += matchLength;
				}
			}
		}

		memcpy(decompressed, compressed, SKANDA_LAST_BYTES);
		progress->progress(SKANDA_LAST_BYTES + 1);
		return 0;
	}
}

#endif  //SKANDA_IMPLEMENTATION

#endif  //__SKANDA__
