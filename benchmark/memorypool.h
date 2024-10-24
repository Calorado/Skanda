/*
MIT License

Copyright (c) 2023-2024 Carlos de Diego

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

#pragma once

#include <mutex>
#include <cstdint>
#include "lib/parallel_hashmap/btree.h"

#define IS_64BIT (UINTPTR_MAX > UINT32_MAX)

namespace mempool {

    //Undefined behaviour when value == 0
    size_t unsafe_bit_scan_forward(const size_t value) {
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

    //Very simple memory pool with fixed size blocks. allocate() and free() are O(1).
    //Memory overhead is 2 bits per block.
    class FixedMemoryPool
    {
        //To reduce memory use group blocks into chunks. Each chunk has a bitmap
        // which shows which blocks in it are free. We can quickly find a free block
        // in that bitmap using a bit scan. Lastly, there is a stack which keeps
        // track of which chunks have at least one block free.
#define CHUNK_SIZE (sizeof(size_t) * 8)
        std::mutex mtx;
        uint8_t* data = nullptr;
        size_t* nonFullChunks = nullptr;
        size_t* chunkBitmaps = nullptr;
        size_t nonFullCount = 0;
        size_t numberBlocks = 0;
        size_t blockSize = 1;
        bool newFallback = false;
        bool threadSafe = true;

    public:

        FixedMemoryPool() {}
        FixedMemoryPool(size_t _blockSize, size_t _numberBlocks, bool _newFallback = false, bool _threadSafe = true) {
            reinit(_blockSize, _numberBlocks, _newFallback, _threadSafe);
        }
        ~FixedMemoryPool() {
            delete[] data;
            delete[] nonFullChunks;
            delete[] chunkBitmaps;
        }

        void reinit(size_t _blockSize, size_t _numberBlocks, bool _newFallback = false, bool _threadSafe = true)
        {
            delete[] data;
            delete[] nonFullChunks;
            delete[] chunkBitmaps;

            size_t numberChunks = (_numberBlocks + CHUNK_SIZE - 1) / CHUNK_SIZE;
            data = new uint8_t[_blockSize * _numberBlocks];
            nonFullChunks = new size_t[numberChunks];
            chunkBitmaps = new size_t[numberChunks];

            blockSize = _blockSize;
            numberBlocks = _numberBlocks;
            nonFullCount = numberChunks;
            newFallback = _newFallback;
            threadSafe = _threadSafe;

            for (size_t i = 0; i < numberChunks; i++)
                nonFullChunks[i] = i;
            for (size_t i = 0; i < numberChunks; i++)
                chunkBitmaps[i] = (size_t)-1;
            //If we dont have a multiple of CHUNK_SIZE mark some blocks in last chunk as used
            if (numberBlocks % CHUNK_SIZE)
                chunkBitmaps[numberChunks - 1] >>= (CHUNK_SIZE - numberBlocks % CHUNK_SIZE);
        }
        void* allocate()
        {
            if (threadSafe)
                mtx.lock();

            if (nonFullCount == 0) {
                if (threadSafe)
                    mtx.unlock();
                if (!newFallback)
                    throw std::bad_alloc();
                return new uint8_t[blockSize];
            }

            size_t chunkIndex = nonFullChunks[nonFullCount - 1];
            size_t bitmap = chunkBitmaps[chunkIndex];

            size_t freeBlock = unsafe_bit_scan_forward(bitmap);
            bitmap ^= (size_t)1 << freeBlock;

            chunkBitmaps[chunkIndex] = bitmap;
            nonFullCount -= (bitmap == 0);

            if (threadSafe)
                mtx.unlock();

            uint8_t* ptr = data + blockSize * (chunkIndex * CHUNK_SIZE + freeBlock);
            return ptr;
        }
        void free(void* ptr)
        {
            if (ptr == nullptr)
                return;
            size_t index = size_t((uint8_t*)ptr - data) / blockSize;
            //Pointer came from new allocation
            if (index >= numberBlocks) {
                delete[] (uint8_t*)ptr;
                return;
            }

            if (threadSafe)
                mtx.lock();

            size_t chunkIndex = index / CHUNK_SIZE;
            size_t blockIndex = index % CHUNK_SIZE;

            size_t bitmap = chunkBitmaps[chunkIndex];
            //This chunk now has one free block; append to stack
            if (bitmap == 0) {
                nonFullChunks[nonFullCount] = chunkIndex;
                nonFullCount += (bitmap == 0);
            }
            bitmap |= (size_t)1 << blockIndex;
            chunkBitmaps[chunkIndex] = bitmap;

            if (threadSafe)
                mtx.unlock();
        }

        size_t block_size() {
            return blockSize;
        }
    };

    //Used by the binary tree in VariableMemoryPool
    template<typename T>
    class MyPoolAlloc {
    public:

        FixedMemoryPool* leafPool;
        FixedMemoryPool* internalPool;

        typedef size_t     size_type;
        typedef ptrdiff_t  difference_type;
        typedef T* pointer;
        typedef const T* const_pointer;
        typedef T& reference;
        typedef const T& const_reference;
        typedef T          value_type;
        typedef std::true_type propagate_on_container_move_assignment;
        typedef std::true_type is_always_equal;

        template<typename X>
        using rebind = MyPoolAlloc<X>;

        MyPoolAlloc(FixedMemoryPool* _leafPool, FixedMemoryPool* _internalPool) {
            leafPool = _leafPool;
            internalPool = _internalPool;
        }

        MyPoolAlloc() throw() {
            leafPool = nullptr;
            internalPool = nullptr;
        }

        MyPoolAlloc(const MyPoolAlloc& alloc) throw() {
            leafPool = alloc.leafPool;
            internalPool = alloc.internalPool;
        }

        template<typename X>
        MyPoolAlloc(const MyPoolAlloc<X>& alloc) throw() {
            leafPool = alloc.leafPool;
            internalPool = alloc.internalPool;
        }

        ~MyPoolAlloc() throw() {};

        pointer address(reference __x) const { return &__x; }

        const_pointer address(const_reference __x) const { return &__x; }

        pointer allocate(size_type __n, const void* hint = 0) {
            if (__n * sizeof(T) <= leafPool->block_size())
                return reinterpret_cast<T*>(leafPool->allocate());
            return reinterpret_cast<T*>(internalPool->allocate());
        }

        void deallocate(pointer __p, size_type __n) {
            if (__n * sizeof(T) <= leafPool->block_size())
                leafPool->free(reinterpret_cast<uint8_t*>(__p));
            else
                internalPool->free(reinterpret_cast<uint8_t*>(__p));
        }

        size_type max_size() const throw() {
            return SIZE_MAX;
        }

        void construct(pointer __p, const T& __val) {
            ::new(__p) T(__val);
        }

        void destroy(pointer __p) {
            __p->~T();
        }

        template<typename X>
        bool operator==(const MyPoolAlloc<X>& alloc) noexcept {
            return this->pool == alloc.pool;
        }
        template<typename X>
        bool operator!=(const MyPoolAlloc<X>& alloc) noexcept {
            return this->pool != alloc.pool;
        }
    };

    //A memory pool with fixed size blocks, but it can allocate/free any number of contiguous blocks.
    //It requires about 32 bytes of additional space for each block.
    //allocate() and free() are O(log n), with n the number of blocks in the pool.
    class VariableMemoryPool
    {
        std::mutex mtx;
        size_t blockSize = 1;
        size_t numberBlocks = 0;
        uint8_t* data = nullptr;
        bool newFallback = false;
        bool threadSafe = true;

        struct BlockNode {
            size_t isFree : 1;
#if IS_64BIT
            size_t length : 63;
#else
            size_t length : 31;
#endif
            size_t previous;
        };

        struct FreeChunk {
            size_t length;
            size_t index;

            FreeChunk(size_t len, size_t idx) {
                length = len;
                index = idx;
            }
            bool operator<(const FreeChunk& other) const {
                if (this->length == other.length)
                    return this->index < other.index;
                return this->length < other.length;
            }
            //This operator is for lower_bound()
            bool operator<(const size_t len) const {
                return this->length < len;
            }
        };

        FixedMemoryPool bTreeLeafPool;
        FixedMemoryPool bTreeInternalPool;
        MyPoolAlloc<FreeChunk> bTreeAlloc;
        BlockNode* blockNodes = nullptr;
        phmap::btree_set<FreeChunk, std::less<>, MyPoolAlloc<FreeChunk>> freeChunks;

    public:

        VariableMemoryPool()
            : bTreeAlloc(&bTreeLeafPool, &bTreeInternalPool), freeChunks(std::less<>(), bTreeAlloc) {}
        VariableMemoryPool(size_t _blockSize, size_t _numberBlocks, bool _newFallback = false, bool _threadSafe = true)
            : bTreeAlloc(&bTreeLeafPool, &bTreeInternalPool), freeChunks(std::less<>(), bTreeAlloc) {
            reinit(_blockSize, _numberBlocks, _newFallback, _threadSafe);
        }
        ~VariableMemoryPool() {
            delete[] data;
            delete[] blockNodes;
        }

        void reinit(size_t _blockSize, size_t _numberBlocks, bool _newFallback = false, bool _threadSafe = true)
        {
            delete[] data;
            delete[] blockNodes;

            data = new uint8_t[_blockSize * _numberBlocks];
            blockNodes = new BlockNode[_numberBlocks];

            //Maximum ammount of free chunks we can have is ~(numberBlocks / 2), with alternating 
            // used and free blocks, otherwise they would get merged.
            size_t maxFreeChunks = (_numberBlocks + 1) / 2;

            //A leaf node of the binary tree can hold up to 256 bytes of data. This includes space 
            // for 2 qwords or dwords for 64 or 32 bit architectures. So the number of elements per 
            // leaf node is given by kNodeValues = (256 - sizeof(size_t) * 2) / sizeof(ValueType), 
            // with a minimum value of 3. The size it occupies is kNodeValues * sizeof(ValueType) + sizeof(size_t) * 2
            int kNodeValues = (256 - sizeof(size_t) * 2) / sizeof(FreeChunk);
            if (kNodeValues < 3)
                kNodeValues = 3;
            int kLeafNodeSize = kNodeValues * sizeof(FreeChunk) + sizeof(size_t) * 2;
            //A non leaf node can also hold kNodeValues, but instead of 2 qwords/dwords it holds
            // kNodeValues + 3 of them. Each of them can have up to kNodeValues + 1 children.
            int kInternalNodeSize = kNodeValues * sizeof(FreeChunk) + sizeof(size_t) * (kNodeValues + 3);

            //These are approximations, real count is usually much lower 
            // but we want to avoid running out of nodes in the memory pools.
            size_t kMaxLeafCount = maxFreeChunks / ((kNodeValues + 1) / 2) + 1;
            size_t kMaxInternalCount = kMaxLeafCount / ((kNodeValues + 1) / 2) + 1;

            bTreeLeafPool.reinit(kLeafNodeSize, kMaxLeafCount, false, false);
            bTreeInternalPool.reinit(kInternalNodeSize, kMaxInternalCount, false, false);

            blockSize = _blockSize;
            numberBlocks = _numberBlocks;
            newFallback = _newFallback;
            threadSafe = _threadSafe;
            blockNodes[0].isFree = true;
            blockNodes[0].length = _numberBlocks;
            blockNodes[0].previous = 0;
            freeChunks.insert({ _numberBlocks, 0 });
        }

        void* allocate(size_t size)
        {
            if (size == 0)
                return nullptr;
            size_t blocksRequested = (size + blockSize - 1) / blockSize;

            if (threadSafe)
                mtx.lock();

            auto freeChunk = freeChunks.lower_bound(blocksRequested);
            if (freeChunk == freeChunks.end()) {
                if (threadSafe)
                    mtx.unlock();
                if (!newFallback)
                    throw std::bad_alloc();
                return new uint8_t[size];
            }

            size_t blockIndex = freeChunk->index;
            size_t chunkLength = freeChunk->length;
            freeChunks.erase(freeChunk);

            blockNodes[blockIndex].isFree = false;
            blockNodes[blockIndex].length = blocksRequested;

            if (chunkLength > blocksRequested) {
                blockNodes[blockIndex + blocksRequested].isFree = true;
                blockNodes[blockIndex + blocksRequested].length = chunkLength - blocksRequested;
                blockNodes[blockIndex + blocksRequested].previous = blockIndex;
                freeChunks.insert({ chunkLength - blocksRequested, blockIndex + blocksRequested });
            }
            //Update the previous link of the next chunk
            if (blockIndex + chunkLength != numberBlocks) {
                blockNodes[blockIndex + chunkLength].previous =
                    blockIndex + (chunkLength > blocksRequested ? blocksRequested : 0);
            }

            if (threadSafe)
                mtx.unlock();

            return data + blockIndex * blockSize;
        }

        void free(void* ptr)
        {
            if (ptr == nullptr)
                return;

            size_t blockIndex = size_t((uint8_t*)ptr - data) / blockSize;
            //Pointer came from new allocation
            if (blockIndex >= numberBlocks) {
                delete[] (uint8_t*)ptr;
                return;
            }

            if (threadSafe)
                mtx.lock();

            size_t chunkLength = blockNodes[blockIndex].length;

            //Merge with next chunk
            if (blockIndex + chunkLength != numberBlocks) {
                if (blockNodes[blockIndex + chunkLength].isFree) {
                    freeChunks.erase({ blockNodes[blockIndex + chunkLength].length, blockIndex + chunkLength });
                    chunkLength += blockNodes[blockIndex + chunkLength].length;
                }
            }
            //Merge with previous chunk
            if (blockIndex != 0) {
                size_t previousChunk = blockNodes[blockIndex].previous;
                if (blockNodes[previousChunk].isFree) {
                    freeChunks.erase({ blockNodes[previousChunk].length, previousChunk });
                    chunkLength += blockNodes[previousChunk].length;
                    blockIndex = previousChunk;
                }
            }

            //Update the previous link of the next chunk
            if (blockIndex + chunkLength != numberBlocks)
                blockNodes[blockIndex + chunkLength].previous = blockIndex;

            blockNodes[blockIndex].isFree = true;
            blockNodes[blockIndex].length = chunkLength;
            freeChunks.insert({ chunkLength, blockIndex });

            if (threadSafe)
                mtx.unlock();
        }
    };
}