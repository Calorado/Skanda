# Skanda

Skanda is an experimental compression algorithm, trying to improve upon the Lizard algorithm. It usually gets higher ratios, very close to Deflate on larger windows, while keeping 75-80% of the decompression speed of LZ4. 

Note that due to being experimental I cannot guarantee backwards/forwards compatibility, or good support for older or less common compiler/architectures.

# How to use

Simply add the file Skanda.h to your project and create a .cpp file with the following:
```cpp
#define SKANDA_IMPLEMENTATION
#include "Skanda.h"
```
Then simply add the header file anywhere you need.

The API is very simple and straightforward. To compress you might do something like this:
```cpp
uint8_t* outputBuf = new uint8_t[skanda::compress_bound(inputSize)];
size_t compressedSize = skanda::compress(inputBuf, inputSize, outputBuf);
if (compressedSize == -1)
  std::cout << "Error while compressing data";
```
And to decompress:
```cpp
int res = skanda::decompress(compressedBuf, compressedSize, decompressedBuf, uncompressedSize);
if (res == -1)
  std::cout << "Error while decompressing data";
```

If you want to keep track of the progress, you can create a child class from ProgressCallbacks, and then pass a pointer of the object to the functions:
```cpp
class MyCallback : public skanda::ProgressCallback {
  size_t fileSize;
  
public:
  MyCallback(size_t _fileSize) {
    fileSize = _fileSize;
  }
  bool progress(size_t bytes) {
    std::cout << "Current progress: " << bytes << "/" << fileSize << "\n";
    return false;
  }
}

int main() {
  //...
  MyCallback myCallback(inputSize);
  size_t compressedSize = skanda::compress(input, inputSize, output, level, window, &myCallback);
  //...
}
```

Note: on GCC and Clang it should be compiled with -mbmi and -mbmi2 for maximum decompression speed.

# Benchmarks

The algorithm was benchmarked using [lzbench](https://github.com/inikep/lzbench) on Windows 11, on a Ryzen 6900HX@3.3GHz and compiled with Visual Studio 2022. The file used was produced by tarring the [Silesia corpus](http://sun.aei.polsl.pl/~sdeor/index.php?page=silesia). The only additional parameter was -t16,16.

| Compressor name         | Compression| Decompress.| Compr. size | Ratio |
| ---------------         | -----------| -----------| ----------- | ----- | 
| memcpy                  | 18569 MB/s | 18756 MB/s |   211948544 |100.00 |  
| lz4 1.9.4               |   557 MB/s |  3169 MB/s |   100880983 | 47.60 |  
| lz4hc 1.9.4 -1          |   104 MB/s |  2832 MB/s |    83804013 | 39.54 |  
| lz4hc 1.9.4 -4          |    69 MB/s |  2965 MB/s |    79808158 | 37.65 |  
| lz4hc 1.9.4 -9          |    27 MB/s |  3069 MB/s |    77884698 | 36.75 |  
| lz4hc 1.9.4 -12         |  9.77 MB/s |  3146 MB/s |    77262872 | 36.45 |  
| libdeflate 1.14 -1      |   194 MB/s |   764 MB/s |    73505591 | 34.68 |  
| libdeflate 1.14 -4      |   112 MB/s |   801 MB/s |    69471403 | 32.78 |  
| libdeflate 1.14 -8      |    32 MB/s |   799 MB/s |    66765105 | 31.50 |  
| libdeflate 1.14 -12     |  5.59 MB/s |   808 MB/s |    64686830 | 30.52 |  
| lizard 1.0 -20          |   354 MB/s |  2025 MB/s |    96927491 | 45.73 |  
| lizard 1.0 -23          |    54 MB/s |  2075 MB/s |    82079451 | 38.73 |  
| lizard 1.0 -26          |  5.46 MB/s |  2110 MB/s |    73469355 | 34.66 |  
| lizard 1.0 -29          |  1.81 MB/s |  2063 MB/s |    69678229 | 32.88 |  
| lizard 1.0 -40          |   255 MB/s |  1129 MB/s |    80849296 | 38.15 |  
| lizard 1.0 -43          |    50 MB/s |  1352 MB/s |    71818260 | 33.88 |  
| lizard 1.0 -46          |  8.27 MB/s |  1319 MB/s |    66462863 | 31.36 |  
| lizard 1.0 -49          |  1.63 MB/s |  1401 MB/s |    61370631 | 28.96 |  
| **skanda 0.5 -0**       |   561 MB/s |  2473 MB/s |    98832490 | 46.63 |  
| **skanda 0.5 -3**       |   105 MB/s |  2206 MB/s |    78453193 | 37.02 |  
| **skanda 0.5 -5**       |    23 MB/s |  2352 MB/s |    72169933 | 34.05 |  
| **skanda 0.5 -7**       |  6.23 MB/s |  2521 MB/s |    68840176 | 32.48 |  
| **skanda 0.5 -9**       |  3.20 MB/s |  2544 MB/s |    67675046 | 31.93 |  
| zstd 1.5.2 -1           |   372 MB/s |   853 MB/s |    73422932 | 34.64 |  
| zstd 1.5.2 -6           |    75 MB/s |   817 MB/s |    61481995 | 29.01 |  
| zstd 1.5.2 -12          |    21 MB/s |   866 MB/s |    58196278 | 27.46 |  
| zstd 1.5.2 -17          |  4.21 MB/s |   811 MB/s |    54281107 | 25.61 |  
| zstd 1.5.2 -22          |  1.66 MB/s |   782 MB/s |    52476925 | 24.76 |  
