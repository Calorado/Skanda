# Skanda

Skanda is a compression algorithm based on LZ4 and Lizard, but designed for higher ratios, more similar to DEFLATE, while still keeping >1GB/s of decompression speed. This puts it somewhere between LZ4 and Zstd in the size/speed tradeoff, and makes it one of the strongest entropyless, byte-aligned LZs out there.

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
  void progress(size_t bytes) {
    std::cout << "Current progress: " << bytes << "/" << fileSize << "\n";
  }
}

int main() {
  //...
  MyCallback myCallback(inputSize);
  size_t compressedSize = skanda::compress(input, inputSize, output, level, window, &myCallback);
  //...
}
```

You can also get the amount of extra memory used by the compression using estimate_memory().

# Benchmarks

The algorithm was benchmarked using [lzbench](https://github.com/inikep/lzbench) on Windows 10, on an i5-6300HQ and compiled with Visual Studio 2019. The file used was produced by tarring the [Silesia corpus](http://sun.aei.polsl.pl/~sdeor/index.php?page=silesia). The only additional parameter was -t16,16.

| Compressor name         | Compression| Decompress.| Compr. size | Ratio |
| ---------------         | -----------| -----------| ----------- | ----- | 
| memcpy                  | 11963 MB/s | 12223 MB/s |   211948544 |100.00 |  
| lz4 1.9.4               |   488 MB/s |  3023 MB/s |   100880983 | 47.60 |
| lz4hc 1.9.4 -1          |    91 MB/s |  2730 MB/s |    83804013 | 39.54 |
| lz4hc 1.9.4 -4          |    59 MB/s |  2853 MB/s |    79808158 | 37.65 |
| lz4hc 1.9.4 -9          |    24 MB/s |  2949 MB/s |    77884698 | 36.75 |
| lz4hc 1.9.4 -12         |  8.51 MB/s |  3002 MB/s |    77262872 | 36.45 |
| libdeflate 1.13 -1      |   178 MB/s |   545 MB/s |    73505591 | 34.68 |
| libdeflate 1.13 -4      |    99 MB/s |   564 MB/s |    69471403 | 32.78 |
| libdeflate 1.13 -8      |    32 MB/s |   581 MB/s |    66765105 | 31.50 |
| libdeflate 1.13 -12     |  4.78 MB/s |   582 MB/s |    64686830 | 30.52 |
| lizard 1.0 -20          |   307 MB/s |  1904 MB/s |    96927491 | 45.73 |
| lizard 1.0 -23          |    43 MB/s |  1930 MB/s |    82079451 | 38.73 |
| lizard 1.0 -26          |  5.13 MB/s |  1988 MB/s |    72564738 | 34.24 |
| lizard 1.0 -29          |  1.72 MB/s |  1833 MB/s |    68942591 | 32.53 |
| lizard 1.0 -40          |   234 MB/s |   987 MB/s |    80849296 | 38.15 |
| lizard 1.0 -43          |    41 MB/s |  1203 MB/s |    71818260 | 33.88 |
| lizard 1.0 -46          |  7.85 MB/s |  1203 MB/s |    65573603 | 30.94 |
| lizard 1.0 -49          |  1.64 MB/s |  1218 MB/s |    60861708 | 28.72 |
| **skanda 1.4.0 -0**     |   551 MB/s |  2089 MB/s |    97997954 | 46.24 | 
| **skanda 1.4.0 -3**     |    88 MB/s |  1922 MB/s |    76225105 | 35.96 |
| **skanda 1.4.0 -5**     |    16 MB/s |  1975 MB/s |    70977964 | 33.49 | 
| **skanda 1.4.0 -7**     |  5.16 MB/s |  1987 MB/s |    68332864 | 32.24 | 
| **skanda 1.4.0 -9**     |  3.44 MB/s |  1981 MB/s |    66699101 | 31.47 | 
| zstd 1.5.2 -1           |   319 MB/s |   787 MB/s |    73422932 | 34.64 |
| zstd 1.5.2 -6           |    73 MB/s |   777 MB/s |    61481995 | 29.01 |
| zstd 1.5.2 -12          |    20 MB/s |   823 MB/s |    58196278 | 27.46 |
| zstd 1.5.2 -17          |  3.76 MB/s |   771 MB/s |    54281107 | 25.61 |
| zstd 1.5.2 -22          |  1.67 MB/s |   690 MB/s |    52476925 | 24.76 |
