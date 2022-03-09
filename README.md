# Skanda

Skanda is an LZ4-like compression algorithm, intended for higher compression ratios (similar to DEFLATE) while still keeping a very high decompression speed. This puts it somewhere in between LZ4 and Zstd in the size/speed tradeoff.

# How to use

Simply add the file Skanda.h to your project and create a .cpp file with the following:
```cpp
#define SKANDA_IMPLEMENTATION
#include "Skanda.h"
```
Then simply add the header file anywhere you need.

The API is very simple and straighforward. To compress you might do something like this:
```cpp
uint8_t* outputBuf = new uint8_t[skanda::skanda_compress_bound(inputSize)];
size_t compressedSize = skanda::skanda_compress(inputBuf, inputSize, outputBuf, level);
if (compressedSize == -1)
  std::cout << "Error while compressing data";
```
And to decompress:
```cpp
int res = skanda::skanda_decompress(compressedBuf, compressedSize, decompressedBuf, uncompressedSize);
if (res == -1)
  std::cout << "Error while decompressing data";
```

If you want to keep track of the progress, you can create a child class from ProgressCallbacks, and then pass a pointer of the object to the functions:
```cpp
class MyCallback : public skanda::ProgressCallback {
  size_t fileSize;
  size_t processed;
  
public:
  MyCallback(size_t _fileSize) {
    fileSize = _fileSize;
    processed = 0;
  }
  void progress(size_t bytes) {
    processed += bytes;
    std::cout << "Current progress: " << processed << "/" << fileSize << "\n";
  }
}

int main() {
  //...
  MyCallback myCallback(inputSize);
  size_t compressedSize = skanda::skanda_compress(input, inputSize, output, level, &myCallback);
  //...
}
```

You can also get the amount of extra memory used by the compression using skanda_estimate_memory().

# Benchmarks

The algorithm was benchmarked using [lzbench](https://github.com/inikep/lzbench) on Windows 10, on an i5-6300HQ and compiled with Visual Studio 2019. The file used was produced by tarring the [Silesia corpus](http://sun.aei.polsl.pl/~sdeor/index.php?page=silesia).

| Compressor name         | Compression| Decompress.| Compr. size | Ratio |
| ---------------         | -----------| -----------| ----------- | ----- |
| memcpy                  | 11344 MB/s | 11475 MB/s |   211948544 |100.00 |
| lz4 1.9.3               |   499 MB/s |  3233 MB/s |   100880983 | 47.60 |
| lz4hc 1.9.3 -1          |    94 MB/s |  2956 MB/s |    83804013 | 39.54 |
| lz4hc 1.9.3 -4          |    61 MB/s |  3075 MB/s |    79808158 | 37.65 |
| lz4hc 1.9.3 -9          |    24 MB/s |  3183 MB/s |    77884698 | 36.75 |
| lz4hc 1.9.3 -12         |  8.75 MB/s |  3234 MB/s |    77262872 | 36.45 |
| lizard 1.0 -20          |   320 MB/s |  2036 MB/s |    96927491 | 45.73 |
| lizard 1.0 -23          |    46 MB/s |  2079 MB/s |    82079451 | 38.73 |
| lizard 1.0 -26          |  5.19 MB/s |  2115 MB/s |    72971214 | 34.43 |
| lizard 1.0 -29          |  1.70 MB/s |  1979 MB/s |    69333974 | 32.71 |
| lizard 1.0 -40          |   239 MB/s |  1109 MB/s |    80849296 | 38.15 |
| lizard 1.0 -43          |    43 MB/s |  1334 MB/s |    71818260 | 33.88 |
| lizard 1.0 -46          |  8.08 MB/s |  1330 MB/s |    65572865 | 30.94 |
| lizard 1.0 -49          |  1.65 MB/s |  1337 MB/s |    60862599 | 28.72 |
| libdeflate 1.10 -1      |   169 MB/s |   554 MB/s |    73505591 | 34.68 |
| libdeflate 1.10 -4      |    95 MB/s |   574 MB/s |    69471403 | 32.78 |
| libdeflate 1.10 -8      |    32 MB/s |   594 MB/s |    66765105 | 31.50 |
| libdeflate 1.10 -12     |  4.74 MB/s |   595 MB/s |    64686830 | 30.52 |
| **skanda 1.0.0 -0**     |   415 MB/s |  1772 MB/s |    95588364 | 45.10 |
| **skanda 1.0.0 -3**     |    78 MB/s |  1648 MB/s |    77845984 | 36.73 |
| **skanda 1.0.0 -5**     |    11 MB/s |  1745 MB/s |    71806406 | 33.88 |
| **skanda 1.0.0 -7**     |  2.87 MB/s |  1609 MB/s |    66408477 | 31.33 |
| **skanda 1.0.0 -9**     |  1.51 MB/s |  1439 MB/s |    63038043 | 29.74 |
| zstd 1.5.2 -1           |   321 MB/s |   775 MB/s |    73422932 | 34.64 |
| zstd 1.5.2 -6           |    75 MB/s |   728 MB/s |    61481995 | 29.01 |
| zstd 1.5.2 -12          |    20 MB/s |   771 MB/s |    58196278 | 27.46 |
| zstd 1.5.2 -17          |  3.85 MB/s |   724 MB/s |    54281107 | 25.61 |
| zstd 1.5.2 -22          |  1.75 MB/s |   670 MB/s |    52476925 | 24.76 |
