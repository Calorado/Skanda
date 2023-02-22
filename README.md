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
  size_t compressedSize = skanda::compress(input, inputSize, output, level, &myCallback);
  //...
}
```

Note: on GCC and Clang it should be compiled with -mbmi and -mbmi2 for maximum decompression speed.

# Benchmarks

The algorithm was benchmarked on Windows 11, on a Ryzen 6900HX@3.3GHz and compiled with Visual Studio 2022. The file used was produced by tarring the [Silesia corpus](http://sun.aei.polsl.pl/~sdeor/index.php?page=silesia).

| Compressor name         | Compression| Decompress.| Compr. size | Ratio |
| ---------------         | -----------| -----------| ----------- | ----- | 
| **skanda 0.6 -0** | 521.84MiB/s | 2258.46MiB/s | 98832868 | 46.63 |
| **skanda 0.6 -3** | 97.12MiB/s | 2036.82MiB/s | 77206627 | 36.43 |
| **skanda 0.6 -5** | 20.01MiB/s | 2133.87MiB/s | 69991203 | 33.02 |
| **skanda 0.6 -7** | 3.61MiB/s | 2027.14MiB/s | 65049273 | 30.69 |
| **skanda 0.6 -9** | 2.16MiB/s | 2015.62MiB/s | 63498349 | 29.96 |
| lz4 1.9.4 | 532.59MiB/s | 2985.82MiB/s | 100880983 | 47.60 |
| lz4hc 1.9.4 -1 | 100.03MiB/s | 2657.34MiB/s | 83804013 | 39.54 |
| lz4hc 1.9.4 -4 | 66.71MiB/s | 2775.14MiB/s | 79808158 | 37.65 |
| lz4hc 1.9.4 -8 | 31.68MiB/s | 2854.57MiB/s | 77957732 | 36.78 |
| lz4hc 1.9.4 -12 | 9.21MiB/s | 2937.22MiB/s | 77262872 | 36.45 |
| lizard 1.0 -20 | 332.35MiB/s | 1920.20MiB/s | 96927491 | 45.73 |
| lizard 1.0 -23 | 50.40MiB/s | 1976.04MiB/s | 82079451 | 38.73 |
| lizard 1.0 -26 | 5.05MiB/s | 2034.15MiB/s | 72564738 | 34.24 |
| lizard 1.0 -29 | 1.64MiB/s | 1970.55MiB/s | 68942591 | 32.53 |
| lizard 1.0 -40 | 234.32MiB/s | 1065.81MiB/s | 80847695 | 38.14 |
| lizard 1.0 -43 | 46.93MiB/s | 1273.47MiB/s | 71816638 | 33.88 |
| lizard 1.0 -46 | 7.69MiB/s | 1263.81MiB/s | 65572024 | 30.94 |
| lizard 1.0 -49 | 1.49MiB/s | 1315.67MiB/s | 60859056 | 28.71 |
| libdeflate 1.17 -1 | 187.95MiB/s | 742.01MiB/s | 73505591 | 34.68 |
| libdeflate 1.17 -4 | 105.74MiB/s | 776.12MiB/s | 69471403 | 32.78 |
| libdeflate 1.17 -8 | 31.86MiB/s | 767.90MiB/s | 66765105 | 31.50 |
| libdeflate 1.17 -12 | 4.67MiB/s | 775.15MiB/s | 64678485 | 30.52 |
| zstd 1.5.4 -1 | 361.02MiB/s | 903.88MiB/s | 73423309 | 34.64 |
| zstd 1.5.4 -6 | 82.46MiB/s | 807.23MiB/s | 61481995 | 29.01 |
| zstd 1.5.4 -12 | 22.76MiB/s | 870.69MiB/s | 58196278 | 27.46 |
| zstd 1.5.4 -17 | 4.28MiB/s | 798.03MiB/s | 54284479 | 25.61 |
| zstd 1.5.4 -22 | 1.95MiB/s | 767.13MiB/s | 52473367 | 24.76 |
