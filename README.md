# Skanda

Skanda is an experimental compression algorithm, kinda like an spiritual successor to Lizard. Depending on the speed/ratio tradeoff selected it can range from 80% of LZ4 speed with close to Deflate ratios, to similar to Zstd ratios at double the decode speed.

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
  size_t compressedSize = skanda::compress(input, inputSize, output, level, speedBias, &myCallback);
  //...
}
```

# Benchmarks

The algorithm was benchmarked on Windows 11, on a Ryzen 6900HX@3.3GHz and compiled with Visual Studio 2022. The file used was produced by tarring the [Silesia corpus](http://sun.aei.polsl.pl/~sdeor/index.php?page=silesia).

| Compressor name         | Compression| Decompress.| Compr. size | Ratio |
| ---------------         | -----------| -----------| ----------- | ----- | 
| **skanda 0.8 -speed-bias=1 -0** | 533.16MiB/s | 2656.77MiB/s | 94135957 | 44.41 |
| **skanda 0.8 -speed-bias=1 -3** | 100.73MiB/s | 2451.41MiB/s | 78139070 | 36.87 |
| **skanda 0.8 -speed-bias=1 -6** | 14.42MiB/s | 2597.87MiB/s | 71423098 | 33.70 |
| **skanda 0.8 -speed-bias=1 -9** | 3.01MiB/s | 3038.08MiB/s | 67760675 | 31.97 |
| **skanda 0.8 -speed-bias=0.5 -0** | 405.20MiB/s | 1883.58MiB/s | 80246607 | 37.86 |
| **skanda 0.8 -speed-bias=0.5 -3** | 96.89MiB/s | 1936.61MiB/s | 67420657 | 31.81 |
| **skanda 0.8 -speed-bias=0.5 -6** | 8.57MiB/s | 2086.66MiB/s | 61720628 | 29.12 |
| **skanda 0.8 -speed-bias=0.5 -9** | 0.64MiB/s | 2094.89MiB/s | 57123123 | 26.95 |
| **skanda 0.8 -speed-bias=0 -0** | 335.04MiB/s | 1356.30MiB/s | 74659786 | 35.23 |
| **skanda 0.8 -speed-bias=0 -3** | 92.94MiB/s | 1407.94MiB/s | 63615902 | 30.01 |
| **skanda 0.8 -speed-bias=0 -6** | 8.53MiB/s | 1519.94MiB/s | 57811563 | 27.28 |
| **skanda 0.8 -speed-bias=0 -9** | 0.55MiB/s | 1481.58MiB/s | 52771100 | 24.90 |
| lz4 1.9.4 -1 | 553.23MiB/s | 3382.70MiB/s | 100880983 | 47.60 |
| lz4 1.9.4 -2 | 104.09MiB/s | 3009.71MiB/s | 83804013 | 39.54 |
| lz4 1.9.4 -4 | 69.34MiB/s | 3151.82MiB/s | 79808158 | 37.65 |
| lz4 1.9.4 -8 | 32.83MiB/s | 3263.93MiB/s | 77957732 | 36.78 |
| lz4 1.9.4 -12 | 9.68MiB/s | 3336.39MiB/s | 77262872 | 36.45 |
| lizard 1.0 -20 | 330.36MiB/s | 2049.21MiB/s | 96927491 | 45.73 |
| lizard 1.0 -23 | 50.10MiB/s | 2093.62MiB/s | 82079451 | 38.73 |
| lizard 1.0 -26 | 5.04MiB/s | 2147.43MiB/s | 72564738 | 34.24 |
| lizard 1.0 -29 | 1.64MiB/s | 2069.80MiB/s | 68942591 | 32.53 |
| lizard 1.0 -40 | 235.59MiB/s | 1103.63MiB/s | 80847695 | 38.14 |
| lizard 1.0 -43 | 46.60MiB/s | 1325.95MiB/s | 71816638 | 33.88 |
| lizard 1.0 -46 | 7.70MiB/s | 1313.61MiB/s | 65572024 | 30.94 |
| lizard 1.0 -49 | 1.39MiB/s | 1367.12MiB/s | 60859056 | 28.71 |
| libdeflate 1.18 -1 | 193.71MiB/s | 741.68MiB/s | 73505591 | 34.68 |
| libdeflate 1.18 -4 | 109.05MiB/s | 780.81MiB/s | 69471403 | 32.78 |
| libdeflate 1.18 -8 | 30.72MiB/s | 780.20MiB/s | 66765105 | 31.50 |
| libdeflate 1.18 -12 | 4.82MiB/s | 789.02MiB/s | 64678485 | 30.52 |
| zstd 1.5.5 -1 | 360.13MiB/s | 861.60MiB/s | 73423309 | 34.64 |
| zstd 1.5.5 -6 | 74.28MiB/s | 796.59MiB/s | 61543204 | 29.04 |
| zstd 1.5.5 -12 | 19.94MiB/s | 812.60MiB/s | 58211131 | 27.46 |
| zstd 1.5.5 -17 | 4.09MiB/s | 757.63MiB/s | 54284479 | 25.61 |
| zstd 1.5.5 -22 | 1.69MiB/s | 724.07MiB/s | 52473367 | 24.76 |
