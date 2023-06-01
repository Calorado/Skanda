# Skanda

Skanda is an experimental compression algorithm, trying to improve upon the Lizard algorithm. Depending on the speed/ratio tradeoff selected it can range from 75% of LZ4 speed with Deflate ratios to a bit faster than Zstd with a bit less compression.

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
| **skanda 0.7 -0 speed_bias=1** | 519.11MiB/s | 2383.56MiB/s | 94113718 | 44.40 |
| **skanda 0.7 -3 speed_bias=1** | 96.42MiB/s | 2128.53MiB/s | 77248707 | 36.45 |
| **skanda 0.7 -5 speed_bias=1** | 19.84MiB/s | 2139.77MiB/s | 70610209 | 33.31 |
| **skanda 0.7 -7 speed_bias=1** | 3.60MiB/s | 2065.94MiB/s | 65134961 | 30.73 |
| **skanda 0.7 -9 speed_bias=1** | 2.09MiB/s | 2006.48MiB/s | 63540928 | 29.98 |
| **skanda 0.7 -0 speed_bias=0.35** | 344.33MiB/s | 1474.11MiB/s | 76696378 | 36.19 |
| **skanda 0.7 -3 speed_bias=0.35** | 91.37MiB/s | 1630.88MiB/s | 66321779 | 31.29 |
| **skanda 0.7 -5 speed_bias=0.35** | 19.78MiB/s | 1670.19MiB/s | 61546799 | 29.04 |
| **skanda 0.7 -7 speed_bias=0.35** | 3.18MiB/s | 1689.51MiB/s | 58003260 | 27.37 |
| **skanda 0.7 -9 speed_bias=0.35** | 1.92MiB/s | 1640.28MiB/s | 56591343 | 26.70 |
| lz4 1.9.4 -1 | 534.21MiB/s | 2992.14MiB/s | 100880983 | 47.60 |
| lz4 1.9.4 -2 | 100.48MiB/s | 2677.99MiB/s | 83804013 | 39.54 |
| lz4 1.9.4 -4 | 66.39MiB/s | 2803.24MiB/s | 79808158 | 37.65 |
| lz4 1.9.4 -8 | 31.06MiB/s | 2900.54MiB/s | 77957732 | 36.78 |
| lz4 1.9.4 -12 | 9.35MiB/s | 2979.16MiB/s | 77262872 | 36.45 |
| lizard 1.0 -20 | 329.14MiB/s | 1914.49MiB/s | 96927491 | 45.73 |
| lizard 1.0 -23 | 49.60MiB/s | 1958.62MiB/s | 82079451 | 38.73 |
| lizard 1.0 -26 | 4.93MiB/s | 2003.82MiB/s | 72564738 | 34.24 |
| lizard 1.0 -29 | 1.62MiB/s | 1939.29MiB/s | 68942591 | 32.53 |
| lizard 1.0 -40 | 237.66MiB/s | 1058.37MiB/s | 80847695 | 38.14 |
| lizard 1.0 -43 | 45.45MiB/s | 1235.25MiB/s | 71816638 | 33.88 |
| lizard 1.0 -46 | 7.57MiB/s | 1221.37MiB/s | 65572024 | 30.94 |
| lizard 1.0 -49 | 1.48MiB/s | 1266.49MiB/s | 60859056 | 28.71 |
| libdeflate 1.18 -1 | 183.02MiB/s | 731.20MiB/s | 73505591 | 34.68 |
| libdeflate 1.18 -4 | 104.96MiB/s | 766.39MiB/s | 69471403 | 32.78 |
| libdeflate 1.18 -8 | 30.99MiB/s | 764.57MiB/s | 66765105 | 31.50 |
| libdeflate 1.18 -12 | 4.63MiB/s | 774.50MiB/s | 64678485 | 30.52 |
| zstd 1.5.5 -1 | 349.89MiB/s | 860.82MiB/s | 73423309 | 34.64 |
| zstd 1.5.5 -6 | 76.19MiB/s | 795.90MiB/s | 61481995 | 29.01 |
| zstd 1.5.5 -12 | 21.84MiB/s | 837.13MiB/s | 58196278 | 27.46 |
| zstd 1.5.5 -17 | 3.91MiB/s | 776.80MiB/s | 54284479 | 25.61 |
| zstd 1.5.5 -22 | 1.54MiB/s | 725.87MiB/s | 52473367 | 24.76 |
