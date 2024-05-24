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
if (skanda::is_error(compressedSize))
  std::cout << "Error while compressing data";
```
And to decompress:
```cpp
size_t err = skanda::decompress(compressedBuf, compressedSize, decompressedBuf, uncompressedSize);
if (skanda::is_error(err))
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
| **skanda 0.9 -speed=1 -0** | 539.88MiB/s | 3114.32MiB/s | 93538228 | 44.13 |
| **skanda 0.9 -speed=1 -2** | 193.47MiB/s | 2640.16MiB/s | 80049550 | 37.77 |
| **skanda 0.9 -speed=1 -5** | 26.59MiB/s | 2864.41MiB/s | 72495740 | 34.20 |
| **skanda 0.9 -speed=1 -7** | 5.36MiB/s | 2972.98MiB/s | 68823978 | 32.47 |
| **skanda 0.9 -speed=1 -9** | 2.94MiB/s | 2998.12MiB/s | 67729324 | 31.96 |
| **skanda 0.9 -speed=0.5 -0** | 457.00MiB/s | 1853.22MiB/s | 79301574 | 37.42 |
| **skanda 0.9 -speed=0.5 -2** | 184.42MiB/s | 1809.57MiB/s | 68352128 | 32.25 |
| **skanda 0.9 -speed=0.5 -5** | 13.22MiB/s | 1921.22MiB/s | 62695404 | 29.58 |
| **skanda 0.9 -speed=0.5 -7** | 2.88MiB/s | 2034.08MiB/s | 58205631 | 27.46 |
| **skanda 0.9 -speed=0.5 -9** | 1.47MiB/s | 2036.71MiB/s | 57089458 | 26.94 |
| **skanda 0.9 -speed=0 -0** | 402.50MiB/s | 1348.53MiB/s | 73494683 | 34.68 |
| **skanda 0.9 -speed=0 -2** | 178.53MiB/s | 1331.73MiB/s | 64429195 | 30.40 |
| **skanda 0.9 -speed=0 -5** | 15.64MiB/s | 1411.07MiB/s | 58037106 | 27.38 |
| **skanda 0.9 -speed=0 -7** | 2.49MiB/s | 1419.57MiB/s | 53558506 | 25.27 |
| **skanda 0.9 -speed=0 -9** | 1.27MiB/s | 1409.58MiB/s | 52102474 | 24.58 |
| lz4 1.9.4 | 551.14MiB/s | 3113.20MiB/s | 100880983 | 47.60 |
| lz4hc 1.9.4 -1 | 102.44MiB/s | 2780.29MiB/s | 83804013 | 39.54 |
| lz4hc 1.9.4 -4 | 67.45MiB/s | 2892.35MiB/s | 79808158 | 37.65 |
| lz4hc 1.9.4 -8 | 29.39MiB/s | 2981.40MiB/s | 77957732 | 36.78 |
| lz4hc 1.9.4 -12 | 9.46MiB/s | 3063.99MiB/s | 77262872 | 36.45 |
| zstd 1.5.6 -1 | 365.46MiB/s | 812.80MiB/s | 73423309 | 34.64 |
| zstd 1.5.6 -3 | 200.14MiB/s | 724.92MiB/s | 66524095 | 31.39 |
| zstd 1.5.6 -6 | 75.77MiB/s | 765.67MiB/s | 61543204 | 29.04 |
| zstd 1.5.6 -10 | 31.49MiB/s | 797.64MiB/s | 58680128 | 27.69 |
| zstd 1.5.6 -15 | 7.06MiB/s | 834.98MiB/s | 57177019 | 26.98 |
| zstd 1.5.6 -19 | 2.11MiB/s | 715.91MiB/s | 52996635 | 25.00 |
| zstd 1.5.6 -22 | 1.56MiB/s | 704.20MiB/s | 52473367 | 24.76 |

