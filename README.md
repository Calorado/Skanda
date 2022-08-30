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
uint8_t* outputBuf = new uint8_t[skanda::skanda_compress_bound(inputSize)];
size_t compressedSize = skanda::skanda_compress(inputBuf, inputSize, outputBuf);
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
  size_t compressedSize = skanda::skanda_compress(input, inputSize, output, level, window, &myCallback);
  //...
}
```

You can also get the amount of extra memory used by the compression using skanda_estimate_memory().

# Benchmarks

The algorithm was benchmarked using [lzbench](https://github.com/inikep/lzbench) on Windows 10, on an i5-6300HQ and compiled with Visual Studio 2019. The file used was produced by tarring the [Silesia corpus](http://sun.aei.polsl.pl/~sdeor/index.php?page=silesia). The only additional parameter was -t16,16.

| Compressor name         | Compression| Decompress.| Compr. size | Ratio |
| ---------------         | -----------| -----------| ----------- | ----- | 
| memcpy                  | 11963 MB/s | 12223 MB/s |   211948544 |100.00 |  
| lz4 1.9.3               |   496 MB/s |  2977 MB/s |   100880983 | 47.60 |  
| lz4hc 1.9.3 -1          |    90 MB/s |  2693 MB/s |    83804013 | 39.54 |  
| lz4hc 1.9.3 -4          |    59 MB/s |  2813 MB/s |    79808158 | 37.65 |  
| lz4hc 1.9.3 -9          |    24 MB/s |  2905 MB/s |    77884698 | 36.75 |  
| lz4hc 1.9.3 -12         |  8.53 MB/s |  2953 MB/s |    77262872 | 36.45 |  
| lizard 1.0 -20          |   312 MB/s |  1825 MB/s |    96927491 | 45.73 |  
| lizard 1.0 -23          |    45 MB/s |  1867 MB/s |    82079451 | 38.73 |  
| lizard 1.0 -26          |  5.12 MB/s |  1895 MB/s |    72564556 | 34.24 |  
| lizard 1.0 -29          |  1.69 MB/s |  1777 MB/s |    68942077 | 32.53 |  
| lizard 1.0 -40          |   236 MB/s |   979 MB/s |    80849296 | 38.15 |  
| lizard 1.0 -43          |    42 MB/s |  1182 MB/s |    71818260 | 33.88 |  
| lizard 1.0 -46          |  8.07 MB/s |  1175 MB/s |    65572865 | 30.94 |  
| lizard 1.0 -49          |  1.64 MB/s |  1187 MB/s |    60862558 | 28.72 |  
| libdeflate 1.10 -1      |   171 MB/s |   552 MB/s |    73505591 | 34.68 |  
| libdeflate 1.10 -4      |    96 MB/s |   573 MB/s |    69471403 | 32.78 |  
| libdeflate 1.10 -8      |    32 MB/s |   592 MB/s |    66765105 | 31.50 |  
| libdeflate 1.10 -12     |  4.72 MB/s |   593 MB/s |    64686830 | 30.52 |  
| **skanda 1.3.3 -0**     |   533 MB/s |  2187 MB/s |    99772761 | 47.07 |  
| **skanda 1.3.3 -1**     |   318 MB/s |  1795 MB/s |    86679562 | 40.90 |
| **skanda 1.3.3 -3**     |    92 MB/s |  1874 MB/s |    77091874 | 36.37 |
| **skanda 1.3.3 -5**     |    17 MB/s |  1978 MB/s |    71429192 | 33.70 |
| **skanda 1.3.3 -7**     |  5.79 MB/s |  1980 MB/s |    68657211 | 32.39 |
| **skanda 1.3.3 -9**     |  3.60 MB/s |  1989 MB/s |    66961512 | 31.59 |
| **skanda 1.3.3 -10**    |  1.26 MB/s |  1987 MB/s |    66419728 | 31.34 |  
| zstd 1.5.2 -1           |   324 MB/s |   729 MB/s |    73422932 | 34.64 |  
| zstd 1.5.2 -6           |    74 MB/s |   691 MB/s |    61481995 | 29.01 |  
| zstd 1.5.2 -12          |    20 MB/s |   737 MB/s |    58196278 | 27.46 |  
| zstd 1.5.2 -17          |  3.78 MB/s |   694 MB/s |    54281107 | 25.61 |  
| zstd 1.5.2 -22          |  1.68 MB/s |   654 MB/s |    52476925 | 24.76 |  
