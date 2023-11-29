
#include "htk.h"
#ifdef HTK_USE_CUDA

int _cudaMemoryListIdx = 0;

size_t _cudaMallocSize = 0;

htkCUDAMemory_t _cudaMemoryList[_cudaMemoryListSize];

char *htkRandom_list(size_t sz) {
  size_t ii;
  char *rands = htkNewArray(char, sz);
  int *irands = (int *)rands;
  for (ii = 0; ii < sz / sizeof(int); ii++) {
    irands[ii] = rand();
  }
  while (ii < sz) {
    rands[ii] = (char)(rand() % 255);
    ii++;
  }
  return rands;
}

#endif /* HTK_USE_CUDA */
