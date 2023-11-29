
#ifndef __HTK_CUDA_H__
#define __HTK_CUDA_H__

#ifdef HTK_USE_CUDA
#ifdef __PGI
#define __GNUC__ 4
#endif /* __PGI */
#include <cuda.h>
#include <cuda_runtime.h>

typedef struct st_htkCUDAMemory_t {
  void *mem;
  size_t sz;
} htkCUDAMemory_t;

#define _cudaMemoryListSize 1024

extern size_t _cudaMallocSize;
extern htkCUDAMemory_t _cudaMemoryList[];
extern int _cudaMemoryListIdx;

char *htkRandom_list(size_t sz);

static inline cudaError_t htkCUDAMalloc(void **devPtr, size_t sz) {
  int idx = _cudaMemoryListIdx;

  cudaError_t err = cudaMalloc(devPtr, sz);

  if (idx == 0) {
    srand(time(NULL));
    memset(_cudaMemoryList, 0,
           sizeof(htkCUDAMemory_t) * _cudaMemoryListSize);
  }

  if (err == cudaSuccess) {
#if 0
    char * rands = htkRandom_list(sz);
    // can use curand here, but do not want to invoke a kernel
    err = cudaMemcpy(*devPtr, rands, sz, cudaMemcpyHostToDevice);
    htkFree(rands);
#else
    err = cudaMemset(*devPtr, 0, sz);
#endif
  }

  _cudaMallocSize += sz;
  _cudaMemoryList[idx].mem = *devPtr;
  _cudaMemoryList[idx].sz  = sz;
  _cudaMemoryListIdx++;
  return err;
}

static inline cudaError_t htkCUDAFree(void *mem) {
  int idx = _cudaMemoryListIdx;
  if (idx == 0) {
    memset(_cudaMemoryList, 0,
           sizeof(htkCUDAMemory_t) * _cudaMemoryListSize);
  }
  for (int ii = 0; ii < idx; ii++) {
    if (_cudaMemoryList[ii].mem != nullptr &&
        _cudaMemoryList[ii].mem == mem) {
      cudaError_t err = cudaFree(mem);
      _cudaMallocSize -= _cudaMemoryList[ii].sz;
      _cudaMemoryList[ii].mem = nullptr;
      return err;
    }
  }
  return cudaErrorMemoryAllocation;
}

#define cudaMalloc(elem, err) htkCUDAMalloc((void **)elem, err)
#define cudaFree htkCUDAFree

#endif /* HTK_USE_CUDA */

#endif /* __HTK_CUDA_H__ */
