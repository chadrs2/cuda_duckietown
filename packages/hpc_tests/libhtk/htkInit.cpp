
#include "htk.h"

#define MB (1 << 20)
#ifndef HTK_DEFAULT_HEAP_SIZE
#define HTK_DEFAULT_HEAP_SIZE (1024 * MB)
#endif /* HTK_DEFAULT_HEAP_SIZE */

static bool _initializedQ = htkFalse;

#ifndef HTK_USE_WINDOWS
//__attribute__((__constructor__))
#endif /* HTK_USE_WINDOWS */
void htk_init(int *
#ifdef HTK_USE_MPI
                 argc
#endif /* HTK_USE_MPI */
             ,
             char ***
#ifdef HTK_USE_MPI
                 argv
#endif /* HTK_USE_MPI */
             ) {
  if (_initializedQ == htkTrue) {
    return;
  }
#ifdef HTK_USE_MPI
  htkMPI_Init(argc, argv);
#endif /* HTK_USE_MPI */

  _envSessionId();
#ifdef HTK_USE_CUDA
  CUresult err = cuInit(0);

/* Select a random GPU */

#ifdef HTK_USE_MPI
  if (rankCount() > 1) {
    int deviceCount;
    cudaGetDeviceCount(&deviceCount);
    srand(time(NULL));
    cudaSetDevice(htkMPI_getRank() % deviceCount);
  } else {
    int deviceCount;
    cudaGetDeviceCount(&deviceCount);

    srand(time(NULL));
    cudaSetDevice(rand() % deviceCount);
  }
#else
  {
    int deviceCount;
    cudaGetDeviceCount(&deviceCount);

    srand(time(NULL));
    cudaSetDevice(rand() % deviceCount);
  }
#endif /* HTK_USE_MPI */

  cudaDeviceSetLimit(cudaLimitPrintfFifoSize, 1 * MB);
  cudaDeviceSetLimit(cudaLimitMallocHeapSize, HTK_DEFAULT_HEAP_SIZE);

  cudaDeviceSynchronize();

#endif /* HTK_USE_CUDA */

#ifdef HTK_USE_WINDOWS
  QueryPerformanceFrequency((LARGE_INTEGER *)&_hrtime_frequency);
#endif /* _MSC_VER */

  _hrtime();

  _timer        = htkTimer_new();
  _logger       = htkLogger_new();
  _initializedQ = htkTrue;

  htkFile_init();

  solutionJSON = nullptr;

#ifdef HTK_USE_MPI
  atexit(htkMPI_Exit);
#else  /* HTK_USE_MPI */
  atexit(htk_atExit);
#endif /* HTK_USE_MPI */
}
