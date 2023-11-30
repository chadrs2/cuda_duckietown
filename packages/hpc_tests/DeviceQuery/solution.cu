#include "htk.h"

//@@ The purpose of this code is to become familiar with the submission
//@@ process. Do not worry if you do not understand all the details of
//@@ the code.

int main(int argc, char **argv) {
  int deviceCount;

  htkArg_read(argc, argv);

  cudaGetDeviceCount(&deviceCount);

  htkTime_start(GPU, "Getting GPU Data."); //@@ start a timer

  for (int dev = 0; dev < deviceCount; dev++) {
    cudaDeviceProp deviceProp;

    cudaGetDeviceProperties(&deviceProp, dev);

    if (dev == 0) {
      if (deviceProp.major == 9999 && deviceProp.minor == 9999) {
        htkLog(TRACE, "No CUDA GPU has been detected");
        return -1;
      } else if (deviceCount == 1) {
        //@@ htkLog is a provided logging API (similar to Log4J).
        //@@ The logging function htkLog takes a level which is either
        //@@ OFF, FATAL, ERROR, WARN, INFO, DEBUG, or TRACE and a
        //@@ message to be printed.
        htkLog(TRACE, "There is 1 device supporting CUDA");
      } else {
        htkLog(TRACE, "There are ", deviceCount,
              " devices supporting CUDA");
      }
    }

    htkLog(TRACE, "Device ", dev, " name: ", deviceProp.name);
    htkLog(TRACE, " Computational Capabilities: ", deviceProp.major, ".",
          deviceProp.minor);
    htkLog(TRACE, " Maximum global memory size: ",
          deviceProp.totalGlobalMem);
    htkLog(TRACE, " Maximum constant memory size: ",
          deviceProp.totalConstMem);
    htkLog(TRACE, " Maximum shared memory size per block: ",
          deviceProp.sharedMemPerBlock);
    htkLog(TRACE, " Maximum block dimensions: ",
          deviceProp.maxThreadsDim[0], " x ", deviceProp.maxThreadsDim[1],
          " x ", deviceProp.maxThreadsDim[2]);
    htkLog(TRACE, " Maximum grid dimensions: ", deviceProp.maxGridSize[0],
          " x ", deviceProp.maxGridSize[1], " x ",
          deviceProp.maxGridSize[2]);
    htkLog(TRACE, " Warp size: ", deviceProp.warpSize);
  }

  htkTime_stop(GPU, "Getting GPU Data."); //@@ stop the timer

  return 0;
}
