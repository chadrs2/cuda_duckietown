#include "htk.h"

#if defined(USE_DOUBLE)
#define EPSILON 0.00005
#define FABS fabs
typedef double real_t;

#else
#define EPSILON 0.00005f
#define FABS fabsf
typedef float real_t;
#endif

#define val(arry, i, j) arry[(i)*width + (j)]

int iterations;
__device__ int d_run;

//@@ Kernel for hot_plate
__global__ void hot_plate_kernel(real_t *output, real_t *input, 
                                  int width, int height) {
  // Hotplate code for CUDA
  d_run = 0;
  int r_run = 0;
  int Row = blockIdx.y*blockDim.y + threadIdx.y;
  int Col = blockIdx.x*blockDim.x + threadIdx.x;
  if ((Row < height-1) && (Row > 0) && (Col < width-1) && (Col > 0)) {
    real_t avg = ( val(input,Row-1,Col) + val(input,Row+1,Col) 
                        + val(input,Row,Col-1) + val(input,Row,Col+1) ) / (real_t)4;
    val(output, Row, Col) = avg;
    r_run = (EPSILON < FABS(avg - val(input,Row,Col)));
    __syncthreads_or(r_run);
    atomicOr(&d_run, r_run);
  }
}

static void hot_plate(real_t *hostOutputData, real_t *hostInputData, int width, int height, real_t epsilon) {
  //@@ Device variable definitions
  float *deviceInputData;
  float *deviceOutputData;

  //@@ Allocate GPU memory here
  int size = width * height * sizeof(float);
  cudaMalloc((void **) &deviceInputData, size);
  cudaMalloc((void **) &deviceOutputData, size);

  //@@ Copy memory to GPU here
  cudaMemcpy(deviceInputData, hostInputData, size, cudaMemcpyHostToDevice);
  cudaMemcpy(deviceOutputData, hostInputData, size, cudaMemcpyHostToDevice);

  //@@ Initialize the grid and block dimensions here
  dim3 block_dim(16,16,1); // blocks are inside of the grid
  dim3 grid_dim(ceil((double) width / block_dim.x), 
                ceil((double) height / block_dim.y));

  int h_run = 1;

  // Iterate until the new (W) and old (U) solution differ by no more than epsilon.
  iterations = 0;
  htkTime_start(Compute, "Doing the computation");
  while (h_run) {
    // std::cout << iterations << std::endl;
    // std::cout << d_run << std::endl;

    // Determine the new estimate of the solution at the interior points.
    // The new solution W is the average of north, south, east and west neighbors.
    h_run = 0;
    
    hot_plate_kernel <<< grid_dim, block_dim >>> (deviceOutputData,
                                                  deviceInputData,
                                                  width, height);
    cudaDeviceSynchronize(); // wait for all blocks in the launch to finish processing
    cudaMemcpyFromSymbol(&h_run, d_run, sizeof(int));

    real_t *t = deviceOutputData; 
    deviceOutputData = deviceInputData; 
    deviceInputData = t; // swap input and output

    iterations++;
  }
  htkTime_stop(Compute, "Doing the computation");

  // Save solution to output. 
  //@@ Copy GPU memory back to CPU host memory
  cudaMemcpy(hostOutputData, deviceOutputData, size, cudaMemcpyDeviceToHost);

  //@@ Free GPU memory
  cudaFree(deviceInputData);
  cudaFree(deviceOutputData);
}

/*
4. Adapt the OR reduction to work on the GPU. Remember that blocks don't 
normally communicate with each other except through global memory. 
One way to do this is to call __syncthreads_or() to do an OR reduction across 
all threads in a block, and then call atomicOr() to continue the reduction 
between blocks. Two variables are needed: r_run is a variable local to the 
kernel declared as an int, and d_run is a global in device memory declared 
with "__device__ int d_run;".

5. If your kernel is launched once for each iteration step, you will need to 
copy the value of the d_run variable from device memory to the host so it can 
be accessed from the host code. You can do this with a call to 
cudaMemcpyFromSymbol(&h_run, d_run, sizeof(int)). You can also set the value 
of d_run from the host with a call to 
cudaMemcpyToSymbol(d_run, &h_run, sizeof(int)).

6. The iteration count must be printed by the parallel version of your program 
before termination, just like the sequential version.
*/


int main(int argc, char *argv[]) {
  htkArg_t args;
  int width;
  int height;
  int channels;
  char *inputFile;
  htkImage_t input;
  htkImage_t output;
  float *hostInputData;
  float *hostOutputData;

  args = htkArg_read(argc, argv);
  if (args.inputCount != 1) {htkLog(ERROR, "Missing input"); return 1;}

  htkTime_start(IO, "Importing data and creating memory on host");
  inputFile = htkArg_getInputFile(args, 0);
  input = htkImport(inputFile);
  width  = htkImage_getWidth(input);
  height = htkImage_getHeight(input);
  channels  = htkImage_getChannels(input);
  if (channels != 1) {htkLog(ERROR, "Expecting gray scale image"); return 1;}
  output = htkImage_new(width, height, channels);
  hostInputData  = htkImage_getData(input);
  hostOutputData = htkImage_getData(output);
  htkTime_stop(IO, "Importing data and creating memory on host");
  htkLog(TRACE, "Image dimensions WxH are ", width, " x ", height);

  //@@ Launch the GPU Kernel here
  // hot_plate_kernel <<< grid_dim, block_dim >>> (deviceInputData,
  //                                               deviceOutputData,
  //                                               width, height);
  // cudaDeviceSynchronize(); // wait for all blocks in the launch to finish processing
  hot_plate(hostOutputData, hostInputData, width, height, EPSILON);

  htkLog(TRACE, "Solution iterations: ", iterations);

  htkSolution(args, output);

  htkImage_delete(output);
  htkImage_delete(input);

  return 0;
}
