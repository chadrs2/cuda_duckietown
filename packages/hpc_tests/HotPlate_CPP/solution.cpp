#include "htk.h"
#include <cstdlib>
#include<thread>
#include<vector>
#include<algorithm>

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
int num_threads;
#include<atomic>
std::atomic<int> atomic_run;

void parallel_plate(
   int t_idx,
   real_t *out,
   real_t *in,
   int t_height,
   int width,
   int height
  ) {
  int run = 0;
  for (int row = t_idx*t_height; row < (t_idx+1)*t_height; ++row) {
    for (int col = 1; col < width-1; ++col) {

      if ((row > 0) && (row < height-1)) {
        real_t avg = ( val(in,row-1,col) + val(in,row+1,col)
                        + val(in,row,col-1) + val(in,row,col+1) ) / (real_t)4;
        val(out, row, col) = avg;
        // t_results[t_idx] |= (EPSILON < FABS(avg - val(in,row,col)));
        run |= (EPSILON < FABS(avg - val(in,row,col)));
      }
    }
  }
  atomic_run |= run;
}


static void hot_plate(real_t *out, real_t *in, int width, int height, real_t epsilon) {
 real_t *d_in = in;
 real_t *d_out = out;
  atomic_run = 1;

  if (const char* str_p = std::getenv("NUM_THREADS")) {
   num_threads = atoi(str_p);
  } else {
   num_threads = std::thread::hardware_concurrency();
  }
 int thread_height = std::ceil((float)height/num_threads);

 // Initialize output from input (need border pixels)
 memcpy(out, in, sizeof(real_t) * width * height);
 // Iterate until the new (W) and old (U) solution differ by no more than epsilon.
 iterations = 0;
  while (atomic_run) {
    // Determine the new estimate of the solution at the interior points.
    // The new solution W is the average of north, south, east and west neighbors.
    atomic_run = 0;

    // Initialize and run threads
    std::vector<std::thread> threads;
    for (int i = 0; i < num_threads; ++i) {
      threads.push_back(std::thread(
        parallel_plate,
        i,
        d_out,
        d_in,
        thread_height,
        width,
        height
      ));
    }

    // loop again to join the threads
    for (auto& t : threads) t.join();

    real_t *t = d_out;
    d_out = d_in;
    d_in = t; // swap input and output

    iterations++;
 }
 // Save solution to output.
 if (d_in != out) {
   memcpy(out, d_in, sizeof(real_t) * width * height);
 }
}

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


 htkTime_start(Compute, "Doing the computation");
 hot_plate(hostOutputData, hostInputData, width, height, EPSILON);
 htkTime_stop(Compute, "Doing the computation");
 htkLog(TRACE, "Solution iterations: ", iterations);


 htkSolution(args, output);


 htkImage_delete(output);
 htkImage_delete(input);


 return 0;
}