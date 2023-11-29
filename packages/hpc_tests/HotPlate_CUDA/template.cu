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

static void hot_plate(real_t *out, real_t *in, int width, int height, real_t epsilon) {
  real_t *u = in;
  real_t *w = out;
  int run = 1;

  // Initialize output from input (need border pixels)
  memcpy(out, in, sizeof(real_t) * width * height);
  // Iterate until the new (W) and old (U) solution differ by no more than epsilon.
  iterations = 0;
  while (run) {
    // Determine the new estimate of the solution at the interior points.
    // The new solution W is the average of north, south, east and west neighbors.
    run = 0;
    for (int i = 1; i < height - 1; ++i) {
      for (int j = 1; j < width - 1; ++j) {
        val(w,i,j) = ( val(u,i-1,j) + val(u,i+1,j) + val(u,i,j-1) + val(u,i,j+1) ) / (real_t)4;
        if (epsilon < FABS(val(w,i,j) - val(u,i,j))) run |= 1;
      }
    }
    {real_t *t = w; w = u; u = t;} // swap u and w
    iterations++;
  }
  // Save solution to output.
  if (u != out) {
    memcpy(out, u, sizeof(real_t) * width * height);
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
