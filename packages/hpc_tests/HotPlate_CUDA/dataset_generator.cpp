
#include "htk.h"

#define MAX_VAL 255
#define Max(a, b) ((a) < (b) ? (b) : (a))
#define Min(a, b) ((a) > (b) ? (b) : (a))
#define Clamp(a, start, end) Max(Min(a, end), start)
#define val(arry, i, j) arry[(i)*width + (j)]

#define HOT  212
#define WARM  72
#define COLD   0

#if defined(USE_DOUBLE)
#define EPSILON 0.00005
#define FABS fabs
typedef double real_t;

#else
#define EPSILON 0.00005f
#define FABS fabsf
typedef float real_t;
#endif

static char *base_dir;


static void compute(unsigned char *out, unsigned char *in, int width, int height) {
  real_t *u = (real_t *)malloc(sizeof(real_t) * width * height);
  real_t *w = (real_t *)malloc(sizeof(real_t) * width * height);
  real_t epsilon = EPSILON;
  int run = 1;
  int iterations = 0;

  // Initialize U & W from input
  real_t scale = (real_t)1 / (real_t)MAX_VAL;
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      val(w,i,j) = val(u,i,j) = (real_t)val(in,i,j) * scale;
    }
  }

  // Iterate until the new (W) and old (U) solution differ by no more than epsilon.
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
  printf("image WxH: %dx%d, iterations: %d\n", width, height, iterations);

  // Save solution to output
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      val(out,i,j) = (unsigned char)(Clamp(val(u,i,j), 0, 1) * MAX_VAL + (real_t)0.5);
    }
  }

  free(w);
  free(u);
}

static unsigned char *generate_data(int width, int height) {
  unsigned char *data = (unsigned char *)malloc(sizeof(unsigned char) * width * height);
  //  Set the boundary values
  for (int i = 1; i < height-1; ++i) {
    val(data, i,       0) = HOT; // left
    val(data, i, width-1) = HOT; // right
  }
  for (int j = 0; j < width; ++j) {
    val(data, 0       , j) = COLD; // top
    val(data, height-1, j) = HOT; // bottom
  }
  //  Initialize the interior
  for (int i = 1; i < height-1; ++i) {
    for (int j = 1; j < width-1; ++j) {
        val(data, i, j) = WARM;
    }
  }
  return data;
}

static void write_data(char *file_name, unsigned char *data, int width, int height) {
  FILE *handle = fopen(file_name, "wb");
  fprintf(handle, "P5\n");
  fprintf(handle, "%d %d\n", width, height);
  fprintf(handle, "255\n");
  fwrite(data, width * sizeof(unsigned char), height, handle);
  fclose(handle);
}

static void create_dataset(int datasetNum, int width, int height) {

  const char *dir_name = htkDirectory_create(htkPath_join(base_dir, datasetNum));

  char *input_file_name  = htkPath_join(dir_name, "input.pgm");
  char *output_file_name = htkPath_join(dir_name, "expect.pgm");

  unsigned char *input_data  = generate_data(width, height);
  unsigned char *output_data = (unsigned char *)calloc(sizeof(unsigned char), width * height);

  compute(output_data, input_data, width, height);

  write_data(input_file_name, input_data, width, height);
  write_data(output_file_name, output_data, width, height);

  free(input_data);
  free(output_data);
}

int main() {
  base_dir = htkPath_join(htkDirectory_current(), "data");
  create_dataset(0, 50, 32);
  create_dataset(1, 768, 768);
  create_dataset(2, 1000, 750);
  create_dataset(3, 2048, 1024);
  return 0;
}
