
#include "htk.h"

enum {
  htkMPI_timerTag          = 2,
  htkMPI_loggerTag         = 4,
  htkMPI_solutionExistsTag = 8,
  htkMPI_solutionTag       = 16
};

void htk_atExit(void) {
  using std::cout;
  using std::endl;

#ifdef HTK_USE_CUDA
  // cudaDeviceSynchronize();
#endif /* HTK_USE_CUDA */

  int nranks = rankCount();
  if (nranks > 1) {
#ifdef HTK_USE_MPI
    if (isMasterQ) {
#if defined(htkExit_print)
#if defined(JSON_OUTPUT)
      cout << "==$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << endl;

      cout << "{\n";
      cout << htkString_quote("timer") << ":";
      cout << "[\n";
      for (int ii = 0; ii < nranks; ii++) {
        if (ii == 0) {
          cout << htkTimer_toJSON();
        } else {
          const char *msg = htkMPI_getStringFromRank(ii, htkMPI_timerTag);
          if (msg != nullptr && strlen(msg) != 0) {
            cout << ",\n";
            cout << msg;
            //		 free(msg);
          }
        }
      }
      cout << "]" << endl; // close timer

      cout << "," << endl; // start logger
      cout << htkString_quote("logger") << ":";
      cout << "[\n";
      for (int ii = 0; ii < nranks; ii++) {
        if (ii == 0) {
          cout << htkLogger_toJSON();
        } else {
          const char *msg = htkMPI_getStringFromRank(ii, htkMPI_loggerTag);
          if (msg != nullptr && strlen(msg) != 0) {
            cout << ",\n";
            cout << msg;
            //		 free(msg);
          }
        }
      }
      cout << "]" << endl; // close logger

      cout << "," << endl; // start solutionExists
      cout << htkString_quote("cuda_memory") << ":" << _cudaMallocSize
           << ",\n";

      if (solutionJSON) {
        cout << htkString_quote("solution_exists") << ": true,\n";
        cout << htkString_quote("solution") << ":" << solutionJSON << "\n";
      } else {
        cout << htkString_quote("solution_exists") << ": false\n";
      }
      cout << "}" << endl; // close json

    } else {
      htkMPI_sendStringToMaster(htkTimer_toJSON().c_str(), htkMPI_timerTag);
      htkMPI_sendStringToMaster(htkLogger_toJSON().c_str(), htkMPI_loggerTag);
    }
#endif /* JSON_OUTPUT */
#endif /* htkExit_print */

#endif /* HTK_USE_MPI */
  } else {
#if defined(htkExit_print)
#if defined(JSON_OUTPUT)
    cout << "==$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << endl;

    cout << "{\n" << htkString_quote("timer") << ":[" << htkTimer_toJSON()
         << "],\n" << htkString_quote("logger") << ":[" << htkLogger_toJSON()
         << "],\n";

#ifdef HTK_USE_CUDA
    cout << htkString_quote("cuda_memory") << ":" << _cudaMallocSize
         << ",\n";
#endif /* HTK_USE_CUDA */

    if (solutionJSON) {
      cout << htkString_quote("solution_exists") << ": true,\n";
      cout << htkString_quote("solution") << ":" << solutionJSON << "\n";
    } else {
      cout << htkString_quote("solution_exists") << ": false\n";
    }
    cout << "}" << endl;
#endif /* JSON_OUTPUT */
#endif /* htkExit_print */
  }

  // htkTimer_delete(_timer);
  // htkLogger_delete(_logger);

  _timer  = nullptr;
  _logger = nullptr;

// htkFile_atExit();

#ifdef HTK_USE_CUDA
  cudaDeviceReset();
#endif

  exit(0);

  // assert(0);

  return;
}
