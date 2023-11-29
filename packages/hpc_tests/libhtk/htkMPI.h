
#ifndef __HTK_MPI_H__
#define __HTK_MPI_H__

#ifdef HTK_USE_MPI

#include <cstring>
#include <mpi/mpi.h>
#include <string>

#define isMasterQ ((htkMPI_getRank()) == 0)

extern int htkMPI_getRank();

extern int rankCount();

extern const char *htkMPI_getStringFromRank(int rank, int tag);
extern void htkMPI_sendStringToMaster(const char *str, int tag);

extern int htkMPI_Init(int *argc, char ***argv);

extern bool finalizedQ;

extern "C" int htkMPI_Finalize(void);
extern "C" void htkMPI_Exit(void);

#define MPI_Finalize htkMPI_Finalize

#else  /* HTK_USE_MPI */
static inline int rankCount() {
  return 1;
}
static inline int htkMPI_getRank() {
  return 0;
}
#endif /* HTK_USE_MPI */
#endif /* __HTK_MPI_H__ */
