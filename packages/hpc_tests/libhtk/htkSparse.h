
#ifndef __HTK_SPARSE_H__
#define __HTK_SPARSE_H__

EXTERN_C void CSRToJDS(int dim, int *csrRowPtr, int *csrColIdx,
                       float *csrData, int **jdsRowPerm, int **jdsRowNNZ,
                       int **jdsColStartIdx, int **jdsColIdx,
                       float **jdsData);

#endif /* __HTK_SPARSE_H__ */
