

#ifndef __HTK_SOLUTION_H__
#define __HTK_SOLUTION_H__

typedef struct st_htkSolution_t {
  char * id;
  char * session_id;
  char *type;
  char *outputFile;
  void *data;
  int rows;
  int columns;
  int depth;
} htkSolution_t;

#define htkSolution_getId(sol) ((sol).id)
#define htkSolution_getSessionId(sol) ((sol).session_id)
#define htkSolution_getType(sol) ((sol).type)
#define htkSolution_getOutputFile(sol) ((sol).outputFile)
#define htkSolution_getData(sol) ((sol).data)
#define htkSolution_getRows(sol) ((sol).rows)
#define htkSolution_getColumns(sol) ((sol).columns)
#define htkSolution_getDepth(sol) ((sol).depth)

#define htkSolution_getHeight htkSolution_getRows
#define htkSolution_getWidth htkSolution_getColumns
#define htkSolution_getChannels htkSolution_getDepth

#define htkSolution_setId(sol, val) (htkSolution_getId(sol) = val)
#define htkSolution_setSessionId(sol, val)                                 \
  (htkSolution_getSessionId(sol) = val)
#define htkSolution_setType(sol, val) (htkSolution_getType(sol) = val)
#define htkSolution_setOutputFile(sol, val)                                \
  (htkSolution_getOutputFile(sol) = val)
#define htkSolution_setData(sol, val) (htkSolution_getData(sol) = val)
#define htkSolution_setRows(sol, val) (htkSolution_getRows(sol) = val)
#define htkSolution_setColumns(sol, val) (htkSolution_getColumns(sol) = val)
#define htkSolution_setDepth(sol, val) (htkSolution_getDepth(sol) = val)

htkBool htkSolution(char *expectedOutputFile, char *outputFile, char *type0,
                  void *data, int rows, int columns);
htkBool htkSolution(htkArg_t arg, void *data, int rows, int columns);
EXTERN_C htkBool htkSolution(htkArg_t arg, void *data, int rows);
htkBool htkSolution(htkArg_t arg, htkImage_t img);

#endif /* __HTK_SOLUTION_H__ */
