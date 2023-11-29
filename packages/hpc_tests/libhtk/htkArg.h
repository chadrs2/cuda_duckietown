

#ifndef __HTK_ARG_H__
#define __HTK_ARG_H__

struct st_htkArg_t {
  char * sessionId;
  int inputCount;
  char **inputFiles;
  char *outputFile;
  char *expectedOutput;
  char *type;
};

#define htkArg_getInputCount(wa) ((wa).inputCount)
#define htkArg_getInputFiles(wa) ((wa).inputFiles)
/* FIXME: htkArg_getInputFile: check for .inputFiles == nullptr or .inputCount < ii+1 */
#define htkArg_getInputFile(wa, ii) (htkArg_getInputFiles(wa)[ii])
#define htkArg_getOutputFile(wa) ((wa).outputFile)
#define htkArg_getSessionId(wa) ((wa).sessionId)
#define htkArg_getExpectedOutputFile(wa) ((wa).expectedOutput)
#define htkArg_getType(wa) ((wa).type)

#define htkArg_setSessionId(wa, val) (htkArg_getSessionId(wa) = val)
#define htkArg_setInputCount(wa, val) (htkArg_getInputCount(wa) = val)
#define htkArg_setInputFiles(wa, val) (htkArg_getInputFiles(wa) = val)
#define htkArg_setInputFile(wa, ii, val) (htkArg_getInputFile(wa, ii) = val)
#define htkArg_setOutputFile(wa, val) (htkArg_getOutputFile(wa) = val)
#define htkArg_setExpectedOutputFile(wa, val) \
  (htkArg_getExpectedOutputFile(wa) = val)
#define htkArg_setType(wa, val) (htkArg_getType(wa) = val)

EXTERN_C htkArg_t htkArg_new(int *argc, char ***argv);
EXTERN_C void htkArg_delete(htkArg_t arg);
EXTERN_C htkArg_t htkArg_read(int argc, char **argv);

char* sessionId();
char * _envSessionId();

#endif /* __HTK_ARG_H__ */
