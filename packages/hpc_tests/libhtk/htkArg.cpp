
#include "htk.h"

#ifdef HTK_USE_SESSION_ID
static char * _sessionId = nullptr;
char * _envSessionId() {
#ifdef HTK_USE_UNIX
  if (_sessionId != nullptr) {
    char *env = std::getenv("SESSION_ID");
    if (env) {
      _sessionId = htkString_duplicate(env);
    }
  }
#endif /* HTK_USE_UNIX */
  return htkString_duplicate(_sessionId);
}
char * sessionId() {
  if (_sessionId != "") {
    return htkString_duplicate(_sessionId);
  }
  return htkString_duplicate(_envSessionId());
}
#else /* HTK_USE_SESSION_ID */
char * _envSessionId() {
  return htkString_duplicate("session_id_disabled");
}
char * sessionId() {
  return htkString_duplicate("session_id_disabled");
}
#endif /* HTK_USE_SESSION_ID */

htkArg_t htkArg_new(int *argc, char ***argv) {
  htkArg_t arg;

  htk_init(argc, argv);

  htkArg_setSessionId(arg, _envSessionId());
  htkArg_setInputCount(arg, 0);
  htkArg_setInputFiles(arg, NULL);
  htkArg_setOutputFile(arg, NULL);
  htkArg_setType(arg, NULL);
  htkArg_setExpectedOutputFile(arg, NULL);
  return arg;
}

void htkArg_delete(htkArg_t arg) {
  if (htkArg_getInputCount(arg) > 0 && htkArg_getInputFiles(arg) != nullptr) {
    int ii;
    for (ii = 0; ii < htkArg_getInputCount(arg); ii++) {
      htkDelete(htkArg_getInputFile(arg, ii));
    }
    htkDelete(htkArg_getInputFiles(arg));
    htkArg_setInputCount(arg, 0);
    htkArg_setInputFiles(arg, NULL);
  }
  if (htkArg_getOutputFile(arg)) {
    htkDelete(htkArg_getOutputFile(arg));
    htkArg_setOutputFile(arg, NULL);
  }
  if (htkArg_getExpectedOutputFile(arg)) {
    htkDelete(htkArg_getExpectedOutputFile(arg));
    htkArg_setExpectedOutputFile(arg, NULL);
  }
  if (htkArg_getType(arg)) {
    htkDelete(htkArg_getType(arg));
    htkArg_setType(arg, NULL);
  }
  return;
}

static int getInputFileCount(char *arg) {
  int count = 1;
  while (*arg != '\0' && *arg != '-') {
    if (*arg == ',') {
      count++;
    }
    arg++;
  }
  return count;
}

static char **parseInputFiles(char *arg, int *resCount) {
  int count;
  int ii = 0;
  char **files;
  char *token;

  count = getInputFileCount(arg);

  files = htkNewArray(char *, count);

  token = strtok(arg, ",");
  while (token != nullptr) {
    files[ii++] = htkString_duplicate(token);
    token       = strtok(NULL, ",");
  }
  *resCount = ii;
  return files;
}

static char *parseString(char *arg) {
  return htkString_duplicate(arg);
}

static void parseSessionId(char *arg) {
#ifdef HTK_USE_SESSION_ID
  _sessionId = std::string(arg);
#endif /* HTK_USE_SESSION_ID */
}

htkArg_t htkArg_read(int argc, char **argv) {
  int ii;
  htkArg_t arg;

  arg = htkArg_new(&argc, &argv);
  for (ii = 0; ii < argc; ii++) {
    if (htkString_startsWith(argv[ii], "-s")) {
      parseSessionId(argv[++ii]);
      htkArg_setSessionId(arg, sessionId());
    } else if (htkString_startsWith(argv[ii], "-i")) {
      int fileCount;
      char **files;

      files = parseInputFiles(argv[++ii], &fileCount);

      htkArg_setInputCount(arg, fileCount);
      htkArg_setInputFiles(arg, files);
    } else if (htkString_startsWith(argv[ii], "-o")) {
      char *file = parseString(argv[++ii]);
      htkArg_setOutputFile(arg, file);
    } else if (htkString_startsWith(argv[ii], "-e")) {
      char *file = parseString(argv[++ii]);
      htkArg_setExpectedOutputFile(arg, file);
    } else if (htkString_startsWith(argv[ii], "-t")) {
      char *type = parseString(argv[++ii]);
      htkArg_setType(arg, type);
    } else if (htkString_startsWith(argv[ii], "-h")){
      htkLog(ERROR, "./htkexample -s <sessionid> -e <file> -i <file1,...> -o <file> -t <type>\n");
    } else if (argv[ii][0] == '-') {
      htkLog(ERROR, "Unexpected program option ", argv[ii]);
    }
  }

  return arg;
}
