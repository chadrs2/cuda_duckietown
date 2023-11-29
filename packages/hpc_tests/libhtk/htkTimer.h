

#ifndef __HTK_TIMER_H__
#define __HTK_TIMER_H__

#ifdef HTK_USE_WINDOWS
extern uint64_t _hrtime_frequency;
#endif /* _WIN32 */

extern htkTimer_t _timer;

typedef enum en_htkTimerKind_t {
  htkTimerKind_Generic,
  htkTimerKind_IO,
  htkTimerKind_GPU,
  htkTimerKind_Copy,
  htkTimerKind_Driver,
  htkTimerKind_CopyAsync,
  htkTimerKind_Compute,
  htkTimerKind_CPUGPUOverlap,
} htkTimerKind_t;

struct st_htkTimerNode_t {
  int idx;
  char * id;
  char * session_id;
  int mpiRank;
  int level;
  htkBool stoppedQ;
  htkTimerKind_t kind;
  uint64_t startTime;
  uint64_t endTime;
  uint64_t elapsedTime;
  int startLine;
  int endLine;
  const char *startFunction;
  const char *endFunction;
  const char *startFile;
  const char *endFile;
  htkTimerNode_t next;
  htkTimerNode_t prev;
  htkTimerNode_t parent;
  char *msg;
};

struct st_htkTimer_t {
  char * id;
  char * session_id;
  size_t length;
  htkTimerNode_t head;
  htkTimerNode_t tail;
  uint64_t startTime;
  uint64_t endTime;
  uint64_t elapsedTime;
};

#define htkTimerNode_getIdx(node) ((node)->idx)
#define htkTimerNode_getId(node) ((node)->id)
#define htkTimerNode_getSessionId(node) ((node)->session_id)
#define htkTimerNode_getMPIRank(node) ((node)->mpiRank)
#define htkTimerNode_getLevel(node) ((node)->level)
#define htkTimerNode_getStoppedQ(node) ((node)->stoppedQ)
#define htkTimerNode_getKind(node) ((node)->kind)
#define htkTimerNode_getStartTime(node) ((node)->startTime)
#define htkTimerNode_getEndTime(node) ((node)->endTime)
#define htkTimerNode_getElapsedTime(node) ((node)->elapsedTime)
#define htkTimerNode_getStartLine(node) ((node)->startLine)
#define htkTimerNode_getEndLine(node) ((node)->endLine)
#define htkTimerNode_getStartFunction(node) ((node)->startFunction)
#define htkTimerNode_getEndFunction(node) ((node)->endFunction)
#define htkTimerNode_getStartFile(node) ((node)->startFile)
#define htkTimerNode_getEndFile(node) ((node)->endFile)
#define htkTimerNode_getNext(node) ((node)->next)
#define htkTimerNode_getPrevious(node) ((node)->prev)
#define htkTimerNode_getParent(node) ((node)->parent)
#define htkTimerNode_getMessage(node) ((node)->msg)

#define htkTimerNode_setIdx(node, val) (htkTimerNode_getIdx(node) = val)
#define htkTimerNode_setId(node, val) (htkTimerNode_getId(node) = val)
#define htkTimerNode_setSessionId(node, val)                               \
  (htkTimerNode_getSessionId(node) = val)
#define htkTimerNode_setMPIRank(node, val)                                 \
  (htkTimerNode_getMPIRank(node) = val)
#define htkTimerNode_setLevel(node, val) (htkTimerNode_getLevel(node) = val)
#define htkTimerNode_setStoppedQ(node, val)                                \
  (htkTimerNode_getStoppedQ(node) = val)
#define htkTimerNode_setKind(node, val) (htkTimerNode_getKind(node) = val)
#define htkTimerNode_setStartTime(node, val)                               \
  (htkTimerNode_getStartTime(node) = val)
#define htkTimerNode_setEndTime(node, val)                                 \
  (htkTimerNode_getEndTime(node) = val)
#define htkTimerNode_setElapsedTime(node, val)                             \
  (htkTimerNode_getElapsedTime(node) = val)
#define htkTimerNode_setStartLine(node, val)                               \
  (htkTimerNode_getStartLine(node) = val)
#define htkTimerNode_setEndLine(node, val)                                 \
  (htkTimerNode_getEndLine(node) = val)
#define htkTimerNode_setStartFunction(node, val)                           \
  (htkTimerNode_getStartFunction(node) = val)
#define htkTimerNode_setEndFunction(node, val)                             \
  (htkTimerNode_getEndFunction(node) = val)
#define htkTimerNode_setStartFile(node, val)                               \
  (htkTimerNode_getStartFile(node) = val)
#define htkTimerNode_setEndFile(node, val)                                 \
  (htkTimerNode_getEndFile(node) = val)
#define htkTimerNode_setNext(node, val) (htkTimerNode_getNext(node) = val)
#define htkTimerNode_setPrevious(node, val)                                \
  (htkTimerNode_getPrevious(node) = val)
#define htkTimerNode_setParent(node, val)                                  \
  (htkTimerNode_getParent(node) = val)
#define htkTimerNode_setMessage(node, val)                                 \
  (htkTimerNode_getMessage(node) = val)

#define htkTimerNode_stoppedQ(node)                                        \
  (htkTimerNode_getStoppedQ(node) == htkTrue)
#define htkTimerNode_hasNext(node) (htkTimerNode_getNext(node) != nullptr)
#define htkTimerNode_hasPrevious(node)                                     \
  (htkTimerNode_getPrevious(node) != nullptr)
#define htkTimerNode_hasParent(node) (htkTimerNode_getParent(node) != nullptr)

uint64_t _hrtime(void);

htkTimer_t htkTimer_new(void);
void htkTimer_delete(htkTimer_t timer);

string htkTimer_toJSON(htkTimer_t timer);
string htkTimer_toJSON();

string htkTimer_toXML(htkTimer_t timer);
string htkTimer_toXML();

htkTimerNode_t htkTimer_start(htkTimerKind_t kind, const char *file,
                            const char *fun, int line);
htkTimerNode_t htkTimer_start(htkTimerKind_t kind, string msg,
                            const char *file, const char *fun, int line);
void htkTimer_stop(htkTimerKind_t kind, string msg, const char *file,
                  const char *fun, int line);
void htkTimer_stop(htkTimerKind_t kind, const char *file, const char *fun,
                  int line);

#define htkTime_start(kind, ...)                                           \
  htkTimer_start(htkTimerKind_##kind, htkString(__VA_ARGS__), htkFile,        \
                htkFunction, htkLine)
#define htkTime_stop(kind, ...)                                            \
  htkTimer_stop(htkTimerKind_##kind, htkString(__VA_ARGS__), htkFile,         \
               htkFunction, htkLine)

#endif /* __HTK_TIMER_H__ */
