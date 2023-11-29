
#ifndef __HTK_LOGGER_H__
#define __HTK_LOGGER_H__

#include <stdint.h>

typedef enum en_htkLogLevel_t {
  htkLogLevel_unknown = -1,
  htkLogLevel_OFF     = 0,
  htkLogLevel_FATAL,
  htkLogLevel_ERROR,
  htkLogLevel_WARN,
  htkLogLevel_INFO,
  htkLogLevel_DEBUG,
  htkLogLevel_TRACE
} htkLogLevel_t;

struct st_htkLogEntry_t {
  char * id;
  char * session_id;
  int line;
  int mpiRank;
  char *msg;
  uint64_t time;
  const char *fun;
  const char *file;
  htkLogLevel_t level;
  htkLogEntry_t next;
};

struct st_htkLogger_t {
  char * id;
  char * session_id;
  int length;
  htkLogEntry_t head;
  htkLogLevel_t level;
};

#define htkLogEntry_getId(elem) ((elem)->id)
#define htkLogEntry_getSessionId(elem) ((elem)->session_id)
#define htkLogEntry_getMessage(elem) ((elem)->msg)
#define htkLogEntry_getMPIRank(elem) ((elem)->mpiRank)
#define htkLogEntry_getTime(elem) ((elem)->time)
#define htkLogEntry_getLevel(elem) ((elem)->level)
#define htkLogEntry_getNext(elem) ((elem)->next)
#define htkLogEntry_getLine(elem) ((elem)->line)
#define htkLogEntry_getFunction(elem) ((elem)->fun)
#define htkLogEntry_getFile(elem) ((elem)->file)

#define htkLogEntry_setId(elem, val) (htkLogEntry_getId(elem) = val)
#define htkLogEntry_setSessionId(elem, val)                                \
  (htkLogEntry_getSessionId(elem) = val)
#define htkLogEntry_setMessage(elem, val)                                  \
  (htkLogEntry_getMessage(elem) = val)
#define htkLogEntry_setMPIRank(elem, val)                                  \
  (htkLogEntry_getMPIRank(elem) = val)
#define htkLogEntry_setTime(elem, val) (htkLogEntry_getTime(elem) = val)
#define htkLogEntry_setLevel(elem, val) (htkLogEntry_getLevel(elem) = val)
#define htkLogEntry_setNext(elem, val) (htkLogEntry_getNext(elem) = val)
#define htkLogEntry_setLine(elem, val) (htkLogEntry_getLine(elem) = val)
#define htkLogEntry_setFunction(elem, val)                                 \
  (htkLogEntry_getFunction(elem) = val)
#define htkLogEntry_setFile(elem, val) (htkLogEntry_getFile(elem) = val)

#define htkLogger_getId(log) ((log)->id)
#define htkLogger_getSessionId(log) ((log)->session_id)
#define htkLogger_getLength(log) ((log)->length)
#define htkLogger_getHead(log) ((log)->head)
#define htkLogger_getLevel(log) ((log)->level)

#define htkLogger_setId(log, val) (htkLogger_getId(log) = val)
#define htkLogger_setSessionId(log, val) (htkLogger_getSessionId(log) = val)
#define htkLogger_setLength(log, val) (htkLogger_getLength(log) = val)
#define htkLogger_setHead(log, val) (htkLogger_getHead(log) = val)

#define htkLogger_incrementLength(log) (htkLogger_getLength(log)++)
#define htkLogger_decrementLength(log) (htkLogger_getLength(log)--)

#define htkLog(level, ...)                                                 \
  htkLogger_append(htkLogLevel_##level, htkString(__VA_ARGS__), htkFile,      \
                  htkFunction, htkLine)

extern htkLogger_t _logger;

htkLogger_t htkLogger_new();

void htkLogger_clear(htkLogger_t logger);

void htkLogger_delete(htkLogger_t logger);

void htkLogger_append(htkLogLevel_t level, string msg, const char *file,
                     const char *fun, int line);

string htkLogger_toXML(htkLogger_t logger);
string htkLogger_toXML();

string htkLogger_toJSON(htkLogger_t logger);
string htkLogger_toJSON();

#endif /* __HTK_LOGGER_H__ */
