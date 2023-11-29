#include "htk.h"
#include "htkLogger.h"

#ifdef HTK_USE_WINDOWS
uint64_t _hrtime_frequency = 0;
#endif /* HTK_USE_WINDOWS */
htkTimer_t _timer = nullptr;

#ifdef HTK_USE_DARWIN
static double o_timebase    = 0;
static uint64_t o_timestart = 0;
#endif /* HTK_USE_DARWIN */

uint64_t _hrtime(void) {
#define NANOSEC ((uint64_t)1e9)
#ifdef HTK_USE_WINDOWS
  LARGE_INTEGER counter;
  if (!QueryPerformanceCounter(&counter)) {
    return 0;
  }
  return ((uint64_t)counter.LowPart * NANOSEC / _hrtime_frequency) +
         (((uint64_t)counter.HighPart * NANOSEC / _hrtime_frequency)
          << 32);
#else
  struct timespec ts;
#ifdef HTK_USE_DARWIN
#define O_NANOSEC (+1.0E-9)
#define O_GIGA UINT64_C(1000000000)
  if (!o_timestart) {
    mach_timebase_info_data_t tb{};
    mach_timebase_info(&tb);
    o_timebase = tb.numer;
    o_timebase /= tb.denom;
    o_timestart = mach_absolute_time();
  }
  double diff = (mach_absolute_time() - o_timestart) * o_timebase;
  ts.tv_sec   = diff * O_NANOSEC;
  ts.tv_nsec  = diff - (ts.tv_sec * O_GIGA);
#undef O_NANOSEC
#undef O_GIGA
#else  /* HTK_USE_DARWIN */
  clock_gettime(CLOCK_MONOTONIC, &ts);
#endif /* HTK_USE_DARWIN */
  return (((uint64_t)ts.tv_sec) * NANOSEC + ts.tv_nsec);
#endif /* HTK_USE_WINDOWS */
#undef NANOSEC
}

static inline uint64_t getTime(void) {
#ifdef HTK_USE_CUDA
  cudaDeviceSynchronize();
#endif /* HTK_USE_CUDA */
  return _hrtime();
}

static inline htkTimerNode_t htkTimerNode_new(int idx, htkTimerKind_t kind,
                                            const char *file,
                                            const char *fun,
                                            int startLine) {
  htkTimerNode_t node = htkNew(struct st_htkTimerNode_t);
  htkTimerNode_setId(node, uuid());
  htkTimerNode_setIdx(node, idx);
  htkTimerNode_setSessionId(node, sessionId());
  htkTimerNode_setMPIRank(node, htkMPI_getRank());
  htkTimerNode_setLevel(node, 0);
  htkTimerNode_setStoppedQ(node, htkFalse);
  htkTimerNode_setKind(node, kind);
  htkTimerNode_setStartTime(node, 0);
  htkTimerNode_setEndTime(node, 0);
  htkTimerNode_setElapsedTime(node, 0);
  htkTimerNode_setStartLine(node, startLine);
  htkTimerNode_setEndLine(node, 0);
  htkTimerNode_setStartFunction(node, fun);
  htkTimerNode_setEndFunction(node, NULL);
  htkTimerNode_setStartFile(node, file);
  htkTimerNode_setEndFile(node, NULL);
  htkTimerNode_setNext(node, NULL);
  htkTimerNode_setPrevious(node, NULL);
  htkTimerNode_setParent(node, NULL);
  htkTimerNode_setMessage(node, NULL);
  return node;
}

static inline void htkTimerNode_delete(htkTimerNode_t node) {
  if (node != nullptr) {
    if (htkTimerNode_getMessage(node)) {
      htkDelete(htkTimerNode_getMessage(node));
    }
    htkDelete(node);
  }
}

static inline const char *_nodeKind(htkTimerKind_t kind) {
  switch (kind) {
    case htkTimerKind_Generic:
      return "Generic";
    case htkTimerKind_IO:
      return "IO";
    case htkTimerKind_GPU:
      return "GPU";
    case htkTimerKind_Copy:
      return "Copy";
    case htkTimerKind_Driver:
      return "Driver";
    case htkTimerKind_CopyAsync:
      return "CopyAsync";
    case htkTimerKind_Compute:
      return "Compute";
    case htkTimerKind_CPUGPUOverlap:
      return "CPUGPUOverlap";
  }
  return "Undefined";
}

static inline json11::Json htkTimerNode_toJSONObject(htkTimerNode_t node) {
  if (node == nullptr) {
    return json11::Json{};
  }
  int parent_id = htkTimerNode_hasParent(node)
                      ? htkTimerNode_getIdx(htkTimerNode_getParent(node))
                      : -1;
  json11::Json json = json11::Json::object{
      {"id", htkTimerNode_getId(node)},
      {"session_id", htkTimerNode_getSessionId(node)},
      {"idx", htkTimerNode_getIdx(node)},
      {"mpi_rank", htkTimerNode_getMPIRank(node)},
      {"stopped", htkTimerNode_stoppedQ(node)},
      {"kind", _nodeKind(htkTimerNode_getKind(node))},
      {"start_time", htkTimerNode_getStartTime(node)},
      {"end_time", htkTimerNode_getEndTime(node)},
      {"elapsed_time", htkTimerNode_getElapsedTime(node)},
      {"start_line", htkTimerNode_getStartLine(node)},
      {"end_line", htkTimerNode_getEndLine(node)},
      {"start_function", htkTimerNode_getStartFunction(node)},
      {"end_function", htkTimerNode_getEndFunction(node)},
      {"start_file", htkTimerNode_getStartFile(node)},
      {"end_file", htkTimerNode_getEndFile(node)},
      {"parent_id", parent_id},
      {"message", htkTimerNode_getMessage(node)},
  };
  return json;
}

static inline void htkTimerNode_toLogger(htkTimerNode_t node) {
  if (node == nullptr) {
    return ;
  }
  int parent_id = htkTimerNode_hasParent(node)
                      ? htkTimerNode_getIdx(htkTimerNode_getParent(node))
                      : -1;
  htkLog(INFO, "[TIME][", _nodeKind(htkTimerNode_getKind(node)),
           "][", htkTimerNode_getMessage(node),
           "][", htkTimerNode_getEndFile(node),
           ": ", htkTimerNode_getStartLine(node), "-", htkTimerNode_getEndLine(node),
           "] Elapsed time: ", static_cast<double>(htkTimerNode_getElapsedTime(node)) * 1e-6, " ms");
  return ;
}

static inline string htkTimerNode_toJSON(htkTimerNode_t node) {
  if (node == nullptr) {
    return "";
  } else if (HTK_USE_JSON11) {
    json11::Json json = htkTimerNode_toJSONObject(node);
    return json.string_value();
  } else {
    stringstream ss;

    ss << "{\n";
    ss << htkString_quote("idx") << ":" << htkTimerNode_getIdx(node)
       << ",\n";
    ss << htkString_quote("id") << ":"
       << htkString_quote(htkTimerNode_getId(node)) << ",\n";
    ss << htkString_quote("session_id") << ":"
       << htkString_quote(htkTimerNode_getSessionId(node)) << ",\n";
    ss << htkString_quote("mpi_rank") << ":" << htkTimerNode_getMPIRank(node)
       << ",\n";
    ss << htkString_quote("stopped") << ":"
       << htkString(htkTimerNode_stoppedQ(node) ? "true" : "false") << ",\n";
    ss << htkString_quote("kind") << ":"
       << htkString_quote(_nodeKind(htkTimerNode_getKind(node))) << ",\n";
    ss << htkString_quote("start_time") << ":"
       << htkTimerNode_getStartTime(node) << ",\n";
    ss << htkString_quote("end_time") << ":" << htkTimerNode_getEndTime(node)
       << ",\n";
    ss << htkString_quote("elapsed_time") << ":"
       << htkTimerNode_getElapsedTime(node) << ",\n";
    ss << htkString_quote("start_line") << ":"
       << htkTimerNode_getStartLine(node) << ",\n";
    ss << htkString_quote("end_line") << ":" << htkTimerNode_getEndLine(node)
       << ",\n";
    ss << htkString_quote("start_function") << ":"
       << htkString_quote(htkTimerNode_getStartFunction(node)) << ",\n";
    ss << htkString_quote("end_function") << ":"
       << htkString_quote(htkTimerNode_getEndFunction(node)) << ",\n";
    ss << htkString_quote("start_file") << ":"
       << htkString_quote(htkTimerNode_getStartFile(node)) << ",\n";
    ss << htkString_quote("end_file") << ":"
       << htkString_quote(htkTimerNode_getEndFile(node)) << ",\n";
    ss << htkString_quote("parent_id") << ":"
       << htkString(htkTimerNode_hasParent(node)
                       ? htkTimerNode_getIdx(htkTimerNode_getParent(node))
                       : -1)
       << ",\n";
    ss << htkString_quote("message") << ":"
       << htkString_quote(htkTimerNode_getMessage(node)) << "\n";
    ss << "}";

    return ss.str();
  }
}

static inline string htkTimerNode_toXML(htkTimerNode_t node) {
  if (node == nullptr) {
    return "";
  } else {
    stringstream ss;

    ss << "<node>\n";
    ss << "<idx>" << htkTimerNode_getIdx(node) << "</id>\n";
    ss << "<id>" << htkTimerNode_getId(node) << "</id>\n";
    ss << "<session_id>" << htkTimerNode_getSessionId(node) << "</id>\n";
    ss << "<stoppedQ>"
       << htkString(htkTimerNode_stoppedQ(node) ? "true" : "false")
       << "</stoppedQ>\n";
    ss << "<kind>" << _nodeKind(htkTimerNode_getKind(node)) << "</kind>\n";
    ss << "<start_time>" << htkTimerNode_getStartTime(node)
       << "</start_time>\n";
    ss << "<end_time>" << htkTimerNode_getEndTime(node) << "</end_time>\n";
    ss << "<elapsed_time>" << htkTimerNode_getElapsedTime(node)
       << "</elapsed_time>\n";
    ss << "<start_line>" << htkTimerNode_getStartLine(node)
       << "</start_line>\n";
    ss << "<end_line>" << htkTimerNode_getEndLine(node) << "</end_line>\n";
    ss << "<start_function>" << htkTimerNode_getStartFunction(node)
       << "</start_function>\n";
    ss << "<end_function>" << htkTimerNode_getEndFunction(node)
       << "</end_function>\n";
    ss << "<start_file>" << htkTimerNode_getStartFile(node)
       << "</start_file>\n";
    ss << "<end_file>" << htkTimerNode_getEndFile(node) << "</end_file>\n";
    ss << "<parent_id>"
       << htkString(htkTimerNode_hasParent(node)
                       ? htkTimerNode_getIdx(htkTimerNode_getParent(node))
                       : -1)
       << "</parent_id>\n";
    ss << "<message>" << htkTimerNode_getMessage(node) << "</message>\n";
    ss << "</node>\n";

    return ss.str();
  }
}

#define htkTimer_getId(timer) ((timer)->id)
#define htkTimer_getSessionId(timer) ((timer)->session_id)
#define htkTimer_getLength(timer) ((timer)->length)
#define htkTimer_getHead(timer) ((timer)->head)
#define htkTimer_getTail(timer) ((timer)->tail)
#define htkTimer_getStartTime(timer) ((timer)->startTime)
#define htkTimer_getEndTime(timer) ((timer)->endTime)
#define htkTimer_getElapsedTime(timer) ((timer)->elapsedTime)

#define htkTimer_setId(timer, val) (htkTimer_getId(timer) = val)
#define htkTimer_setSessionId(timer, val)                                  \
  (htkTimer_getSessionId(timer) = val)
#define htkTimer_setLength(timer, val) (htkTimer_getLength(timer) = val)
#define htkTimer_setHead(timer, val) (htkTimer_getHead(timer) = val)
#define htkTimer_setTail(timer, val) (htkTimer_getTail(timer) = val)
#define htkTimer_setStartTime(node, val) (htkTimer_getStartTime(node) = val)
#define htkTimer_setEndTime(node, val) (htkTimer_getEndTime(node) = val)
#define htkTimer_setElapsedTime(node, val)                                 \
  (htkTimer_getElapsedTime(node) = val)

#define htkTimer_incrementLength(timer) (htkTimer_getLength(timer)++)
#define htkTimer_decrementLength(timer) (htkTimer_getLength(timer)--)

#define htkTimer_emptyQ(timer) (htkTimer_getLength(timer) == 0)

void htkTimer_delete(htkTimer_t timer) {
  if (timer != nullptr) {
    htkTimerNode_t tmp, iter;

    iter = htkTimer_getHead(timer);
    while (iter) {
      tmp = htkTimerNode_getNext(iter);
      htkTimerNode_delete(iter);
      iter = tmp;
    }

    htkDelete(timer);
  }
}

static json11::Json htkTimer_toJSONObject(htkTimer_t timer) {

  stringstream ss;
  htkTimerNode_t iter;
  uint64_t currentTime;
  std::vector<json11::Json> elems;

  currentTime = getTime();

  htkTimer_setEndTime(timer, currentTime);
  htkTimer_setElapsedTime(timer, currentTime - htkTimer_getStartTime(timer));

  for (iter = htkTimer_getHead(timer); iter != nullptr;
       iter = htkTimerNode_getNext(iter)) {
    if (!htkTimerNode_stoppedQ(iter)) {
      htkTimerNode_setEndTime(iter, currentTime);
      htkTimerNode_setElapsedTime(iter, currentTime -
                                           htkTimerNode_getStartTime(iter));
    }
    elems.push_back(htkTimerNode_toJSONObject(iter));
  }
  return json11::Json(elems);
}

string htkTimer_toJSON(htkTimer_t timer) {
  if (timer == nullptr) {
    return "";
  } else if (HTK_USE_JSON11) {
    json11::Json json = htkTimer_toJSONObject(timer);
    return json.string_value();
  } else {
    stringstream ss;
    htkTimerNode_t iter;
    uint64_t currentTime;

    currentTime = getTime();

    htkTimer_setEndTime(timer, currentTime);
    htkTimer_setElapsedTime(timer,
                           currentTime - htkTimer_getStartTime(timer));

    for (iter = htkTimer_getHead(timer); iter != nullptr;
         iter = htkTimerNode_getNext(iter)) {
      if (!htkTimerNode_stoppedQ(iter)) {
        htkTimerNode_setEndTime(iter, currentTime);
        htkTimerNode_setElapsedTime(
            iter, currentTime - htkTimerNode_getStartTime(iter));
      }
      ss << htkTimerNode_toJSON(iter);
      if (htkTimerNode_getNext(iter) != nullptr) {
        ss << ",\n";
      }
    }

    return ss.str();
  }
}

string htkTimer_toJSON() {
  return htkTimer_toJSON(_timer);
}

string htkTimer_toXML(htkTimer_t timer) {
  if (timer == nullptr) {
    return "";
  } else {
    stringstream ss;
    htkTimerNode_t iter;
    uint64_t currentTime;

    currentTime = getTime();

    htkTimer_setEndTime(timer, currentTime);
    htkTimer_setElapsedTime(timer,
                           currentTime - htkTimer_getStartTime(timer));

    ss << "<timer>\n";
    ss << "<start_time>" << htkTimer_getStartTime(timer)
       << "</start_time>\n";
    ss << "<end_time>" << htkTimer_getEndTime(timer) << "</end_time>\n";
    ss << "<elapsed_time>" << htkTimer_getElapsedTime(timer)
       << "</elapsed_time>\n";
    ss << "<elements>\n";
    for (iter = htkTimer_getHead(timer); iter != nullptr;
         iter = htkTimerNode_getNext(iter)) {
      if (!htkTimerNode_stoppedQ(iter)) {
        htkTimerNode_setEndTime(iter, currentTime);
        htkTimerNode_setElapsedTime(
            iter, currentTime - htkTimerNode_getStartTime(iter));
      }
      ss << htkTimerNode_toXML(iter);
    }
    ss << "</elements>\n";
    ss << "</timer>\n";

    return ss.str();
  }
}

string htkTimer_toXML() {
  return htkTimer_toXML(_timer);
}

htkTimer_t htkTimer_new(void) {
  htkTimer_t timer = htkNew(struct st_htkTimer_t);
  htkTimer_setId(timer, uuid());
  htkTimer_setSessionId(timer, sessionId());
  htkTimer_setLength(timer, 0);
  htkTimer_setHead(timer, NULL);
  htkTimer_setTail(timer, NULL);
  htkTimer_setStartTime(timer, getTime());
  htkTimer_setEndTime(timer, 0);
  htkTimer_setElapsedTime(timer, 0);

  return timer;
}

static inline htkTimerNode_t _findParent(htkTimer_t timer) {
  htkTimerNode_t iter;

  for (iter = htkTimer_getTail(timer); iter != nullptr;
       iter = htkTimerNode_getPrevious(iter)) {
    if (!htkTimerNode_stoppedQ(iter)) {
      return iter;
    }
  }
  return NULL;
}

static inline void _insertIntoList(htkTimer_t timer, htkTimerNode_t node) {
  if (htkTimer_emptyQ(timer)) {
    htkTimer_setHead(timer, node);
    htkTimer_setTail(timer, node);
  } else {
    htkTimerNode_t end = htkTimer_getTail(timer);
    htkTimer_setTail(timer, node);
    htkTimerNode_setNext(end, node);
    htkTimerNode_setPrevious(node, end);
  }
  htkTimer_incrementLength(timer);
}

htkTimerNode_t htkTimer_start(htkTimerKind_t kind, const char *file,
                            const char *fun, int line) {
  int id;
  uint64_t currentTime;
  htkTimerNode_t node;
  htkTimerNode_t parent;

  // htk_init(NULL, NULL);

  currentTime = getTime();

  id = htkTimer_getLength(_timer);

  node = htkTimerNode_new(id, kind, file, fun, line);

  parent = _findParent(_timer);
  _insertIntoList(_timer, node);

  htkTimerNode_setStartTime(node, currentTime);
  htkTimerNode_setParent(node, parent);
  if (parent != nullptr) {
    htkTimerNode_setLevel(node, htkTimerNode_getLevel(parent) + 1);
  }

  return node;
}

htkTimerNode_t htkTimer_start(htkTimerKind_t kind, string msg,
                            const char *file, const char *fun, int line) {
  htkTimerNode_t node = htkTimer_start(kind, file, fun, line);
  htkTimerNode_setMessage(node, htkString_duplicate(msg));
  return node;
}

static inline htkTimerNode_t _findNode(htkTimer_t timer, htkTimerKind_t kind,
                                      string msg) {
  htkTimerNode_t iter;

  for (iter = htkTimer_getTail(timer); iter != nullptr;
       iter = htkTimerNode_getPrevious(iter)) {
    if (msg == "") {
      if (!htkTimerNode_stoppedQ(iter) &&
          htkTimerNode_getKind(iter) == kind) {
        return iter;
      }
    } else {
      if (!htkTimerNode_stoppedQ(iter) &&
          htkTimerNode_getKind(iter) == kind &&
          msg == htkTimerNode_getMessage(iter)) {
        return iter;
      }
    }
  }
  return NULL;
}

void htkTimer_stop(htkTimerKind_t kind, string msg, const char *file,
                  const char *fun, int line) {
  uint64_t currentTime;
  htkTimerNode_t node;

  currentTime = getTime();

  node = _findNode(_timer, kind, msg);

  htkAssert(node != nullptr);
  if (node == nullptr) {
    return;
  }

  htkTimerNode_setEndTime(node, currentTime);
  htkTimerNode_setElapsedTime(node,
                             currentTime - htkTimerNode_getStartTime(node));
  htkTimerNode_setEndLine(node, line);
  htkTimerNode_setEndFunction(node, fun);
  htkTimerNode_setEndFile(node, file);
  htkTimerNode_setStoppedQ(node, htkTrue);

  if (node) {
#if defined(htkTimer_print)
#if defined(JSON_OUTPUT)
    json11::Json json = json11::Json::object{
      {"type", "timer"},
      {"id", htkTimerNode_getId(node)},
      {"session_id", htkTimerNode_getSessionId(node)},
      {"data", htkTimerNode_toJSONObject(node)}};
    std::cout << json.dump() << std::endl;
#else
    printf("[%s] Elapsed time: %.3f ms\n",
      _nodeKind(htkTimerNode_getKind(node)),
      static_cast<double>(htkTimerNode_getElapsedTime(node)) * 1e-6);
#endif /* JSON_OUTPUT */
#endif /* htkTimer_print */
#if defined(htkTimer_log)
    htkTimerNode_toLogger(node);
#endif /* htkTimer_log */
  }
  return;
}

void htkTimer_stop(htkTimerKind_t kind, const char *file, const char *fun,
                  int line) {
  htkTimer_stop(kind, "", file, fun, line);
}
