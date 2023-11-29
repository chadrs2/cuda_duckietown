
#include "htk.h"

htkLogger_t _logger = nullptr;

static inline htkBool htkLogEntry_hasNext(htkLogEntry_t elem) {
  return htkLogEntry_getNext(elem) != nullptr;
}

static inline htkLogEntry_t htkLogEntry_new() {
  htkLogEntry_t elem;

  elem = htkNew(struct st_htkLogEntry_t);

  htkLogEntry_setId(elem, uuid());
  htkLogEntry_setSessionId(elem, sessionId());
  htkLogEntry_setMessage(elem, NULL);
  htkLogEntry_setMPIRank(elem, htkMPI_getRank());
  htkLogEntry_setTime(elem, _hrtime());

  htkLogEntry_setLevel(elem, htkLogLevel_TRACE);

  htkLogEntry_setNext(elem, NULL);

  htkLogEntry_setLine(elem, -1);
  htkLogEntry_setFile(elem, NULL);
  htkLogEntry_setFunction(elem, NULL);

  return elem;
}

static inline htkLogEntry_t
htkLogEntry_initialize(htkLogLevel_t level, string msg, const char *file,
                      const char *fun, int line) {
  htkLogEntry_t elem;

  elem = htkLogEntry_new();

  htkLogEntry_setLevel(elem, level);

  htkLogEntry_setMessage(elem, htkString_duplicate(msg));

  htkLogEntry_setLine(elem, line);
  htkLogEntry_setFile(elem, file);
  htkLogEntry_setFunction(elem, fun);

  return elem;
}

static inline void htkLogEntry_delete(htkLogEntry_t elem) {
  if (elem != nullptr) {
    if (htkLogEntry_getMessage(elem) != nullptr) {
      htkFree(htkLogEntry_getMessage(elem));
    }
    htkDelete(elem);
  }
  return;
}

static inline const char *getLevelName(htkLogLevel_t level) {
  switch (level) {
    case htkLogLevel_unknown:
      return "Unknown";
    case htkLogLevel_OFF:
      return "Off";
    case htkLogLevel_FATAL:
      return "Fatal";
    case htkLogLevel_ERROR:
      return "Error";
    case htkLogLevel_WARN:
      return "Warn";
    case htkLogLevel_INFO:
      return "Info";
    case htkLogLevel_DEBUG:
      return "Debug";
    case htkLogLevel_TRACE:
      return "Trace";
  }
  return NULL;
}

static inline json11::Json htkLogEntry_toJSONObject(htkLogEntry_t elem) {
  if (elem == nullptr) {
    return json11::Json{};
  }
  json11::Json json = json11::Json::object{
      {"id", htkLogEntry_getId(elem)},
      {"session_id", htkLogEntry_getSessionId(elem)},
      {"mpi_rank", htkLogEntry_getMPIRank(elem)},
      {"level", getLevelName(htkLogEntry_getLevel(elem))},
      {"file", htkLogEntry_getFile(elem)},
      {"function", htkLogEntry_getFunction(elem)},
      {"line", htkLogEntry_getLine(elem)},
      {"time", htkLogEntry_getTime(elem)},
      {"message", htkLogEntry_getMessage(elem)},
  };
  return json;
}

static inline string htkLogEntry_toJSON(htkLogEntry_t elem) {
  if (elem == NULL) {
    return "\"\"";
  } else if (HTK_USE_JSON11) {
    json11::Json json = htkLogEntry_toJSONObject(elem);
    return json.string_value();
  } else {
    stringstream ss;

    ss << "{\n";
    ss << htkString_quote("id") << ":"
       << htkString_quote(htkLogEntry_getId(elem)) << ",\n";
    ss << htkString_quote("session_id") << ":"
       << htkString_quote(htkLogEntry_getSessionId(elem)) << ",\n";
    ss << htkString_quote("mpi_rank") << ":"
       << htkString(htkLogEntry_getMPIRank(elem)) << ",\n";
    ss << htkString_quote("level") << ":"
       << htkString_quote(getLevelName(htkLogEntry_getLevel(elem))) << ",\n";
    ss << htkString_quote("message") << ":"
       << htkString_quote(htkLogEntry_getMessage(elem)) << ",\n";
    ss << htkString_quote("file") << ":"
       << htkString_quote(htkLogEntry_getFile(elem)) << ",\n";
    ss << htkString_quote("function") << ":"
       << htkString_quote(htkLogEntry_getFunction(elem)) << ",\n";
    ss << htkString_quote("line") << ":" << htkLogEntry_getLine(elem)
       << ",\n";
    ss << htkString_quote("time") << ":" << htkLogEntry_getTime(elem)
       << "\n";
    ss << "}";

    return ss.str();
  }
  return "";
}

static inline string htkLogEntry_toXML(htkLogEntry_t elem) {
  if (elem != nullptr) {
    stringstream ss;

    ss << "<entry>\n";
    ss << "<type>"
       << "LoggerElement"
       << "</type>\n";
    ss << "<id>" << htkLogEntry_getId(elem) << "</id>\n";
    ss << "<session_id>" << htkLogEntry_getSessionId(elem)
       << "</session_id>\n";
    ss << "<level>" << htkLogEntry_getLevel(elem) << "</level>\n";
    ss << "<message>" << htkLogEntry_getMessage(elem) << "</message>\n";
    ss << "<file>" << htkLogEntry_getFile(elem) << "</file>\n";
    ss << "<function>" << htkLogEntry_getFunction(elem) << "</function>\n";
    ss << "<line>" << htkLogEntry_getLine(elem) << "</line>\n";
    ss << "<time>" << htkLogEntry_getTime(elem) << "</time>\n";
    ss << "</entry>\n";

    return ss.str();
  }
  return "";
}

htkLogger_t htkLogger_new() {
  htkLogger_t logger;

  logger = htkNew(struct st_htkLogger_t);

  htkLogger_setId(logger, uuid());
  htkLogger_setSessionId(logger, sessionId());
  htkLogger_setLength(logger, 0);
  htkLogger_setHead(logger, NULL);

  htkLogger_getLevel(logger) = htkLogLevel_TRACE;

  return logger;
}

static inline void _htkLogger_setLevel(htkLogger_t logger,
                                      htkLogLevel_t level) {
  htkLogger_getLevel(logger) = level;
}

static inline void _htkLogger_setLevel(htkLogLevel_t level) {
  _htkLogger_setLevel(_logger, level);
}

#define htkLogger_setLevel(level) _htkLogger_setLevel(htkLogLevel_##level)

void htkLogger_clear(htkLogger_t logger) {
  if (logger != nullptr) {
    htkLogEntry_t tmp;
    htkLogEntry_t iter;

    iter = htkLogger_getHead(logger);
    while (iter != nullptr) {
      tmp = htkLogEntry_getNext(iter);
      htkLogEntry_delete(iter);
      iter = tmp;
    }

    htkLogger_setLength(logger, 0);
    htkLogger_setHead(logger, NULL);
  }
}

void htkLogger_delete(htkLogger_t logger) {
  if (logger != nullptr) {
    htkLogger_clear(logger);
    htkDelete(logger);
  }
  return;
}

void htkLogger_append(htkLogLevel_t level, string msg, const char *file,
                     const char *fun, int line) {
  htkLogEntry_t elem;
  htkLogger_t logger;

  htk_init(NULL, NULL);

  logger = _logger;

  if (htkLogger_getLevel(logger) < level) {
    return;
  }

  elem = htkLogEntry_initialize(level, msg, file, fun, line);

#if defined(htkLogger_print)
  if (elem) {
#if defined(JSON_OUTPUT)
    json11::Json json = json11::Json::object{
        {"type", "logger"},
        {"id", htkLogEntry_getId(elem)},
        {"session_id", htkLogEntry_getSessionId(elem)},
        {"data", htkLogEntry_toJSONObject(elem)}};
    std::cout << json.dump() << std::endl;
#else
  printf("%s\n", htkLogEntry_getMessage(elem));
#endif /* JSON_OUTPUT */
  }
#endif /* htkLogger_print */
#if !defined(htkLogger_log)
  return;
#endif /* htkLogger_log */

  if (htkLogger_getHead(logger) == nullptr) {
    htkLogger_setHead(logger, elem);
  } else {
    htkLogEntry_t prev = htkLogger_getHead(logger);

    while (htkLogEntry_hasNext(prev)) {
      prev = htkLogEntry_getNext(prev);
    }
    htkLogEntry_setNext(prev, elem);
  }

#if 0
  if (elem) {
    const char *levelName = getLevelName(level);

    fprintf(stderr, "= LOG: %s: %s (In %s:%s on line %d). =\n", levelName,
            htkLogEntry_getMessage(elem), htkLogEntry_getFile(elem),
            htkLogEntry_getFunction(elem), htkLogEntry_getLine(elem));
  }
#endif

  htkLogger_incrementLength(logger);

  return;
}

string htkLogger_toJSON() {
  return htkLogger_toJSON(_logger);
}

static json11::Json htkLogger_toJSONObject(htkLogger_t logger) {
  std::vector<json11::Json> elems{};

  if (logger != nullptr) {
    htkLogEntry_t iter;
    stringstream ss;

    for (iter = htkLogger_getHead(logger); iter != nullptr;
         iter = htkLogEntry_getNext(iter)) {
      elems.push_back(htkLogEntry_toJSONObject(iter));
    }
  }
  return json11::Json(elems);
}

string htkLogger_toJSON(htkLogger_t logger) {
  if (logger != nullptr) {
    htkLogEntry_t iter;
    stringstream ss;

    for (iter = htkLogger_getHead(logger); iter != nullptr;
         iter = htkLogEntry_getNext(iter)) {
      ss << htkLogEntry_toJSON(iter);
      if (htkLogEntry_getNext(iter) != nullptr) {
        ss << ",\n";
      }
    }

    return ss.str();
  }
  return "";
}

string htkLogger_toXML() {
  return htkLogger_toXML(_logger);
}

string htkLogger_toXML(htkLogger_t logger) {
  if (logger != nullptr) {
    htkLogEntry_t iter;
    stringstream ss;

    ss << "<logger>\n";
    ss << "<type>"
       << "Logger"
       << "</type>\n";
    ss << "<id>" << htkLogger_getId(logger) << "</id>\n";
    ss << "<session_id>" << htkLogger_getSessionId(logger)
       << "</session_id>\n";
    ss << "<elements>\n";
    for (iter = htkLogger_getHead(logger); iter != nullptr;
         iter = htkLogEntry_getNext(iter)) {
      ss << htkLogEntry_toXML(iter);
    }
    ss << "</elements>\n";
    ss << "</logger>\n";

    return ss.str();
  }
  return "";
}
