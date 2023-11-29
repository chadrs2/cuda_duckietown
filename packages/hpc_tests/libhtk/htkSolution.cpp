
#include "htk.h"

char *solutionJSON = nullptr;
static string _solution_correctQ("");

static void _onUnsameImageFunction(string str) {
  _solution_correctQ = str;
}

template <typename T>
static htkBool htkSolution_listCorrectQ(const char *expectedOutputFile,
                                      htkSolution_t sol, const char *type) {
  htkBool res;
  T *expectedData;
  int expectedRows, expectedColumns;

  expectedData = (T *)htkImport(expectedOutputFile, &expectedRows,
                               &expectedColumns, type);

  if (expectedData == nullptr) {
    _solution_correctQ = "Failed to open expected output file.";
    res                = htkFalse;
  } else if (expectedRows != htkSolution_getRows(sol)) {
    htkLog(TRACE, "Number of rows in the solution is ",
          htkSolution_getRows(sol), ". Expected number of rows is ",
          expectedRows, ".");
    _solution_correctQ =
        "The number of rows in the solution did not match "
        "that of the expected results.";
    res = htkFalse;
  } else if (expectedColumns != htkSolution_getColumns(sol)) {
    htkLog(TRACE, "Number of columns in the solution is ",
          htkSolution_getColumns(sol), ". Expected number of columns is ",
          expectedColumns, ".");
    _solution_correctQ = "The number of columns in the solution did not "
                         "match that of the expected results.";
    res = htkFalse;
  } else {
    int ii, jj, idx;
    T *solutionData;

    solutionData = (T *)htkSolution_getData(sol);
    if (htkSolution_getType(sol) == "integral_vector/sorted") {
        htkSort(solutionData, expectedRows * expectedColumns);
    }

    for (ii = 0; ii < expectedRows; ii++) {
      for (jj = 0; jj < expectedColumns; jj++) {
        idx = ii * expectedColumns + jj;
        if (htkUnequalQ(expectedData[idx], solutionData[idx])) {
          string str;
          if (expectedColumns == 1) {
            str = htkString(
                "The solution did not match the expected results at row ",
                ii, ". Expecting ", expectedData[idx], " but got ",
                solutionData[idx], ".");
          } else {
            str = htkString("The solution did not match the expected "
                           "results at column ",
                           jj, " and row ", ii, ". Expecting ",
                           expectedData[idx], " but got ",
                           solutionData[idx], ".");
          }
          _solution_correctQ = str;
          res                = htkFalse;
          goto matrixCleanup;
        }
      }
    }

    res = htkTrue;
  matrixCleanup:
    if (expectedData != nullptr) {
      htkFree(expectedData);
    }
  }
  return res;
}

static htkBool htkSolution_correctQ(char *expectedOutputFile,
                                  htkSolution_t sol) {
  if (expectedOutputFile == nullptr) {
    _solution_correctQ = "Failed to determined the expected output file.";
    return htkFalse;
  } else if (!htkFile_existsQ(expectedOutputFile)) {
    _solution_correctQ =
        htkString("The file ", expectedOutputFile, " does not exist.");
    return htkFalse;
  } else if (htkString_sameQ(htkSolution_getType(sol), "image")) {
    htkBool res;
    htkImage_t solutionImage = nullptr;
    htkImage_t expectedImage = htkImport(expectedOutputFile);
    if (expectedImage == nullptr) {
      _solution_correctQ = "Failed to open expected output file.";
      res                = htkFalse;
    } else if (htkImage_getWidth(expectedImage) !=
               htkSolution_getWidth(sol)) {
      _solution_correctQ =
          "The image width of the expected image does not "
          "match that of the solution.";
      res = htkFalse;
    } else if (htkImage_getHeight(expectedImage) !=
               htkSolution_getHeight(sol)) {
      _solution_correctQ =
          "The image height of the expected image does not "
          "match that of the solution.";
      res = htkFalse;
    } else if (htkImage_getChannels(expectedImage) !=
               htkSolution_getChannels(sol)) {
      _solution_correctQ =
          "The image channels of the expected image does not "
          "match that of the solution.";
      res = htkFalse;
    } else {
      solutionImage = (htkImage_t)htkSolution_getData(sol);
      htkAssert(solutionImage != nullptr);
      res = htkImage_sameQ(solutionImage, expectedImage,
                          _onUnsameImageFunction);
    }
    if (expectedImage != nullptr) {
      htkImage_delete(expectedImage);
    }
    return res;
  } else if (htkString_sameQ(htkSolution_getType(sol), "histogram")) {
    return htkSolution_listCorrectQ<unsigned char>(expectedOutputFile, sol,
                                                  "Integer");
  } else if (htkString_sameQ(htkSolution_getType(sol), "integral_vector/sorted") ||
             htkString_sameQ(htkSolution_getType(sol), "integral_vector")) {
    return htkSolution_listCorrectQ<int>(expectedOutputFile, sol,
                                        "Integer");
  } else if (htkString_sameQ(htkSolution_getType(sol), "vector") ||
             htkString_sameQ(htkSolution_getType(sol), "matrix")) {
    return htkSolution_listCorrectQ<htkReal_t>(expectedOutputFile, sol,
                                             "Real");
  } else {
    htkAssert(htkFalse);
    return htkFalse;
  }
}

htkBool htkSolution(char *expectedOutputFile, char *outputFile, char *type0,
                  void *data, int rows, int columns, int depth) {
  char *type;
  htkBool res;
  htkSolution_t sol;

  if (expectedOutputFile == nullptr || data == nullptr || type0 == nullptr) {
    htkLog(ERROR, "Failed to grade solution, %s %s %s", expectedOutputFile, data, type0);
    return htkFalse;
  }

  type = htkString_toLower(type0);

  if (_solution_correctQ != "") {
    _solution_correctQ = "";
  }

  htkSolution_setOutputFile(sol, outputFile);
  htkSolution_setId(sol, uuid());
  htkSolution_setSessionId(sol, sessionId());
  htkSolution_setType(sol, type);
  htkSolution_setData(sol, data);
  htkSolution_setRows(sol, rows);
  htkSolution_setColumns(sol, columns);
  htkSolution_setDepth(sol, depth);

  res = htkSolution_correctQ(expectedOutputFile, sol);

  if (outputFile != nullptr) {
    if (htkString_sameQ(type, "image")) {
      htkImage_t inputImage = (htkImage_t)data;
      htkImage_t img        = htkImage_new(htkImage_getWidth(inputImage),
                                  htkImage_getHeight(inputImage),
                                  htkImage_getChannels(inputImage));
      memcpy(htkImage_getData(img), htkImage_getData(inputImage),
             rows * columns * depth * sizeof(htkReal_t));
      htkExport(outputFile, img);
      htkImage_delete(img);
    } else if (htkString_sameQ(type, "integral_vector/sort")) {
      htkSort((int *)data, rows*columns);
      htkExport(outputFile, (int *)data, rows, columns);
    } else if (htkString_sameQ(type, "integral_vector")) {
      htkExport(outputFile, (int *)data, rows, columns);
    } else if (htkString_sameQ(type, "vector") ||
               htkString_sameQ(type, "matrix")) {
      htkExport(outputFile, (htkReal_t *)data, rows, columns);
    } else if (htkString_sameQ(type, "histogram")) {
      htkExport(outputFile, (unsigned char *)data, rows, columns);
    } else if (htkString_sameQ(type, "text")) {
      htkExport_text(outputFile, (unsigned char *)data, rows * columns);
    }
  }

  htkFree(type);

  return res;
}

htkBool htkSolution(char *expectedOutputFile, char *outputFile, char *type0,
                  void *data, int rows, int columns) {
  return htkSolution(expectedOutputFile, outputFile, type0, data, rows,
                    columns, 1);
}

htkBool htkSolution(htkArg_t arg, void *data, int rows, int columns,
                  int depth) {
  char *type;
  htkBool res;
  char *expectedOutputFile;
  char *outputFile;
  stringstream ss;

  expectedOutputFile = htkArg_getExpectedOutputFile(arg);
  outputFile         = htkArg_getOutputFile(arg);
  type               = htkArg_getType(arg);

  htkAssert(type != nullptr);
  htkAssert(expectedOutputFile != nullptr);
  htkAssert(outputFile != nullptr);

  res = htkSolution(expectedOutputFile, outputFile, type, data, rows,
                   columns, depth);

#if defined(htkSolution_print)
#if defined(JSON_OUTPUT)
  if (HTK_USE_JSON11) {
    json11::Json json;
    if (res) {
      json = json11::Json::object{{"correctq", true},
                                  {"message", "The solution is correct"}};
    } else {
      json = json11::Json::object{{"correctq", false},
                                  {"message", _solution_correctQ}};
    }
    json11::Json e =
        json11::Json::object{{"type", "solution"}, {"data", json}};
    ss << e.dump() << std::endl;
    solutionJSON = htkString_duplicate(json.string_value());
  } else {
    if (res) {
      ss << "{\n";
      ss << htkString_quote("correctq") << ": true,\n";
      ss << htkString_quote("message") << ": "
         << htkString_quote("Solution is correct.") << "\n";
      ss << "}";
    } else {
      ss << "{\n";
      ss << htkString_quote("correctq") << ": false,\n";
      ss << htkString_quote("message") << ": "
         << htkString_quote(_solution_correctQ) << "\n";
      ss << "}";
    }
    solutionJSON = htkString_duplicate(ss.str());
  }
  std::cout << ss.str();
#else
  printf("Solution is %scorrect\n", res ? "" : "NOT ");
#endif /* JSON_OUTPUT */
#endif /* htkSolution_print */
#if defined(htkSolution_log)
  htkLog(INFO, "Solution is ", res ? "" : "NOT ", "correct\n");
#endif /* htkSolution_log */

  return res;
}

htkBool htkSolution(htkArg_t arg, void *data, int rows, int columns) {
  return htkSolution(arg, data, rows, columns, 1);
}

EXTERN_C htkBool htkSolution(htkArg_t arg, void *data, int rows) {
  return htkSolution(arg, data, rows, 1);
}

htkBool htkSolution(htkArg_t arg, htkImage_t img) {
  return htkSolution(arg, img, htkImage_getHeight(img),
                    htkImage_getWidth(img), htkImage_getChannels(img));
}
