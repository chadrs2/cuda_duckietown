
#include "htk.h"
#include "vendor/catch.hpp"

TEST_CASE("Can create Raw dataset", "[DataGenerator]") {
  htkGenerateParams_t params;
  params.raw.rows   = 2;
  params.raw.cols   = 300;
  params.raw.minVal = 0;
  params.raw.maxVal = 30;
  params.raw.type   = htkType_integer;
  htkDataset_generate(
      htkPath_join(htkDirectory_current(), "test-dataset", "test.raw"),
      htkExportKind_raw, params);
}

TEST_CASE("Can create Text dataset", "[DataGenerator]") {
  htkGenerateParams_t params;
  params.text.length = 2000;
  htkDataset_generate(
      htkPath_join(htkDirectory_current(), "test-dataset", "test.txt"),
      htkExportKind_text, params);
}
