// Copyright 2021 Garena Online Private Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core.h"

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <queue>
#include <random>
#include <tuple>
#include <utility>

// TEST(OlympicsBaseTEST, TESTBUILD) { OlympicsBase(); }
TEST(CurlingTEST, TESTSTEP) {
  curling c;
  c.reset();
  // for (size_t i = 0; i < 1000; i++) {
  c.step({{200, 0}, {200, 0}});
  // }
}
