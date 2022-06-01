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

#include <benchmark/benchmark.h>

#include <queue>
#include <random>
#include <tuple>
#include <utility>

#include "core.h"

static void BM_StepAndReset(benchmark::State& state) {
  // Perform setup here
  curling c("/app/envpool/classic_control/testhelper/scenario.json");
  c.reset();
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> powerdis(-100.0, 200.0);
  std::uniform_real_distribution<> angledis(-30.0, 30.0);
  // double p1 = powerdis(gen), a1 = angledis(gen), p2 = powerdis(gen),
  //        a2 = angledis(gen);
  // std::cout << "step:" << i << std::endl;
  // std::cin >> p1 >> a1 >> p2 >> a2;
  // std::cout << p1 << ' ' << a1 << ' ' << p2 << ' ' << a2 << std::endl;
  // c.step({{p1, a1}, {p2, a2}});

  for (auto _ : state) {
    // This code gets timed
    c.step({{80, 0}, {80, 0}});
    c.reset();
  }
}
// Register the function as a benchmark
BENCHMARK(BM_StepAndReset);

// TEST(OlympicsBaseTEST, TESTBUILD) { OlympicsBase(); }
// TEST(CurlingTEST, TESTSTEP) {
//   curling c;
//   c.reset();
//   std::random_device rd;
//   std::mt19937 gen(rd());
//   std::uniform_real_distribution<> powerdis(-100.0, 200.0);
//   std::uniform_real_distribution<> angledis(-30.0, 30.0);
//   for (size_t i = 0; i < 500; i++) {
//     // double p1 = powerdis(gen), a1 = angledis(gen), p2 = powerdis(gen),
//     //        a2 = angledis(gen);
//     // std::cout << "step:" << i << std::endl;
//     // std::cin >> p1 >> a1 >> p2 >> a2;
//     // std::cout << p1 << ' ' << a1 << ' ' << p2 << ' ' << a2 << std::endl;
//     // c.step({{p1, a1}, {p2, a2}});
//     c.step({{200, 0}, {200, 0}});
//   }
// }
