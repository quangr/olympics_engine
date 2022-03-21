/*
 * Copyright 2021 Garena Online Private Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
// https://github.com/openai/gym/blob/master/gym/envs/classic_control/Curling.py

#ifndef ENVPOOL_CLASSIC_CONTROL_Curling_H_
#define ENVPOOL_CLASSIC_CONTROL_Curling_H_

#include <cmath>
#include <random>

#include "core.h"
#include "envpool/core/async_envpool.h"
#include "envpool/core/env.h"

namespace classic_control {

class CurlingEnvFns {
 public:
  static decltype(auto) DefaultConfig() {
    return MakeDict("max_episode_steps"_.bind(2000),
                    "reward_threshold"_.bind(195.0), "img_height"_.bind(30),
                    "img_width"_.bind(30), "stack_num"_.bind(2));
  }
  template <typename Config>
  static decltype(auto) StateSpec(const Config& conf) {
    return MakeDict(
        "obs"_.bind(Spec<uint8_t>(
            {conf["stack_num"_], conf["img_height"_], conf["img_width"_]},
            {0, 7})),
        "info:reward"_.bind(Spec<double>({2})),
        "info:release"_.bind(Spec<bool>({1})),
        "info:pos"_.bind(Spec<double>({2})), "info:v"_.bind(Spec<double>({2})),
        "info:curteam"_.bind(Spec<int>({1})));
  }
  template <typename Config>
  static decltype(auto) ActionSpec(const Config& conf) {
    return MakeDict("action"_.bind(Spec<double>({4})));
  }
};

typedef class EnvSpec<CurlingEnvFns> CurlingEnvSpec;
typedef Spec<uint8_t> FrameSpec;
class CurlingEnv : public Env<CurlingEnvSpec>, public curling {
 protected:
  bool done_;
  int stack_num_;
  std::deque<Array> stack_buf_;
  std::vector<Array> maxpool_buf_;
  FrameSpec raw_spec_, resize_spec_, transpose_spec_;

 public:
  CurlingEnv(const Spec& spec, int env_id)
      : Env<CurlingEnvSpec>(spec, env_id),
        curling(),
        stack_num_(spec.config["stack_num"_]),
        done_(true),
        raw_spec_({30, 30}),
        resize_spec_({spec.config["img_height"_], spec.config["img_width"_]}),
        transpose_spec_(
            {spec.config["img_height"_], spec.config["img_width"_]}) {
    for (int i = 0; i < 2; ++i) {
      maxpool_buf_.push_back(std::move(Array(raw_spec_)));
    }
    for (int i = 0; i < stack_num_; ++i) {
      stack_buf_.push_back(Array(transpose_spec_));
    }
  }

  bool IsDone() override { return done_; }

  void Reset() override {
    // WONDERWHY
    // auto&& a = curling::reset(true);
    auto&& a = curling::reset();
    done_ = false;
    State state = Allocate();
    state["reward"_] = 0.0f;
    state["info:reward"_][0] = 0.0;
    state["info:reward"_][1] = 0.0;
    state["info:pos"_][0] = 0.0;
    state["info:pos"_][1] = 0.0;
    state["info:v"_][0] = 0.0;
    state["info:v"_][1] = 0.0;
    state["info:curteam"_] = 1;
    state["info:release"_] = false;
    PushStack(false, false);
    WriteObs(state);
  }

  void Step(const Action& action) override {
    State state = Allocate();
    std::cout << 1;
    // state["reward"_] = 0.0d;
    auto [ta, tb, tc, td] =
        curling::step({{action["action"_][0], action["action"_][1]},
                       {action["action"_][2], action["action"_][3]}});
    state["reward"_] = 0.0f;
    state["info:reward"_][0] = std::get<0>(tb);
    state["info:reward"_][1] = std::get<1>(tb);
    state["info:pos"_][0] = agent_pos[cur_ball][0];
    state["info:pos"_][1] = agent_pos[cur_ball][1];
    state["info:v"_][0] = agent_v[cur_ball][0];
    state["info:v"_][1] = agent_v[cur_ball][1];
    state["info:curteam"_] = current_team;
    state["info:release"_] = release;
    PushStack(false, false);
    WriteObs(state);
  }

 private:
  void PushStack(bool push_all, bool maxpool) {
    uint8_t* ptr = static_cast<uint8_t*>(obs_list.data());
    // if (maxpool) {
    //   uint8_t* ptr1 = static_cast<uint8_t*>(maxpool_buf_[1].data());
    //   for (std::size_t i = 0; i < maxpool_buf_[0].size; ++i) {
    //     ptr[i] = std::max(ptr[i], ptr1[i]);
    //   }
    // }
    stack_buf_.pop_front();
    stack_buf_.push_back(Array(transpose_spec_));
    memcpy(stack_buf_[stack_buf_.size() - 1].data(), ptr,
           stack_buf_[stack_buf_.size() - 1].size);
    // if (push_all) {
    //   for (auto& s : stack_buf_) {
    //     uint8_t* ptr_s = static_cast<uint8_t*>(s.data());
    //     if (ptr != ptr_s) {
    //       memcpy(ptr_s, ptr, size);
    //     }
    //   }
    // }
  };
  void WriteObs(State& state) {  // NOLINT
    for (int i = 0; i < stack_num_; ++i) {
      state["obs"_].Slice(i, i + 1).Assign(stack_buf_[i]);
    }
  }
};

typedef AsyncEnvPool<CurlingEnv> CurlingEnvPool;

}  // namespace classic_control

#endif  // ENVPOOL_CLASSIC_CONTROL_Curling_H_