// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <atomic>
#include <memory>
#include <string>
namespace wildos_ackermann_rpp {
struct PathProgress {
  std::atomic<bool> active{false};
  std::atomic<bool> finish_allowed{false};
};
// Both plugins live in controller_server and share its exact node/controller key.
std::shared_ptr<PathProgress> path_progress(const std::string & key);
}
