// Copyright 2017-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pacmod_interface/pacmod_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

#ifdef USE_AGNOCAST_ENABLED
#include "agnocast/agnocast_callback_isolated_executor.hpp"

#include <cstdlib>
#endif

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PacmodInterface>();

#ifdef USE_AGNOCAST_ENABLED
  const char * enable_agnocast = std::getenv("ENABLE_AGNOCAST");
  if (enable_agnocast && std::string(enable_agnocast) == "1") {
    agnocast::CallbackIsolatedAgnocastExecutor executor;
    executor.add_node(node);
    executor.spin();
  } else {
    rclcpp::spin(node);
  }
#else
  rclcpp::spin(node);
#endif

  rclcpp::shutdown();
  return 0;
}
