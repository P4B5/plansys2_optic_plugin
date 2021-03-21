// Copyright 2019 Intelligent Robotics Lab
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

#ifndef PLANSYS2_OPTIC_PLAN_SOLVER__OPTIC_PLAN_SOLVER_HPP_
#define PLANSYS2_OPTIC_PLAN_SOLVER__OPTIC_PLAN_SOLVER_HPP_

#include <optional>
#include <memory>
#include <string>

#include "plansys2_core/PlanSolverBase.hpp"

namespace plansys2
{

class OPTICPlanSolver : public PlanSolverBase
{
public:
  OPTICPlanSolver();

  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr &, const std::string &) {}

  std::optional<Plan> getPlan(
    const std::string & domain, const std::string & problem,
    const std::string & node_namespace = "");

  std::string check_domain(
    const std::string & domain,
    const std::string & node_namespace = "");
};

}  // namespace plansys2

#endif  // PLANSYS2_OPTIC_PLAN_SOLVER__OPTIC_PLAN_SOLVER_HPP_
