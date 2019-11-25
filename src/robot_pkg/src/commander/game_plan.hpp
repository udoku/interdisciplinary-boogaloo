#ifndef __GAME_PLAN_HPP__
#define __GAME_PLAN_HPP__
#include "common/common_header.hpp"
#include <functional>
#include <map>

#include "fallback_seq.hpp"
#include "lambda_task.hpp"
#include "task_manager_base.hpp"
#include "task_seq.hpp"

#include "managers/demo_manager.hpp"
#include "managers/dropper_test_manager.hpp"

map<string, TaskPtr> tasks;

void setupTasks() {
    tasks["demo"] = TaskPtr(new DemoManager());
    tasks["drop"] = TaskPtr(new DropperTestManager());
}
#endif
