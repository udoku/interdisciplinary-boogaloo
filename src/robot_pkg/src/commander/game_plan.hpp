#ifndef __GAME_PLAN_HPP__
#define __GAME_PLAN_HPP__
#include "common/common_header.hpp"
#include <functional>
#include <map>

#include "fallback_seq.hpp"
#include "lambda_task.hpp"
#include "task_manager_base.hpp"
#include "task_seq.hpp"

#include "managers/finish_manager.hpp"

#include "managers/demo_manager.hpp"
#include "managers/dropper_test_manager.hpp"
#include "managers/move_test_manager.hpp"
#include "managers/ultrasonic_test_manager.hpp"

#include "managers/start_manager.hpp"
#include "managers/plateau_manager.hpp"
#include "managers/obstacle_manager.hpp"
#include "managers/mobility_manager.hpp"
#include "managers/line_manager.hpp"
#include "managers/end_manager.hpp"

map<string, TaskPtr> tasks;

void setupTasks() {
    // NOTE: This task will be automatically appended to any task sequence
    // Don't modify it
    tasks["finish"] = TaskPtr(new FinishManager());

    // Testing tasks
    tasks["demo"] = TaskPtr(new DemoManager());
    tasks["drop"] = TaskPtr(new DropperTestManager());
    tasks["move"] = TaskPtr(new MoveTestManager());
    tasks["ultrasonic"] = TaskPtr(new UltrasonicTestManager());

    // Game task definitions
    tasks["start"] = TaskPtr(new StartManager());
    tasks["plateau"] = TaskPtr(new PlateauManager());
    tasks["obstacle"] = TaskPtr(new ObstacleManager());
    tasks["mobility"] = TaskPtr(new MobilityManager());
    tasks["line"] = TaskPtr(new LineManager());
    tasks["end"] = TaskPtr(new EndManager());

    // Game task ordering
    // NOTE: THIS IS THE ONLY PART OF THE CODE THAT SHOULD BE
    // MODIFIED BEFORE COMPETITION
    // TODO: SET THE NUMBERS TO THE ACTUAL TABLE ORDER
    tasks["task3"] = TaskPtr(tasks["plateau"]);
    tasks["task4"] = TaskPtr(tasks["obstacle"]);
    tasks["task1"] = TaskPtr(tasks["mobility"]);
    tasks["task2"] = TaskPtr(tasks["line"]);
    tasks["escape"] = TaskPtr(new TaskSeq({tasks["start"],
                                           tasks["task1"],
                                           tasks["task2"],
                                           tasks["task3"],
                                           tasks["task4"],
                                           tasks["end"]}));
}
#endif
