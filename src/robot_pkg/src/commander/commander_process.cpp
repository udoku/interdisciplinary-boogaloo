#include <signal.h>

#include "commander_process.hpp"
#include "game_plan.hpp"
#include "utils.hpp"

int main(int argc, char* argv[]) {
    // TODO: Make sure arguments to each constructor are correct
    // Init ros
    ros::init(argc, argv, string(COMMAND_PROCESS));

    ros::NodeHandle nh;
    // Init message passing stuff
    Motion::init(nh);
    Actuators::init(nh);
    Vision::init(nh);
    // Init process and run
    CommanderProcess p(argc > 1 ? string(argv[1]) : "");
    p.run();
    return 0;
}

// Ensure that a system-wide reset is sent if this process is killed
void sigHandler(int s __attribute__((unused))) {
    exit(0);
}

CommanderProcess* CommanderProcess::context_ = NULL;
string CommanderProcess::task_to_run_ = "";

CommanderProcess::CommanderProcess(string _taskToRun) {
    task_to_run_ = _taskToRun;
    assert(context_ == NULL);
    context_ = this;
    setupTasks();
}

CommanderProcess::~CommanderProcess() { context_ = NULL; }

int CommanderProcess::run() {
    if (tasks.find(task_to_run_) == tasks.end()) {
        ROS_ERROR("Task \"%s\" not found. Check spelling and/or game_plan.hpp.", task_to_run_.c_str());
        return 1;
    }

    // Set up the signal handler
    struct sigaction sig_int_handler;
    sig_int_handler.sa_handler = sigHandler;
    sigemptyset(&sig_int_handler.sa_mask);
    sig_int_handler.sa_flags = 0;
    sigaction(SIGINT, &sig_int_handler, NULL);

    // Wait for the hardware process
    // if (!getenv("IGNORE_HARDWARE")) {
    //     ROS_INFO("Waiting for current state");
    //     while (!Motion::currentStateInit()) {
    //         usleep(100 * 1000);
    //         ros::spinOnce();
    //     }
    // }

    // Wait for the kill switch

    // if (getenv("WAIT_FOR_KILL_SWITCH_REMOVAL") && !getenv("IGNORE_HARDWARE")) {
    //     ROS_INFO("Waiting for kill switch to be taken OFF");
    //     while (!Motion::getCurrentState().killed_ && !DRY_RUN) {
    //         usleep(100 * 1000);
    //         ros::spinOnce();
    //     }
    // }

    while (true) {

        // Wait for the kill switch
        // if (!getenv("IGNORE_HARDWARE")) {
        //     ROS_INFO("Waiting for kill switch to be put ON");
        //     while (Motion::getCurrentState().killed_ && !DRY_RUN) {
        //         usleep(100 * 1000);
        //         ros::spinOnce();
        //     }
        // }

        // Give whoever set the kill switch a few seconds to move away
        // if (getenv("IGNORE_HARDWARE")) {
        //     ROS_INFO("Ignoring kill switch; starting now");
        //     usleep(1000 * 1000);
        // } else {
        //     ROS_INFO("Kill switch on. Waiting 6 seconds...");
        //     usleep(6000 * 1000);
        // }

        // Reset all processes and then start the mission in a separate thread
        ROS_INFO("About to start commander");
        // System::systemWideReset();
        pthread_create(&commander_thread_, NULL, CommanderProcess::startTaskManagerSequence,
                       NULL);
        ROS_INFO("Created commander");
        // Wait for us to be killed
        //while (DRY_RUN || !Motion::getCurrentState().killed_ || getenv("IGNORE_HARDWARE")) {
        while (true) {
            usleep(100 * 1000);
            // ROS_INFO("Commander Spin");
            ros::spinOnce();
        }

        ROS_ERROR("Commander noticed that the kill switch was removed.");

        // Now we need to kill the task manager that we spawned.

        // Note, this does not garantee an immediate kill. However, the thread will
        // be killed if it ever tries to send a message sleep, or do certain other
        // system calls. TODO: Is it bad if a thread is killed when it makes the
        // send() system call from within the message passer?
        pthread_cancel(commander_thread_);

        // Tell other processes to stop PID'ing and looking for targets
        // System::systemWideReset();

        // Reset all the tasks
        setupTasks();
    }

    return 0;
}

string CommanderProcess::getProcessName() { return string(COMMAND_PROCESS); }

// This is the entry point for the therad that runs the task managers
void* CommanderProcess::startTaskManagerSequence(void* arg __attribute__((unused))) {
    ROS_ERROR("Running %s", task_to_run_.c_str());

    // Give the diver a moment and get setup for the run
    // System::systemWideReset();

    usleep(200 * 1000);

    // Run main task
    tasks[task_to_run_]->run();

    // System::systemWideReset();

    ROS_ERROR("Finished. Exited Task Thread.");
    return 0;
}
