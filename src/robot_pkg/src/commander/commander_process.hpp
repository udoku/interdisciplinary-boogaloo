#ifndef __COMMANDER_PROCESS__
#define __COMMANDER_PROCESS__

#include "common/common_header.hpp"

class CommanderProcess {
  public:
    /* Start a command process. Mode represents what actions the robot should do */
    CommanderProcess(string _taskToRun);
    virtual ~CommanderProcess();
    virtual int run(void);
    virtual string getProcessName(void);

    // Allow non-member code to call non-static functions inside this class
    static CommanderProcess* context_;

    // Send out the appropriate messages to reset the states of all other process
    void resetOtherProcesses();

  private:
    // This is the entry point for the thread that runs the task managers
    static void* startTaskManagerSequence(void* arg);
    pthread_t commander_thread_;
    static string task_to_run_;
};

#endif /** #ifndef __COMMANDER_PROCESS__ **/
