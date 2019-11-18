#include "utils.hpp"
//#include "commander_process.hpp"

namespace Timeout {
bool g_timeout_enabled = false;
long long int g_time_limit = 0;

void exitCheck() {
    pthread_testcancel();
    if (g_timeout_enabled && getCurrentMsCount() > g_time_limit) {
        ROS_ERROR("TIMELIMIT EXCEEDED. THROWING.");
        throw 0;
    }
}

bool runWithTimeout(function<bool()> code, int timeout, string name) {
    ROS_ERROR("Starting: %s", name.c_str());
    if (timeout > 0) {
        ROS_ERROR("(with timeout = %d seconds)", timeout);
        g_timeout_enabled = true;
        g_time_limit = timeout * 1000LL + getCurrentMsCount();
    }

    bool ret_val;
    try {
        ret_val = code();
        ROS_ERROR("Done calling: %s", name.c_str());
    } catch (const int i) {
        ROS_ERROR("Timedout on call: %s", name.c_str());
        ret_val = false;
        if (timeout <= 0) {
            throw;
        }
    }

    g_timeout_enabled = false;
    return ret_val;
}
} // namespace Timeout

namespace Motion {
/** Stores the most recent target state (ie the last target state sent to
 * Mobiltity) */
// RobotStateCommand g_last_target;

/** Ros message passing components */
//ros::Publisher g_state_command_pub;
//ros::Subscriber g_robot_state_sub;

void init(ros::NodeHandle nh) {
//     g_state_command_pub = nh.advertise<crt_system::RobotStateCommandMsg>(ROBOT_STATE_COMMAND_TOPIC, 1000);
//     g_robot_state_sub = nh.subscribe(ROBOT_STATE_TOPIC, 1000, &handleNewRobotStateMsg);
}


} // namespace Motion

namespace Recipes {

} // namespace Recipes

namespace Vision {
// pthread_mutex_t g_vision_mutex = PTHREAD_MUTEX_INITIALIZER;

// /** Stores the most recent detection for each object type */
// map<DetectionType, vector<Detection>> g_vision_data;

// /** Ros message passing components */
// ros::Publisher g_detector_type_pub;
// ros::Subscriber g_detections_sub;

void init(ros::NodeHandle nh) {
    // g_detector_type_pub = nh.advertise<crt_system::Detector>(DESIRED_DETECTOR_TOPIC, 1000);
    // g_detections_sub = nh.subscribe(DETECTIONS_TOPIC, 1, &handleNewVisionData);
}

// void handleNewVisionData(const crt_system::Detections::ConstPtr& detections) {
//     clearVisionData();
//     // ROS_INFO("New Vision Data");
//     for (crt_system::DetectionMsg det : detections->detections) {
//         // If there is not already a map entry for this detection type,
//         // add it into the map.
//         if (g_vision_data.find(det.detection_type) == g_vision_data.end()) {
//             pthread_mutex_lock(&g_vision_mutex);
//             g_vision_data[det.detection_type] = vector<Detection>();
//             pthread_mutex_unlock(&g_vision_mutex);
//         }
//         // Add the detection to the map's vector for the type
//         Detection detection(toVector3d(det.position), toVector3d(det.xDir), toVector3d(det.yDir));
//         pthread_mutex_lock(&g_vision_mutex);
//         g_vision_data[det.detection_type].push_back(detection);
//         pthread_mutex_unlock(&g_vision_mutex);
//     }
// }

// void clearVisionData() {
//     // Purge away any old left-over detections
//     pthread_mutex_lock(&g_vision_mutex);
//     g_vision_data.clear();
//     pthread_mutex_unlock(&g_vision_mutex);
// }

// void setDetector(DetectorType detector_type, int mode) {
//     Timeout::exitCheck();

//     if (detector_type == crt_system::Detector::NO_DETECTOR) {
//         ROS_INFO("Stopping vision");
//     } else {
//         ROS_INFO("Changing or starting a detector");
//     }

//     crt_system::Detector detector;
//     detector.detector_name = detector_type;
//     detector.detector_mode = mode;
//     g_detector_type_pub.publish(detector);

//     // Purge away any old left-over detections
//     clearVisionData();
// }

// vector<Detection> getCurrentVisionData(DetectionType object_type) {
//     Timeout::exitCheck();

//     pthread_mutex_lock(&g_vision_mutex);

//     auto detection = g_vision_data.find(object_type);
//     vector<Detection> ret_val;
//     if (detection == g_vision_data.end()) {
//         ret_val = vector<Detection>();
//     } else {
//         ret_val = (*detection).second;
//     }

//     pthread_mutex_unlock(&g_vision_mutex);

//     return ret_val;
// }

// bool waitVisionData(DetectionType object_type, double timeout, vector<Detection>* result,
//                       unsigned int min_num_detections, unsigned int max_num_detections) {
//     Timeout::exitCheck();

//     long long int cur_time = getCurrentMsCount();
//     vector<Detection> detection = getCurrentVisionData(object_type);

//     while (getCurrentMsCount() - cur_time < timeout * 1000 &&
//            (detection.size() < min_num_detections || detection.size() > max_num_detections)) {
//         ros::Duration(0.002).sleep();
//         detection = getCurrentVisionData(object_type);
//     }

//     bool success = detection.size() >= min_num_detections && detection.size() <= max_num_detections;

//     if (success && result) {
//         *result = detection;
//     }

//     return success;
// }

// bool waitVisionData(vector<DetectionType> object_types, double timeout, vector<Detection>* result,
//                       unsigned int min_num_detections, unsigned int max_num_detections) {
//     Timeout::exitCheck();

//     long long int cur_time = getCurrentMsCount();

//     vector<Detection> detections;
//     for (const auto& obj_type : object_types) {
//         vector<Detection> detection = getCurrentVisionData(obj_type);
//         for (const auto& det : detection) {
//             detections.push_back(det);
//         }
//     }

//     while (getCurrentMsCount() - cur_time < timeout * 1000 &&
//            (detections.size() < min_num_detections || detections.size() > max_num_detections)) {
//         detections.clear();
//         ros::Duration(0.002).sleep();
//         for (const auto& obj_type : object_types) {
//             vector<Detection> detection = getCurrentVisionData(obj_type);
//             for (const auto& det : detection) {
//                 detections.push_back(det);
//             }
//         }
//     }

//     bool success =
//         detections.size() >= min_num_detections && detections.size() <= max_num_detections;

//     if (success && result) {
//         *result = detections;
//     }

//     return success;
// }
} // namespace Vision

namespace Actuators {

void init(ros::NodeHandle nh) {
}


} // namespace Actuators
