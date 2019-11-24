#include "vision_process.hpp"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, string(VISION_PROCESS));

    VisionProcess* p = new VisionProcess();
    p->run();
}

VisionProcess::VisionProcess() {
    detector_sub_ = nh_.subscribe(DESIRED_DETECTOR_TOPIC, 1, &VisionProcess::changeDetector, this);
    images_sub_ = nh_.subscribe(CAMERA_IMAGES_TOPIC, 1, &VisionProcess::processImage, this);
    
    
    detections_pub_ = nh_.advertise<robot_pkg::Detections>(DETECTIONS_TOPIC, 1);
    detection_image_pub_ = nh_.advertise<sensor_msgs::Image>(DETECTION_IMAGE_TOPIC, 10);
    current_detector_.detector_type = robot_pkg::Detector::NO_DETECTOR;
    new_detector_ = current_detector_;
}

VisionProcess::~VisionProcess() {}

int VisionProcess::run() {
    ROS_INFO("Vision setup. Waiting for frames");
    while (!current_frame_.valid) {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Obtained first set of frames");

    detector_timer_ = nh_.createTimer(ros::Duration(1.0/10.0), &VisionProcess::runDetectors, this);

    ros::spin();
    return 0;
}

void VisionProcess::runDetectors(const ros::TimerEvent& time) {
    checkSwitchDetectors();

    robot_pkg::RunDetector detector_service;
    detector_service.request.detector = current_detector_;
    detector_service.request.frame = current_frame_;
    detector_service.response.success = false;

    while (!ros::service::call(DETECTOR_RUN_TOPIC, detector_service)) {
        std::cout << "python service not available" << std::endl;
        ros::Duration(.5).sleep();
    }
    if (!detector_service.response.success) {
        if (current_detector_.detector_type != robot_pkg::Detector::NO_DETECTOR) {
            ROS_ERROR("The detector %d crashed or does not exist: can't run",
                        current_detector_.detector_type);
        }
        return;
    } else {
        std::vector<robot_pkg::Detection> global_dets;
        for (const auto& det : detector_service.response.detections.detections) {
            // TODO: This assumes vision converts to global. Maybe what we want, maybe not
            global_dets.push_back(det);
        }
        detector_service.response.detections.detections = global_dets;
    }

    detection_image_pub_.publish(detector_service.response.feedback);
    detections_pub_.publish(detector_service.response.detections);
}

void VisionProcess::changeDetector(const robot_pkg::Detector::ConstPtr& msg) {
    new_detector_.detector_type = msg->detector_type;
    new_detector_.detector_mode = msg->detector_mode;
}

void VisionProcess::processImage(const robot_pkg::Frame::ConstPtr& msg) {
    current_frame_.header = msg->header;
    current_frame_.image = msg->image;
    current_frame_.state = msg->state;
}

void VisionProcess::checkSwitchDetectors() {
    if (current_detector_.detector_type != new_detector_.detector_type ||
        current_detector_.detector_mode != new_detector_.detector_mode) {
        
        robot_pkg::SetupDetector setup_service;
        setup_service.request.detector = new_detector_;
        setup_service.response.success = false;
        std::cout << "attempting to initialize detector from " << current_detector_.detector_type
                  << " to " << new_detector_.detector_type << std::endl;
        
        while (!ros::service::call(DETECTOR_SETUP_TOPIC, setup_service)) {
            std::cout << "python service not available" << std::endl;
            ros::Duration(.5).sleep();
        }

        ROS_INFO("python handled setup? %d", setup_service.response.success);
        if (!setup_service.response.success) {
            ROS_ERROR("The detector %d does not exist: can't initialize",
                        new_detector_.detector_type);
        }
    }
    current_detector_ = new_detector_;
}