/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 *
 * The use of the code within this file and all code within files that
 * make up the software that is ElasticFusion is permitted for
 * non-commercial purposes only.  The full terms and conditions that
 * apply to the code within this file are detailed within the LICENSE.txt
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/>
 * unless explicitly stated.  By downloading this file you agree to
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#ifndef SPARTANLOGREADER_H_
#define SPARTANLOGREADER_H_

#include <Utils/Resolution.h>
#include <Utils/Stopwatch.h>
#include <pangolin/utils/file_utils.h>

#include "LogReader.h"

#include <cassert>
#include <zlib.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <stack>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgcodecs.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "yaml-cpp/yaml.h"
#include <fstream>

// Helper functions to enable loading of calibration params before creating a LogReader instance
bool isSpartanLog(std::string const& value);
void spartanGetParams(const std::string camera_info_filename, int& pixels_width, int& pixels_height, double& fx, double& fy, double& cx, double& cy);

class SpartanLogReader : public LogReader
{
    public:
        SpartanLogReader(const std::string & log_folder, bool flipColors);

        virtual ~SpartanLogReader();

        void getNext();

        void getBack();

        int getForwardKinematicsPose();

        int getNumFrames();

        bool hasMore();

        bool rewound();

        void rewind();

        void fastForward(int frame);

        const std::string getFile();

        void setAuto(bool value);

        std::stack<int> filePointers;

    private:
        void getCore();
        YAML::Node config_yaml;
};

#endif /* SPARTANLOGREADER_H_ */
