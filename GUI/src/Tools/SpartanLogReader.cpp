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

#include "SpartanLogReader.h"

bool isSpartanLog(std::string const& value)
{
    std::string ending = ".bag";
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

void spartanGetParams(const SpartanLogData & log_data, int& pixels_width, int& pixels_height, double& fx, double& fy, double& cx, double& cy) {
    rosbag::Bag bag;
    
    bag.open(log_data.ros_bag_filename, rosbag::bagmode::Read);
    
    // Pete ToDo: provide CLI for setting topic names (done by manuelli!)
    std::string cam_info_topic = log_data.cam_info_topic;
    
    std::vector<std::string> topics;
    topics.push_back(cam_info_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    { 
        if (m.getTopic() == cam_info_topic || ("/" + m.getTopic() == cam_info_topic)) {
            sensor_msgs::CameraInfo::ConstPtr cam_info = m.instantiate<sensor_msgs::CameraInfo>();
            if (cam_info != NULL) {
                pixels_width = cam_info->width;
                pixels_height = cam_info->height;
                fx = cam_info->K[0];
                fy = cam_info->K[4];
                cx = cam_info->K[2];
                cy = cam_info->K[5];
                bag.close();
                return;
            }
        }
    }
    std::cout << "Did not find camera info!" << std::endl;
    exit(0);
}

SpartanLogReader::SpartanLogReader(const SpartanLogData & log_data, bool flipColors)
 : LogReader(log_data.ros_bag_filename, flipColors)
{
    assert(pangolin::FileExists(log_data.ros_bag_filename.c_str()));

    // not sure why we need to do this . . . 
    fp = fopen(log_data.ros_bag_filename.c_str(), "rb");

    currentFrame = 0;

    // Need to set this later
    numFrames = 741;

    depthReadBuffer = new unsigned char[numPixels * 2];
    imageReadBuffer = new unsigned char[numPixels * 3];
    decompressionBufferDepth = new Bytef[numPixels * 2];
    decompressionBufferImage = new Bytef[numPixels * 3];
}

SpartanLogReader::~SpartanLogReader()
{
    delete [] depthReadBuffer;
    delete [] imageReadBuffer;
    delete [] decompressionBufferDepth;
    delete [] decompressionBufferImage;

    fclose(fp);
}

void SpartanLogReader::getBack()
{
    currentFrame = numFrames;
    getCore();
}

void SpartanLogReader::getNext()
{
    getCore();
}

std::string ZeroPadNumber(int num)
{
    std::stringstream ss;
    
    // the number is converted to string with the help of stringstream
    ss << num; 
    std::string ret;
    ss >> ret;
    
    // Append zero chars
    int str_length = ret.length();
    for (int i = 0; i < 6 - str_length; i++)
        ret = "0" + ret;
    return ret;
}

#include <sys/stat.h>
// from https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exist-using-standard-c-c11-c
inline bool image_exists (const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

void SpartanLogReader::getCore()
{

    std::string config_filename = "/home/peteflo/spartan/sandbox/fusion/fusion_1521222309.47/images/pose_data.yaml";
    YAML::Node config_yaml = YAML::LoadFile(config_filename);
    std::cout << config_yaml[0]["camera_to_world"] << std::endl;

    std::cout << "current frame " << currentFrame << std::endl;
    std::cout << "padded " << ZeroPadNumber(currentFrame) << std::endl;

    // Depth 
    std::string depth_filename = "/home/peteflo/spartan/sandbox/fusion/fusion_1521222309.47/images/"+ZeroPadNumber(currentFrame)+"_depth.png";
    if (image_exists(depth_filename))
    {
        cv::Mat cv_depth;
        cv_depth = cv::imread(depth_filename, CV_16UC1);
        memcpy(&decompressionBufferDepth[0], cv_depth.ptr(), numPixels * 2);
    }
    else
    {
        std::cout << "This filename didn't exist " << depth_filename << std::endl;
        exit(0);
    }

    // RGB
    std::string rgb_filename = "/home/peteflo/spartan/sandbox/fusion/fusion_1521222309.47/images/"+ZeroPadNumber(currentFrame)+"_rgb.png";
    if (image_exists(rgb_filename))
    {
        cv::Mat cv_rgb;
        cv_rgb = cv::imread(rgb_filename, cv::IMREAD_COLOR);
        memcpy(&decompressionBufferImage[0], cv_rgb.ptr(), numPixels * 3);
    } else {
        std::cout << "This filename didn't exist " << rgb_filename << std::endl;
        exit(0);
    }

    rgb = (unsigned char *)&decompressionBufferImage[0];
    depth = (unsigned short *)&decompressionBufferDepth[0];

    if(flipColors)
    {
        for(int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3)
        {
            std::swap(rgb[i + 0], rgb[i + 2]);
        }
    }

    currentFrame++;
}

void SpartanLogReader::fastForward(int frame)
{
  std::cout << "ROSBagReader::fastForward not implemented" << std::endl;
}

int SpartanLogReader::getNumFrames()
{
    return numFrames;
}

bool SpartanLogReader::hasMore()
{
    return currentFrame + 1 < numFrames;
}


void SpartanLogReader::rewind()
{
    currentFrame = 0;
}

bool SpartanLogReader::rewound()
{
    return (currentFrame == 0);
}

const std::string SpartanLogReader::getFile()
{
    return file;
}

void SpartanLogReader::setAuto(bool value)
{

}
