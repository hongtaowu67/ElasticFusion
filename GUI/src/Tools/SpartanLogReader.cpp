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

#include <sys/stat.h>
// from https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exist-using-standard-c-c11-c
inline bool file_exists (const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

bool isSpartanLog(std::string const& value)
{
    if (file_exists(value+"/pose_data.yaml")) {
        return true;
    }
    else {
        return false;
    }
}

void spartanGetParams(const std::string camera_info_filename, int& pixels_width, int& pixels_height, double& fx, double& fy, double& cx, double& cy) {
    YAML::Node camera_info_yaml = YAML::LoadFile(camera_info_filename);
    
    pixels_width = camera_info_yaml["image_width"].as<int>();
    pixels_height = camera_info_yaml["image_height"].as<int>();

    fx = camera_info_yaml["camera_matrix"]["data"][0].as<double>();
    fy = camera_info_yaml["camera_matrix"]["data"][4].as<double>();
    cx = camera_info_yaml["camera_matrix"]["data"][2].as<double>();
    cy = camera_info_yaml["camera_matrix"]["data"][5].as<double>();

    std::cout << "Loaded these params:" << std::endl;
    std::cout << "pixels_width, pixels_height: "<< pixels_width << ", " << pixels_height << std::endl;
    std::cout << "fx, fy, cx, cy: " << fx << " " << fy << " " << cx << " " << cy << std::endl;
}

SpartanLogReader::SpartanLogReader(const std::string & log_folder, bool flipColors)
 : LogReader(log_folder, flipColors)
{
    
    std::string config_filename = log_folder + "/pose_data.yaml";
    assert(pangolin::FileExists(config_filename.c_str()));
    fp = fopen(config_filename.c_str(), "rb");      // not sure why we need to do this . . . 
    config_yaml = YAML::LoadFile(config_filename);

    currentFrame = 0;
    numFrames = config_yaml.size();

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

int SpartanLogReader::getForwardKinematicsPose() {
    return currentFrame;
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

void SpartanLogReader::getCore()
{

    // Depth 
    std::string depth_filename = file+"/"+ZeroPadNumber(currentFrame)+"_depth.png";
    if (file_exists(depth_filename))
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
    std::string rgb_filename = file+"/"+ZeroPadNumber(currentFrame)+"_rgb.png";
    if (file_exists(rgb_filename))
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
  std::cout << "SpartanLogReader::fastForward not implemented" << std::endl;
}

int SpartanLogReader::getNumFrames()
{
    return numFrames;
}

bool SpartanLogReader::hasMore()
{
    return currentFrame + 1 <= numFrames;
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
