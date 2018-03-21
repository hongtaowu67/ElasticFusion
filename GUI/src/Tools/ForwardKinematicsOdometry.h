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

#ifndef FORWARDKINEMATICSODOMETRY_H_
#define FORWARDKINEMATICSODOMETRY_H_

#include "yaml-cpp/yaml.h"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <iostream>
#include <fstream>
#include <map>

#ifdef WIN32
#  include <cstdint>
#endif 

class ForwardKinematicsOdometry
{
    public:
        ForwardKinematicsOdometry(const std::string & filename);

        virtual ~ForwardKinematicsOdometry();

        Eigen::Matrix4f getPose(uint64_t frame_number);

        Eigen::Matrix4f getTransformation(uint64_t frame_number);

    private:
    	YAML::Node poses_yaml;
    	void loadPoses(const std::string & filename);
    	std::map<uint64_t, Eigen::Isometry3f> camera_trajectory;
        
};

#endif /* FORWARDKINEMATICSODOMETRY_H_ */
