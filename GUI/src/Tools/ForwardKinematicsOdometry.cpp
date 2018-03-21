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

#include "ForwardKinematicsOdometry.h"

ForwardKinematicsOdometry::ForwardKinematicsOdometry(const std::string & filename)
{
    loadPoses(filename);
}

ForwardKinematicsOdometry::~ForwardKinematicsOdometry()
{

}

void ForwardKinematicsOdometry::loadPoses(const std::string & filename)
{
    std::cout << "Loading forward kin poses for file " << filename << std::endl;
    poses_yaml = YAML::LoadFile(filename);
    std::cout << "Loaded forward kin poses" << std::endl;
    
    for (size_t i = 0; i < poses_yaml.size(); i++) {

        YAML::Node current_pose = poses_yaml[i];

        float x, y, z, qx, qy, qz, qw;

        x = current_pose["camera_to_world"]["translation"]["x"].as<float>();
        y = current_pose["camera_to_world"]["translation"]["y"].as<float>();
        z = current_pose["camera_to_world"]["translation"]["z"].as<float>();

        qx = current_pose["camera_to_world"]["quaternion"]["x"].as<float>();
        qy = current_pose["camera_to_world"]["quaternion"]["y"].as<float>();
        qz = current_pose["camera_to_world"]["quaternion"]["z"].as<float>();
        qw = current_pose["camera_to_world"]["quaternion"]["w"].as<float>();
        
        Eigen::Quaternionf q(qw, qx, qy, qz);
        Eigen::Vector3f t(x, y, z);

        Eigen::Isometry3f T;
        T.setIdentity();
        T.pretranslate(t).rotate(q);
        camera_trajectory[i] = T;
    }
}

Eigen::Matrix4f ForwardKinematicsOdometry::getTransformation(uint64_t frame_number)
{

    Eigen::Matrix4f pose4f = Eigen::Matrix4f::Identity();

    if(frame_number != 0)
    {
        Eigen::Isometry3f pose3f = camera_trajectory[frame_number];

        Eigen::Matrix4f M = Eigen::Matrix4f::Identity();

        pose4f = M * camera_trajectory[0].inverse() * M * pose3f * M;

    }

    return pose4f;
}