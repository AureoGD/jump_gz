#ifndef TOOLS_GZ_H
#define TOOLS_GZ_H

#include <memory>
#include <set>
#include <string>

#include <OsqpEigen/OsqpEigen.h>
#include "gz/custom_msgs/vector.pb.h"
#include <gz/msgs.hh>

class ToolsGZ
{
public:
    // Read vector data from a sdf file
    void VectorDataFromSDF(const std::string _data, Eigen::VectorXd &_vec);

    // write new msg from eigeen vector data
    // void EigenVec2VecMsg(const Eigen::VectorXd &mat, gz::custom_msgs::vector *msg);
    void EigenVec2VecMsg(const Eigen::VectorXd &mat, gz::msgs::Double_V *msg);

    // write eigeen vector from msg data
    // void VecMsg2VecEigen(const gz::custom_msgs::vector &msg, Eigen::VectorXd *vec);
    void VecMsg2VecEigen(const gz::msgs::Double_V &msg, Eigen::VectorXd *vec);
};

#endif