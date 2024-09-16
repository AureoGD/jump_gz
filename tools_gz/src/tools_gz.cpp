#include "tools_gz/tools_gz.h"

void ToolsGZ::VectorDataFromSDF(const std::string _data, Eigen::VectorXd &_vec)
{
    std::string aux;
    int pos = 0;
    for (int idx = 0; idx <= _data.size(); idx++)
    {
        if (_data[idx] != ',' && _data[idx] != '\0')
        {
            aux += _data[idx];
        }
        else
        {
            _vec[pos] = std::stod(aux);
            pos++;
            aux = "";
        }
    }
}

// write new msg from eigeen vector data
// void ToolsGZ::EigenVec2VecMsg(const Eigen::VectorXd &mat, gz::custom_msgs::vector *msg)
// {
//     msg->mutable_data()->Reserve(mat.rows());
//     for (int ii = 0; ii < mat.rows(); ii++)
//     {
//         msg->add_data(mat(ii));
//     }
// }

// write eigeen vector from msg data
// void ToolsGZ::VecMsg2VecEigen(const gz::custom_msgs::vector &msg, Eigen::VectorXd *vec)
// {
//     vec->resize(msg.data_size());
//     for (int ii = 0; ii < msg.data_size(); ii++)
//     {
//         vec->operator()(ii) = msg.data(ii);
//     }
// }

void ToolsGZ::EigenVec2VecMsg(const Eigen::VectorXd &mat, gz::msgs::Double_V *msg)
{
    msg->mutable_data()->Reserve(mat.rows());
    for (int ii = 0; ii < mat.rows(); ii++)
    {
        msg->add_data(mat(ii));
    }
}

void ToolsGZ::VecMsg2VecEigen(const gz::msgs::Double_V &msg, Eigen::VectorXd *vec)
{
    vec->resize(msg.data_size());
    for (int ii = 0; ii < msg.data_size(); ii++)
    {
        vec->operator()(ii) = msg.data(ii);
    }
}