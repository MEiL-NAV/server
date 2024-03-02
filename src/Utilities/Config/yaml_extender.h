#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>

namespace YAML 
{
    template<>
    struct convert<Eigen::Vector3f> 
    {
        static Node encode(const Eigen::Vector3f& rhs) 
        {
            Node node;
            node.push_back(rhs.x());
            node.push_back(rhs.y());
            node.push_back(rhs.z());
            return node;
        }

        static bool decode(const Node& node, Eigen::Vector3f& rhs) 
        {
            if(!node.IsSequence() || node.size() != 3) 
            {
                return false;
            }

            rhs(0) = node[0].as<double>();
            rhs(1) = node[1].as<double>();
            rhs(2) = node[2].as<double>();
            return true;
        }
    };

    template<>
    struct convert<Eigen::Matrix3f> 
    {
        static Node encode(const Eigen::Matrix3f& rhs) 
        {
            Node node;
            node.push_back(rhs(0,0));
            node.push_back(rhs(0,1));
            node.push_back(rhs(0,2));
            node.push_back(rhs(1,0));
            node.push_back(rhs(1,1));
            node.push_back(rhs(1,2));
            node.push_back(rhs(2,0));
            node.push_back(rhs(2,1));
            node.push_back(rhs(2,2));
            return node;
        }

        static bool decode(const Node& node, Eigen::Matrix3f& rhs) 
        {
            if(!node.IsSequence() || node.size() != 9) 
            {
                return false;
            }

            rhs(0,0) = node[0].as<double>();
            rhs(0,1) = node[1].as<double>();
            rhs(0,2) = node[2].as<double>();
            rhs(1,0) = node[3].as<double>();
            rhs(1,1) = node[4].as<double>();
            rhs(1,2) = node[5].as<double>();
            rhs(2,0) = node[6].as<double>();
            rhs(2,1) = node[7].as<double>();
            rhs(2,2) = node[8].as<double>();
            return true;
        }
    };
}