#pragma once



#include <string>
#include <Eigen/Core>

#include <Eigen/Geometry>

class Util{

    public:
        static void DebugPrint(std::string str);
        static void PrintVector(Eigen::Vector3f vec);
        static float GenerateRandomInRange(float min, float max);

    private:
};