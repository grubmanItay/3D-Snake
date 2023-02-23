
#include "Util.h"
#include "Utility.h"
#include <random>
#define DEBUG true



void Util::DebugPrint(std::string str){
    if (DEBUG)
        cg3d::debug_print(str);
}

void Util::PrintVector(Eigen::Vector3f vec)
{
    float x = vec(0);
    float y = vec(1);
    float z = vec(2);
    DebugPrint("Vector is : " +  std::to_string(x) + " " + std::to_string(y) + " " +  std::to_string(z) + " ");
}

float Util::GenerateRandomInRange(float min, float max)
{
    std::random_device randDevice;
	std::mt19937       randGenerator(randDevice());
	std::uniform_real_distribution<> distribute(min, max);
    return distribute(randGenerator);
}

