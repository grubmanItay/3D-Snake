#pragma once

#include "Pickup.h"

namespace Game{

class HealthPickup : public Pickup{
    public:
        HealthPickup(std::shared_ptr<cg3d::Material> material, std::shared_ptr<cg3d::Model> model, SnakeGame* scene);
        static HealthPickup* SpawnObject(float xCoordinate, float yCoordinate, float zCoordinate, std::shared_ptr<cg3d::Material> material, std::shared_ptr<cg3d::Model> model, SnakeGame *scene);
        float healthValue = 5;

        void RunAction();
};

}