#pragma once

#include "Pickup.h"
#include "SpawnManager.h"

namespace Game{

class HealthPickup : public Pickup{
    public:
        HealthPickup(std::shared_ptr<cg3d::Material> material, std::shared_ptr<cg3d::Model> model, SnakeGame* scene);
        static HealthPickup* SpawnObject(float xCoordinate, float yCoordinate, float zCoordinate, std::shared_ptr<cg3d::Material> material, std::shared_ptr<cg3d::Model> model, SnakeGame *scene);
        float healthValue = 5;
        SpawnManager::InteractableObjects getType() { return type; }


        void RunAction();
        SpawnManager::InteractableObjects type = SpawnManager::InteractableObjects::HEALTHPICKUP;
};

}