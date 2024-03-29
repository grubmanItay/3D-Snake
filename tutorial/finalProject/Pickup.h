#pragma once

#include "MovingObject.h"
#include "SpawnManager.h"

namespace Game
{
class Pickup : public MovingObject
{
public:
    Pickup(std::shared_ptr<cg3d::Material> material, std::shared_ptr<cg3d::Model> model, SnakeGame* scene);
    void OnCollision();
    virtual void RunAction();
    static Pickup *SpawnObject(float xCoordinate, float yCoordinate, float zCoordinate, std::shared_ptr<cg3d::Material> material, std::shared_ptr<cg3d::Model> model, SnakeGame *scene);
    void Update();
    SpawnManager::InteractableObjects getType() { return type; }

private:
    int score = 5;
    SpawnManager::InteractableObjects type = SpawnManager::InteractableObjects::PICKUP;

}; 

} // namespace Game
