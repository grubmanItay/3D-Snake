#pragma once

#include "MovingObject.h"

namespace Game
{
class Obstacle : public MovingObject
{
public:
    Obstacle(std::shared_ptr<cg3d::Material> material, std::shared_ptr<cg3d::Model> model, SnakeGame* scene);
    void OnCollision();
    static Obstacle *SpawnObject(float xCoordinate, float yCoordinate, float zCoordinate, std::shared_ptr<cg3d::Material> material, std::shared_ptr<cg3d::Model> model, SnakeGame *scene);
    virtual void RunAction();
    void Update();

private:
    int damage;

}; 

} // namespace Game
