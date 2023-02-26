#include "Obstacle.h"
#include <utility>
#include "Utility.h"
#include "SnakeGame.h"
#include "GameManager.h"
#include "GameObject.h"
#include "Util.h"
#include "Snake.h"
#include "SoundManager.h"
#include "SpawnManager.h"

#define OBSTACLE_CYCLES 50
#define DAMAGE 5

namespace Game{

Game::Obstacle::Obstacle(std::shared_ptr<cg3d::Material> material, std::shared_ptr<cg3d::Model> model, SnakeGame* scene) : Game::MovingObject(material, model, scene)
{
    this->cycles = OBSTACLE_CYCLES;
    this->damage = DAMAGE;
}

void Game::Obstacle::OnCollision()
{
    Util::DebugPrint(name + " " + "collided");
    RunAction();

    
}

Obstacle* Game::Obstacle::SpawnObject(float xCoordinate, float yCoordinate, float zCoordinate, std::shared_ptr<cg3d::Material> material, std::shared_ptr<cg3d::Model> model, SnakeGame* scene){
    //create object
	Game::Obstacle* movingObject = new Game::Obstacle{material, model, scene};
	//move to location
	scene->root->AddChild(movingObject->model);
	Eigen::Vector3f posVec{xCoordinate, yCoordinate, zCoordinate};
	movingObject->model->Translate(posVec);
	//add to logs
	Util::DebugPrint(movingObject->name + " added at : " + std::to_string(xCoordinate) + ", " + std::to_string(yCoordinate) + ", " + std::to_string(zCoordinate));
	return movingObject;
}

void Game::Obstacle::RunAction() {
    model->isHidden = true;
    model->Scale(Eigen::Vector3f({ 0.0001f, 0.0001f, 0.0001f }));
    permaGone = true;
    SetTimeOut();
    scene->gameManager->snake->GetHit(damage);
    scene->gameManager->spawnManager->PickupDestroyed(this);
    
    if (scene->IsActive())
        scene->gameManager->soundManager->PlayHitSound();
    else
        scene->gameManager->soundManager->PlayGameEndSound();
}

void Game::Obstacle::Update()
{
    
    if(!AdvanceTime()) //if not time to move or is not active, do not proceed
        return;
    // proceed to check collisions with other objects
   for (int i = 0; i < scene->gameManager->gameObjects.size(); i++) {
            auto elem = scene->gameManager->gameObjects.at(i);
        if (elem->name == this->name ) //temp - do not collide with self
            continue;
        if(isActive && CollidingWith(elem))
            if(elem->partOfSnake){
                OnCollision();
            }
    }
    // Move();
    
    
}

}