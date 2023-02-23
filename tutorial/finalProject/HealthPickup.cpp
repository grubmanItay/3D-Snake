#include "HealthPickup.h"
#include "GameManager.h"
#include "SnakeGame.h"
#include "Snake.h"
#include "Pickup.h"
#include "Util.h"
#include "SoundManager.h"

#define HEALTH_CYCLES 50

namespace Game{

Game::HealthPickup::HealthPickup(std::shared_ptr<cg3d::Material> material, std::shared_ptr<cg3d::Model> model, SnakeGame* scene) : Game::Pickup(material, model, scene)
{
    this->cycles = HEALTH_CYCLES;
}

HealthPickup* Game::HealthPickup::SpawnObject(float xCoordinate, float yCoordinate, float zCoordinate, std::shared_ptr<cg3d::Material> material, std::shared_ptr<cg3d::Model> model, SnakeGame* scene){
	Game::HealthPickup* movingObject = new Game::HealthPickup{material, model, scene};
	//move to location
	scene->root->AddChild(movingObject->model);
	Eigen::Vector3f posVec{xCoordinate, yCoordinate, zCoordinate};
	movingObject->model->Translate(posVec);
	//add to logs
	Util::DebugPrint(movingObject->name + " added at : " + std::to_string(xCoordinate) + ", " + std::to_string(yCoordinate) + ", " + std::to_string(zCoordinate));
	return movingObject;
}



void Game::HealthPickup::RunAction(){
    
    // hide
    model->isHidden=true;
    //kill and timeout
    permaGone = true;
     SetTimeOut();
    // update health
    scene->gameManager->snake->IncreaseHealth(healthValue);
    // notify game manager
    scene->gameManager->soundManager->PlayHealthSound();

}
}

