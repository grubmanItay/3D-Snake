#include "Pickup.h"
#include <utility>
#include "Utility.h"
#include "SnakeGame.h"
#include "GameManager.h"
#include "GameObject.h"
#include "SpawnManager.h"
#include "Util.h"
#include "SoundManager.h"


#define PICKUP_CYCLES 50

namespace Game {

    Game::Pickup::Pickup(std::shared_ptr<cg3d::Material> material, std::shared_ptr<cg3d::Model> model, SnakeGame* scene) : Game::MovingObject(material, model, scene)
    {
        this->cycles = PICKUP_CYCLES;
    }

    void Game::Pickup::OnCollision()
    {
        Util::DebugPrint(name + " " + "collided");
        RunAction();


    }

    void Game::Pickup::RunAction() {
        // hide
        model->isHidden = true;
        //kill and timeout
        permaGone = true;
        SetTimeOut();
        // update score
        scene->gameManager->IncreaseScore(score);
        // notify game manager
        // log this
        Util::DebugPrint("Pickup " + name + " Destroyed");
        scene->gameManager->spawnManager->PickupDestroyed(this);
        //remove from objects
        
        scene->gameManager->soundManager->PlayPickupSound();

    }

    Pickup* Game::Pickup::SpawnObject(float xCoordinate, float yCoordinate, float zCoordinate, std::shared_ptr<cg3d::Material> material, std::shared_ptr<cg3d::Model> model, SnakeGame* scene) {
        Game::Pickup* movingObject = new Game::Pickup{ material, model, scene };
        //move to location
        scene->root->AddChild(movingObject->model);
        Eigen::Vector3f posVec{ xCoordinate, yCoordinate, zCoordinate };
        movingObject->model->Translate(posVec);
        //add to logs
        Util::DebugPrint(movingObject->name + " added at : " + std::to_string(xCoordinate) + ", " + std::to_string(yCoordinate) + ", " + std::to_string(zCoordinate));
        return movingObject;

    }


    void Game::Pickup::Update()
    {
        SetSpeed(scene->gameManager->GetCurrWave() * 1.5f);
        if (AdvanceTime()) {
            // proceed to check collisions with other objects
            for (int i = 0; i < scene->gameManager->gameObjects.size(); i++) {
                auto elem = scene->gameManager->gameObjects.at(i);
                if (elem->name == this->name) //temp - do not collide with self
                    continue;
                if (isActive && CollidingWith(elem))
                    if (elem->partOfSnake) {
                        OnCollision();
                        return;
                    }
            }
        } //if not time to move or is not active, do not proceed
        // change direction every random interval of time
        int randModifier = Util::GenerateRandomInRange(30, 60);
        if (ticks % (cycles * randModifier) == 0)
            // GenerateMoveVec();
            GenerateBezierCurve();
        if (isActive)
            Move();


    }

}