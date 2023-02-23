#pragma once 

#include <random>
#include  "SnakeGame.h"


// class SnakeGame;
// class Model;
namespace Game{

class SpawnManager{

    public:
        enum InteractableObjects{
            OBSTACLE,
            PICKUP,
            HEALTHPICKUP
        };

        SpawnManager(float xSize, float ySize, float zSize, SnakeGame* scene);
        void Spawnlevel(int level);
        void Spawnlevel(int pickups, int obstacles, int health);
        void PickupDestroyed(MovingObject* interactable);

        float xSize, ySize, zSize;
        SnakeGame* scene;

        int activePickups;
    private:
        void InitAssets();

        std::shared_ptr<cg3d::Model> CreatePickupModel();
        std::shared_ptr<cg3d::Model> CreateHealthPickupModel();
        std::shared_ptr<cg3d::Model> CreateObstacleModel();


        
        void SpawnPickupRandom(InteractableObjects type);
        void SpawnPickup(float x, float y, float z, InteractableObjects type);

        std::shared_ptr<cg3d::Material> pickupMaterial;
        std::shared_ptr<cg3d::Mesh> pickupMesh;
        std::shared_ptr<cg3d::Material> healthMaterial;
        std::shared_ptr<cg3d::Mesh> healthMesh;
        std::shared_ptr<cg3d::Material> obstacleMaterial;
        std::shared_ptr<cg3d::Mesh> obstacleMesh;
        
        std::shared_ptr<cg3d::Program> modelShader;

};
    

}