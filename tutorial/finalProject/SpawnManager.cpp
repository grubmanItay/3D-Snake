#include  "SpawnManager.h"
#include  "SnakeGame.h"
#include  "Obstacle.h"
#include  "Pickup.h"
#include  "Model.h"
#include  "HealthPickup.h"
#include  "GameManager.h"
#include  "Util.h"
#include "IglMeshLoader.h"

using namespace cg3d;
namespace Game{

SpawnManager::SpawnManager(float xSize, float ySize, float zSize, SnakeGame* scene){
    this->xSize = xSize;
    this->ySize = ySize;
    this->zSize = zSize;
    this->scene = scene;
    this->activePickups = 0;
    InitAssets();
}

void SpawnManager::InitAssets(){
    modelShader = std::make_shared<Program>("shaders/basicShader"); 
    obstacleMaterial = std::make_shared<Material>("bricks", modelShader);
    healthMaterial = std::make_shared<Material>("grass", modelShader);
    pickupMaterial = std::make_shared<Material>("pickup", modelShader);
    
    obstacleMaterial->AddTexture(0, "textures/wall2.jpg", 2);
    healthMaterial->AddTexture(0, "textures/heart.png", 2);
    pickupMaterial->AddTexture(0, "textures/apple.png", 2);
    
    obstacleMesh = Mesh::Cube(); //IglLoader::MeshFromFiles("cubeMesh", "data/cube.off");
    pickupMesh = Mesh::Cube(); //IglLoader::MeshFromFiles("sphereMesh", "data/sphere.obj");
    healthMesh = Mesh::Cube(); //IglLoader::MeshFromFiles("cylMesh", "data/xcylinder.obj");
}

void SpawnManager::Spawnlevel(int level){
    // set amount of each spawns according to level num
    this->activePickups = 0;
    int pickups = level + 1;
    int obstacles = level == 1 ? 2 : 1;
    int health = level %2 == 1 ? 1 : 0; // every odd level

    Spawnlevel(pickups, obstacles, health);
    Util::DebugPrint("Manager Spawning...");

}
void SpawnManager::Spawnlevel(int pickups, int obstacles, int health)
{
    for(int i = 0; i<pickups; i++){
        SpawnPickupRandom(PICKUP);
    }
    for(int i = 0; i<obstacles; i++){
        SpawnPickupRandom(OBSTACLE);
    }
    for(int i = 0; i<health; i++){
        SpawnPickupRandom(HEALTHPICKUP);
    }
}

void SpawnManager::PickupDestroyed(MovingObject* interactable)
{
    if (!interactable->name.starts_with("Obstacle"))
        activePickups = activePickups - 1 >= 0 ? activePickups - 1 : 0;
    //scene->RemoveInteractable(interactable);

    if (activePickups == 0) {
        Util::DebugPrint("Spawn Manager detected no more pickups, alerting Game Manager");
        scene->gameManager->shouldSpawnNextlevel = true;
        scene->gameManager->soundManager->PlayNextLevel();
    }
}

std::shared_ptr<cg3d::Model> SpawnManager::CreatePickupModel()
{
    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
        // return 0;
        return model->GetMesh()->data.size()-1;
    };
    auto model  = Model::Create("Pickup", pickupMesh, pickupMaterial);
    // auto model  = Model::Create("Pickup", pickupMesh, healthMaterial);
    return AutoMorphingModel::Create(*model, morphFunc);
}

std::shared_ptr<cg3d::Model> SpawnManager::CreateObstacleModel()
{
    return Model::Create("Obstacle", Mesh::Cube(), obstacleMaterial);
}

std::shared_ptr<cg3d::Model> SpawnManager::CreateHealthPickupModel()
{
    return Model::Create("HealthPickup", healthMesh, healthMaterial);
}

void SpawnManager::SpawnPickupRandom(InteractableObjects type){
    float randX = Util::GenerateRandomInRange(-xSize, xSize);
    float randY = Util::GenerateRandomInRange(-ySize, ySize);
    float randZ = Util::GenerateRandomInRange(-zSize, zSize);
    SpawnPickup(randX, randY, randZ, type);
}

void SpawnManager::SpawnPickup(float x, float y, float z, InteractableObjects type){
    //create a model
    std::shared_ptr<cg3d::Model> model;
    std::shared_ptr<MovingObject> pickup;

    // spawn the object
    switch(type){
        case PICKUP:
        //TODO - update manager
        model = CreatePickupModel();
        model->isHidden = false;
        pickup = std::make_shared<Pickup>(*Pickup::SpawnObject(x, y, z, pickupMaterial, model, scene));
        // update the manager
        activePickups ++;
        break;

        case OBSTACLE:
        model = CreateObstacleModel();
        model->Scale(1.5f, Movable::Axis::XYZ);
        pickup = std::make_shared<Obstacle>(*Obstacle::SpawnObject(x, y, z, obstacleMaterial, model, scene));
        break;

        case HEALTHPICKUP:
        model = CreateHealthPickupModel();
        pickup = std::make_shared<HealthPickup>(*HealthPickup::SpawnObject(x, y, z, healthMaterial, model, scene));
        break;
    }
    scene->AddInteractable(pickup);

}


}