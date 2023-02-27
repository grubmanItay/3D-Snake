#pragma once

#include "SceneWithImGui.h"

#include <memory>
#include <utility>

#define SNAKE_NAME "snake"


    namespace Game{
    class Snake;
    class GameObject;
    class MovingObject;
    class GameManager;
    }
    class SnakeGame : public cg3d::SceneWithImGui
    {

    private:
        void BuildImGui() override;
        std::shared_ptr<cg3d::Material> carbon;
        // bool animate;
        std::vector<std::shared_ptr<cg3d::Camera>> camList{ 2 };
        std::vector<std::shared_ptr<Game::MovingObject>> interactables;
        std::shared_ptr<Game::MovingObject> sphereObj, sphereObj2;
        std::shared_ptr<cg3d::Model> background;
        cg3d::Viewport* viewport = nullptr;
        int ticks = 0;
        void InitPtrs();

        void InitSnake(std::shared_ptr<cg3d::Program> program);
        void InitManagers();

        std::shared_ptr<cg3d::Program> InitSceneEtc();
        void InitCameras(float fov, int width, int height, float near, float far);

    public:
        std::shared_ptr<Game::Snake> snake;
        std::shared_ptr<Movable> root;
        std::shared_ptr<cg3d::Model> cube1, cube2, cylinder, sphere1, sphere2, collisionCube1, collisionCube2;
        std::shared_ptr<Game::GameManager> gameManager;
        float velInterval;
        SnakeGame(std::string name, cg3d::Display* display);
        void AddInteractable(std::shared_ptr<Game::MovingObject> interactable);
        void RemoveInteractable(Game::MovingObject* interactable);
        void Init(float fov, int width, int height, float near, float far);
        void AnimateUntilCollision(std::shared_ptr<Game::Snake> snakeModel);
        void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
        void RestartScene();
        void ActivateGame();
        void EndGame();
        void InitBackground();
        void KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;
        void AddViewportCallback(cg3d::Viewport* _viewport) override;
        void ViewportSizeCallback(cg3d::Viewport* _viewport) override;
        static void PreDecimateMesh(std::shared_ptr<cg3d::Mesh> mesh, bool custom);
        void ShowObject(std::shared_ptr<cg3d::Model> model, bool dir);
        // inline bool IsActive() const { return animate; };
        inline void SetActive(bool _isActive = true) { animate = _isActive; }
        void StopMotion();

    private:
        void LoadObjectFromFileDialog();
        void UpdateXVelocity(bool direction);
        void UpdateYVelocity(bool direction);
        void UpdateZVelocity(bool direction);
        void SetCamera(int index);
        static void DumpMeshData(const Eigen::IOFormat& simple, const cg3d::MeshData& data);
    };

    
