#pragma once

#include "Visitor.h"
#include "Camera.h"
#include "../tutorial/finalProject/SnakeGame.h"
#include "Model.h"
#include <Eigen/Geometry>
#include <utility>
#include <memory>


namespace cg3d
{

    class AnimationVisitor : public Visitor
    {
    public:
        void Run(Scene* scene, Camera* camera) override;
        void Visit(Model* model) override;
        bool drawOutline = true;
        float outlineLineWidth = 5;


    private:

        SnakeGame* scene;
        std::shared_ptr<Game::Snake> snake;
        Eigen::Quaternionf quaternionz, quaterniony;
        int link_index = 0;
        int number_of_links = 0;
    };
}