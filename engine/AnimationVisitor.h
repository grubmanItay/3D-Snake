#pragma once

#include "Visitor.h"
#include "Camera.h"
#include "Scene.h"
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

    Scene* scene;
};
}