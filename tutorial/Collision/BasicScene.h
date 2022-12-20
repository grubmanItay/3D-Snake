#pragma once

#include "Scene.h"
#include "igl/AABB.h"

#include <utility>

using namespace cg3d;

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void InitObjs();
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void DrawCubes(Eigen::AlignedBox<double, 3>& box, std::shared_ptr<cg3d::Model> cube);
    bool isCollision(igl::AABB<Eigen::MatrixXd, 3>* treeA, igl::AABB<Eigen::MatrixXd, 3>* treeB);
    bool boxesIntersect(Eigen::AlignedBox<double, 3>& boxA, Eigen::AlignedBox<double, 3>& boxB);
    void KeyCallback(Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;

private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> obj1, obj2, cube1, cube2, cube3, cube4;
    igl::AABB<Eigen::MatrixXd, 3> tree1, tree2;
    Eigen::Vector3f motion = { 0.001f,0.0f,0.0f };
    bool COLLISION = false;
    bool BUNNY = false;
    bool COW = false;
    bool TRUCK = false;
};
