#pragma once
#include "AutoMorphingModel.h"
#include "Scene.h"

#include <memory>
#include <utility>

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void MouseCallback(cg3d::Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[]) override;
    void ScrollCallback(cg3d::Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[]) override;
    void CursorPosCallback(cg3d::Viewport* viewport, int x, int y, bool dragging, int* buttonState)  override;
    void KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods) override;
    Eigen::Vector3f GetSpherePos();

    Eigen::Vector3f GetDestinationPos();
    Eigen::Vector3f GetTipPos(int link_id);
    Eigen::Vector3f GetSourcePos(int link_id);
    std::vector<Eigen::Matrix3f> GetEulerAnglesMatrices(Eigen::Matrix3f R);

    void CyclicCoordinateDecentMethod();

    void KeySpaceEvent();
    void KeyPEvent();
    void KeyTEvent();
    void KeyDEvent();
    void KeyQEvent();
    void KeyNEvent();
    void KeyEqualEvent();
    void KeyRightEvent();
    void KeyLeftEvent();
    void KeyUpEvent();
    void KeyDownEvent();

    void Reset(int num_of_link);

private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> sphere1;
    std::vector<std::shared_ptr<cg3d::Model>> cyls, axis;
    int pickedIndex = 0;
    int tipIndex = 0;
    Eigen::VectorXi EMAP;
    Eigen::MatrixXi F,E,EF,EI;
    Eigen::VectorXi EQ;
    Eigen::MatrixXd V, C, N, T, points,edges,colors;

    int IK_mode = 0;
    bool PlayPause = false;
    float delta = 0.05;
    float angle_divider = 50.f;

    int first_link_id = 0;
    int last_link_id = 2;
    int num_of_links = 3;
    float link_length = 1.6f;
};
