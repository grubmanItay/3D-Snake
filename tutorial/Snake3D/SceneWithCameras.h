#pragma once

#include "SceneWithImGui.h"
#include "CamModel.h"


class SceneWithCameras : public cg3d::SceneWithImGui
{
private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Material> carbon;
    bool animate = false;
    void BuildImGui() override;
    std::vector<std::shared_ptr<cg3d::Camera>> camList{2};
    std::shared_ptr<cg3d::Model> snake, head;
    cg3d::Viewport* viewport = nullptr;
    std::vector<std::shared_ptr<cg3d::Model>> cyls;
    
    float scaleFactor = 1;
    int first_link_id = 0;
    int defaultSnakeSize = 16;
    int last_link_id;
    int currSnakeSize;
    float link_length = 1.6f;
    Eigen::Vector3f pivotPos;

public:

    SceneWithCameras(std::string name, cg3d::Display* display);
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void AddViewportCallback(cg3d::Viewport* _viewport) override;
    void ViewportSizeCallback(cg3d::Viewport* _viewport) override;
    Eigen::Vector3f GetTipPos(int link_id);

private:
    int score = 0;
    inline bool IsActive() const { return animate; };
    inline void SetActive(bool _isActive = true) { animate = _isActive; }
    void LoadObjectFromFileDialog();
    void SetCamera(int index);
    static std::shared_ptr<CamModel> CreateCameraWithModel(int width, int height, float fov, float near, float far, const std::shared_ptr<cg3d::Material>& material);
    static void DumpMeshData(const Eigen::IOFormat& simple, const cg3d::MeshData& data) ;
};

