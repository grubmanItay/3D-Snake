#include "SceneWithCameras.h"

#include <utility>

#include "ObjLoader.h"
#include "AutoMorphingModel.h"
#include "SceneWithImGui.h"
#include "CamModel.h"
#include "Visitor.h"
#include "Utility.h"
#include "IglMeshLoader.h"

#include "imgui.h"
#include "file_dialog_open.h"
#include "GLFW/glfw3.h"


using namespace cg3d;
void SceneWithCameras::BuildImGui()
{
    int flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
    bool* pOpen = nullptr;

    ImGui::Begin("Menu", pOpen, flags);
    ImGui::SetWindowPos(ImVec2(0, 0), ImGuiCond_Always);
    ImGui::SetWindowSize(ImVec2(0, 0), ImGuiCond_Always);
  

    ImGui::Text("Camera: ");
    for (int i = 0; i < camList.size(); i++) {
        ImGui::SameLine(0);
        bool selectedCamera = camList[i] == camera;
        if (selectedCamera)
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(std::to_string(i + 1).c_str()))
            SetCamera(i);
        if (selectedCamera)
            ImGui::PopStyleColor();
    }
    if (ImGui::Button(!animate ? "Play" : "Pause")) {
        animate = !animate;;
    }

    ImGui::Text("Score: ");
    ImGui::SameLine();
    std::string temp_str = std::to_string(score); //converting number to a string
    char const* chScore = temp_str.c_str(); //converting string to char Array
    ImGui::Text(chScore);

    ImGui::End();
}

void SceneWithCameras::DumpMeshData(const Eigen::IOFormat& simple, const MeshData& data)
{
    std::cout << "vertices mesh: " << data.vertices.format(simple) << std::endl;
    std::cout << "faces mesh: " << data.faces.format(simple) << std::endl;
    std::cout << "vertex normals mesh: " << data.vertexNormals.format(simple) << std::endl;
    std::cout << "texture coordinates mesh: " << data.textureCoords.format(simple) << std::endl;
}

SceneWithCameras::SceneWithCameras(std::string name, Display* display) : SceneWithImGui(std::move(name), display)
{
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.FrameRounding = 5.0f;
}

void SceneWithCameras::SetCamera(int index)
{
    camera = camList[index];
    viewport->camera = camera;
}

void SceneWithCameras::Init(float fov, int width, int height, float near, float far)
{
    // create the basic elements of the scene
    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto program = std::make_shared<Program>("shaders/basicShader"); // TODO: TAL: replace with hard-coded basic program
    carbon = std::make_shared<Material>("carbon", program); // default material
    carbon->AddTexture(0, "textures/carbon.jpg", 2);

    // create the camera objects
    camList.resize(camList.capacity());
    camList[0] = Camera::Create("camera0", fov, float(width) / float(height), near, far);
    for (int i = 1; i < camList.size(); i++) {
        auto camera = Camera::Create("", fov, double(width) / height, near, far);
        root->AddChild(camList[i] = camera);
    }

    camList[0]->Translate(50, Axis::Z);
    camList[0]->Translate(10, Axis::Y);
    camList[0]->RotateByDegree(-20, Axis::X);
    camList[1]->Translate(-50, Axis::X);
    camList[1]->Translate(10, Axis::Z);
    camList[1]->RotateByDegree(90, Axis::X);
    camList[1]->RotateByDegree(-90, Axis::Y);
    camList[1]->RotateByDegree(-20, Axis::X);

    camera = camList[0];

    auto daylight{std::make_shared<Material>("daylight", "shaders/cubemapShader")};
    auto material{ std::make_shared<Material>("material", program) };
    material->AddTexture(0, "textures/snake.jpg", 2);
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);

    auto cylMesh{ IglLoader::MeshFromFiles("cyl_igl","data/zcylinder.obj") };
    auto background{Model::Create("background", Mesh::Cube(), daylight)};
    AddChild(background);

    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
        static float prevDistance = -1;
        float distance = (visitor->view * visitor->norm * model->GetAggregatedTransform()).norm();
        if (prevDistance != distance)
            debug(model->name, " distance from camera: ", prevDistance = distance);
        return distance > 3 ? 1 : 0;
    };


    cyls.push_back(Model::Create("cyl", cylMesh, material));
    cyls[0]->Scale(scaleFactor, Axis::X);
    cyls[0]->SetCenter(Eigen::Vector3f(0, 0, (-link_length / 2) * scaleFactor));
    root->AddChild(cyls[0]);

    for (int i = 1; i < defaultSnakeSize; i++)
    {
        cyls.push_back(Model::Create("cyl", cylMesh, material));
        cyls[i]->Scale(scaleFactor, Axis::X);
        cyls[i]->Translate(link_length * scaleFactor, Axis::Z);
        cyls[i]->SetCenter(Eigen::Vector3f(0, 0, (-link_length/2) * scaleFactor));
        cyls[i - 1]->AddChild(cyls[i]);

    }
    currSnakeSize = defaultSnakeSize;
    last_link_id = currSnakeSize - 1;
    head = cyls[last_link_id];
    cyls[0]->Translate({ 0,-link_length * currSnakeSize,0 });
    cyls[0]->RotateByDegree(90, Axis::X);
    cyls[0]->RotateByDegree(180, Axis::Y);
    root->RotateByDegree(90, Eigen::Vector3f(-1, 0, 0));

    //pickedModel = snake;
    background->Scale(100, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();
}

void SceneWithCameras::Update(const Program& p, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(p, proj, view, model);

    if (animate) {
        //for (int i = 0; i < currSnakeSize; i++) {
        //    cyls[i]->Translate(Eigen::Vector3f{0.0f,0.0f,0.01f});
        //}
        //snake->Translate(0.001f, Axis::Y);

        for (int i = last_link_id; i > 0; i--) {
            if (GetTipPos(i) == pivotPos)
                cyls[i]->RotateByDegree(-90, Axis::Y);
        }
        cyls[0]->Translate(0.01f, Axis::Y);
        //cyls[1]->RotateByDegree(-90, Axis::X);

        animate = !animate;
    }
}

void SceneWithCameras::KeyCallback(Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        if (key == GLFW_KEY_SPACE || key == GLFW_KEY_P)
            SetActive(!IsActive());

        if (key == GLFW_KEY_1 || key == GLFW_KEY_2) {
            if (int index; (index = (key - GLFW_KEY_1 + 10) % 10) < camList.size())
                SetCamera(index);
        }

        if (key == GLFW_KEY_RIGHT) {
            pivotPos = GetTipPos(last_link_id);
        }
    }

}

void SceneWithCameras::ViewportSizeCallback(Viewport* _viewport)
{
    for (auto& cam : camList)
        cam->SetProjection(float(_viewport->width) / float(_viewport->height));

    // note: we don't need to call Scene::ViewportSizeCallback since we are setting the projection of all the cameras
}

void SceneWithCameras::AddViewportCallback(Viewport* _viewport)
{
    viewport = _viewport;

    Scene::AddViewportCallback(viewport);
}

Eigen::Vector3f SceneWithCameras::GetTipPos(int link_id) {
    Eigen::Vector3f length = Eigen::Vector3f(0, 0, 0.8f);

    Eigen::Matrix4f armAggregatedTransform = cyls[link_id]->GetAggregatedTransform();
    Eigen::Vector3f armCenter = Eigen::Vector3f(armAggregatedTransform.col(3).x(), armAggregatedTransform.col(3).y(), armAggregatedTransform.col(3).z());
    Eigen::Vector3f ans = armCenter + cyls[link_id]->GetRotation() * length;

    return ans;
}