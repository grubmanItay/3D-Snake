#include "BasicScene.h"
#include <Eigen/src/Core/Matrix.h>
#include <edges.h>
#include <memory>
#include <per_face_normals.h>
#include <read_triangle_mesh.h>
#include <utility>
#include <vector>
#include "GLFW/glfw3.h"
#include "Mesh.h"
#include "PickVisitor.h"
#include "Renderer.h"
#include "ObjLoader.h"
#include "IglMeshLoader.h"

#include "igl/per_vertex_normals.h"
#include "igl/per_face_normals.h"
#include "igl/unproject_onto_mesh.h"
#include "igl/edge_flaps.h"
#include "igl/loop.h"
#include "igl/upsample.h"
#include "igl/AABB.h"
#include "igl/parallel_for.h"
#include "igl/shortest_edge_and_midpoint.h"
#include "igl/circulation.h"
#include "igl/edge_midpoints.h"
#include "igl/collapse_edge.h"
#include "igl/edge_collapse_is_valid.h"
#include "igl/write_triangle_mesh.h"

using namespace cg3d;

void BasicScene::Init(float fov, int width, int height, float near, float far){
    camera = Camera::Create("camera", fov, float(width) / height, near, far);
    
    AddChild(root = Movable::Create("root")); 
    auto daylight{std::make_shared<Material>("daylight", "shaders/cubemapShader")}; 
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{Model::Create("background", Mesh::Cube(), daylight)};
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();
 
    auto program = std::make_shared<Program>("shaders/phongShader");
    auto program1 = std::make_shared<Program>("shaders/pickingShader");
    
    auto material{std::make_shared<Material>("material", program)}; 
    auto material1{std::make_shared<Material>("material", program1)}; 
 
    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj")};
    auto cylMesh{IglLoader::MeshFromFiles("cyl_igl","data/zcylinder.obj")};
    auto cubeMesh{IglLoader::MeshFromFiles("cube_igl","data/cube_old.obj")};
    sphere1 = Model::Create("sphere",sphereMesh, material);    
    cube = Model::Create("cube", cubeMesh, material);
    
    Eigen::MatrixXd vertices(6,3);
    vertices << -1,0,0,1,0,0,0,-1,0,0,1,0,0,0,-1,0,0,1;
    Eigen::MatrixXi faces(3,2);
    faces << 0,1,2,3,4,5;
    Eigen::MatrixXd vertexNormals = Eigen::MatrixXd::Ones(6, 3);
    Eigen::MatrixXd textureCoords = Eigen::MatrixXd::Ones(6, 2);
    std::shared_ptr<Mesh> coordsys = std::make_shared<Mesh>("coordsys", vertices, faces, vertexNormals, textureCoords);
    axis.push_back(Model::Create("axis", coordsys, material1));
    axis[0]->mode = 1;   
    axis[0]->Scale(4,Axis::XYZ);
    root->AddChild(axis[0]);
    float scaleFactor = 1; 
    cyls.push_back(Model::Create("cyl", cylMesh, material));
    cyls[0]->Scale(scaleFactor,Axis::X);
    cyls[0]->SetCenter(Eigen::Vector3f(0,0,-0.8f*scaleFactor));
    root->AddChild(cyls[0]);

    for(int i = 1; i < 3; i++)
    { 
        cyls.push_back(Model::Create("cyl", cylMesh, material));
        cyls[i]->Scale(scaleFactor,Axis::X);   
        cyls[i]->Translate(1.6f*scaleFactor,Axis::Z);
        cyls[i]->SetCenter(Eigen::Vector3f(0,0,-0.8f*scaleFactor));
        cyls[i-1]->AddChild(cyls[i]);

        axis.push_back(Model::Create("axis", coordsys, material1));
        axis[i]->mode = 1;
        axis[i]->Scale(4, Axis::XYZ);
        cyls[i-1]->AddChild(axis[i]);
        axis[i]->Translate(0.8f*scaleFactor, Axis::Z);
    }
    cyls[0]->Translate({0,0,0.8f*scaleFactor});
    root->RotateByDegree(90, Eigen::Vector3f(-1, 0, 0));

    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
      return model->meshIndex; 
    };
    autoCube = AutoMorphingModel::Create(*cube, morphFunc);
  
    sphere1->showWireframe = true;
    autoCube->Translate({-6,0,0});
    autoCube->Scale(1.5f);
    sphere1->Translate({5,0,0});

    autoCube->showWireframe = true;
    camera->Translate(22, Axis::Z);
    root->AddChild(sphere1);
    root->AddChild(autoCube);
    cube->mode = 1; 
    auto mesh = cube->GetMeshList();

    int num_collapsed;

    V = mesh[0]->data[0].vertices;
    F = mesh[0]->data[0].faces;
    igl::edge_flaps(F,E,EMAP,EF,EI);
    std::cout<< "vertices: \n" << V <<std::endl;
    std::cout<< "faces: \n" << F <<std::endl;
    
    std::cout<< "edges: \n" << E.transpose() <<std::endl;
    std::cout<< "edges to faces: \n" << EF.transpose() <<std::endl;
    std::cout<< "faces to edges: \n "<< EMAP.transpose()<<std::endl;
    std::cout<< "edges indices: \n" << EI.transpose() <<std::endl;

    autoCube->Translate({ 0,0,0 });
    sphere1->Translate({ 0,0,0 });
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model){
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 0.8f, 0.3f, 0.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 0.3f, 0.6f, 1.0f);
    program.SetUniform4f("Kdi", 0.5f, 0.5f, 0.0f, 1.0f);
    program.SetUniform1f("specular_exponent", 5.0f);
    program.SetUniform4f("light_position", 0.0, 15.0f, 0.0, 1.0f);
    cube->Rotate(0.1f, Axis::XYZ);

    CyclicCoordinateDecentMethod();
}

void BasicScene::MouseCallback(Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[]){
    if (action == GLFW_PRESS) { 
        PickVisitor visitor;
        visitor.Init();
        renderer->RenderViewportAtPos(x, y, &visitor); 
        auto modelAndDepth = visitor.PickAtPos(x, renderer->GetWindowHeight() - y);
        renderer->RenderViewportAtPos(x, y); 
        pickedModel = modelAndDepth.first ? std::dynamic_pointer_cast<Model>(modelAndDepth.first->shared_from_this()) : nullptr;
        pickedModelDepth = modelAndDepth.second;
        camera->GetRotation().transpose();
        xAtPress = x;
        yAtPress = y;

        if (pickedModel && !pickedModel->isPickable)
            pickedModel = nullptr; 

        if (pickedModel)
            pickedToutAtPress = pickedModel->GetTout();
        else
            cameraToutAtPress = camera->GetTout();
    }
}

void BasicScene::ScrollCallback(Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[]){
    auto system = camera->GetRotation().transpose();
    if (pickedModel) {
        bool arm_selected = false;
        for (int i = 0; i < num_of_links && !arm_selected; i++) {
            if (pickedModel == cyls[i]) {
                cyls[first_link_id]->TranslateInSystem(system * cyls[first_link_id]->GetRotation(), { 0, 0, -float(yoffset) });
                pickedToutAtPress = pickedModel->GetTout();
                arm_selected = true;
            }
        }
        if (!arm_selected) {
            pickedModel->TranslateInSystem(system * pickedModel->GetRotation(), { 0, 0, -float(yoffset) });
            pickedToutAtPress = pickedModel->GetTout();
        }
    }
    else {
        camera->TranslateInSystem(system, { 0, 0, -float(yoffset) });
        cameraToutAtPress = camera->GetTout();
    }
}

void BasicScene::CursorPosCallback(Viewport* viewport, int x, int y, bool dragging, int* buttonState){
    if (dragging) {
        auto system = camera->GetRotation().transpose() * GetRotation();
        auto moveCoeff = camera->CalcMoveCoeff(pickedModelDepth, viewport->width);
        auto angleCoeff = camera->CalcAngleCoeff(viewport->width);
        if (pickedModel) {
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE) {
                bool arm_selected = false;
                for (int i = 0; i < num_of_links && !arm_selected; i++) {
                    if (pickedModel == cyls[i]) {
                        cyls[first_link_id]->TranslateInSystem(system * cyls[first_link_id]->GetRotation(), { -float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0 });
                        arm_selected = true;
                    }
                }
                if (!arm_selected) {
                    pickedModel->TranslateInSystem(system * pickedModel->GetRotation(), { -float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0 });
                }
            }
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                bool arm_selected = false;
                for (int i = 0; i < num_of_links && !arm_selected; i++) {
                    if (pickedModel == cyls[i]) {
                        Eigen::Matrix3f R = pickedModel->GetRotation();
                        Eigen::Matrix3f R_without_root = root->GetRotation().transpose() * R;
                        Eigen::Vector3f euler_angles = R_without_root.eulerAngles(2, 0, 2);

                        float z_angle = -float(xAtPress - x) / angleCoeff;
                        float x_angle = float(yAtPress - y) / angleCoeff;

                        Eigen::AngleAxisf phi(euler_angles[0] + z_angle, Eigen::Vector3f::UnitZ());
                        Eigen::AngleAxisf theta(euler_angles[1] + x_angle, Eigen::Vector3f::UnitX());
                        Eigen::AngleAxisf psi(euler_angles[2], Eigen::Vector3f::UnitZ());

                        Eigen::Matrix3f R_new = root->GetRotation() * Eigen::Quaternionf(phi * theta * psi).toRotationMatrix();
                        pickedModel->Rotate(R.transpose() * R_new);

                        arm_selected = true;
                    }
                }
                if (!arm_selected) {
                    pickedModel->RotateInSystem(system * pickedModel->GetRotation(), -float(xAtPress - x) / angleCoeff, Axis::Y);
                    pickedModel->RotateInSystem(system * pickedModel->GetRotation(), -float(yAtPress - y) / angleCoeff, Axis::X);
                }
            }
        }
        else {
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
                root->TranslateInSystem(system, { -float(xAtPress - x) / moveCoeff / 10.0f, float(yAtPress - y) / moveCoeff / 10.0f, 0 });
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                root->RotateInSystem(system, float(x - xAtPress) / 180.0f, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                root->RotateInSystem(system, float(x - xAtPress) / angleCoeff, Axis::Y);
                root->RotateInSystem(system, float(y - yAtPress) / angleCoeff, Axis::X);
            }
        }
        xAtPress = x;
        yAtPress = y;
    }
}

void BasicScene::KeyCallback(Viewport* viewport, int x, int y, int key, int scancode, int action, int mods){
    auto system = camera->GetRotation().transpose();

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) 
        {
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                break;
            case GLFW_KEY_B:
                camera->TranslateInSystem(system, {0, 0, 0.1f});
                break;
            case GLFW_KEY_F:
                camera->TranslateInSystem(system, {0, 0, -0.1f});
                break;
            case GLFW_KEY_SPACE: // IK solver
                KeySpaceEvent();
                break;
            case GLFW_KEY_P: // Prints rotation matrices
                std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
                KeyPEvent();
                std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
                break;
            case GLFW_KEY_T: // Prints arms tip positions
                std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
                KeyTEvent();
                std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
                break;
            case GLFW_KEY_D: // Prints destination position
                std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
                KeyDEvent();
                std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
                break;
            case GLFW_KEY_Q: // Prints the number of links
                std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
                KeyQEvent();
                std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
                break;
            case GLFW_KEY_N: // Pick the next link, or the first one in case the last link is picked
                KeyNEvent();
                break;
            case GLFW_KEY_RIGHT: // Rotates picked link around the previous link Y axis
                KeyRightEvent();
                break;
            case GLFW_KEY_LEFT: // Rotates picked link around the previous link Y axis
                KeyLeftEvent();
                break;
            case GLFW_KEY_UP: // Rotates picked link around the current X axis
                KeyUpEvent();
                break;
            case GLFW_KEY_DOWN: // Rotates picked link around the current X axis
                KeyDownEvent();
                break;
            case GLFW_KEY_EQUAL:
                KeyEqualEvent(); // Gets a number from the user and build arm with that number of links
                break;
            case GLFW_KEY_R: // Reset arms positions
                Reset(num_of_links);
                break;
            case GLFW_KEY_1: case GLFW_KEY_KP_1: // Build 1 arm
                Reset(1);
                break;
            case GLFW_KEY_2: case GLFW_KEY_KP_2: // Build 2 arms
                Reset(2);
                break;
            case GLFW_KEY_3: case GLFW_KEY_KP_3: // Build 3 arms
                Reset(3);
                break;
            case GLFW_KEY_4: case GLFW_KEY_KP_4: // Build 4 arms
                Reset(4);
                break;
            case GLFW_KEY_5: case GLFW_KEY_KP_5: // Build 5 arms
                Reset(5);
                break;
            case GLFW_KEY_6: case GLFW_KEY_KP_6: // Build 6 arms
                Reset(6);
                break;
            case GLFW_KEY_7: case GLFW_KEY_KP_7: // Build 7 arms
                Reset(7);
                break;
            case GLFW_KEY_8: case GLFW_KEY_KP_8: // Build 8 arms
                Reset(8);
                break;
            case GLFW_KEY_9: case GLFW_KEY_KP_9: // Build 9 arms
                Reset(9);
                break;
            case GLFW_KEY_KP_ADD: // Add 1 more arm
                Reset(num_of_links + 1);
                break;
            case GLFW_KEY_KP_SUBTRACT: // Remove 1 arm
                if (num_of_links != 1) {
                    Reset(num_of_links - 1);
                }
                break;
        }
    }
}

Eigen::Vector3f BasicScene::GetSpherePos(){
      Eigen::Vector3f l = Eigen::Vector3f(1.6f,0,0);
      Eigen::Vector3f ans;
      ans = cyls[tipIndex]->GetRotation()*l;
      return ans;
}

Eigen::Vector3f BasicScene::GetDestinationPos() {
    Eigen::Matrix4f AggregatedTransform = sphere1->GetAggregatedTransform();
    Eigen::Vector3f ans = Eigen::Vector3f(AggregatedTransform.col(3).x(), AggregatedTransform.col(3).y(), AggregatedTransform.col(3).z());

    return ans;
}

Eigen::Vector3f BasicScene::GetTipPos(int link_id){
    Eigen::Vector3f length = Eigen::Vector3f(0, 0, 0.8f);

    Eigen::Matrix4f armAggregatedTransform = cyls[link_id]->GetAggregatedTransform();
    Eigen::Vector3f armCenter = Eigen::Vector3f(armAggregatedTransform.col(3).x(), armAggregatedTransform.col(3).y(), armAggregatedTransform.col(3).z());
    Eigen::Vector3f ans = armCenter + cyls[link_id]->GetRotation() * length;

    return ans;
}

Eigen::Vector3f BasicScene::GetSourcePos(int link_id) {
    Eigen::Vector3f length = Eigen::Vector3f(0, 0, 0.8f);

    Eigen::Matrix4f armAggregatedTransform = cyls[link_id]->GetAggregatedTransform();
    Eigen::Vector3f armCenter = Eigen::Vector3f(armAggregatedTransform.col(3).x(), armAggregatedTransform.col(3).y(), armAggregatedTransform.col(3).z());
    Eigen::Vector3f ans = armCenter - cyls[link_id]->GetRotation() * length;

    return ans;
}

std::vector<Eigen::Matrix3f> BasicScene::GetEulerAnglesMatrices(Eigen::Matrix3f R) {
    Eigen::Vector3f zxz = R.eulerAngles(2, 0, 2);

    Eigen::Matrix3f phi;
    phi.row(0) = Eigen::Vector3f(cos(zxz.x()), -sin(zxz.x()), 0);
    phi.row(1) = Eigen::Vector3f(sin(zxz.x()), cos(zxz.x()), 0);
    phi.row(2) = Eigen::Vector3f(0, 0, 1);

    Eigen::Matrix3f theta;
    theta.row(0) = Eigen::Vector3f(1, 0, 0);
    theta.row(1) = Eigen::Vector3f(0, cos(zxz.y()), -sin(zxz.y()));
    theta.row(2) = Eigen::Vector3f(0, sin(zxz.y()), cos(zxz.y()));

    Eigen::Matrix3f psi;
    psi.row(0) = Eigen::Vector3f(cos(zxz.z()), -sin(zxz.z()), 0);
    psi.row(1) = Eigen::Vector3f(sin(zxz.z()), cos(zxz.z()), 0);
    psi.row(2) = Eigen::Vector3f(0, 0, 1);

    std::vector<Eigen::Matrix3f> euler_angles_matrices;
    euler_angles_matrices.push_back(phi);
    euler_angles_matrices.push_back(theta);
    euler_angles_matrices.push_back(psi);

    return euler_angles_matrices;
}

void BasicScene::CyclicCoordinateDecentMethod() {
    if (PlayPause && animate) {
        Eigen::Vector3f D = GetDestinationPos();
        Eigen::Vector3f first_link_position = GetSourcePos(first_link_id);

        if ((D - first_link_position).norm() > link_length * num_of_links) {
            std::cout << "cannot reach" << std::endl;
            PlayPause = false;
            return;
        }

        int curr_link = last_link_id;

        while (curr_link != -1) {
            Eigen::Vector3f R = GetSourcePos(curr_link);
            Eigen::Vector3f E = GetTipPos(last_link_id);
            Eigen::Vector3f RD = D - R;
            Eigen::Vector3f RE = E - R;
            float distance = (D - E).norm();

            if (distance < delta) {
                std::cout << "distance: " << distance << std::endl;
                PlayPause = false;
                return;
            }

            Eigen::Vector3f normal = RE.normalized().cross(RD.normalized()); 

            float dot = RD.normalized().dot(RE.normalized());

            if (dot > 1) dot = 1;
            if (dot < -1) dot = -1;

            float angle = (acosf(dot) * (180.f / 3.14f)) / angle_divider;
            Eigen::Vector3f rotation_vector = cyls[curr_link]->GetAggregatedTransform().block<3, 3>(0, 0).inverse() * normal;
            cyls[curr_link]->RotateByDegree(angle, rotation_vector);

            curr_link--;
        }
        animate = false;
    }
}

void BasicScene::SolverHelper(int link_id, Eigen::Vector3f D) {
    Eigen::Vector3f R = GetSourcePos(link_id);
    Eigen::Vector3f E = GetTipPos(link_id);
    Eigen::Vector3f RD = D - R;
    Eigen::Vector3f RE = E - R;

    Eigen::Vector3f normal = RE.normalized().cross(RD.normalized());

    float dot = RD.normalized().dot(RE.normalized()); 

    if (dot > 1) dot = 1;
    if (dot < -1) dot = 1;

    float angle = (acos(dot) * (180.f / 3.14f)) / angle_divider;
    Eigen::Vector3f rotation_vector = cyls[link_id]->GetAggregatedTransform().block<3, 3>(0, 0).inverse() * normal;
    cyls[link_id]->RotateByDegree(angle, rotation_vector);
}

void BasicScene::KeySpaceEvent()
{
    if (IK_mode == 0) {
        PlayPause = !PlayPause;
    }
}

void BasicScene::KeyPEvent()
{
    for (int i = 0; i < num_of_links; i++) {
        if (pickedModel == cyls[i]) {
            Eigen::Matrix3f rotation = pickedModel->GetRotation();

            std::cout << "Arm" << i << " Rotation: " << std::endl
                << "(" << rotation.row(0).x() << "," << rotation.row(0).y() << "," << rotation.row(0).z() << ")" << std::endl
                << "(" << rotation.row(1).x() << "," << rotation.row(1).y() << "," << rotation.row(1).z() << ")" << std::endl
                << "(" << rotation.row(2).x() << "," << rotation.row(2).y() << "," << rotation.row(2).z() << ")" << std::endl
                << std::endl;

            Eigen::Vector3f angles = rotation.eulerAngles(2, 0, 2) * (180.f / 3.14f);
            std::vector<Eigen::Matrix3f> matrices = GetEulerAnglesMatrices(rotation);

            std::cout << "Arm" << i << " Euler Angles: " << std::endl
                << "phi: " << angles.x() << " (Deg)" << std::endl
                << "theta: " << angles.y() << " (Deg)" << std::endl
                << "psi: " << angles.z() << " (Deg)" << std::endl
                << std::endl;

            std::cout << "phi matrix: " << std::endl
                << "(" << matrices[0].row(0).x() << "," << matrices[0].row(0).y() << "," << matrices[0].row(0).z() << ")" << std::endl
                << "(" << matrices[0].row(1).x() << "," << matrices[0].row(1).y() << "," << matrices[0].row(1).z() << ")" << std::endl
                << "(" << matrices[0].row(2).x() << "," << matrices[0].row(2).y() << "," << matrices[0].row(2).z() << ")" << std::endl
                << std::endl;

            std::cout << "theta matrix: " << std::endl
                << "(" << matrices[1].row(0).x() << "," << matrices[1].row(0).y() << "," << matrices[1].row(0).z() << ")" << std::endl
                << "(" << matrices[1].row(1).x() << "," << matrices[1].row(1).y() << "," << matrices[1].row(1).z() << ")" << std::endl
                << "(" << matrices[1].row(2).x() << "," << matrices[1].row(2).y() << "," << matrices[1].row(2).z() << ")" << std::endl
                << std::endl;

            std::cout << "psi matrix: " << std::endl
                << "(" << matrices[2].row(0).x() << "," << matrices[2].row(0).y() << "," << matrices[2].row(0).z() << ")" << std::endl
                << "(" << matrices[2].row(1).x() << "," << matrices[2].row(1).y() << "," << matrices[2].row(1).z() << ")" << std::endl
                << "(" << matrices[2].row(2).x() << "," << matrices[2].row(2).y() << "," << matrices[2].row(2).z() << ")" << std::endl;

            return;
        }
    }
    Eigen::Matrix3f rotation = root->GetRotation();

    std::cout << "Scene Rotation: " << std::endl
        << "(" << rotation.row(0).x() << "," << rotation.row(0).y() << "," << rotation.row(0).z() << ")" << std::endl
        << "(" << rotation.row(1).x() << "," << rotation.row(1).y() << "," << rotation.row(1).z() << ")" << std::endl
        << "(" << rotation.row(2).x() << "," << rotation.row(2).y() << "," << rotation.row(2).z() << ")" << std::endl;
}

void BasicScene::KeyTEvent()
{
    for (int i = 0; i < num_of_links; i++) {
        Eigen::Vector3f pos = GetTipPos(i);

        std::cout << "Arm" << i << " Tip Position: "
            << "(" << pos.x()
            << ", " << pos.y()
            << ", " << pos.z()
            << ")" << std::endl;
    }
}

void BasicScene::KeyDEvent() 
{
    Eigen::Vector3f pos = GetDestinationPos();

    std::cout << "Destination Position: "
        << "(" << pos.x()
        << ", " << pos.y()
        << ", " << pos.z()
        << ")" << std::endl;
}

void BasicScene::KeyQEvent()
{
    std::cout << "Number of links: " << num_of_links << std::endl;
}

void BasicScene::KeyNEvent()
{
    bool SELECTED = false;
    for (int i = 0; i < num_of_links && !SELECTED; i++) {
        if (pickedModel == cyls[i]) {
            // Last link
            if (i == num_of_links - 1) {
                i = -1;
            }
            pickedModel = cyls[i + 1];
            SELECTED = true;
        }
    }
    // None of the arms were selected
    if (!SELECTED) {
        pickedModel = cyls[0];
    }
}

void BasicScene::KeyEqualEvent()
{
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    printf("Enter a number >");
    int x;
    scanf("%d", &x);
    //printf("got the number: %d", x);
    Reset(x);
}

void BasicScene::KeyRightEvent()
{
    auto system = camera->GetRotation().transpose();

    bool SELECTED = false;
    for (int i = 0; i < num_of_links && !SELECTED; i++) {
        if (pickedModel == cyls[i]) {
            Eigen::Matrix3f R = pickedModel->GetRotation();
            std::vector<Eigen::Matrix3f> matrices = GetEulerAnglesMatrices(R);

            float angle = 0.1f;
            Eigen::Matrix3f z;
            z.row(0) = Eigen::Vector3f(cos(angle), -sin(angle), 0);
            z.row(1) = Eigen::Vector3f(sin(angle), cos(angle), 0);
            z.row(2) = Eigen::Vector3f(0, 0, 1);

            Eigen::Matrix3f newR = matrices[0] * matrices[1] * matrices[2] * z;
            pickedModel->Rotate(R.inverse() * newR);

            SELECTED = true;
        }
    }
    if (!SELECTED) {
        root->RotateInSystem(system, -0.1f, Axis::Y);
    }
}

void BasicScene::KeyLeftEvent()
{
    auto system = camera->GetRotation().transpose();

    bool SELECTED = false;
    for (int i = 0; i < num_of_links && !SELECTED; i++) {
        if (pickedModel == cyls[i]) {
            Eigen::Matrix3f R = pickedModel->GetRotation();
            std::vector<Eigen::Matrix3f> matrices = GetEulerAnglesMatrices(R);

            float angle = -0.1f;
            Eigen::Matrix3f z;
            z.row(0) = Eigen::Vector3f(cos(angle), -sin(angle), 0);
            z.row(1) = Eigen::Vector3f(sin(angle), cos(angle), 0);
            z.row(2) = Eigen::Vector3f(0, 0, 1);

            Eigen::Matrix3f newR = matrices[0] * matrices[1] * matrices[2] * z;
            pickedModel->Rotate(R.inverse() * newR);

            SELECTED = true;
        }
    }
    if (!SELECTED) {
        root->RotateInSystem(system, 0.1f, Axis::Y);
    }
}

void BasicScene::KeyUpEvent()
{
    auto system = camera->GetRotation().transpose();

    bool SELECTED = false;
    for (int i = 0; i < num_of_links && !SELECTED; i++) {
        if (pickedModel == cyls[i]) {
            Eigen::Matrix3f R = pickedModel->GetRotation();
            std::vector<Eigen::Matrix3f> matrices = GetEulerAnglesMatrices(R);

            float angle = 0.1f;
            Eigen::Matrix3f x;
            x.row(0) = Eigen::Vector3f(1, 0, 0);
            x.row(1) = Eigen::Vector3f(0, cos(angle), -sin(angle));
            x.row(2) = Eigen::Vector3f(0, sin(angle), cos(angle));

            Eigen::Matrix3f newR = matrices[0] * matrices[1] * x * matrices[2];
            pickedModel->Rotate(R.inverse() * newR);

            SELECTED = true;
        }
    }
    if (!SELECTED) {
        root->RotateInSystem(system, -0.1f, Axis::X);
    }
}

void BasicScene::KeyDownEvent()
{
    auto system = camera->GetRotation().transpose();

    bool SELECTED = false;
    for (int i = 0; i < num_of_links && !SELECTED; i++) {
        if (pickedModel == cyls[i]) {
            Eigen::Matrix3f R = pickedModel->GetRotation();
            std::vector<Eigen::Matrix3f> matrices = GetEulerAnglesMatrices(R);

            float angle = -0.1f;
            Eigen::Matrix3f x;
            x.row(0) = Eigen::Vector3f(1, 0, 0);
            x.row(1) = Eigen::Vector3f(0, cos(angle), -sin(angle));
            x.row(2) = Eigen::Vector3f(0, sin(angle), cos(angle));

            Eigen::Matrix3f newR = matrices[0] * matrices[1] * x * matrices[2];
            pickedModel->Rotate(R.inverse() * newR);

            SELECTED = true;
        }
    }
    if (!SELECTED) {
        root->RotateInSystem(system, 0.1f, Axis::X);
    }
}

void BasicScene::Reset(int num_of_link) {
    PlayPause = false;
    root->RemoveChild(axis[0]);
    root->RemoveChild(cyls[0]);
    axis.clear();
    cyls.clear();
    pickedModel = NULL;


    auto program = std::make_shared<Program>("shaders/phongShader");
    auto program1 = std::make_shared<Program>("shaders/pickingShader");

    auto material{ std::make_shared<Material>("material", program) }; // empty material
    auto material1{ std::make_shared<Material>("material", program1) }; // empty material

    material->AddTexture(0, "textures/box0.bmp", 2);
    auto cylMesh{ IglLoader::MeshFromFiles("cyl_igl","data/zcylinder.obj") };

    Eigen::MatrixXd vertices(6, 3);
    vertices << -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1;
    Eigen::MatrixXi faces(3, 2);
    faces << 0, 1, 2, 3, 4, 5;
    Eigen::MatrixXd vertexNormals = Eigen::MatrixXd::Ones(6, 3);
    Eigen::MatrixXd textureCoords = Eigen::MatrixXd::Ones(6, 2);
    std::shared_ptr<Mesh> coordsys = std::make_shared<Mesh>("coordsys", vertices, faces, vertexNormals, textureCoords);
    axis.push_back(Model::Create("axis", coordsys, material1));
    axis[0]->mode = 1;
    axis[0]->Scale(4, Axis::XYZ);
    root->AddChild(axis[0]);
    float scaleFactor = 1;
    cyls.push_back(Model::Create("cyl", cylMesh, material));
    cyls[0]->Scale(scaleFactor, Axis::X);
    cyls[0]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
    root->AddChild(cyls[0]);

    for (int i = 1; i < num_of_link; i++)
    {
        cyls.push_back(Model::Create("cyl", cylMesh, material));
        cyls[i]->Scale(scaleFactor, Axis::X);
        cyls[i]->Translate(1.6f * scaleFactor, Axis::Z);
        cyls[i]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
        cyls[i - 1]->AddChild(cyls[i]);

        axis.push_back(Model::Create("axis", coordsys, material1));
        axis[i]->mode = 1;
        axis[i]->Scale(4, Axis::XYZ);
        cyls[i - 1]->AddChild(axis[i]);
        axis[i]->Translate(0.8f * scaleFactor, Axis::Z);
    }
    cyls[0]->Translate({ 0,0,0.8f * scaleFactor });
    root->Translate({ 0,0,0 });

    first_link_id = 0;
    last_link_id = num_of_link - 1;
    num_of_links = num_of_link;
    link_length = 1.6f;
}
