#include "SnakeGame.h"

#include <utility>
#include <vector>
#include <iterator>

#include "igl/vertex_triangle_adjacency.h"
#include "igl/per_face_normals.h"
#include "igl/edge_flaps.h"
#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/decimate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>

#include "ObjLoader.h"
#include "Renderer.h"
#include "AutoMorphingModel.h"
#include "SceneWithImGui.h"
#include "CamModel.h"
#include "Visitor.h"
#include "IglMeshLoader.h"
#include "Utility.h"
#include "Util.h"
#include "Snake.h"
#include "MovingObject.h"
#include "Pickup.h"
#include "HealthPickup.h"
#include "Obstacle.h"
#include "Snake.h"
#include "GameManager.h"
#include "GameObject.h"
#include "SpawnManager.h"

#include "imgui.h"
#include "file_dialog_open.h"
#include "GLFW/glfw3.h"

#define DECIMATION_MULT 0.03
#define STAGE_SIZE 15
#define DEBUG_GUI false


using namespace std;
using namespace Eigen;
using namespace igl;
using namespace cg3d;
using namespace Game;

void SnakeGame::BuildImGui()
{
    int flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
    bool* pOpen = nullptr;

    if(DEBUG_GUI){
        ImGui::Begin("Menu", pOpen, flags);
        ImGui::SetWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(0, 0), ImGuiCond_Always);
        if (ImGui::Button("Load object"))
            LoadObjectFromFileDialog();

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
        ImGui::SameLine();
        if (ImGui::Button("Center"))
            camera->SetTout(Eigen::Affine3f::Identity());
        if (pickedModel) {
            ImGui::Text("Picked model: %s", pickedModel->name.c_str());
            ImGui::SameLine();
            if (ImGui::Button("Drop"))
                pickedModel = nullptr;
            if (pickedModel) {
                if (ImGui::CollapsingHeader("Draw options", ImGuiTreeNodeFlags_DefaultOpen)) {
                    ImGui::Checkbox("Show wireframe", &pickedModel->showWireframe);
                    if (pickedModel->showWireframe) {
                        ImGui::Text("Wireframe color:");
                        ImGui::SameLine();
                        ImGui::ColorEdit4("Wireframe color", pickedModel->wireframeColor.data(), ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel);
                    }
                    ImGui::Checkbox("Show faces", &pickedModel->showFaces);
                    ImGui::Checkbox("Show textures", &pickedModel->showTextures);
                    if (ImGui::Button("Scale down"))
                        pickedModel->Scale(0.9f);
                    ImGui::SameLine();
                    if (ImGui::Button("Scale up"))
                        pickedModel->Scale(1.1f);
                }
                if (ImGui::Button("Dump model mesh data")) {
                    std::cout << "model name: " << pickedModel->name << std::endl;
                    if (pickedModel->meshIndex > 0)
                        std::cout << "mesh index in use: " << pickedModel->meshIndex;
                    for (auto& mesh: pickedModel->GetMeshList()) {
                        Eigen::IOFormat simple(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");
                        std::cout << "mesh name: " << mesh->name << std::endl;
                        for (int i = 0; i < mesh->data.size(); i++) {
                            if (mesh->data.size() > 1)
                                std::cout << "mesh #" << i + 1 << ":" << std::endl;
                            DumpMeshData(simple, mesh->data[i]);
                        }
                    }
                }
                if (ImGui::Button("Dump model transformations")) {
                    Eigen::IOFormat format(2, 0, ", ", "\n", "[", "]");
                    const Eigen::Matrix4f& transform = pickedModel->GetAggregatedTransform();
                    std::cout << "Tin:" << std::endl << pickedModel->Tin.matrix().format(format) << std::endl
                            << "Tout:" << std::endl << pickedModel->Tout.matrix().format(format) << std::endl
                            << "Transform:" << std::endl << transform.matrix().format(format) << std::endl
                            << "--- Transform Breakdown ---" << std::endl
                            << "Rotation:" << std::endl << Movable::GetTranslation(transform).matrix().format(format) << std::endl
                            << "Translation:" << std::endl << Movable::GetRotation(transform).matrix().format(format) << std::endl
                            << "Rotation x Translation:" << std::endl << Movable::GetTranslationRotation(transform).matrix().format(format)
                            << std::endl << "Scaling:" << std::endl << Movable::GetScaling(transform).matrix().format(format) << std::endl;
                }
            }
        }

        ImGui::End();
    }
    else{
        ImGui::Begin("Menu", pOpen, flags);
        ImGui::SetWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetWindowSize(ImVec2(0, 0), ImGuiCond_Always);
        //Cameras
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

        //Score Text
        ImGui::Text("High Score: %f", gameManager->GetHighScore());
        if (!snake->isAlive) {
            ImGui::Text("Game Over, Play Again?");
        }

        if (ImGui::Button(!gameManager->GetisStarted() ? "Play":"Restart")) {
            if (!gameManager->GetisStarted()) {
                gameManager->GameStart();
            }
            else
            {
                float HS = gameManager->GetHighScore();
                RestartScene();
                gameManager->Restart();
                gameManager->SetHighScore(HS);
            }
        }

        
        ImGui::SameLine();
        if(gameManager->shouldSpawnNextlevel){
            if(ImGui::Button("Next level")){
                gameManager->Nextlevel();
            }
            else {
                SetActive(false);
                int lvl = gameManager->GetCurrlevel();
                ImGui::Text("Well Done! You have completed level %d", lvl);
            }
        }
        if(IsActive()){
            ImGui::Text("Current level: %d", gameManager->GetCurrlevel());
            ImGui::Text("Health: %f", snake->currHealth);
            ImGui::Text("Score %f", gameManager->GetScore());
        }
        else if (gameManager->GetisStarted()) {
            ImGui::Text("Paused");
        }

        ImGui::End();
    }
}

void SnakeGame::DumpMeshData(const Eigen::IOFormat& simple, const MeshData& data)
{
    std::cout << "vertices mesh: " << data.vertices.format(simple) << std::endl;
    std::cout << "faces mesh: " << data.faces.format(simple) << std::endl;
    std::cout << "vertex normals mesh: " << data.vertexNormals.format(simple) << std::endl;
    std::cout << "texture coordinates mesh: " << data.textureCoords.format(simple) << std::endl;
}

SnakeGame::SnakeGame(std::string name, Display* display) : SceneWithImGui(std::move(name), display)
{
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.FrameRounding = 5.0f;
}

void SnakeGame::SetCamera(int index)
{
    camera = camList[index];
    viewport->camera = camera;
}

void SnakeGame::PreDecimateMesh(std::shared_ptr<cg3d::Mesh> mesh, bool custom)
{
    float decimationMult = DECIMATION_MULT;
	Eigen::MatrixXd V, OV, VN, FN; 
	Eigen::MatrixXi F, OF;
	double initialSize; 

	std::vector<Eigen::Matrix4d> Qmatrices;

	OV = mesh->data.back().vertices; 
	OF = mesh->data.back().faces;

	Eigen::VectorXi EMAP;
	Eigen::MatrixXi E, EF, EI;
	Eigen::VectorXi EQ;
	igl::min_heap<std::tuple<double, int, int>> Q; 

	Eigen::MatrixXd C;
	int num_collapsed;

#pragma region customCollapse
	auto computeQ = [&]() 
	{
		// this is used to initialize the Q matrices of each vertex 
		std::vector<std::vector<int>> V2F; // vertex to face vector
		std::vector<std::vector<int>> NI;  // for adj

		int n = V.rows();
		Qmatrices.resize(n);
		igl::vertex_triangle_adjacency(n, F, V2F, NI);
		igl::per_face_normals(V, F, FN);
		for (int i = 0; i < n; i++)
		{ // for each vertex in q
			Qmatrices[i] = Eigen::Matrix4d::Zero();
			for (int j = 0; j < V2F[i].size(); j++)
			{														   // for each vertex face
				Eigen::Vector3d norm = FN.row(V2F[i][j]).normalized(); // get the face normal
				Eigen::Vector4d p;
				p << norm, (V.row(i) * norm) * -1;
				Eigen::Matrix4d Kp = p * p.transpose(); // Kp = pp^T
				Qmatrices[i] += Kp; //from the given formula
			}
		}
	};

	// select all valid pairs and compute optimal contraction for target v' for each (v1,v2)
	auto costCalc = [&](
						const int edge,
						const Eigen::MatrixXd &V,
						const Eigen::MatrixXi & /*F*/,
						const Eigen::MatrixXi &E,
						const Eigen::VectorXi & /*EMAP*/,
						const Eigen::MatrixXi & /*EF*/,
						const Eigen::MatrixXi & /*EI*/,
						double &cost,
						Eigen::RowVectorXd &p)
	{
		int v1 = E(edge, 0); // get vertices of edge
		int v2 = E(edge, 1);
		
		std::pair<Eigen::Vector4d, int> mincost;
		Eigen::Matrix4d Qtag = Qmatrices[v1] + Qmatrices[v2];
		Eigen::Matrix4d Qcoor = Qtag;
		Eigen::Vector4d vtag;

		Qcoor.row(3) = Eigen::Vector4d(0, 0, 0, 1);
		bool inversable;
		Qcoor.computeInverseWithCheck(Qcoor, inversable);
		if (inversable)
		{
			vtag = Qcoor * (Eigen::Vector4d(0, 0, 0, 1));
			cost = vtag.transpose() * Qtag * vtag;
		}
		else
		{
			Eigen::Vector4d v1coor;
			Eigen::Vector4d v2coor;
			Eigen::Vector4d v1v2coor;
			v1coor << V.row(v1).x(),V.row(v1).y(), V.row(v1).z() , 1; // make into 4
			v2coor << V.row(v2).x(),V.row(v2).y(), V.row(v2).z() , 1; // make into 4
			Eigen::Vector3d v1v2 = (V.row(v1) + V.row(v2)) / 2 ;
			v1v2coor << v1v2 , 1; // make into 4
			double cost1 = v1coor.transpose() * Qtag * v1coor;
			double cost2 = v2coor.transpose() * Qtag * v2coor;
			double cost3 = v1v2coor.transpose() * Qtag * v1v2coor;
			mincost = std::min({pair(v1coor, cost1), pair(v2coor, cost2), pair(v1v2coor, cost3)}, [](pair<Eigen::Vector4d, int> a, pair<Eigen::Vector4d, int> b)
							   { return a.second > b.second; }); //calculate minimal error
			
			cost = mincost.second;
			vtag = mincost.first;
		}
		Eigen::Vector3d vtagCoor;
		vtagCoor << vtag[0], vtag[1], vtag[2];
		p = vtagCoor;
	};

#pragma endregion

#pragma region reset
	auto reset = [&](bool custom)
	{
		F = OF;
		V = OV;
		edge_flaps(F, E, EMAP, EF, EI);
		C.resize(E.rows(), V.cols());
		VectorXd costs(E.rows());
		if (custom)
			computeQ();
		Q = {};
		EQ = Eigen::VectorXi::Zero(E.rows());
		{
			Eigen::VectorXd costs(E.rows());
			igl::parallel_for(
				E.rows(), [&](const int e)
				{
						double cost = e;
						RowVectorXd p(1, 3);
						// decimation algorithm chosen by flag
						if(!custom)
							shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
						else {
							costCalc(e, V, F, E, EMAP, EF, EI, cost, p);
						}
						C.row(e) = p;
						costs(e) = cost; },
				10000);
			for (int e = 0; e < E.rows(); e++)
			{
				Q.emplace(costs(e), e, 0);
			}
		}
		initialSize = Q.size(); // initial size is the first edge queue
		debug_print(mesh->name, " initial edge count: ", initialSize);
		num_collapsed = 0;
	};

#pragma endregion

#pragma region collapse
	auto cutedges = [&](bool custom)
	{
		bool collapsed = false;
		if (!Q.empty())
		{
			// collapse edge
			if(custom)
				computeQ(); //compute q matrix after each round of collapsing - better result without this for simple shapes
			const int max_iter = std::ceil(decimationMult * initialSize);
			for (int j = 0; j < max_iter; j++)
			{
				Eigen::RowVectorXd pos(1, 3);
				double cost = std::get<0>(Q.top());
				int e = std::get<1>(Q.top());
				if (custom)
				{
					// UNCOMMENT TO GET PRINTS
					// costCalc(e, V, F, E, EMAP, EF, EI, cost, pos);
					if (!collapse_edge(costCalc, V, F, E, EMAP, EF, EI, Q, EQ, C)) // use calculation according to flag
						break;
				}
				else
				{
					// UNCOMMENT TO GET PRINTS
					// costCalc(e, V, F, E, EMAP, EF, EI, cost, pos);
					if (!collapse_edge(shortest_edge_and_midpoint, V, F, E, EMAP, EF, EI, Q, EQ, C))
						break;
				}
				// UNCOMMENT TO GET PRINTS

				// cout << "Edge removed is: " << e
				// 	 << ", Cost was: " << cost
				// 	 << ", New v position is: (" << pos[0] << "," << pos[1] << "," << pos[2] << ")"
				// 	 << std::endl;

				collapsed = true;
				num_collapsed++;
				initialSize--; // edges removed are decreased from total to remove
			}
			// get the missing vars for a mesh
			if (collapsed)
			{
				Eigen::MatrixXd VN;
				igl::per_vertex_normals(V, F, VN);
				Eigen::MatrixXd TC = Eigen::MatrixXd::Zero(V.rows(), 2);
				mesh->data.push_back(MeshData{V, F, VN, TC});
				debug_print("collapses successful so far: ", num_collapsed);
			}
		}
	};

#pragma endregion

	reset(custom);
	for (int i = 0; i < 10; i++)
	{
		cutedges(custom);
	}
}

void SnakeGame::Init(float fov, int width, int height, float _near, float _far)
{
    InitManagers();
    auto program = InitSceneEtc();
    InitBackground();
    InitSnake(program);
    InitCameras(fov, width, height, _near, _far);

    auto bricks{std::make_shared<Material>("bricks", program)};
    bricks->AddTexture(0, "textures/bricks.jpg", 2);
    auto collisionCube1Mesh {IglLoader::MeshFromFiles("collisioncube1mesh", "data/cube.off")};
    collisionCube1 = Model::Create("collisioncube1", collisionCube1Mesh, bricks);
    collisionCube1->isHidden = true;

	snake->GetModel()->AddChild(collisionCube1);   
}

void SnakeGame::AnimateUntilCollision(std::shared_ptr<Game::Snake> snakeModel){
	if(!snakeModel->IsColliding()){
        Eigen::Vector3d moveVector;
        moveVector = Vector3d{-0.01,0,0};
		//snake->Skinning(moveVector);
	}
}

void SnakeGame::RemoveInteractable(Game::MovingObject* interactable){
    auto it = std::find_if(interactables.begin(), interactables.end(), [&](std::shared_ptr<MovingObject> p){return p.get()==interactable;});
    if (it != interactables.end())
        interactables.erase(it);

}

void SnakeGame::InitManagers(){
    float sz = STAGE_SIZE;
    SpawnManager* spawnManager = new SpawnManager(sz, sz, sz, this);
    SoundManager* soundManager = new SoundManager();
    GameManager* gameManagerPtr = new GameManager(spawnManager, soundManager);
    this->gameManager = std::make_shared<GameManager>(*gameManagerPtr);
}

void SnakeGame::InitBackground(){
    if (background != NULL)
        RemoveChild(background);

    int level = gameManager->GetCurrlevel();
    std::string bgName;
    if (level <= 1) { bgName = "Earth"; }
    else if (level == 2) { bgName = "Bridge"; }
    else if (level == 3) { bgName = "Tenerife"; }
    else if (level == 4) { bgName = "Vindelalven"; }
    else if (level == 5) { bgName = "Daylight Box"; }

    auto bgCube{std::make_shared<Material>(bgName, "shaders/cubemapShader")};
    bgCube->AddTexture(0, "textures/cubemaps/" + bgName + "_", 3);
    background = Model::Create("background", Mesh::Cube(), bgCube);
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();
}

std::shared_ptr<cg3d::Program> SnakeGame::InitSceneEtc(){
    // initialize common values and return a shader program to be used 
    this->core().animation_max_fps = 30;
    this->velInterval =0.1f;
    this->animate = false;
    // create the basic elements of the scene
    AddChild(this->root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto program = std::make_shared<Program>("shaders/basicShader");
    return program; 
}

void SnakeGame::InitCameras(float fov, int width, int height, float _near, float _far){
    //First Person Camera
    camList.resize(camList.capacity());
    camList[0] = Camera::Create("camera0", fov, float(width) / float(height), _near, _far);
    camList[0]->Translate(-1, Axis::X);
    camList[0]->RotateByDegree(90, Axis::Y);
    snake->autoSnake->AddChild(camList[0]);

    camList[1] = Camera::Create("camera" + std::to_string(1), fov, double(width) / height, _near, _far);
    root->AddChild(camList[1]);
    camList[1]->Translate(50, Axis::Y);
    camList[1]->RotateByDegree(-90, Axis::X);
    this->camera = camList[1];
    
}

void SnakeGame::InitSnake(std::shared_ptr<cg3d::Program> program){
   //init snake object
    auto snakeMaterial{std::make_shared<Material>(SNAKE_NAME, program)};
    snakeMaterial->AddTexture(0, "textures/snake1.png", 2);
    auto snakeMesh = Mesh::Cylinder(); 
    snakeMesh->data.push_back(snakeMesh->data[0]);
    auto snakeModel{Model::Create(SNAKE_NAME, snakeMesh, snakeMaterial)};

    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
        return 0;
    };

    std::shared_ptr<AutoMorphingModel> autoSnake = AutoMorphingModel::Create(*snakeModel, morphFunc);
    autoSnake->showWireframe = false;
    autoSnake->RotateByDegree(90, Eigen::Vector3f(0,1,0));
    root->AddChild(autoSnake);
    this->snake = Game::Snake::CreateSnake(snakeMaterial, autoSnake, 16, this);
    AnimateUntilCollision(this->snake);
    gameManager->snake = this->snake;
}

void SnakeGame::InitPtrs(){
    int sz = root->children.size();
    for (int i=0 ; i< sz ; i++){
        auto child = root->children.at(0);
        root->RemoveChild(child);
    }
}

void SnakeGame::Update(const Program& p, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(p, proj, view, model);
    p.SetUniform4f("lightColor", 0.8f, 0.3f, 0.0f, 0.5f);
    p.SetUniform4f("Kai", 1.0f, 0.3f, 0.6f, 1.0f);
    p.SetUniform4f("Kdi", 0.5f, 0.5f, 0.0f, 1.0f);
    p.SetUniform1f("specular_exponent", 5.0f);
    p.SetUniform4f("light_position", 0.0, 15.0f, 0.0, 1.0f);
    if (animate) {
        for (int i = 0; i < interactables.size(); i++) {
            auto element = interactables.at(i);
            element->Update();

        }
    }
}

void SnakeGame::AddInteractable(std::shared_ptr<Game::MovingObject> interactable)
{
    interactables.push_back(interactable);
}

void SnakeGame::LoadObjectFromFileDialog()
{
    std::string filename = igl::file_dialog_open();
    if (filename.length() == 0) return;

    auto shape = Model::Create(filename, carbon);
}

// TEMP Motion

void SnakeGame::UpdateXVelocity(bool direction){ //function for changing velocity on button click

	float change = direction ? velInterval*1 : velInterval*-1;
	snake->AddVelocity(change, Axis::X);
}

void SnakeGame::UpdateYVelocity(bool direction){
	float change = direction ? velInterval*1 : velInterval*-1;
	snake->AddVelocity(change, Axis::Y);
}

void SnakeGame::UpdateZVelocity(bool direction){
	float change = direction ? velInterval*1 : velInterval*-1;
	snake->AddVelocity(change, Axis::Z);
}

void SnakeGame::StopMotion(){
	snake->StopMoving();
}

void SnakeGame::RestartScene(){
    InitPtrs();
    const int DISPLAY_WIDTH = 800;
    const int DISPLAY_HEIGHT = 800;
    const float CAMERA_ANGLE = 45.0f;
    const float _NEAR = 0.1f;
    const float _FAR = 120.0f;
    this->Init(CAMERA_ANGLE, DISPLAY_WIDTH, DISPLAY_HEIGHT, _NEAR, _FAR);
    SetCamera(1);
}

void SnakeGame::KeyCallback(Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods)
{

    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        Eigen::MatrixX3f system = camList[0]->GetRotation().transpose();

		if (key == GLFW_KEY_SPACE){
            animate = !animate;
            if (!snake->isAlive) {
                float HS = gameManager->GetHighScore();
                RestartScene();
                gameManager->Restart();
                gameManager->SetHighScore(HS);
            }

            if (!gameManager->GetisStarted()) {
                gameManager->GameStart();
            }
            
			StopMotion();
        }
		// if (key == GLFW_KEY_UP)
		// 	snake->moveDir = Eigen::Vector3d(0, snake->GetMoveSpeed(), 0);
		// if (key == GLFW_KEY_DOWN)
        //     snake->moveDir = Eigen::Vector3d(0, -1*snake->GetMoveSpeed(), 0);
		
		// if (key == GLFW_KEY_LEFT)
		// 	snake->moveDir = Eigen::Vector3d(-1*snake->GetMoveSpeed(), 0, 0);
		
		// if (key == GLFW_KEY_RIGHT)
		// 	snake->moveDir = Eigen::Vector3d(snake->GetMoveSpeed(), 0, 0);
		if (key == GLFW_KEY_R){
           RestartScene();
        }
		
        
        
        if (key == GLFW_KEY_UP) 
            // snake->GetModel()->RotateByDegree(3, Axis::Z);
            snake->GetModel()->RotateInSystem(system, 0.1f, Axis::X);
			// UpdateYVelocity(true);
		if (key == GLFW_KEY_DOWN) 
            // snake->GetModel()->RotateByDegree(-3, Axis::Z);
            snake->GetModel()->RotateInSystem(system, -0.1f, Axis::X);
			// UpdateYVelocity(false);
		if (key == GLFW_KEY_LEFT)
            // snake->GetModel()->RotateByDegree(3, Axis::Y);
            snake->GetModel()->RotateInSystem(system, 0.1f, Axis::Y);
			// UpdateXVelocity(true);
        if (key == GLFW_KEY_RIGHT)
            // snake->GetModel()->RotateByDegree(-3, Axis::Y);
            snake->GetModel()->RotateInSystem(system, -0.1f, Axis::Y);
			// UpdateXVelocity(false);
        // keys 1-9 are objects 1-9 (objects[0] - objects[8]), key 0 is object 10 (objects[9])
        if (key >= GLFW_KEY_0 && key <= GLFW_KEY_9) {
            if (int index; (index = (key - GLFW_KEY_1 + 10) % 10) < camList.size())
                SetCamera(index);
        }
    }

    //SceneWithImGui::KeyCallback(nullptr, x, y, key, scancode, action, mods);
}

void SnakeGame::ViewportSizeCallback(Viewport* _viewport)
{
    for (auto& cam: camList)
        cam->SetProjection(float(_viewport->width) / float(_viewport->height));

    // note: we don't need to call Scene::ViewportSizeCallback since we are setting the projection of all the cameras
}

void SnakeGame::AddViewportCallback(Viewport* _viewport)
{
    viewport = _viewport;

    Scene::AddViewportCallback(viewport);
}

void SnakeGame::ShowObject(std::shared_ptr<cg3d::Model> model, bool dir){
    model->showFaces = dir;
    model->showWireframe = dir;
    model->showTextures = dir;
    model->isHidden = !dir;
}