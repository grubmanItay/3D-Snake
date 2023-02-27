#include "Snake.h"
#include <memory>
#include <per_face_normals.h>
#include <read_triangle_mesh.h>
#include <utility>
#include <string.h>
#include <vector>
#include "GLFW/glfw3.h"
#include "Mesh.h"
#include "PickVisitor.h"
#include "Renderer.h"
#include "ObjLoader.h"
#include "Movable.h"
#include "AutoMorphingModel.h"
#include "IglMeshLoader.h"

#include "igl/per_vertex_normals.h"
#include "igl/dqs.h"
#include "igl/deform_skeleton.h"
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
#include "GameObject.h"

#include "Utility.h"
#include "Util.h"

using namespace cg3d;
using namespace std;

#define MAX_HEALTH 15.0 //TODO make this config file property
#define MOVE_SPEED 0.005 //TODO make this config file property

namespace Game{


Snake::Snake(std::shared_ptr<cg3d::Material> program, std::shared_ptr<cg3d::AutoMorphingModel> model, int links, SnakeGame* scene): autoSnake(model) {
    // snakeMesh = mesh;
    this->scene = scene;
    this->gameManager = scene->gameManager;
    snakeProgram = program;
    numOfLinks = links;
    V = model->GetMeshList()[0]->data.back().vertices;
    InitSnake();
}

int Snake::GetNumOfLinks(){
    return numOfLinks;
}

std::shared_ptr<AutoMorphingModel> Snake::GetModel(){
    return autoSnake;
 }


std::shared_ptr<Snake> Snake::CreateSnake(std::shared_ptr<cg3d::Material> program, std::shared_ptr<cg3d::AutoMorphingModel> model, int numOfLinks, SnakeGame* scene){
    // return std::shared_ptr<Snake>{new Snake{program, model, numOfLinks}};
    return std::shared_ptr<Snake>{new Snake(program, model, numOfLinks, scene)};
 }

 void Snake::GetHit(float amount)
 {
    DecreaseHealth(amount);
 }

 void Snake::Die()
 {
    isAlive = false;
    Util::DebugPrint("snake dead");
    gameManager->GameEnd();
 }

 void Snake::AddVelocity(float amount, cg3d::Model::Axis axis)
 {
    //handle x velocity
    if (axis == cg3d::Model::Axis::X || axis == cg3d::Model::Axis::XY || axis == cg3d::Model::Axis::XZ || axis == cg3d::Model::Axis::XYZ) 
        velocityX += amount;
    //handle y velocity
    if (axis == cg3d::Model::Axis::Y || axis == cg3d::Model::Axis::XY || axis == cg3d::Model::Axis::YZ || axis == cg3d::Model::Axis::XYZ) 
        velocityY += amount;
    //handle z velocity
    if (axis == cg3d::Model::Axis::Z || axis == cg3d::Model::Axis::XZ || axis == cg3d::Model::Axis::YZ || axis == cg3d::Model::Axis::XYZ) 
        velocityZ += amount;
 }

 void Snake::StopMoving()
 {
    velocityX = velocityY = velocityZ = 0;
    moveDir = Eigen::Vector3d{0,0,0};
 }

 float Snake::GetMoveSpeed()
 {
    return moveSpeed;
 }

 bool Snake::IsColliding()
 {
    return false;
 }

 Eigen::Vector3f Snake::GetMoveDirection()
 {
    return Eigen::Vector3f(velocityX,velocityY,velocityZ).normalized();
 }

void Snake::InitSnake(){
   
    float scaleFactor = 0.3f;
    linkLen = 0.8f*scaleFactor;
    GameObject* link;

    link = new GameObject(snakeProgram, autoSnake, scene);
    link->partOfSnake = true;
    links.push_back(std::make_shared<Game::GameObject>(*link));
    auto cylMesh = Mesh::Cylinder();
    
    cyls.push_back( Model::Create("cyl 0",cylMesh, snakeProgram));
    cyls[0]->Scale(scaleFactor,cg3d::Movable::Axis::X);
    
    link = new GameObject(snakeProgram, cyls[0], scene);
    link->partOfSnake = true;
    links.push_back(std::make_shared<Game::GameObject>(*link));
    cyls[0]->SetCenter(Eigen::Vector3f(-linkLen,0,0));
    scene->root->AddChild(cyls[0]);
    //autoSnake->AddChild(cyls[0]);

    for(int i = 1;i < numOfLinks; i++)
        { 
        //cyls
        cyls.push_back(Model::Create("cyl " + std::to_string(i), cylMesh, snakeProgram));
        cyls[i]->Scale(scaleFactor,cg3d::Movable::Axis::X);   
        cyls[i]->Translate(1.6f*scaleFactor,cg3d::Movable::Axis::X);
        cyls[i]->SetCenter(Eigen::Vector3f(-linkLen,0,0));
        cyls[i-1]->AddChild(cyls[i]);   
        link = new GameObject(snakeProgram, cyls[i], scene);
        link->partOfSnake = true;
        links.push_back(std::make_shared<Game::GameObject>(*link));
     
    }
    InitGameValues();
    initJoints();
    
}

void Snake::DecreaseHealth(float amount)
{
    currHealth = (currHealth - amount)>=0 ? currHealth-amount : 0;
    Util::DebugPrint("snake health: " + std::to_string(currHealth));
    if(currHealth<=0){
        Die();
    }
}

void Snake::IncreaseHealth(float amount)
{
    currHealth = (currHealth + amount)<=maxHealth ? currHealth+amount : maxHealth;
    Util::DebugPrint("snake health: " + std::to_string(currHealth));
}

void Snake::InitGameValues(){
    isAlive = true;
    shouldAnimate = false;
    maxHealth = MAX_HEALTH;
    currHealth = maxHealth - 10;
    moveSpeed = MOVE_SPEED;
    StopMoving();
}

void Snake::initJoints(){
	vC.resize(numOfLinks + 1);
	vT.resize(numOfLinks + 1);
    Eigen::Vector3d min = V.colwise().minCoeff();
	double min_z = min[2];
	vQ.resize(numOfLinks + 1,Eigen::Quaterniond::Identity());
    BE.resize(16, 2);
	Cp.resize(17, 3);
	CT.resize(32, 3);
	BE << 0, 1,
		1, 2,
		2, 3,
		3, 4,
		4, 5,
		5, 6,
		6, 7,
		7, 8,
		8, 9,
		9,10,
		10, 11,
		11, 12,
		12, 13,
		13, 14,
		14, 15,
		15, 16;
       

	RestartSnake();

    CalcWeight(V, min_z);

}
void Snake::RestartSnake(){
    // scene->data(0).set_vertices(V);
	Eigen::Vector3d min = V.colwise().minCoeff();
	Eigen::Vector3d max = V.colwise().maxCoeff();
	float min_z = min[2];
	float max_z = max[2];
	// Eigen::Vector3d pos(0, 0, min_z);
	Eigen::MatrixXd points(17, 3);
	for (int i = 0; i < numOfLinks + 1; i++) {
	    // Eigen::Vector3d pos(links[i]->model->GetTranslation().cast<double>());
	    Eigen::Vector3d pos(links[i]->model->GetAggregatedTransform().block<3,1>(0,3).cast<double>());
		vC[i] = pos;
		vT[i] = pos;
		points.row(i) = pos;
		// pos = pos + Eigen::Vector3d(0, 0, linkLen);
	}
	// for (int i = 1; i < numOfLinks + 1; i++) {
	// 	links[i]->model->SetCenter(Eigen::Vector3f(0, 0, min_z + (i - 1) * linkLen));
	// 	links[i]->model->Translate(vC[i - 1].cast<float>());
	// }
	for (int i = 0; i < numOfLinks + 1; i++) {
		Cp.row(i) = vC[i];
	}
	for (int i = 1; i < numOfLinks + 1; i++) {
		CT.row(i * 2 - 1) = vC[i];
	}
}

void Snake::Skinning(Eigen::Vector3d t) {
	Move(t);
	const int dim = Cp.cols();
	Eigen::MatrixXd T(BE.rows() * (dim + 1), dim);
	for (int e = 0; e < BE.rows(); e++)
	{
		Eigen::Affine3d a = Eigen::Affine3d::Identity();
		a.translate(vT[e]);
		a.rotate(vQ[e]);
		T.block(e * (dim + 1), 0, dim + 1, dim) =
			a.matrix().transpose().block(0, 0, dim + 1, dim);
	}
	igl::dqs(V, W, vQ, vT, U);
	//move joints according to T, returns new position in CT and BET
	igl::deform_skeleton(Cp, BE, T, CT, BET);
    
    scene->data().set_vertices(U);
    scene->data().set_edges(CT, BET,Eigen::RowVector3d(70. / 255., 252. / 255., 167. / 255.));
    // scene->data(0).set_vertices(U);
    // scene->data(0).set_edges(CT, BET,Eigen::RowVector3d(70. / 255., 252. / 255., 167. / 255.));
    for (int i = 0; i < numOfLinks+ 1; i++) 
        vC[i] = vT[i];

    for (int i =1; i < numOfLinks + 1; i++){
        Eigen::MatrixX3f system = links[i]->model->GetRotation();
        // links[i]->model->Translate(CT.cast<float>().row(2*i-1)*0.0001f);
        links[i]->model->TranslateInSystem(system ,CT.cast<float>().row(2*i-1)*0.0001f);

    }

	
	
}

void Snake::CalcWeight(Eigen::MatrixXd& V, double min_z){
	int n = V.rows();
	W=Eigen::MatrixXd::Zero( n,numOfLinks+1);
	for (int i = 0; i < n; i++) {
		double curr_z = V.row(i)[2];
		for (int j = 0; j < numOfLinks+1; j++) {
			if (curr_z >= min_z + linkLen * j && curr_z <= min_z + linkLen * (j + 1)) {
				double res = abs(curr_z - (min_z + linkLen * (j + 1))) * 10;
				W.row(i)[j] = res;
				W.row(i)[j + 1] = 1-res ;
				break;
			}
		}

	}

}


void Snake::Move(Eigen::Vector3d t){
	for (int i = 0; i < numOfLinks; i++) 
		vT[i] = vT[i] + (vT[i + 1] - vT[i]) / 10;
	vT[numOfLinks] = vT[numOfLinks] + t;
	//update the joints
	for (int i = 1; i < numOfLinks + 1; i++) {
		quat = Eigen::Quaterniond::FromTwoVectors(vC[i] - vC[i - 1], vT[i] - vC[i - 1]);
        auto euler = quat.toRotationMatrix();
		// links[i]->model->RotateInSystem(links[i-1]->model->GetRotation(),euler.cast<float>());
		links[i]->model->Rotate(euler.cast<float>());
		// Eigen::Matrix3d mat =links[i]->model->GetTranslation();
        // links[i]->model->RotateByDegree	
	}
	
}
}