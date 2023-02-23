#include "GameObject.h"
#include "GameManager.h"
#include "GameManager.h"
#include "Utility.h"
#include "Util.h"
#include "SnakeGame.h"

#define GENERIC_CYCLES 50

namespace Game{

GameObject::GameObject(std::shared_ptr<cg3d::Material> material, std::shared_ptr<cg3d::Model> model, SnakeGame* scene)
{
    this->scene = scene;
    this->material = material;
    this->model = model;
	this->index = scene->gameManager->gameObjects.size();
    this->name = model->name + "-" + std::to_string(this->index);
    InitCollider();
    this->isActive = true;
    this->timeout = 0;
    this->permaGone = false;
    this->cycles = GENERIC_CYCLES;
	this->partOfSnake = (material->name)==SNAKE_NAME;
    this->thisShared = std::make_shared<GameObject>(*this);
    this->scene->gameManager->gameObjects.push_back(thisShared);
}

void GameObject::InitCollider()
{
    // if(model->GetMeshList()[0]->data.size()<2 && model->GetMeshList()[0]->name != "snakeMesh")
    //     SnakeGame::PreDecimateMesh(model->GetMeshList()[0], false);
    Eigen::MatrixXd Ve;
	Eigen::MatrixXi Fa;
          
    Ve = (model->GetMeshList()[0]->data[model->GetMeshList()[0]->data.size()-1].vertices); // push vertices of collapsed mesh
    Fa = (model->GetMeshList()[0]->data[model->GetMeshList()[0]->data.size()-1].faces);
    colliderBoxTree.init(Ve, Fa);


}


void GameObject::DrawBox(Eigen::AlignedBox<double, 3>& box, int color,std::shared_ptr<cg3d::Model> model){
    //TEMP JSUT FOR DEBUG
	Eigen::RowVector3d colorVector;
	if (color == 1) {
		// model->material->program->SetUniform4f("lightColor", 0.0f, 0.8f,0.0f, 0.5f);
		model->material->AddTexture(0, "textures/grass.bmp", 2);
		//model->material->BindTextures();
	}
	else if (color == 2) {
		// model->material->program->SetUniform4f("lightColor", 0.8f, 0.0f,0.0f, 0.5f);
		model->material->AddTexture(0, "textures/box0.bmp", 2);
		// model->material->BindTextures();
	}
		
	Eigen::RowVector3d BottomRightCeil = box.corner(box.BottomRightCeil);
	Eigen::RowVector3d BottomRightFloor = box.corner(box.BottomRightFloor);
	Eigen::RowVector3d BottomLeftCeil = box.corner(box.BottomLeftCeil);
	Eigen::RowVector3d BottomLeftFloor = box.corner(box.BottomLeftFloor);
	Eigen::RowVector3d TopRightCeil = box.corner(box.TopRightCeil);
	Eigen::RowVector3d TopRightFloor = box.corner(box.TopRightFloor);
	Eigen::RowVector3d TopLeftCeil = box.corner(box.TopLeftCeil);
	Eigen::RowVector3d TopLeftFloor = box.corner(box.TopLeftFloor);
	Eigen::MatrixXd V, VN, T;
    Eigen::MatrixXi F;

    F.resize(12, 3); // 3d cubes have 12 faces 
    V.resize(8, 3); // and 8 vertices
    
	#pragma region cubemap
	// this maps the cube coords to vertices and faces to vertices
	V.row(0) = BottomLeftFloor;
    V.row(1) = BottomRightFloor;
    V.row(2) = TopLeftFloor;
    V.row(3) = TopRightFloor;
    V.row(4) = BottomLeftCeil;
    V.row(5) = BottomRightCeil;
    V.row(6) = TopLeftCeil;
    V.row(7) = TopRightCeil;
    F.row(0) = Eigen::Vector3i(0, 1, 3);
    F.row(1) = Eigen::Vector3i(3, 2, 0);
    F.row(2) = Eigen::Vector3i(4, 5, 7);
    F.row(3) = Eigen::Vector3i(7, 6, 4);
    F.row(4) = Eigen::Vector3i(0, 4, 6);
    F.row(5) = Eigen::Vector3i(6, 2, 0);
    F.row(6) = Eigen::Vector3i(5, 7, 3);
    F.row(7) = Eigen::Vector3i(7, 3, 1);
    F.row(8) = Eigen::Vector3i(2, 6, 7);
    F.row(9) = Eigen::Vector3i(7, 3, 2);
    F.row(10) = Eigen::Vector3i(4, 5, 1);
    F.row(11) = Eigen::Vector3i(1, 0, 4);

	#pragma endregion

    igl::per_vertex_normals(V, F, VN);
    T = Eigen::MatrixXd::Zero(V.rows(), 2);

    auto mesh = model->GetMeshList();
    mesh[0]->data.push_back({ V, F, VN, T }); // push new cube mesh to draw
    model->SetMeshList(mesh);
    model->meshIndex = mesh[0]->data.size()-1; // use last mesh pushed
    model->isHidden = false;
}

bool GameObject::CollidingWith(std::shared_ptr<Game::GameObject> otherObject){
    return ObjectsCollided(&(this->colliderBoxTree), &(otherObject->colliderBoxTree), thisShared, otherObject);
}

bool GameObject::ObjectsCollided(igl::AABB<Eigen::MatrixXd, 3>* boxTreeA, igl::AABB<Eigen::MatrixXd, 3>* boxTreeB, std::shared_ptr<Game::GameObject> thisObject, std::shared_ptr<Game::GameObject> otherObject){
    
    if (boxTreeA == nullptr || boxTreeB == nullptr){ // if no boxes at all
		return false;
	}
	
	if (!BoxesIntersect(boxTreeA->m_box, boxTreeB->m_box, thisObject, otherObject)) { //if boxes of the two models dont intersect
		return false;
	}
	if (boxTreeA->is_leaf() && boxTreeB->is_leaf()) {
		//if the two boxes are leaves and they intersect, draw them
		if(BoxesIntersect(boxTreeA->m_box, boxTreeB->m_box, thisObject, otherObject)){
            // DrawBox(boxTreeA->m_box, 2, scene->collisionCube2);
	    	// DrawBox(boxTreeB->m_box,2, scene->collisionCube1);
			// objectsCollided = true;
			return true;
		}
		else { //else leaves do not intersect
			return false;
		}

	}
	 if (boxTreeA->is_leaf() && !boxTreeB->is_leaf()) {
		 return ObjectsCollided(boxTreeA, boxTreeB-> m_right, thisObject, otherObject) ||
			 ObjectsCollided(boxTreeA, boxTreeB->m_left, thisObject, otherObject);
	}
	 if (!boxTreeA->is_leaf() && boxTreeB->is_leaf()) {
		 return ObjectsCollided(boxTreeA->m_right, boxTreeB, thisObject, otherObject) ||
			  ObjectsCollided(boxTreeA->m_left, boxTreeB, thisObject, otherObject);
	 }

     

	// check recursive conditions
    return ObjectsCollided(boxTreeA->m_left, boxTreeB->m_left, thisObject, otherObject) ||
		   ObjectsCollided(boxTreeA->m_left, boxTreeB->m_right, thisObject, otherObject) ||
		   ObjectsCollided(boxTreeA->m_right, boxTreeB->m_left, thisObject, otherObject) ||
		   ObjectsCollided(boxTreeA->m_right, boxTreeB->m_right, thisObject, otherObject);
}

bool GameObject::BoxesIntersect(Eigen::AlignedBox<double, 3>& boxA, Eigen::AlignedBox<double, 3>& boxB , std::shared_ptr<Game::GameObject> thisObject, std::shared_ptr<Game::GameObject> otherObject){
	// check matrices overlap as described in slides
	
    std::shared_ptr<cg3d::Model> model = thisObject->model;
    std::shared_ptr<cg3d::Model> otherModel = otherObject->model;
	// matrix A
	Eigen::Matrix3d A = model->GetRotation().cast<double>();
	Eigen::Vector3d A0 = A.col(0);
	Eigen::Vector3d A1 = A.col(1);
	Eigen::Vector3d A2 = A.col(2);

	// matrix B
	Eigen::Matrix3d B = otherModel->GetRotation().cast<double>();
	Eigen::Vector3d B0 = B.col(0);
	Eigen::Vector3d B1 = B.col(1);
	Eigen::Vector3d B2 = B.col(2);
	
	// C=A^T*B
	Eigen::Matrix3d C = A.transpose() * B;
	
	// aabb edge length is size/2
	Eigen::Vector3d a = boxA.sizes();
	Eigen::Vector3d b = boxB.sizes();
	a = a / 2;
	b = b / 2;
	// matrix D
	Eigen::Vector4d CenterA = Eigen::Vector4d(boxA.center()[0], boxA.center()[1], boxA.center()[2], 1);
	Eigen::Vector4d CenterB = Eigen::Vector4d(boxB.center()[0], boxB.center()[1], boxB.center()[2], 1);
	// Eigen::Vector4d D4d = otherModel->GetTransform().cast<double>() * CenterB - model->GetTransform().cast<double>() * CenterA;
	Eigen::Vector4d D4d = otherModel->GetAggregatedTransform().cast<double>() * CenterB - model->GetAggregatedTransform().cast<double>() * CenterA;
	Eigen::Vector3d D = D4d.head(3);
	
	// A conditions
	if (a(0) + (b(0) * abs(C.row(0)(0)) + b(1) * abs(C.row(0)(1)) + b(2) * abs(C.row(0)(2))) < abs(A0.transpose() * D))
	// if (a(0) + b.dot(C.row(0)) < abs(A0.transpose() * D))
		return false;
	if (a(1) + (b(0) * abs(C.row(1)(0)) + b(1) * abs(C.row(1)(1)) + b(2) * abs(C.row(1)(2))) < abs(A1.transpose() * D))
	// if (a(1) + b.dot(C.row(1)) < abs(A1.transpose() * D))
		return false;
	if (a(2) + (b(0) * abs(C.row(2)(0)) + b(1) * abs(C.row(2)(1)) + b(2) * abs(C.row(2)(2))) < abs(A2.transpose() * D))
	// if (a(2) + b.dot(C.row(2)) < abs(A2.transpose() * D))
		return false;
	// B conditions
	if (b(0) + (a(0) * abs(C.row(0)(0)) + a(1) * abs(C.row(1)(0)) + a(2) * abs(C.row(2)(0))) < abs(B0.transpose() * D))
	// if (b(0) + a.dot(C.row(0)) < abs(B0.transpose() * D))
		return false;
	if (b(1) + (a(0) * abs(C.row(0)(1)) + a(1) * abs(C.row(1)(1)) + a(2) * abs(C.row(2)(1))) < abs(B1.transpose() * D))
	// if (b(1) + a.dot(C.row(1)) < abs(B1.transpose() * D))
		return false;
	if (b(2) + (a(0) * abs(C.row(0)(2)) + a(1) * abs(C.row(1)(2)) + a(2) * abs(C.row(2)(2))) < abs(B2.transpose() * D))
	// if (b(2) + a.dot(C.row(2)) < abs(B2.transpose() * D))
		return false;
	// A0 conditions
	double R = C.row(1)(0) * A2.transpose() * D;
	R-=C.row(2)(0) * A1.transpose() * D;
	if (a(1) * abs(C.row(2)(0)) + a(2) * abs(C.row(1)(0)) + b(1) * abs(C.row(0)(2))+ b(2) * abs(C.row(0)(1)) < abs(R))
		return false;

	R = C.row(1)(1) * A2.transpose() * D;
	R -= C.row(2)(1) * A1.transpose() * D;
	if (a(1) * abs(C.row(2)(1)) + a(2) * abs(C.row(1)(1)) + b(0) * abs(C.row(0)(2)) + b(2) * abs(C.row(0)(0)) < abs(R))
		return false;

	R = C.row(1)(2) * A2.transpose() * D;
	R -= C.row(2)(2) * A1.transpose() * D;
	if (a(1) * abs(C.row(2)(2)) + a(2) * abs(C.row(1)(2)) + b(0) * abs(C.row(0)(1)) + b(1) * abs(C.row(0)(0)) < abs(R))
		return false;
	// A1 conditions

	R = C.row(2)(0) * A0.transpose() * D;
	R -= C.row(0)(0) * A2.transpose() * D;
	if (a(0) * abs(C.row(2)(0)) + a(2) * abs(C.row(0)(0)) + b(1) * abs(C.row(1)(2)) + b(2) * abs(C.row(1)(1)) < abs(R))
		return false;

	R = C.row(2)(1) * A0.transpose() * D;
	R -= C.row(0)(1) * A2.transpose() * D;
	if (a(0) * abs(C.row(2)(1)) + a(2) * abs(C.row(0)(1)) + b(0) * abs(C.row(1)(2)) + b(2) * abs(C.row(1)(0)) < abs(R))
		return false;

	R = C.row(2)(2) * A0.transpose() * D;
	R -= C.row(0)(2) * A2.transpose() * D;
	if (a(0) * abs(C.row(2)(2)) + a(2) * abs(C.row(0)(2)) + b(0) * abs(C.row(1)(1)) + b(1) * abs(C.row(1)(0)) < abs(R))
		return false;
	// A2 conditions

	R = C.row(0)(0) * A1.transpose() * D;
	R -= C.row(1)(0) * A0.transpose() * D;
	if (a(0) * abs(C.row(1)(0)) + a(1) * abs(C.row(0)(0)) + b(1) * abs(C.row(2)(2)) + b(2) * abs(C.row(2)(1)) < abs(R))
		return false;

	R = C.row(0)(1) * A1.transpose() * D;
	R -= C.row(1)(1) * A0.transpose() * D;
	if (a(0) * abs(C.row(1)(1)) + a(1) * abs(C.row(0)(1)) + b(0) * abs(C.row(2)(2)) + b(2) * abs(C.row(2)(0)) < abs(R))
		return false;
	R = C.row(0)(2) * A1.transpose() * D;
	R -= C.row(1)(2) * A0.transpose() * D;
	if (a(0) * abs(C.row(1)(2)) + a(1) * abs(C.row(0)(2)) + b(0) * abs(C.row(2)(1)) + b(1) * abs(C.row(2)(0)) < abs(R))
		return false;
	
	return true;

}

bool GameObject::AdvanceTime(){
    // advances cycles and checks if can act
    ticks+=1;
    if(timeout>0)
        timeout--;
    if(timeout == 0)
        Reactivate();
    return (ticks % cycles == 0 );
        
}

void GameObject::Reactivate(){
    if(!permaGone)
        isActive=true;
}

GameObject::GameObject()
{
}
}