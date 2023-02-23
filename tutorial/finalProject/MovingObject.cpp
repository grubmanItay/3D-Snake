#include "MovingObject.h"
#include "GameManager.h"
#include "Util.h"
#include "ViewerData.h"
#include "ViewerCore.h"



#define DAMPENER 0.0007
#define TIMEOUT 1000


Game::MovingObject::MovingObject(std::shared_ptr<cg3d::Material> material, std::shared_ptr<cg3d::Model> model, SnakeGame* scene) : Game::GameObject(material, model, scene)
{

    xCoordinate = model->GetTranslation()(0);
    yCoordinate = model->GetTranslation()(1);
    zCoordinate = model->GetTranslation()(2);
	this->moveBackwards = false;
	this->t = 0;
	InitCurveModel();
    this->GenerateBezierCurve();
	this->moveVec = GenerateBezierVec();
	// this->moveVec = GenerateMoveVec();
}

void Game::MovingObject::GenerateBezierCurve() {
	t=0;
	Eigen::Vector3f pRow1;
	Eigen::Vector3f pRow2;
	Eigen::Vector3f pRow3;
	Eigen::Vector3f pRow4;

	//bezier curves slides

	float minX = -3;
	float minY = -3;
	float minZ = -3;
	float maxX = 3;
	float maxY = 3;
	float maxZ = 3;
	// float minX = 0;
	// float minY = -1;
	// float minZ = -5;
	// float maxX = 4;
	// float maxY = 5;
	// float maxZ = 5;

	// float randX = Util::GenerateRandomInRange(minX,maxX);
	// float randY = Util::GenerateRandomInRange(minY,maxY);
	// float randZ = Util::GenerateRandomInRange(minZ,maxZ);
	// std::uniform_int_distribution<int> distributeX(xCoordinate, xCoordinate + 4);
	// std::uniform_int_distribution<int> distributeY(yCoordinate-1, yCoordinate + 5);
	// std::uniform_int_distribution<int> distributeZ(zCoordinate-5, zCoordinate + 5 );


	pRow1 = Eigen::Vector3f(Util::GenerateRandomInRange(minX,maxX), Util::GenerateRandomInRange(minY,maxY), Util::GenerateRandomInRange(minZ,maxZ));
	pRow2 = Eigen::Vector3f(Util::GenerateRandomInRange(minX,maxX), Util::GenerateRandomInRange(minY,maxY), Util::GenerateRandomInRange(minZ,maxZ));
	pRow3 = Eigen::Vector3f(Util::GenerateRandomInRange(minX,maxX), Util::GenerateRandomInRange(minY,maxY), Util::GenerateRandomInRange(minZ,maxZ));
	pRow4 = Eigen::Vector3f(Util::GenerateRandomInRange(minX,maxX), Util::GenerateRandomInRange(minY,maxY), Util::GenerateRandomInRange(minZ,maxZ));

	curvePoints.row(0) = pRow1.cast<double>();
	curvePoints.row(1) = pRow2.cast<double>();
	curvePoints.row(2) = pRow3.cast<double>();
	curvePoints.row(3) = pRow4.cast<double>();

	M << -1, 3, -3, 1,
		3, -6, 3, 0,
		-3, 3, 0, 0,
		1, 0, 0, 0;

	MG = M * curvePoints;

	// DrawPoints(); 
}

Eigen::Vector3f Game::MovingObject::GenerateBezierVec()
{
	// GenerateBezierCurve();

 	T[0] = powf(t, 3);
	T[1] = powf(t, 2);
	T[2] = t;
	T[3] = 1;

	currentPosition = (T * MG);
	// model->Translate(currentPosition.cast<float>());

	double dampener = DAMPENER;
	// Eigen::Vector3f moveVec = currentPosition.cast<float>().normalized() * dampener * speed;
	Eigen::Vector3f moveVec = currentPosition.cast<float>().normalized() * dampener * speed;
	this->moveVec = moveVec;
	return(moveVec);
}

Eigen::Vector3f Game::MovingObject::GenerateMoveVec()
{
	float min = -10;
	float max = 10;
	float distXY = Util::GenerateRandomInRange(min, max);
	float distZ = Util::GenerateRandomInRange(min, max);
	// TEMP
	double dampener = DAMPENER;
	Eigen::Vector3f moveVec = Eigen::Vector3f(distXY, distXY, distZ).normalized() * dampener * speed;
	
	this->moveVec = moveVec;
	// Util::PrintVector(moveVec);
    return moveVec;
}

void Game::MovingObject::Move()
{	
	GenerateBezierVec();
	model->Translate(this->moveVec);
	t+= 0.3*DAMPENER * speed;
	
}

void Game::MovingObject::SetTimeOut(){
    isActive = false;
    timeout = TIMEOUT;
}

void Game::MovingObject::InitCurveModel(){
	auto modelShader = std::make_shared<cg3d::Program>("shaders/basicShader"); 
    auto obstacleMaterial = std::make_shared<cg3d::Material>("bricks", modelShader);
	this->curveModel = cg3d::Model::Create("Curve", cg3d::Mesh::Cube(), obstacleMaterial);
	curveModel->showFaces = false;
	curveModel->showWireframe = true;
	curveModel->isHidden = true;
	// curveModel->mode = 3;
	curveModel->SetTransform(model->GetAggregatedTransform());
	scene->root->AddChild(curveModel);
	curveModel->SetPickable(true);

    
}

void Game::MovingObject::DrawPoints() {
	// Eigen::Vector3d drawingColor = Eigen::RowVector3d(1, 1, 1);
	Eigen::Matrix3d color = Eigen::Matrix3d::Ones();
	auto mesh = curveModel->GetMeshList();    
	Eigen::MatrixXd verts = mesh[0]->data[0].vertices;
	Eigen::MatrixXd VN, T;
	Eigen::MatrixXi faces = mesh[0]->data[0].faces ;
	faces.resize(3, 3); // 
    verts.resize(4, 3); // 4 vertices
    mesh[0]->data.clear();
    mesh[0]->data.size();
	for (int i = 0; i <= 3; i++){
		verts.row(i) = curvePoints.row(i);
		
	}
	faces.row(0) = Eigen::Vector3i(0, 1, 1);
	faces.row(1) = Eigen::Vector3i(1, 2, 2);
	faces.row(2) = Eigen::Vector3i(2, 3, 3);
	igl::per_vertex_normals(verts, faces, VN);
    T = Eigen::MatrixXd::Zero(verts.rows(), 2);

    mesh[0]->data.push_back({ verts, faces, VN, T }); // push new cube mesh to draw
    curveModel->SetMeshList(mesh);
	// curveModel->viewerDataListPerMesh[0][0].clear_points();
	// for (int i = 0; i < 3; i++){
	// 	curveModel->viewerDataListPerMesh[0][0].add_points(curvePoints.row(i), color);
	// 	curveModel->viewerDataListPerMesh[0][0].add_edges(curvePoints.row(i),curvePoints.row(i+1), color);
		
	// }
	// // scene->data().set_points(curvePoints, color);
	// curveModel->viewerDataListPerMesh[0][0].show_lines=3;
	// curveModel->viewerDataListPerMesh[0][0].show_overlay=3;
	// curveModel->viewerDataListPerMesh[0][0].show_overlay_depth=3;
	// curveModel->viewerDataListPerMesh[0][0].show_texture=3;
    curveModel->meshIndex = mesh[0]->data.size()-1; // use last mesh pushed
	curveModel->SetTransform(model->GetAggregatedTransform());
	curveModel->isHidden = false;
	
}

void Game::MovingObject::DrawCurve() {
	// Eigen::Vector3d color = Eigen::RowVector3d(1, 1, 1);
	// Eigen::Matrix3d color = Eigen::Matrix3d::Ones();
	// auto mesh = curveModel->GetMeshList();    
	// Eigen::MatrixXd verts = mesh[0]->data[0].vertices;
	// Eigen::MatrixXd VN, T;
	// Eigen::MatrixXi faces = mesh[0]->data[0].faces ;
	// faces.resize(99, 3); // 
    // verts.resize(100, 3); // 4 vertices
	// verts.row(0) = curvePoints.row(0);
	// for (int i = 1; i <= 99; i++){
	// 	verts.row(i) = curvePoints.row(i);
		
	// }
	// faces.row(0) = Eigen::Vector3i(0, 1, 1);
	// faces.row(1) = Eigen::Vector3i(1, 2, 2);
	// faces.row(2) = Eigen::Vector3i(2, 3, 3);
	// igl::per_vertex_normals(verts, faces, VN);
    // T = Eigen::MatrixXd::Zero(verts.rows(), 2);

    // mesh[0]->data.push_back({ verts, faces, VN, T }); // push new cube mesh to draw
    // curveModel->SetMeshList(mesh);
    // curveModel->meshIndex = mesh[0]->data.size()-1; // use last mesh pushed
	// curveModel->SetTransform(model->GetAggregatedTransform());
	// curveModel->isHidden = false;
}
	
	

