#include "BasicScene.h"
#include "BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/decimate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/per_vertex_normals.h>
#include <Eigen/Core>
#include "AutoMorphingModel.h"
#include <iostream>
#include <set>

using namespace std;
using namespace Eigen;
using namespace igl;
using namespace cg3d;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
	camera = Camera::Create("camera", fov, float(width) / height, near, far);

	AddChild(root = Movable::Create("root"));
	auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
	daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
	auto background{ Model::Create("background", Mesh::Cube(), daylight) };
	AddChild(background);
	background->Scale(120, Axis::XYZ);
	background->SetPickable(false);
	background->SetStatic();

	// Choose Mesh - sphere is the default
	BUNNY = false;
	COW = false;
	TRUCK = false;
	// --------

	InitObjs();

	tree1.init(obj1->GetMeshList()[0]->data[0].vertices, obj1->GetMeshList()[0]->data[0].faces);
	tree2.init(obj2->GetMeshList()[0]->data[0].vertices, obj2->GetMeshList()[0]->data[0].faces);

	DrawCubes(tree1.m_box, cube1);
	DrawCubes(tree2.m_box, cube2);
}

void BasicScene::InitObjs()
{
	auto program = std::make_shared<Program>("shaders/basicShader");
	auto material{ std::make_shared<Material>("material", program) };
	auto material2{ std::make_shared<Material>("material", program) };

	material->AddTexture(0, "textures/box0.bmp", 2);
	material2->AddTexture(0, "textures/grass.bmp", 2);

	auto sphereMesh1{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };
	auto sphereMesh2{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };

	auto bunnyMesh1{ IglLoader::MeshFromFiles("bunny_igl","data/bunny.off") };
	auto bunnyMesh2{ IglLoader::MeshFromFiles("bunny_igl","data/bunny.off") };

	auto cowMesh1{ IglLoader::MeshFromFiles("cow_igl","data/cow.off") };
	auto cowMesh2{ IglLoader::MeshFromFiles("cow_igl","data/cow.off") };

	auto truckMesh1{ IglLoader::MeshFromFiles("truck_igl","data/truck.obj") };
	auto truckMesh2{ IglLoader::MeshFromFiles("truck_igl","data/truck.obj") };

	auto cubeMesh1{ IglLoader::MeshFromFiles("cube_igl", "data/cube.off") };
	auto cubeMesh2{ IglLoader::MeshFromFiles("cube_igl", "data/cube.off") };
	auto cubeMesh3{ IglLoader::MeshFromFiles("cube_igl", "data/cube.off") };
	auto cubeMesh4{ IglLoader::MeshFromFiles("cube_igl", "data/cube.off") };


	cube1 = Model::Create("cube1", cubeMesh1, material);
	cube2 = Model::Create("cube2", cubeMesh2, material);
	cube3 = Model::Create("cube3", cubeMesh3, material2);
	cube4 = Model::Create("cube4", cubeMesh4, material2);

	if (BUNNY) {
		obj1 = Model::Create("bunny1", bunnyMesh1, material);
		obj2 = Model::Create("bunny2", bunnyMesh2, material);

		obj1->Translate({ -0.3f,0.0f,0.0f });
		cube1->Translate({ -0.3f,0.0f,0.0f });
		obj2->Translate({ 0.3f,0.0f,0.0f });
		cube2->Translate({ 0.3f,0.0f,0.0f });
		camera->Translate(1, Axis::Z);
	}
	else if (COW) {
		obj1 = Model::Create("cow1", cowMesh1, material);
		obj2 = Model::Create("cow2", cowMesh1, material);

		obj2->RotateByDegree(180, Axis::Y);

		obj1->Translate({ -1.0f,0.0f,0.0f });
		cube1->Translate({ -1.0f,0.0f,0.0f });
		obj2->Translate({ 1.0f,0.0f,0.0f });
		cube2->Translate({ 1.0f,0.0f,0.0f });
		
		camera->Translate(3.5, Axis::Z);
	}else if (TRUCK) {
		obj1 = Model::Create("truck1", truckMesh1, material);
		obj2 = Model::Create("truck2", truckMesh2, material);

		obj2->RotateByDegree(180, Axis::Y);

		obj1->Translate({ -1.0f,0.0f,0.0f });
		cube1->Translate({ -1.0f,0.0f,0.0f });
		obj2->Translate({ 1.0f,0.0f,0.0f });
		cube2->Translate({ 1.0f,0.0f,0.0f });

		camera->Translate(4.5, Axis::Z);
	}
	else {
		obj1 = Model::Create("sphere1", sphereMesh1, material);
		obj2 = Model::Create("sphere2", sphereMesh2, material);
		obj1->Translate({ -1.5f,0.0f,0.0f });
		cube1->Translate({ -1.5f,0.0f,0.0f });
		obj2->Translate({ 1.5f,0.0f,0.0f });
		cube2->Translate({ 1.5f,0.0f,0.0f });
		camera->Translate(10, Axis::Z);
	}
	cube1->showFaces = false;
	cube2->showFaces = false;
	obj1->showWireframe = true;
	obj2->showWireframe = true;


	root->AddChild(obj1);
	root->AddChild(obj2);
	root->AddChild(cube1);
	root->AddChild(cube2);
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
	if (!COLLISION && !isCollision(&tree1, &tree2)) {
		obj1->Translate({ 0.001f,0.0f,0.0f });
		cube1->Translate({ 0.001f,0.0f,0.0f });
		obj2->Translate({ -0.001f,0.0f,0.0f });
		cube2->Translate({ -0.001f,0.0f,0.0f });
	}
	else {
		COLLISION = true;
	}
}

void BasicScene::DrawCubes(Eigen::AlignedBox<double, 3>& box, std::shared_ptr<cg3d::Model> cube)
{
	Eigen::RowVector3d BottomRightCeil = box.corner(box.BottomRightCeil);
	Eigen::RowVector3d BottomRightFloor = box.corner(box.BottomRightFloor);
	Eigen::RowVector3d BottomLeftCeil = box.corner(box.BottomLeftCeil);
	Eigen::RowVector3d BottomLeftFloor = box.corner(box.BottomLeftFloor);
	Eigen::RowVector3d TopRightCeil = box.corner(box.TopRightCeil);
	Eigen::RowVector3d TopRightFloor = box.corner(box.TopRightFloor);
	Eigen::RowVector3d TopLeftCeil = box.corner(box.TopLeftCeil);
	Eigen::RowVector3d TopLeftFloor = box.corner(box.TopLeftFloor);

	Eigen::MatrixXi F;
	Eigen::MatrixXd V, VN, T;

	V.resize(8, 3);
	F.resize(12, 3);

	V.row(0) = BottomRightCeil;
	V.row(1) = BottomRightFloor;
	V.row(2) = BottomLeftCeil;
	V.row(3) = BottomLeftFloor;
	V.row(4) = TopRightCeil;
	V.row(5) = TopRightFloor;
	V.row(6) = TopLeftCeil;
	V.row(7) = TopLeftFloor;

	F.row(0) = Eigen::Vector3i(0, 1, 2);
	F.row(1) = Eigen::Vector3i(1, 2, 3);
	F.row(2) = Eigen::Vector3i(4, 5, 6);
	F.row(3) = Eigen::Vector3i(5, 6, 7);
	F.row(4) = Eigen::Vector3i(2, 6, 7);
	F.row(5) = Eigen::Vector3i(2, 3, 7);
	F.row(6) = Eigen::Vector3i(0, 1, 5);
	F.row(7) = Eigen::Vector3i(0, 4, 5);
	F.row(8) = Eigen::Vector3i(1, 3, 7);
	F.row(9) = Eigen::Vector3i(1, 5, 7);
	F.row(10) = Eigen::Vector3i(0, 2, 6);
	F.row(11) = Eigen::Vector3i(0, 4, 6);

	igl::per_vertex_normals(V, F, VN);
	T = Eigen::MatrixXd::Zero(V.rows(), 2);

	auto mesh = cube->GetMeshList();
	mesh[0]->data.push_back({ V,F,VN,T });
	cube->SetMeshList(mesh);
	cube->meshIndex += 1;
}

bool BasicScene::isCollision(igl::AABB<Eigen::MatrixXd, 3>* treeA, igl::AABB<Eigen::MatrixXd, 3>* treeB) {
	//base cases
	if (treeA == nullptr || treeB == nullptr)
		return false;
	if (!boxesIntersect(treeA->m_box, treeB->m_box)) {
		return false;
	}
	if (treeA->is_leaf() && treeB->is_leaf()) {
		DrawCubes(treeA->m_box, cube3);
		DrawCubes(treeB->m_box, cube4);
		obj1->AddChild(cube3);
		obj2->AddChild(cube4);
		return true;

	}
	if (treeA->is_leaf() && !treeB->is_leaf()) {

		return isCollision(treeA, treeB->m_right) ||
			isCollision(treeA, treeB->m_left);
	}
	if (!treeA->is_leaf() && treeB->is_leaf()) {
		return isCollision(treeA->m_right, treeB) ||
			isCollision(treeA->m_left, treeB);
	}

	return isCollision(treeA->m_left, treeB->m_left) ||
		isCollision(treeA->m_left, treeB->m_right) ||
		isCollision(treeA->m_right, treeB->m_left) ||
		isCollision(treeA->m_right, treeB->m_right);
}

bool BasicScene::boxesIntersect(Eigen::AlignedBox<double, 3>& boxA, Eigen::AlignedBox<double, 3>& boxB) {
	// matrix A
	Eigen::Matrix3d A = obj1->GetRotation().cast<double>();
	Eigen::Vector3d A0 = A.col(0);
	Eigen::Vector3d A1 = A.col(1);
	Eigen::Vector3d A2 = A.col(2);

	// matrix B
	Eigen::Matrix3d B = obj2->GetRotation().cast<double>();
	Eigen::Vector3d B0 = B.col(0);
	Eigen::Vector3d B1 = B.col(1);
	Eigen::Vector3d B2 = B.col(2);
	//C=A^T*B
	Eigen::Matrix3d C = A.transpose() * B;
	//get the lengths of the sides of the bounding box
	Eigen::Vector3d a = boxA.sizes();
	Eigen::Vector3d b = boxB.sizes();
	a = a / 2;
	b = b / 2;
	//build matrix D
	Eigen::Vector4d CenterA = Eigen::Vector4d(boxA.center()[0], boxA.center()[1], boxA.center()[2], 1);
	Eigen::Vector4d CenterB = Eigen::Vector4d(boxB.center()[0], boxB.center()[1], boxB.center()[2], 1);
	Eigen::Vector4d D4d = obj2->GetTransform().cast<double>() * CenterB - obj1->GetTransform().cast<double>() * CenterA;
	Eigen::Vector3d D = D4d.head(3);
	//check the 15 conditions
	//check A conditions
	if (a(0) + (b(0) * abs(C.row(0)(0)) + b(1) * abs(C.row(0)(1)) + b(2) * abs(C.row(0)(2))) < abs(A0.transpose() * D))
		return false;
	if (a(1) + (b(0) * abs(C.row(1)(0)) + b(1) * abs(C.row(1)(1)) + b(2) * abs(C.row(1)(2))) < abs(A1.transpose() * D))
		return false;
	if (a(2) + (b(0) * abs(C.row(2)(0)) + b(1) * abs(C.row(2)(1)) + b(2) * abs(C.row(2)(2))) < abs(A2.transpose() * D))
		return false;
	//check B conditions
	if (b(0) + (a(0) * abs(C.row(0)(0)) + a(1) * abs(C.row(1)(0)) + a(2) * abs(C.row(2)(0))) < abs(B0.transpose() * D))
		return false;
	if (b(1) + (a(0) * abs(C.row(0)(1)) + a(1) * abs(C.row(1)(1)) + a(2) * abs(C.row(2)(1))) < abs(B1.transpose() * D))
		return false;
	if (b(2) + (a(0) * abs(C.row(0)(2)) + a(1) * abs(C.row(1)(2)) + a(2) * abs(C.row(2)(2))) < abs(B2.transpose() * D))
		return false;
	//check A0 
	double R = C.row(1)(0) * A2.transpose() * D;
	R -= C.row(2)(0) * A1.transpose() * D;
	if (a(1) * abs(C.row(2)(0)) + a(2) * abs(C.row(1)(0)) + b(1) * abs(C.row(0)(2)) + b(2) * abs(C.row(0)(1)) < abs(R))
		return false;
	R = C.row(1)(1) * A2.transpose() * D;
	R -= C.row(2)(1) * A1.transpose() * D;
	if (a(1) * abs(C.row(2)(1)) + a(2) * abs(C.row(1)(1)) + b(0) * abs(C.row(0)(2)) + b(2) * abs(C.row(0)(0)) < abs(R))
		return false;
	R = C.row(1)(2) * A2.transpose() * D;
	R -= C.row(2)(2) * A1.transpose() * D;
	if (a(1) * abs(C.row(2)(2)) + a(2) * abs(C.row(1)(2)) + b(0) * abs(C.row(0)(1)) + b(1) * abs(C.row(0)(0)) < abs(R))
		return false;
	//check A1 conditions
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
	//check A2 conditions
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