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


	auto program = std::make_shared<Program>("shaders/basicShader");
	auto material{ std::make_shared<Material>("material", program) };

	material->AddTexture(0, "textures/box0.bmp", 2);
	auto sphereMesh{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };

	sphere1 = Model::Create("sphere", sphereMesh, material);

	//Logs();

	sphere1->showWireframe = true;
	camera->Translate(30, Axis::Z);

	auto sphere_mesh = sphere1->GetMeshList();

	V = sphere_mesh[0]->data[0].vertices;
	F = sphere_mesh[0]->data[0].faces;
	OF = F;
	OV = V;
	igl::edge_flaps(F, E, EMAP, EF, EI);

	auto morph_function = [](Model* model, cg3d::Visitor* visitor) {
		int curr = model->meshIndex;
		return (model->GetMeshList())[0]->data.size() * 0 + curr;
	};
	autoModel = AutoMorphingModel::Create(*sphere1, morph_function);
	root->AddChild(autoModel);
	autoModel->Translate({ 0, 0, 25 });

	initData();
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
	Scene::Update(program, proj, view, model);
	program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
	program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);
}

void BasicScene::Reset() {
	index = 0;
	autoModel->meshIndex = index;
	F = OF;
	V = OV;
	edge_flaps(F, E, EMAP, EF, EI);
	C.resize(E.rows(), V.cols());
	VectorXd costs(E.rows());

	Q = {};
	EQ = Eigen::VectorXi::Zero(E.rows());
	{
		Eigen::VectorXd costs(E.rows());
		igl::parallel_for(E.rows(), [&](const int e)
			{
				double cost = e;
				RowVectorXd p(1, 3);
				shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
				C.row(e) = p;
				costs(e) = cost;
			}, 10000);
		for (int e = 0;e < E.rows();e++)
		{
			Q.emplace(costs(e), e, 0);
		}
	}

	num_collapsed = 0;
	data().clear();
	data().set_mesh(V, F);
	data().set_face_based(true);
}

void BasicScene::KeyCallback(Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS || action == GLFW_REPEAT)
	{
		switch (key)
		{
		case GLFW_KEY_SPACE:
			Simplification();
			break;
		case GLFW_KEY_R:
			initData();
			break;
		case GLFW_KEY_UP:
			KeyUpEvent();
			break;
		case GLFW_KEY_DOWN:
			KeyDownEvent();
			break;
		default:
			break;
		}
	}
}

void BasicScene::initData() {
	data().set_mesh(OV, OF);

	index = 0;
	NAVIGATION = false;
	auto mesh = autoModel->GetMeshList();
	for (int i = 1; i < mesh.size();i++)
		mesh[0]->data.pop_back();
	autoModel->meshIndex = index;
	numOfSimplification = 0;
	F = OF;
	V = OV;
	igl::edge_flaps(F, E, EMAP, EF, EI);//init data_structures
	C.resize(E.rows(), V.cols());
	Qit.resize(E.rows()); //number of edges 
	caculateQMatrix(V, F);
	newQ.clear();
	EQ = Eigen::VectorXi::Zero(E.rows());
	num_collapsed = 0;
	//caculate egdes cost
	for (int j = 0; j < E.rows(); j++)
		caculateCostAndPlacment(j, V);
}

void BasicScene::caculateQMatrix(Eigen::MatrixXd& V, Eigen::MatrixXi& F) {
	std::vector<std::vector<int> > VF;// vertex to faces
	std::vector<std::vector<int> > VFi;//not used
	int n = V.rows();
	Qmatrix.resize(n);
	igl::vertex_triangle_adjacency(n, F, VF, VFi);
	Eigen::MatrixXd F_normals = data().F_normals;


	for (int i = 0; i < n; i++) {
		//initialize 
		Qmatrix[i] = Eigen::Matrix4d::Zero();

		//caculate vertex  Q matrix 
		for (int j = 0; j < VF[i].size(); j++) {
			Eigen::Vector3d normal = F_normals.row(VF[i][j]).normalized();//get face normal
			// the equation is ax+by+cz+d=0
			Eigen::Matrix4d curr;
			double a = normal[0];
			double b = normal[1];
			double c = normal[2];
			double d = V.row(i) * normal;
			d *= -1;
			curr.row(0) = Eigen::Vector4d(a * a, a * b, a * c, a * d);
			curr.row(1) = Eigen::Vector4d(a * b, b * b, b * c, b * d);
			curr.row(2) = Eigen::Vector4d(a * c, b * c, c * c, c * d);
			curr.row(3) = Eigen::Vector4d(a * d, b * d, c * d, d * d);
			Qmatrix[i] += curr;
		}

	}
}

void BasicScene::caculateCostAndPlacment(int edge, Eigen::MatrixXd& V)
{
	//vertexes of the edge
	int v1 = E(edge, 0);
	int v2 = E(edge, 1);

	Eigen::Matrix4d Qedge = Qmatrix[v1] + Qmatrix[v2];

	Eigen::Matrix4d Qposition = Qedge; //we will use this to find v` position
	Qposition.row(3) = Eigen::Vector4d(0, 0, 0, 1);
	Eigen::Vector4d vposition;
	double cost;
	bool isInversable;
	Qposition.computeInverseWithCheck(Qposition, isInversable);
	if (isInversable) {
		vposition = Qposition * (Eigen::Vector4d(0, 0, 0, 1));
		cost = vposition.transpose() * Qedge * vposition;
	}
	else {
		//find min error from v1 v2 v1+v2/2
		Eigen::Vector4d v1p;
		v1p << V.row(v1), 1;;
		double cost1 = v1p.transpose() * Qedge * v1p;

		Eigen::Vector4d v2p;
		v2p << V.row(v2), 1;;
		double cost2 = v2p.transpose() * Qedge * v2p;

		Eigen::Vector4d v12p;
		v12p << ((V.row(v1) + V.row(v2)) / 2), 1;;
		double cost3 = v12p.transpose() * Qedge * v12p;
		if (cost1 < cost2 && cost1 < cost3) {
			vposition = v1p;
			cost = cost1;
		}
		else if (cost2 < cost1 && cost2 < cost3) {
			vposition = v2p;
			cost = cost2;
		}
		else {
			vposition = v12p;
			cost = cost3;
		}
	}
	Eigen::Vector3d pos;
	pos[0] = vposition[0];
	pos[1] = vposition[1];
	pos[2] = vposition[2];
	C.row(edge) = pos;
	Qit[edge] = newQ.insert(std::pair<double, int>(cost, edge)).first;
}

void BasicScene::Simplification() {
	if (NAVIGATION) {
		KeyUpEvent();
		return;
	}
	V = data().V;  //vertice matrix
	F = data().F; //faces matrix
	bool something_collapsed = false;
	// collapse edge
	const int max_iter = std::ceil(0.05 * newQ.size());//collapse 5%
	for (int j = 0; j < max_iter; j++)
	{
		if (!new_collapse_edge(V, F)) {
			break;
		}
		something_collapsed = true;
		num_collapsed++;
	}

	if (something_collapsed)
	{
		data().set_mesh(V, F);
		data().set_face_based(true);
		data().dirty = 157;
		index++;
		numOfSimplification++;
		SetMesh();
	}
}

bool BasicScene::new_collapse_edge(Eigen::MatrixXd& V, Eigen::MatrixXi& F) {
	PriorityQueue& curr_Q = newQ;
	std::vector<PriorityQueue::iterator >& curr_Qit = Qit;
	int e1, e2, f1, f2; //be used in the igl collapse_edge function
	if (curr_Q.empty())
	{
		// no edges to collapse
		return false;
	}
	std::pair<double, int> pair = *(curr_Q.begin());
	if (pair.first == std::numeric_limits<double>::infinity())
	{
		// min cost edge is infinite cost
		return false;
	}
	curr_Q.erase(curr_Q.begin()); //delete from the queue
	int e = pair.second; //the lowest cost edge in the queue
	//the 2 vertix of the edge
	int v1 = E.row(e)[0];
	int v2 = E.row(e)[1];

	curr_Qit[e] = curr_Q.end();

	//get the  list of faces around the end point the edge
	std::vector<int> N = igl::circulation(e, true, EMAP, EF, EI);
	std::vector<int> Nd = igl::circulation(e, false, EMAP, EF, EI);
	N.insert(N.begin(), Nd.begin(), Nd.end());

	//collapse the edage
	bool is_collapsed = igl::collapse_edge(e, C.row(e), V, F, E, EMAP, EF, EI, e1, e2, f1, f2);
	if (is_collapsed) {


		// Erase the two, other collapsed edges
		curr_Q.erase(curr_Qit[e1]);
		curr_Qit[e1] = curr_Q.end();
		curr_Q.erase(curr_Qit[e2]);
		curr_Qit[e2] = curr_Q.end();

		//update the Q matrix for the 2 veterixes we collapsed 
		Qmatrix[v1] = Qmatrix[v1] + Qmatrix[v2];
		Qmatrix[v2] = Qmatrix[v1] + Qmatrix[v2];

		Eigen::VectorXd newPosition;
		// update local neighbors
		// loop over original face neighbors
		for (auto n : N)
		{
			if (F(n, 0) != IGL_COLLAPSE_EDGE_NULL ||
				F(n, 1) != IGL_COLLAPSE_EDGE_NULL ||
				F(n, 2) != IGL_COLLAPSE_EDGE_NULL)
			{
				for (int v = 0; v < 3; v++)
				{
					// get edge id
					const  int ei = EMAP(v * F.rows() + n);
					// erase old entry
					curr_Q.erase(curr_Qit[ei]);
					// compute cost and potential placement and place in queue
					caculateCostAndPlacment(ei, V);
					newPosition = C.row(ei);
				}
			}
		}
		std::cout << "edge " << e << ",cost " << pair.first << ",new position (" << newPosition[0] << ","
			<< newPosition[1] << "," << newPosition[2] << ")" << std::endl;
	}
	else
	{
		// reinsert with infinite weight (the provided cost function must **not**
		// have given this un-collapsable edge inf cost already)
		pair.first = std::numeric_limits<double>::infinity();
		curr_Qit[e] = curr_Q.insert(pair).first;
	}
	return is_collapsed;
}

void BasicScene::old_Simplification() {
	// If animating then collapse 10% of edges
	if (!Q.empty())
	{
		bool something_collapsed = false;
		// collapse edge
		const int max_iter = std::ceil(0.01 * Q.size());
		for (int j = 0;j < max_iter;j++)
		{
			if (!collapse_edge(shortest_edge_and_midpoint, V, F, E, EMAP, EF, EI, Q, EQ, C))
			{
				break;
			}
			something_collapsed = true;
			num_collapsed++;
		}

		if (something_collapsed)
		{
			data().clear();
			data().set_mesh(V, F);
			index++;
			SetMesh();
			data().set_face_based(true); //delete
		}
	}
}

void BasicScene::SetMesh() {
	igl::per_vertex_normals(data().V, data().F, VN);
	T = Eigen::MatrixXd::Zero(data().V.rows(), 2);
	auto mesh = autoModel->GetMeshList();
	mesh[0]->data.push_back({ data().V,data().F,VN,T });
	autoModel->SetMeshList(mesh);
	autoModel->meshIndex = index;
}

void BasicScene::Logs() {
	std::cout << "vertices: \n" << V << std::endl;
	std::cout << "faces: \n" << F << std::endl;

	std::cout << "edges: \n" << E.transpose() << std::endl;
	std::cout << "edges to faces: \n" << EF.transpose() << std::endl;
	std::cout << "faces to edges: \n " << EMAP.transpose() << std::endl;
	std::cout << "edges indices: \n" << EI.transpose() << std::endl;
}

void BasicScene::KeyUpEvent() {
	if (numOfSimplification == index)
		NAVIGATION = false;
	if (index < numOfSimplification)
		index++;
	autoModel->meshIndex = index;
}

void BasicScene::KeyDownEvent() {
	NAVIGATION = true;
	if (index > 0)
		index--;
	autoModel->meshIndex = index;
}