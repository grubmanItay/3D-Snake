#pragma once

#include "Scene.h"
#include <utility>
#include "BasicScene.h"
#include <read_triangle_mesh.h>
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
#include <Eigen/Core>
#include <iostream>
#include <set>
#include "igl/aabb.h"


using namespace std;
using namespace Eigen;
using namespace igl;
using namespace cg3d;

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;

    void Reset();

    void KeyCallback(Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void new_Reset();
    void caculateQMatrix(Eigen::MatrixXd& V, Eigen::MatrixXi& F);
    void caculateCostAndPlacment(int edge, Eigen::MatrixXd& V);
    void Simplification();

    bool new_collapse_edge(Eigen::MatrixXd& V, Eigen::MatrixXi& F);

    void old_Simplification();

    void SetMesh();

    void Logs();

    void KeyUpEvent();

    void KeyDownEvent();

private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> cyl, sphere1, cube, bunny;
    Eigen::MatrixXi F, OF;
    Eigen::MatrixXd V, OV, VN, T;
    Eigen::VectorXi EQ;
    igl::opengl::glfw::Viewer viewer;
    int num_collapsed, index;
    std::shared_ptr<cg3d::Model> autoModel;
    igl::min_heap< std::tuple<double, int, int> > Q;
    Eigen::VectorXi EMAP; //edages to faces
    Eigen::MatrixXi E, EF, EI;
    typedef std::set<std::pair<double, int> > PriorityQueue;
    PriorityQueue newQ;		//priority queue - cost for every edge
    std::vector<PriorityQueue::iterator> Qit;
    Eigen::MatrixXd C; ///positions 
    std::vector <Eigen::Matrix4d> Qmatrix; //list of Q matrix for each vertical
    int numOfSimplification;
    bool NAVIGATION;
};
