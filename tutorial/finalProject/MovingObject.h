#pragma once

#include "GameObject.h"
#include <random>
#include "igl/per_vertex_normals.h"
#include "SnakeGame.h"


namespace Game{

class MovingObject: public GameObject{

    public:
        explicit MovingObject(std::shared_ptr<cg3d::Material> material, std::shared_ptr<cg3d::Model> model, SnakeGame* scene);
        void SetSpeed(float s) { speed = s; }
        void  GenerateBezierCurve();
        void  DrawCurve();
        void DrawPoints();
        void Move();
        void SetTimeOut();
        void InitCurveModel();
        Eigen::Vector3f GenerateBezierVec();
        Eigen::Vector3f  GenerateMoveVec();

        Eigen::Vector3f  moveVec;
        // Eigen::Matrix4f M;
        Eigen::Matrix4d M;
        // Eigen::Matrix <float, 4, 3 > curvePoints;
        Eigen::Matrix <double, 4, 3 > curvePoints;
        Eigen::Matrix <double, 4, 3 > MG;
        // Eigen::Matrix <float, 4, 3 > MG;

        // Eigen::RowVector4f T;
        Eigen::RowVector4d T;
        std::vector<Eigen::Vector3d> spawnLocations;
        Eigen::Vector3d currentPosition;
        // Eigen::Vector3f currentPosition;

        bool moveBackwards;
        float t;
        float xCoordinate;
        float yCoordinate;
        float zCoordinate;

        virtual void  Update() =0;
        virtual void  OnCollision() = 0;
        float speed = 1;
    private:
        std::shared_ptr<cg3d::Model> curveModel;

};

}