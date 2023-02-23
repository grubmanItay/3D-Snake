#pragma once

#include <Eigen/Geometry>
#include <utility>
#include "Movable.h"
#include "AutoMorphingModel.h"
#include <vector>
#include <string>
#include <cstdint>

#include "GameManager.h"
#include "GameObject.h"
#include "SnakeGame.h"


namespace Game{

    class Snake{
        
        public:

            //model related
            Snake(std::shared_ptr<cg3d::Material> program, std::shared_ptr<cg3d::AutoMorphingModel> model, int links, SnakeGame* scene);
            void ResetSnake();
            int GetNumOfLinks();
            void GetHit(float amount);
            void Die();
            void AddVelocity(float amount, cg3d::Model::Axis axis);
            void StopMoving();
            float GetMoveSpeed();
            bool IsColliding();
            void InitSnake();
            void IncreaseHealth(float amount);
            void DecreaseHealth(float amount);
            std::shared_ptr<cg3d::AutoMorphingModel> GetModel();
            static std::shared_ptr<Snake> CreateSnake(std::shared_ptr<cg3d::Material> program, std::shared_ptr<cg3d::AutoMorphingModel> model, int links, SnakeGame* scene);
            bool shouldAnimate;
            bool isAlive;
            bool getisAlive() { return isAlive; }
            std::vector<std::shared_ptr<GameObject>> links;
            //game related
            Eigen::Vector3f GetMoveDirection();
            std::shared_ptr<cg3d::AutoMorphingModel> autoSnake;
            SnakeGame* scene;
            void Move(Eigen::Vector3d t);
            void Skinning(Eigen::Vector3d t);

            void CalcWeight(Eigen::MatrixXd &V, double min_z);
            Eigen::Vector3d moveDir;
            float currHealth;
            void InitGameValues();

        private:
            
            std::shared_ptr<cg3d::Material> snakeProgram;
            int numOfLinks;

            //game attributes
            std::shared_ptr<Game::GameManager> gameManager;

            float maxHealth;
            float moveSpeed;
            float velocityY;
            float velocityX;
            float velocityZ;
            bool isColliding;
            float linkLen;


        

            void initJoints();

            void RestartSnake();

            // TEMP - change later
            typedef
                std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
                RotationList;
            Eigen::MatrixXd V;
            Eigen::MatrixXd VHead; // Vertices of the current mesh (#V x 3)
            Eigen::MatrixXi FHead; // Faces of the mesh (#F x 3)
            Eigen::MatrixXd CT;
            Eigen::MatrixXi BET;
            Eigen::MatrixXd W, Cp, U, M;
            Eigen::MatrixXi  BE;
            Eigen::VectorXi P;
            RotationList rest_pose;
            RotationList anim_pose;
            std::vector<RotationList > poses; // rotations of joints for animation
            RotationList vQ;
            std::vector<Eigen::Vector3d> vC;
            std::vector<Eigen::Vector3d> vT;
            Eigen::Quaterniond quat;


    };

}