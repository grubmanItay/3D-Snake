#include "AnimationVisitor.h"



#include <string>
#include "igl/unproject_onto_mesh.h"
#include "Scene.h"
#include "Model.h"
#include "Utility.h"
#include <Eigen/Geometry>


namespace cg3d
{

    void AnimationVisitor::Run(Scene* _scene, Camera* camera)
    {
        scene = (SnakeGame*)_scene;
        snake = scene->snake;
        Visitor::Run(scene, camera);
    }

    void AnimationVisitor::Visit(Model* model)
    {
        Eigen::Matrix3f system = model->GetRotation().transpose();

        Eigen::Vector3f vector1z = Eigen::Vector3f(0, 0, 1);
        Eigen::Vector3f vector2z;

        Eigen::Vector3f vector1y = Eigen::Vector3f(0, 1, 0);
        Eigen::Vector3f vector2y;

        if (scene->IsActive())
        {
            number_of_links = 15;//int(snake->links.size()) - 1;

            if (model->name.find("cyl") != std::string::npos)
            {
                //float movement_speed = float(scene->game_manager->stats->current_movement_speed) * -0.1f;

                if (model->name == std::string("cyl 0"))
                {
                    model->TranslateInSystem(system, Eigen::Vector3f(-0.1f,0,0));
                    //snake.Skinning();
                }
                else {
                    std::string curr_link_name = std::string("cyl ") + std::to_string(link_index + 1);
                   // movement_speed = max(-0.95f, movement_speed + 0.1f);

                    if (model->name == "cyl 1" && link_index == 0) {
                        //Rotate Forward
                        vector2z = model->Tout.rotation() * vector1z;
                        quaternionz = Eigen::Quaternionf::FromTwoVectors(vector2z, vector1z);
                        quaternionz = quaternionz.slerp(0.95, Eigen::Quaternionf::Identity());
                        model->Rotate(quaternionz);

                        vector2y = model->Tout.rotation() * vector1y;
                        quaterniony = Eigen::Quaternionf::FromTwoVectors(vector2y, vector1y);
                        quaterniony = quaterniony.slerp(0.95, Eigen::Quaternionf::Identity());
                        model->Rotate(quaterniony);

                        link_index = (link_index + 1) % number_of_links;
                    }
                    else if (model->name == curr_link_name) {
                        //Rotate Backward
                        quaternionz = quaternionz.conjugate();
                        model->Rotate(quaternionz);

                        quaterniony = quaterniony.conjugate();
                        model->Rotate(quaterniony);


                        //Rotate Forward
                        vector2z = model->Tout.rotation() * vector1z;
                        quaternionz = Eigen::Quaternionf::FromTwoVectors(vector2z, vector1z);
                        quaternionz = quaternionz.slerp(0.95 , Eigen::Quaternionf::Identity());
                        model->Rotate(quaternionz);

                        vector2y = model->Tout.rotation() * vector1y;
                        quaterniony = Eigen::Quaternionf::FromTwoVectors(vector2y, vector1y);
                        quaterniony = quaterniony.slerp(0.95, Eigen::Quaternionf::Identity());
                        model->Rotate(quaterniony);

                        link_index = (link_index + 1) % number_of_links;
                    }
                }
                Visitor::Visit(model); // draw children first
            }
        }
    }
}


