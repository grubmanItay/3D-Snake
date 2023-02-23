#include "AnimationVisitor.h"



#include <string>
#include "igl/unproject_onto_mesh.h"
#include "Scene.h"
#include "Model.h"
#include "Utility.h"
#include <Eigen/Geometry>


namespace cg3d
{

void AnimationVisitor::Run(Scene* _scene, Camera* camera){
    Visitor::Run(scene = _scene, camera);
}

void AnimationVisitor::Visit(Model* model){
    Eigen::MatrixX3f system = model->GetRotation().transpose();
    if(scene->IsActive()){
        if(model->name == std::string("snake"))
            model->TranslateInSystem(system, Eigen::Vector3f(-0.1f,0,0));
    }
}

}

