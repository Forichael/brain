#include "alpha_gazebo/fake_gripper.h"
#include <boost/bind.hpp>

namespace gazebo{
	AlphaFakeGripper::AlphaFakeGripper(){
		l_pub = nh_.advertise<std_msgs::Bool>("lim_sw", 10);
		g_sub = nh_.subscribe<std_msgs::Bool>("grip", 10, &AlphaFakeGripper::grip_cb, this);
		attached = false;

		base_link_name = "base_link";
		object_name = "can";
	}

	AlphaFakeGripper::~AlphaFakeGripper(){

	}

	void AlphaFakeGripper::grip_cb(const std_msgs::BoolConstPtr& msg){
		std::cout << "GRIP_CB CALLED" << std::endl;
		bool grip = msg->data;

		physics::WorldPtr world = model->GetWorld();
		if(!world.get()){
			gzerr << "Alpha Fake Gripper : World is NULL" << std::endl;
			return;
		}

		physics::ModelPtr obj = world->GetModel(object_name);
		if(!obj.get()){
			gzerr << "Alpha Fake Gripper : Object is NULL" << std::endl;
			return;
		}

		base_link = model->GetLink(base_link_name);
		if(!base_link.get()){
			gzerr << "base link doesn't exist??" << std::endl;
			return;
		}

		if(!grip){ // request to open
			if(attached){
				gripper_joint->Detach();
				obj->GetLink()->SetCollideMode("all");
			}
			attached = false;
		}else{
			if(!attached){
				gazebo::math::Pose diff = obj->GetLink()->GetWorldPose() - base_link->GetWorldPose();
				float tolerance = 0.05; // 5cm tolerance
				std::cout << "Pose Diff : " << diff << std::endl;
				if (fabs(.36-diff.pos.x) < tolerance && fabs(0.0 - diff.pos.y) < tolerance){

					gazebo::math::Pose p = obj->GetLink()->GetWorldPose(); // make upright

					p.rot.SetToIdentity();

					//float rz = b_p.rot.z;
					//float dx = .36 * cos(rz);
					//float dy = .36 * sin(rz);
					//p.pos.x = b_p.pos.x + dx;
					//p.pos.y = b_p.pos.y + dy;
					obj->GetLink()->SetWorldPose(p);
					
					diff.pos.x = .36;
					diff.pos.y = 0.0;

					gripper_joint->Load(base_link,obj->GetLink(), diff);
					gripper_joint->Attach(base_link,obj->GetLink());
					gripper_joint->SetAxis(0,gazebo::math::Vector3(1,0,0));
					gripper_joint->SetHighStop(0, 0);
					gripper_joint->SetLowStop(0, 0);

					obj->GetLink()->SetCollideMode("fixed");

					attached = true;
				}
			}
		}
	}

	void AlphaFakeGripper::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
		this->model = _parent;
		this->update_connection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&AlphaFakeGripper::OnUpdate,this,_1)
				);

		physics::PhysicsEnginePtr physics = model->GetWorld()->GetPhysicsEngine();
		this->gripper_joint = physics->CreateJoint("revolute", model);
	}
	void AlphaFakeGripper::OnUpdate(const common::UpdateInfo& _info){
		std_msgs::Bool msg;
		msg.data = attached;
		l_pub.publish(msg);
	}
}
