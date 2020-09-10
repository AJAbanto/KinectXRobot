#pragma once

#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include <math.h>


using namespace ci;
using namespace ci::app;

class robot_manipulator
{
	public:
		
		robot_manipulator();	//default constructor
		void init(vec3 base_pos, float limb_lens[3], float joint_sz ,float link_sz);	//reset/re-initialize parameters
		
		vec3 get_base_pos();// returns (x , y , z)
		vec4 get_end_pos(); // returns (x , y , z , gamma)
		void set_dest(vec4 dest); // accepts (x , y , z , gamma)
		
		void draw();
	private:
		
		float l1_len;		//Link lengths
		float l2_len;
		float l3_len;

		float base_sz;		//Base cube size
		float link_sz;		//Link radius

		float joint_sz;		//joint sphere radius

		float alpha;		//angle at the shoulder
		float beta;			//angle at the elbow
		float theta;		//angle at the wrist
		float gamma;		// alpha + beta + theta
		float theta_0;		//angle at the base joint

		vec3 base_pos;		//current base joint position   (x,y,z)
		vec3 end_pos;		//current end effector position (x,y,z)
		vec4 dest_pos;      //destination of end effector	(x,y,z,gamma)

		//testing shaders and shit
		gl::BatchRef model[5];
		gl::GlslProgRef shader;
		

		vec4 calcIK(vec4 dest);	//calculate Inverse kinematics returns (alpha,beta,theta,thata0)
		bool move_angles(); //updates actual angles
};

