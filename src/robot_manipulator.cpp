#include "robot_manipulator.h"


robot_manipulator::robot_manipulator() {
    //initialize model parameters
    base_pos = vec3(0, 0, 0);

    float def_len = 1.0f;
    float def_jsz = 0.30f;
    float def_lsz = 0.25f;
    float def_base_sz = 1.5f;
    this->l1_len = def_len;
    this->l2_len = def_len;
    this->l3_len = def_len/2;
    this->joint_sz = def_jsz;
    this->link_sz = def_lsz;
    this->base_sz = def_base_sz;

    //note that all angles are in degrees
    this->alpha = 45;
    this->beta = 45;
    this->theta = 45;
    this->theta_0 = 0;
    this->gamma = this->alpha + this->beta + this->theta;

    end_pos = vec3(def_len * 3 + def_jsz, 0, 0);

    //initialize shader
    auto lambert = gl::ShaderDef().lambert().color();
    shader = gl::getStockShader(lambert);

    //initialize geometry for links joints and the base
    auto link1 = geom::Cylinder();
    link1.height(this->l1_len);
    link1.radius(this->link_sz);

    auto link2 = geom::Cylinder();
    link2.height(this->l2_len);
    link2.radius(this->link_sz);

    auto link3 = geom::Cylinder();
    link3.height(this->l3_len);
    link3.radius(this->link_sz);

    auto joint_sphere = geom::Sphere();
    joint_sphere.radius(joint_sz);

    auto base_cube = geom::Cube();
    base_cube.size(vec3(base_sz));

    //push geometry into batch container
    model[0] = gl::Batch::create(base_cube, shader);
    model[1] = gl::Batch::create(joint_sphere, shader);
    model[2] = gl::Batch::create(link1, shader);
    model[3] = gl::Batch::create(link2, shader);
    model[4] = gl::Batch::create(link3, shader);
}

void robot_manipulator::init(vec3 base_pos, float limb_lens[3], float joint_sz , float link_sz)
{
    
    //initialize model parameters
    this->base_pos = base_pos;
    this->l1_len = limb_lens[0];
    this->l2_len = limb_lens[1];
    this->l3_len = limb_lens[2];
    this->joint_sz = joint_sz;
    this->link_sz = link_sz;

    //note that all angles are in degrees
    this->alpha = 0;
    this->beta = 0;
    this->theta = 0;
    this->theta_0 = 0;
    this->gamma = this->alpha + this->beta + this->theta;

    this->end_pos = vec3(0, 0, 0);

}



vec3 robot_manipulator::get_base_pos()
{
    return this->base_pos;
}


vec4 robot_manipulator::get_end_pos()
{
    return vec4(this->end_pos,gamma);
}

void robot_manipulator::draw()
{
    
    //some unit vectors for referrence
    gl::color(Color(1, 0, 0));
    gl::drawVector(vec3(-3, 0, 0), vec3(-2, 0, 0));
    gl::color(Color(0, 1, 0));
    gl::drawVector(vec3(-3, 0, 0), vec3(-3, 1, 0));
    gl::color(Color(0, 0, 1));
    gl::drawVector(vec3(-3, 0, 0), vec3(-3, 0, 1));
    gl::color(Color(1, 1, 1));

    //translate to base render position
    gl::translate(this->base_pos);
    
    this->model[0]->draw();                             //base cube
    gl::translate(vec3(0, base_sz/2, 0));
    
    //rotate base by theta_0 
    gl::rotate(( (-this->theta_0 - 90.0f) * M_PI) / 180.0f, vec3(0, 1, 0));


    //draw first joint accounting for alpha (rotate abt the z axis)
    gl::rotate(( ( this->alpha - 90.0f) * M_PI) / 180.0f, vec3(0, 0, 1));
    gl::translate(vec3(0, joint_sz / 2, 0));

    this->model[1]->draw();
    gl::drawCoordinateFrame();      //draw some unit vectors for referrence

    //draw first link
    gl::translate(vec3(0, joint_sz / 2, 0));
    this->model[2]->draw();

    
    //draw second joint accounting for beta (rotate abt the z axis)
    gl::translate(vec3(0, this->l1_len + (this->joint_sz / 2), 0));
    gl::rotate(( -this->beta * M_PI) / 180.0f, vec3(0, 0, 1));
    this->model[1]->draw();
    gl::drawCoordinateFrame();      //draw some unit vectors for referrence

    //draw second link
    gl::translate(vec3(0, joint_sz / 2, 0));
    this->model[3]->draw();

    //draw third joint accounting for beta (rotate abt the z axis)
    gl::translate(vec3(0, this->l2_len + (this->joint_sz / 2), 0));
    gl::rotate((this->theta * M_PI) / 180.0f, vec3(0, 0, 1));
    this->model[1]->draw();
    gl::drawCoordinateFrame();      //draw some unit vectors for referrence

    //draw third link
    gl::translate(vec3(0, joint_sz / 2, 0));
    this->model[4]->draw();

    

}


vec4 robot_manipulator::calcIK(vec4 dest)
{
    //dest is (x,y,z,gamma)

    //getting needed parameters to calculate angles
    float L1_sqrd = this->l1_len * this->l1_len;
    float L2_sqrd = this->l2_len * this->l2_len;
    float x0 = dest.x - (this->l3_len * cos(dest.w));
    float y0 = dest.y - (this->l3_len * cos(dest.w));
    float r0_sqrd = (x0 * x0) + (y0 * y0);
    float r0 = sqrt(r0_sqrd);
    float rz = sqrt((dest.x * dest.x) + (dest.y * dest.y));

    //calculating actual angles
    float alpha = 180 - acos((L1_sqrd + L2_sqrd - r0_sqrd)/(2 * this->l1_len * this->l2_len));
    float beta = atan(y0/x0) + acos((r0_sqrd + L1_sqrd - L2_sqrd)/(2 * r0 * this->l1_len));
    float theta = dest.w - alpha - beta;
    float theta_0 = acos(dest.z / rz);
    
    //note return -beta
    return vec4(alpha,-beta,theta,theta_0);
}
