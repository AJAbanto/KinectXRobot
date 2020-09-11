#include "robot_manipulator.h"
#define deg_to_rad(deg) (((deg) * M_PI)/ 180.0f)
#define rad_to_deg(rad) (((rad) * 180.0f) / M_PI)

robot_manipulator::robot_manipulator() {
    //initialize model parameters
    base_pos = vec3(0, 0, 0);

    //forward kinematics testing stuff
    forward = vec4(70, -102, 34, 90);
    test_forward = true;

    float def_len = 1.5f;
    float def_jsz = 0.30f;
    float def_lsz = 0.25f;
    float def_base_sz = 1.5f;
    this->l1_len = 1.0;
    this->l2_len = 0.7;
    this->l3_len = 0.4;
    this->joint_sz = def_jsz;
    this->link_sz = def_lsz;
    this->base_sz = def_base_sz;

    this->tot_len = l1_len + l2_len + l3_len + 2 * joint_sz + joint_sz/2;

    //note that all angles are in degrees
    this->alpha = 0;
    this->beta = 0;
    this->theta = 0;
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
    return base_pos;
}


vec4 robot_manipulator::get_end_pos()
{
    return vec4(end_pos,gamma);
}

vec4 robot_manipulator::get_angles()
{
    return vec4(alpha , beta , theta, theta_0);
}

void robot_manipulator::display_info()
{

    string alpha_str = "Alpha:   " + std::to_string(alpha);
    string beta_str = "Beta:    " + std::to_string(beta);
    string theta_str = "Theta:   " + std::to_string(theta);
    string theta_0_str = "Theta_0: " + std::to_string(theta_0);
    string end_pos_str = "Input: (" + std::to_string(end_pos.x) + " , " + std::to_string(end_pos.y) + " , "
        + std::to_string(end_pos.z) + " ) "
        + "gamma : " + std::to_string(gamma);

    ImGui::Begin("Robot info");

    ImGui::Text(alpha_str.c_str());
    ImGui::Text(beta_str.c_str());
    ImGui::Text(theta_str.c_str());
    ImGui::Text(theta_0_str.c_str());
    ImGui::Text(end_pos_str.c_str());
    
    ImGui::Checkbox("Test forward kinematics", &test_forward);

    if(test_forward) ImGui::DragFloat4("Input angles: ", &forward, 1.0f);
    if(ImGui::Button("Reset angle vector")) forward = vec4(70, -102, 34, 90);
    ImGui::End();


    // Test inverse kinematics (x , y , z ,gamma)
    vec4 angles = calcIK(vec4(1.0f, 1.0f, 0.0f, 0.0f));

    alpha = angles.x;
    beta = angles.y;
    theta = angles.z;
    theta_0 = angles.w;

}

void robot_manipulator::draw()
{

    //set destination point and draw sphere there
    /*
    vec3 dest = vec3(1.0f, base_sz/2 + 1.0f, 0);    // should full extend towards the x-axis
    gl::color(0, 0.5f, 0.5f);
    gl::drawSphere(dest, 0.5f);
    */
    
    
    //check if we're testing forward kinematics
    if (test_forward) {
        alpha = forward.x;
        beta = forward.y;
        theta = forward.z;
        theta_0 = forward.w;
    }
    

    

    //some unit vectors for referrence
    gl::color(Color(1, 0, 0));
    gl::drawVector(vec3(-3, 0, 0), vec3(-2, 0, 0));
    gl::color(Color(0, 1, 0));
    gl::drawVector(vec3(-3, 0, 0), vec3(-3, 1, 0));
    gl::color(Color(0, 0, 1));
    gl::drawVector(vec3(-3, 0, 0), vec3(-3, 0, 1));
    gl::color(Color(1, 1, 1));

    //translate to base render position
    gl::translate(base_pos);
    
    model[0]->draw();                             //base cube
    gl::translate(vec3(0, base_sz/2, 0));
    
    //rotate base by theta_0 
    gl::rotate(deg_to_rad(theta_0 - 90.0), vec3(0, 1, 0));


    //draw first joint accounting for alpha (rotate abt the z axis)
    gl::rotate( deg_to_rad(alpha - 90.0), vec3(0, 0, 1));
    gl::translate(vec3(0, joint_sz / 2, 0));

    model[1]->draw();
    gl::drawCoordinateFrame();      //draw some unit vectors for referrence

    //draw first link
    model[2]->draw();

    
    //draw second joint accounting for beta (rotate abt the z axis)
    gl::translate(vec3(0, l1_len, 0));
    gl::rotate(deg_to_rad(beta), vec3(0, 0, 1));
    model[1]->draw();
    gl::drawCoordinateFrame();      //draw some unit vectors for referrence

    //draw second link
    model[3]->draw();

    //draw third joint accounting for beta (rotate abt the z axis)
    gl::translate(vec3(0, l2_len, 0));
    gl::rotate(deg_to_rad(theta), vec3(0, 0, 1));
    model[1]->draw();
    gl::drawCoordinateFrame();      //draw some unit vectors for referrence

    //draw third link
    model[4]->draw();

    

}


vec4 robot_manipulator::calcIK(vec4 dest)
{
    //dest is (x,y,z,gamma)
    //Note: needed to convert all parameters to radians

    //getting needed parameters to calculate angles
    float L1_sqrd = l1_len * l1_len;
    float L2_sqrd = l2_len * l2_len;
    float x0 = dest.x - ( l3_len * cos ( deg_to_rad(dest.w) ));
    float y0 = dest.y - ( l3_len * sin ( deg_to_rad(dest.w) ) );
    float r0_sqrd = (x0 * x0) + (y0 * y0);
    float r0 = sqrt(r0_sqrd);
    float rz = sqrt((dest.x * dest.x) + (dest.y * dest.y));




    //calculating actual angles in degrees
    float b = 180 - rad_to_deg(acos( deg_to_rad((L1_sqrd + L2_sqrd - r0_sqrd)/(2 * l1_len * l2_len)) ));
    float a = rad_to_deg( atan(deg_to_rad(y0/x0))) + rad_to_deg( acos( deg_to_rad(  (r0_sqrd + L1_sqrd - L2_sqrd) / (2 * r0 * l1_len) ) ));
    float t = dest.w - a + b;
    float t0 = rad_to_deg(acos(deg_to_rad(dest.z / rz)));
    


    //panel for debuging 
    //Note: can only use this if function not called in draw loop
    /*
    string x0_str = "x0: " + std::to_string(x0);
    string y0_str = "y0: " + std::to_string(y0);
    string r0_str = "r0: " + std::to_string(r0) + " r0_sqrd: " + std::to_string(r0_sqrd);
    string rz_str = "rz: " + std::to_string(rz);
    string l1_str = "L1: " + std::to_string(l1_len) + " L1_sqrd: " + std::to_string(L1_sqrd);
    string l2_str = "L2: " + std::to_string(l2_len) + " L2_sqrd: " + std::to_string(L2_sqrd);
    string l3_str = "L3: " + std::to_string(l3_len);

    string input_str = "Input: (" + std::to_string(dest.x) + " , " + std::to_string(dest.y) + " , "
        + std::to_string(dest.z) + " ) "
        + "gamma : " + std::to_string(dest.w);


    ImGui::Begin("Inverse kinematics debug");
    ImGui::Text(x0_str.c_str());
    ImGui::Text(y0_str.c_str());
    ImGui::Text(r0_str.c_str());
    ImGui::Text(rz_str.c_str());
    ImGui::Text(l1_str.c_str());
    ImGui::Text(l2_str.c_str());
    ImGui::Text(l3_str.c_str());
    ImGui::Text(input_str.c_str());
    
    ImGui::End();
    */

    //temporarily update end point
    end_pos = dest;


    return vec4(a,-b,t,t0);
}
