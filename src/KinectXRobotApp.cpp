#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/CinderImGui.h"

#include <Windows.h>
#include <synchapi.h>
#include <iostream>
#include <string>
#include <queue>

#include "NuiApi.h"
#include "robot_manipulator.h"
#include "serial.h"

//Taken from stack over flow for debugging printf
#include <sstream>

#define DBOUT( s )            \
{                             \
   std::wostringstream os_;    \
   os_ << s;                   \
   OutputDebugStringW( os_.str().c_str() );  \
}

using namespace ci;
using namespace ci::app;
using namespace std;

class KinectXRobotApp : public App {
  public:
	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void update() override;
	void draw() override;

	//some camera objects to determine view
	CameraPersp left_cam;
	CameraPersp right_cam;

	vec3 left_eye_point;
	vec3 left_look_at;
	vec3 right_eye_point;
	vec3 right_look_at;
	float right_fp;
	float left_fp;

	//set defaults in this function
	void set_cam_def();

	

	static const int res_width = 320;
	static const int res_height = 240;

	//robot manipulator model
	robot_manipulator r1;
	
	//robot model control
	vec4 robot_dest;		//vector to be passed to robot model  
	vec3 robot_home_point;	//home position or initial position of model 
	
	//robot model length parameters
	const float f_incre = 5.0f;
	vec4 default_lens;      //vector containing initial limb and base lengths for robot model

	//robot serial communication
	queue <vec4> gcode_queue;	//queue of vector positions to pass to robot model
	vector <string> gcode_string_vector;	//vector of strings of points in the queue

	SerialPort s1;
	SerialPort s2;
	bool port_opened;			//port1 status bool
	bool port_opened2;			//port2 status bool
	int gcode_queue_len;		//queue length
	bool send_gcode;			//serial communication flag for port 1
	bool send_gcode2;			//serial communication flag for port 2

	//gcode streaming testing tools
	const int S32_incre = 50.0f;
	const int Origin_incre = 1.0f;
	
	int x_temp, y_temp, z_temp;					//Motion capturefiltering tools for target 1
	int x_temp_old, y_temp_old, z_temp_old;

	int x_temp2, y_temp2, z_temp2;				//Motion capturefiltering tools for target 2
	int x_temp2_old, y_temp2_old, z_temp2_old;

	int x_origin, y_origin, z_origin;		//Initial position of robot

	int mv_speed;			//speed up tool

	char gcode_buff[32];	//gcode container before streaming for port 1
	char gcode_buff2[32];	//gcode container before streaming for port 2
	bool apply_mvspeed;		//speed testing
	int loops_since_send;   //loop tracker to space out between sends for port 1
	int loops_since_send2;   //loop tracker to space out between sends for port 2

	//Start of Kinect stuff
	//Flags and handlers
	HRESULT hr;
	HANDLE Skel_dready;

	//Sensor object
	INuiSensor* sensor;

	//for seated tracking mode
	bool seated_tracking;
	
	//Faux origin for setting which point is the displacement if referrenced to
	vec3 faux_origin;
	bool faux_origin_set;

	//Faux origin for second tracking target
	vec3 faux_origin2;
	bool faux_origin2_set;


	int tracking_target;		//int for choosing which target to track for displacement
	int tracking_target2;		//int for second tracking target

	bool apply_displacement;	//flag for applying displacement
	
	vec3 displacement;			//displacement vector calculated relative to origin
	vec3 displacement2;			//displacement vector for second tracking target

	float displacement_mult;	//scalar multiplier to change sensitivity of displacement vector
	



	//Skeleton data 
	NUI_SKELETON_FRAME c_skeletonFrame;
	NUI_SKELETON_DATA c_skeletonData;

	//------Custom functions------

	//initialize kinect sensors and skeletonFrame
	HRESULT init_kinect();

	//update current skeleton frame
	void update_skeletonFrame();

	//function to get coordinate of arbitrary joint
	void get_joint_coordinate();

	//function to set faux origin;
	void set_faux_origin();

	//function to set second faux origin
	void set_faux_origin2();

	//function to draw lines as bones in 3D space
	void draw3D_bone(NUI_SKELETON_DATA skeletonData,
		NUI_SKELETON_POSITION_INDEX joint1,
		NUI_SKELETON_POSITION_INDEX joint2);

	//function to draw skeleton tracked
	void draw_skeleton();



	void cleanup() override;
};

void KinectXRobotApp::setup()
{
	setWindowSize(1280, 480);

	//enable 3rd dimension stuff
	gl::enableDepthRead();
	gl::enableDepthWrite();


	//initialize ImGui panels
	ImGui::Initialize();

	//set camera parameter defaults
	set_cam_def();

	//initialize camera view points
	left_cam.setEyePoint(left_eye_point);
	left_cam.lookAt(left_look_at);

	right_cam.setEyePoint(right_eye_point);
	right_cam.lookAt(right_look_at);


	//initialize kinect tracking stuff
	seated_tracking = true;
	faux_origin_set = false;
	faux_origin2_set = false;

	tracking_target = 0;
	tracking_target2 = 0;
	apply_displacement = false;
	displacement_mult = 1.0f;

	//initialize robot model position to home point
	
	robot_home_point = vec3(420, 320, 0);		//x=320, y=320, z=0;
	robot_dest = vec4(robot_home_point, 0);		//home point , gamma = 0;

	//record initial robot limb lengths
	default_lens = r1.get_limb_lens();			//get initial limb lens

	r1.set_dest(robot_dest);					//move robot to home point


	//initialize serial port status
	gcode_queue_len = 10;						//store 10 points at a time
	port_opened = false;
	port_opened2 = false;

	send_gcode = false;
	send_gcode2 = false;
	s1 = SerialPort();						   //intantiate port object
	s2 = SerialPort();

	//initialize gcode buff for both ports
	memset(gcode_buff, 0, sizeof(gcode_buff));
	memset(gcode_buff2, 0, sizeof(gcode_buff2));

	//set initial position of actual robot
	x_origin = 0;
	y_origin = 320;
	z_origin = 320;

	//initialize filtering variables
	
	//--port1 filters
	x_temp = 0;
	y_temp = 0;
	z_temp = 0;

	x_temp_old = x_temp;
	y_temp_old = y_temp;
	z_temp_old = z_temp;

	//--port2 filters
	x_temp2 = 0;
	y_temp2 = 0;
	z_temp2 = 0;

	x_temp2_old = x_temp2;
	y_temp2_old = y_temp2;
	z_temp2_old = z_temp2;

	apply_mvspeed = false;
	mv_speed = 0;
	loops_since_send = 0;
	loops_since_send2 = 0;

	//initiate kinect device communication
	this->hr = init_kinect();
	if (FAILED(this->hr))
		quit();

	

}

void KinectXRobotApp::mouseDown( MouseEvent event )
{
}

void KinectXRobotApp::update()
{
	
	

	//Get robot model information
	vec4 angles = r1.get_angles();
	string alpha_str = "Alpha:   " + std::to_string(angles.x);
	string beta_str = "Beta:    " + std::to_string(angles.y);
	string theta_str = "Theta:   " + std::to_string(angles.z);
	string theta_0_str = "Theta_0: " + std::to_string(angles.w);
	

	//Update robot end effector destination or reset to home point
	ImGui::Begin("Robot model control panel");

	ImGui::Spacing();
	ImGui::Spacing();
	ImGui::Text("End Effector Coordinates (mm)");

	//Get max coordinates from current robot model
	float u_bound = r1.get_tot_len();

	//Get XYZ coordinates seperately
	ImGui::DragFloat("X", &robot_dest.x, 1.0f, 0.0f, u_bound);
	ImGui::DragFloat("Y", &robot_dest.y, 1.0f, 0.0f, u_bound);
	ImGui::DragFloat("Z", &robot_dest.z, 1.0f, 0.0f, u_bound);
	//Get desired tool angle
	ImGui::DragFloat("tool angle", &robot_dest.w, 1.0f, -90.0f,90.0f);
	//Button to reset end effector to home point
	if (ImGui::Button("Reset to Home")) robot_dest = vec4(robot_home_point, 0);	



	//Change viewing angles
	if (ImGui::TreeNode("change viewing angles")) {
		if (ImGui::Button("Top view")) right_eye_point = vec3(300.0f, 1500.0f, 0.0f);
		if (ImGui::Button("Side view")) right_eye_point = vec3(100.0f, 100.0f, 1500.0f);
		if (ImGui::Button("Reset view")) set_cam_def();
		ImGui::TreePop();
	}
	
	//Show kinematics calculations results
	if (ImGui::TreeNode("show joint angles")) {
		ImGui::Text(alpha_str.c_str());
		ImGui::Text(beta_str.c_str());
		ImGui::Text(theta_str.c_str());
		ImGui::Text(theta_0_str.c_str());
		ImGui::TreePop();
	}

	//Update robot model limb lengths
	
	if (ImGui::TreeNode("change limb lengths")) {
		
		//get current lengths
		vec4 buff_lens = r1.get_limb_lens();
		static float l1_len = buff_lens.x;
		static float l2_len = buff_lens.y;
		static float l3_len = buff_lens.z;
		static float bs_len = buff_lens.w;
		
		//pass to inputs
		ImGui::InputScalar("L1", ImGuiDataType_Float, &l1_len, &f_incre);
		ImGui::InputScalar("L2", ImGuiDataType_Float, &l2_len, &f_incre);
		ImGui::InputScalar("L3", ImGuiDataType_Float, &l3_len, &f_incre);
		ImGui::InputScalar("Base", ImGuiDataType_Float, &bs_len, &f_incre);
		
		//update robot model
		r1.set_limb_lens(vec4(l1_len, l2_len, l3_len, bs_len));

		//reset lengths to initial length
		if (ImGui::Button("Reset limb len")) {
			r1.set_limb_lens(default_lens);
			l1_len = default_lens.x;
			l2_len = default_lens.y;
			l3_len = default_lens.z;
			bs_len = default_lens.w;
		}
		ImGui::TreePop();
	}
	

	ImGui::Separator();
	ImGui::Spacing();
	ImGui::Spacing();
	ImGui::Text("Robot-Kinect control");
	ImGui::DragFloat("sensitivity", &displacement_mult,0.001f, 1.0f,2.0f);
	ImGui::Checkbox("Apply displacement vector", &apply_displacement);
	
	ImGui::Separator();
	ImGui::Spacing();
	ImGui::Spacing();
	ImGui::Text("Communications");
	
	//---------------------------ROBOT ARM CONNECTED TO COM PORT 1----------------------------------
	//Opening and closing of Serial port1 communication
	
	if (!port_opened) {								//Port not yet opened
		
		static char buff[32] = "";
		ImGui::InputText("port", buff,32, ImGuiInputTextFlags_CharsUppercase | ImGuiInputTextFlags_CharsNoBlank);
		
		//Attempt to open port
		if (ImGui::Button("Open port\n")) {
			
			if (s1.open(buff) == -1) ImGui::OpenPopup("Error COM port");	//throw error and prepare error modal window
			else {
				port_opened = true;										//else raise flag to indicate port is opened
				memset(buff, 0, sizeof(buff));							//clear textbox after opening
			}
		}
		//if failed to open port show error modal window
		if (ImGui::BeginPopupModal("Error COM port", NULL, 0)) {
			ImGui::Text("Could not open port. Please try again");
			if (ImGui::Button("close"))
				ImGui::CloseCurrentPopup();
			ImGui::EndPopup();
		}
	}else {

		
		
		
		if (send_gcode == true) {
			ImGui::Text("STREAMING GCODE..");

			//Modify initial position of actual robot and other commands
			if (ImGui::TreeNode("Additional Options")) {

				ImGui::InputScalar("Initial X pos.", ImGuiDataType_S32, &x_origin, &Origin_incre);
				ImGui::InputScalar("Initial Y pos.", ImGuiDataType_S32, &y_origin, &Origin_incre);
				ImGui::InputScalar("Initial Z pos.", ImGuiDataType_S32, &z_origin, &Origin_incre);

				//enable and disable movement speed command in gcode
				ImGui::Checkbox("Apply mv speed", &apply_mvspeed);
				if (apply_mvspeed) ImGui::InputScalar("movement speed", ImGuiDataType_S32, &mv_speed, &S32_incre);

				ImGui::TreePop();
			}
			
			//Int to track how many coordinate values have changed
			int coordinates_changed = 0;
			int displacement_filter = 40;
			int sample_threshold = 2;
			
			//Only record displacement if:
			//----displacement is less than DISPLACEMENT_FILTER units long
			//----new displacement has a delta of atleast SAMPLE_THRESHOLD from the previous displacement value
	
			if ((displacement.x < displacement_filter && displacement.x > -displacement_filter) &&
				((displacement.x >= x_temp_old + sample_threshold)  || (displacement.x <= x_temp_old - sample_threshold))) {
				x_temp_old = displacement_mult * displacement.x;
				coordinates_changed++;
			}

			if ((displacement.y < displacement_filter && displacement.y > -displacement_filter) && 
				((displacement.y >= y_temp_old + sample_threshold) || (displacement.y <= y_temp_old - sample_threshold))) {
				y_temp_old = displacement_mult * displacement.y;
				coordinates_changed++;
			}

			if ((displacement.z < displacement_filter && displacement.z > -displacement_filter) &&
				( (displacement.z >= z_temp_old + sample_threshold) || (displacement.z <= z_temp_old - sample_threshold) )) {
				z_temp_old = displacement_mult * displacement.z;
				coordinates_changed++;
			}


			//Adding displacement to current home position (multiplying displacement with a constant)
			int x_dest = (x_temp_old * 4) + x_origin;
			int y_dest = (y_temp_old * 2) + y_origin;
			int z_dest = (z_temp_old  * 2)+ z_origin;

			//--------Coordinate edge cases-----
			//minimum and maximum X coordinates
			if (x_dest > 300) x_dest = 300;
			if (x_dest < -300) x_dest = -300;
			//minimum and maximum Y coordinates
			if (y_dest < 250) y_dest = 320;
			if (y_dest > 500) y_dest = 500;
			//minimum and maximum Z coordinates
			if (z_dest < 100) z_dest = 100;
			if (z_dest > 320) z_dest = 320;

			//Apply base speed
			if (apply_mvspeed) sprintf(gcode_buff, "G1X%dY%dZ%dF%d", x_dest, y_dest, z_dest, mv_speed);
			else  sprintf(gcode_buff, "G1X%dY%dZ%d", x_dest, y_dest, z_dest);

			//send to robot over serial port if more than 1 coordinate has been updated
			//after sending set a timer
			if (coordinates_changed >= 1 && loops_since_send == 0) {
				s1.write(gcode_buff); 
				loops_since_send = 10;
			}
			
			//decriment timer till next viable gcode send
			if (loops_since_send > 0) loops_since_send -= 1;
			else loops_since_send = 0;

			
			
			ImGui::Text(gcode_buff);
			memset(gcode_buff, 0, sizeof(gcode_buff));

			if (ImGui::Button("Stop gcode stream")) send_gcode = false;
		}
		else {

			//If not streaming gcode , allow user to send discreet gcode commands
			//Note:
			//Each serial message will be sent after appending both NL and CR character at the end of the gcode command string
			static char writebuff[32] = "";
			ImGui::InputText("gcode command", writebuff, 32, ImGuiInputTextFlags_CharsUppercase | ImGuiInputTextFlags_CharsNoBlank );
			ImGui::SameLine();
			
			if (ImGui::Button("send")) {
				s1.write(writebuff);
				memset(writebuff, 0, sizeof(writebuff));
			}

			//Some discreet robot commands in a hidable tree
			if (ImGui::TreeNode("GCODE commands")) {
				if (ImGui::Button("Enable Steppers")) s1.write("M17");
				ImGui::SameLine();
				if (ImGui::Button("Disable Steppers")) s1.write("M18");
				ImGui::TreePop();
			}


			if (ImGui::Button("Start gcode stream")) send_gcode = true;

			if (ImGui::Button("Close port connection")) { //If port already opened give show button to close
				s1.close();								//close port
				port_opened = false;					//reset flag to allow reconnection
			}

		}


		
	}
	//---------------------------END OF ROBOT ARM CONNECTED TO COM PORT 1-----------------------------
	
	//---------------------------ROBOT ARM CONNECTED TO COM PORT 2----------------------------------
	//Opening and closing of Serial port2 communication
	//NOTE: Some features available with port1 will not be available with port 2

	if (!port_opened2) {								//Port not yet opened

		static char buff2[32] = "";
		ImGui::InputText("port2", buff2, 32, ImGuiInputTextFlags_CharsUppercase | ImGuiInputTextFlags_CharsNoBlank);

		//Attempt to open port
		if (ImGui::Button("Open port2\n")) {

			if (s2.open(buff2) == -1) ImGui::OpenPopup("Error COM port2");	//throw error and prepare error modal window
			else {
				port_opened2 = true;										//else raise flag to indicate port is opened
				memset(buff2, 0, sizeof(buff2));							//clear textbox after opening
			}
		}
		//if failed to open port show error modal window
		if (ImGui::BeginPopupModal("Error COM port2", NULL, 0)) {
			ImGui::Text("Could not open port2. Please try again");
			if (ImGui::Button("close port2"))
				ImGui::CloseCurrentPopup();
			ImGui::EndPopup();
		}
	}
	else {




		if (send_gcode2 == true) {
			ImGui::Text("STREAMING GCODE..");


			//NOTES: Removed option to change origin coordinates
			/*
			//Modify initial position of actual robot and other commands
			if (ImGui::TreeNode("Additional Options")) {

				ImGui::InputScalar("Initial X pos.", ImGuiDataType_S32, &x_origin, &Origin_incre);
				ImGui::InputScalar("Initial Y pos.", ImGuiDataType_S32, &y_origin, &Origin_incre);
				ImGui::InputScalar("Initial Z pos.", ImGuiDataType_S32, &z_origin, &Origin_incre);

				//enable and disable movement speed command in gcode
				ImGui::Checkbox("Apply mv speed", &apply_mvspeed);
				if (apply_mvspeed) ImGui::InputScalar("movement speed", ImGuiDataType_S32, &mv_speed, &S32_incre);

				ImGui::TreePop();
			}
			*/

			//Int to track how many coordinate values have changed
			int coordinates_changed2 = 0;
			int displacement_filter2 = 40;
			int sample_threshold2 = 2;
			
		
			//Only record displacement if:
			//----displacement is less than DISPLACEMENT_FILTER units long
			//----new displacement has a delta of atleast SAMPLE_THRESHOLD from the previous displacement value

			if ((displacement2.x < displacement_filter2 && displacement2.x > -displacement_filter2) &&
				((displacement2.x >= x_temp2_old + sample_threshold2) || (displacement2.x <= x_temp2_old - sample_threshold2))) {
				x_temp2_old = displacement_mult * displacement2.x;
				coordinates_changed2++;
			}

			if ((displacement2.y < displacement_filter2 && displacement2.y > -displacement_filter2) &&
				((displacement2.y >= y_temp2_old + sample_threshold2) || (displacement2.y <= y_temp_old - sample_threshold2))) {
				y_temp2_old = displacement_mult * displacement2.y;
				coordinates_changed2++;
			}

			if ((displacement2.z < displacement_filter2 && displacement2.z > -displacement_filter2) &&
				((displacement2.z >= z_temp2_old + sample_threshold2) || (displacement2.z <= z_temp2_old - sample_threshold2))) {
				z_temp2_old = displacement_mult * displacement2.z;
				coordinates_changed2++;
			}

			//NOTES: added "2" to all previously used variables for port 1 so that port 2 has independent 
			// variables

			//Adding displacement to current home position (multiplying displacement with a constant)
			int x_dest2 = (x_temp2_old * 4) + x_origin;
			int y_dest2 = (y_temp2_old * 2) + y_origin;
			int z_dest2 = (z_temp2_old * 2) + z_origin;

			//--------Coordinate edge cases-----
			//minimum and maximum X coordinates
			if (x_dest2 > 300) x_dest2 = 300;
			if (x_dest2 < -300) x_dest2 = -300;
			//minimum and maximum Y coordinates
			if (y_dest2 < 250) y_dest2 = 320;
			if (y_dest2 > 500) y_dest2 = 500;
			//minimum and maximum Z coordinates
			if (z_dest2 < 100) z_dest2 = 100;
			if (z_dest2 > 320) z_dest2 = 320;

			//Apply base speed
			if (apply_mvspeed) sprintf(gcode_buff2, "G1X%dY%dZ%dF%d", x_dest2, y_dest2, z_dest2, mv_speed);
			else  sprintf(gcode_buff2, "G1X%dY%dZ%d", x_dest2, y_dest2, z_dest2);


			//NOTE: independent loop tracker for port 2

			//send to robot over serial port if more than 1 coordinate has been updated
			//after sending set a timer
			if (coordinates_changed2 >= 1 && loops_since_send2 == 0) {
				s2.write(gcode_buff);
				loops_since_send2 = 10;
			}

			//decriment timer till next viable gcode send
			if (loops_since_send2 > 0) loops_since_send2 -= 1;
			else loops_since_send2 = 0;

		

			ImGui::Text(gcode_buff2);
			memset(gcode_buff2, 0, sizeof(gcode_buff2));

			if (ImGui::Button("Stop gcode stream")) send_gcode2 = false;
		}
		else {

			//If not streaming gcode , allow user to send discreet gcode commands
			//Note:
			//Each serial message will be sent after appending both NL and CR character at the end of the gcode command string
			static char writebuff2[32] = "";
			ImGui::InputText("gcode command", writebuff2, 32, ImGuiInputTextFlags_CharsUppercase | ImGuiInputTextFlags_CharsNoBlank);
			ImGui::SameLine();

			if (ImGui::Button("send")) {
				s2.write(writebuff2);
				memset(writebuff2, 0, sizeof(writebuff2));
			}

			//Some discreet robot commands in a hidable tree
			if (ImGui::TreeNode("GCODE commands for port 2")) {
				if (ImGui::Button("Enable Steppers")) s2.write("M17");
				ImGui::SameLine();
				if (ImGui::Button("Disable Steppers")) s2.write("M18");
				ImGui::TreePop();
			}


			if (ImGui::Button("Start gcode stream")) send_gcode2 = true;

			if (ImGui::Button("Close port2 connection")) { //If port already opened give show button to close
				s2.close();								//close port
				port_opened2 = false;					//reset flag to allow reconnection
			}

		}



	}

	
	//---------------------------END OF ROBOT ARM CONNECTED TO COM PORT 2-----------------------------


	

	ImGui::End();

	//----------------------------START OF DISPLACEMENT PROCESSING / GCODE HANDLING---------------------------------------

	//Calculating target displacement to apply to robot model
	vec4 robot_displacement = vec4(displacement_mult) * vec4(robot_home_point + displacement, 0);
	

	
	//--------------------------------END OF DISPLACEMENT PROCESSING / GCODE HANDLING---------------------------------------

	//if apply displacement flag is set then update robot using displacement vector
	if (apply_displacement) {
		robot_dest = robot_displacement;
	}
	//update robot arm destination
	r1.set_dest(robot_dest);

	//Kinect stuff
	if (seated_tracking)
		sensor->NuiSkeletonTrackingEnable(Skel_dready, NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT);
	else
		sensor->NuiSkeletonTrackingEnable(Skel_dready, 0);

	//wait for available data and update skeletonFrame object
	if (WAIT_OBJECT_0 == WaitForSingleObject(Skel_dready, 0)) {
		update_skeletonFrame();
		get_joint_coordinate();
	}

	//Vector for tracking options
	std::vector<std::string> tracking_targets;
	tracking_targets.push_back("Head");
	tracking_targets.push_back("Left hand");
	tracking_targets.push_back("Right hand");
	tracking_targets.push_back("Left foot");
	tracking_targets.push_back("Right foot");

	//Displacement vector 1 info
	string displacement_str = "Displacement: (X" + std::to_string(displacement.x) + " ,Y " + std::to_string(displacement.y) + " ,Z "
		+ std::to_string(displacement.z) + " )";

	//Displacement vector 2 info
	string displacement2_str = "Displacement 2: (X" + std::to_string(displacement2.x) + " ,Y " + std::to_string(displacement2.y) + " ,Z "
		+ std::to_string(displacement2.z) + " )";

	//Faux origin 1 vector info
	string faux_origin_string = "Faux origin ( " + std::to_string(faux_origin.x) + ", "
		+ std::to_string(faux_origin.y) + ", " + std::to_string(faux_origin.z) + ")";

	//Faux origin 2 vector info
	string faux_origin2_string = "Faux origin 2 ( " + std::to_string(faux_origin2.x) + ", "
		+ std::to_string(faux_origin2.y) + ", " + std::to_string(faux_origin2.z) + ")";

	//Kinect tracking control window
	ImGui::Begin("Kinect output");
	ImGui::Checkbox("Seated Tracking", &seated_tracking);

	//----------TRACKING TARGET FOR PORT 1---------------
	ImGui::Combo("Tracking target", &tracking_target, tracking_targets);
	

	ImGui::Text(faux_origin_string.c_str());
	if (ImGui::Button("Set new faux origin")) set_faux_origin();

	ImGui::Text(displacement_str.c_str());
	

	ImGui::Spacing();
	ImGui::Separator();


	//----------TRACKING TARGET FOR PORT 2---------------
	ImGui::Combo("Tracking target", &tracking_target2, tracking_targets);

	ImGui::Text(faux_origin2_string.c_str());
	if (ImGui::Button("Set new faux origin2")) set_faux_origin2();

	ImGui::Text(displacement2_str.c_str());
	

	ImGui::Spacing();
	ImGui::Separator();




	//Camera controls for the skeleton renderer window
	if (ImGui::TreeNode("Renderer camera controls")) {
		ImGui::Spacing();
		ImGui::Spacing();
		ImGui::Text("Kinect Renderer");
		ImGui::DragFloat3("Left eye point", &left_eye_point, 0.01f);
		ImGui::DragFloat3("Left look at", &left_look_at, 0.01f);
		ImGui::DragFloat("Left farplane", &left_fp, 0.01f);
		ImGui::DragFloat3("Right eye point", &right_eye_point, 1.0f);
		ImGui::DragFloat3("Right look at", &right_look_at, 1.0f);
		ImGui::DragFloat("Right far plane", &right_fp, 1.0f);
		ImGui::TreePop();
	}
	
	ImGui::End();


	

	//update camera parameters
	left_cam.setEyePoint(left_eye_point);
	left_cam.lookAt(left_look_at);

	right_cam.setEyePoint(right_eye_point);
	right_cam.lookAt(right_look_at);
	right_cam.setFarClip(right_fp);
	

	//ImGui::ShowDemoWindow();
	
}

void KinectXRobotApp::draw()
{
	//right side of the screen
	gl::clear(Color(0.2f, 0.2f, 0.2f));

	gl::viewport(getWindowWidth() / 2, 0, getWindowWidth() / 2, getWindowHeight());
	gl::color(Color(1, 1, 1));
	gl::setMatrices(right_cam);

	//draw robot
	r1.draw();
	


	//left side of screen
	gl::color(Color(1, 1, 1));
	gl::viewport(0, 0, getWindowWidth() / 2, getWindowHeight());

	gl::setMatrices(left_cam);

	//set a background for the skeleton using camera farclip coordinates
	vec3 wtopleft;
	vec3 wtopright;
	vec3 wbotleft;
	vec3 wbotright;

	left_cam.getFarClipCoordinates(&wtopleft, &wtopright, &wbotleft, &wbotright);
	auto left_cam_farplane = gl::VertBatch(GL_TRIANGLE_STRIP);
	left_cam_farplane.color(ColorA(0.0f, 0.0f, 0.0f, 0.7f));
	left_cam_farplane.vertex(wtopleft);
	left_cam_farplane.vertex(wbotleft);
	left_cam_farplane.vertex(wtopright);
	left_cam_farplane.vertex(wbotright);
	left_cam_farplane.draw();

	//draw skeleton 
	draw_skeleton();
}

void KinectXRobotApp::set_cam_def(){
	left_eye_point = vec3(0.0f, 0.0f, -300.0f);
	left_look_at = vec3(0.0f);
	left_fp = left_cam.getFarClip();

	right_eye_point = vec3(900.0f, 800.0f, 1500.0f);
	right_look_at = vec3(50.0f, 100.0f, 0.0f);
	right_fp = 2000.0f;
}


//function to initialize sensor object and skeletonFrame variable
HRESULT KinectXRobotApp::init_kinect() {

	int sensor_cnt = 0;		//for number of kinects

	//Checking for sensor number
	if (NuiGetSensorCount(&sensor_cnt) < 0 || sensor_cnt < 1) {
		OutputDebugStringA("No Kinects found\n");

		//Make app quit
		return E_FAIL;
	}


	//Connect to kinect found (only first one)
	HRESULT hr = NuiCreateSensorByIndex(0, &sensor);

	if (FAILED(hr)) {
		OutputDebugStringA("Failed to connect to kinect\n");

		//Return Fail
		return E_FAIL;
	}
	else
		OutputDebugStringA("Kinect object created\n");


	//Initialize kinect for skeleton tracking
	sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_SKELETON);

	Skel_dready = CreateEvent(NULL, TRUE, FALSE, NULL);	//Check what this means after besides it creates and event handler

	if (seated_tracking) {
		sensor->NuiSkeletonTrackingEnable(Skel_dready, NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT);	//enable seated tracking
	}
	else
		sensor->NuiSkeletonTrackingEnable(Skel_dready, 0);		//full body tracking only

	//initialize skeleton data
	c_skeletonFrame = { 0 };

	return S_OK;
}


//function to get new skeleton frame every after draw loop
void KinectXRobotApp::update_skeletonFrame()
{

	//get skeleton fram data
	HRESULT hr = sensor->NuiSkeletonGetNextFrame(0, &c_skeletonFrame);
	if (FAILED(hr)) {
		OutputDebugStringA("Couldn't get next frame\n");
		return;
	}

	//smoothen out skeletonframe data
	sensor->NuiTransformSmooth(&c_skeletonFrame, 0);

}


//actually get the skeleton data
void KinectXRobotApp::get_joint_coordinate() {

	//Idea: cycle through all skeleton data, might help stabalize tracking
	//Transfer to proper variable
	NUI_SKELETON_DATA skeletonData = c_skeletonFrame.SkeletonData[0];

	for (int i = 0; skeletonData.eTrackingState == NUI_SKELETON_NOT_TRACKED; i++) {
		if (i >= 6) {
			OutputDebugStringA("No skeleton tracked \n");
			return;
		}
		else
			skeletonData = c_skeletonFrame.SkeletonData[i];

	}

	if (skeletonData.eTrackingState != NUI_SKELETON_NOT_TRACKED) {

		//assume that head is tracked so we get the data
		Vector4 head_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HEAD];
		Vector4 left_hand_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT];
		Vector4 right_hand_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT];

		OutputDebugStringA("skeleton found!\n"); //MOther FuCkEr I Found it!




		return;
	}


}

void KinectXRobotApp::set_faux_origin()
{
	//Idea: cycle through all skeleton data, might help stabalize tracking
	//Transfer to proper variable
	NUI_SKELETON_DATA skeletonData = c_skeletonFrame.SkeletonData[0];

	for (int i = 0; skeletonData.eTrackingState == NUI_SKELETON_NOT_TRACKED; i++) {
		if (i >= 6) return;
		else skeletonData = c_skeletonFrame.SkeletonData[i];
	}

	if (skeletonData.eTrackingState != NUI_SKELETON_NOT_TRACKED) {

		//assume that head is tracked so we get the data
		Vector4 tracked_pos = Vector4();
		
		Vector4 head_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HEAD];
		Vector4 left_hand_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT];
		Vector4 right_hand_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT];
		Vector4 left_foot_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_FOOT_LEFT];
		Vector4 right_foot_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_FOOT_RIGHT];

		//First target for tracking
		switch (tracking_target) {
			case 0:
				tracked_pos = head_pos;
				break;
			case 1:
				tracked_pos = left_hand_pos;
				break;
			case 2:
				tracked_pos = right_hand_pos;
				break;
			case 3:
				tracked_pos = left_foot_pos;
				break;
			case 4:
				tracked_pos = right_foot_pos;
				break;
			default:
				tracked_pos = left_hand_pos;
		}

		

		//set faux origin temporarily via hardcoding and scale to cm from raw m values
		float scalar = 100.0f;
		faux_origin = vec3(tracked_pos.x * scalar, tracked_pos.y * scalar, tracked_pos.z * scalar);

		if (!faux_origin_set) faux_origin_set = true;

		return;
	}
}



void KinectXRobotApp::set_faux_origin2()
{
	//Idea: cycle through all skeleton data, might help stabalize tracking
	//Transfer to proper variable
	NUI_SKELETON_DATA skeletonData = c_skeletonFrame.SkeletonData[0];

	for (int i = 0; skeletonData.eTrackingState == NUI_SKELETON_NOT_TRACKED; i++) {
		if (i >= 6) return;
		else skeletonData = c_skeletonFrame.SkeletonData[i];
	}

	if (skeletonData.eTrackingState != NUI_SKELETON_NOT_TRACKED) {

		//assume that head is tracked so we get the data
		Vector4 tracked_pos2 = Vector4();

		Vector4 head_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HEAD];
		Vector4 left_hand_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT];
		Vector4 right_hand_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT];
		Vector4 left_foot_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_FOOT_LEFT];
		Vector4 right_foot_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_FOOT_RIGHT];

		//First target for tracking
		switch (tracking_target2) {
		case 0:
			tracked_pos2 = head_pos;
			break;
		case 1:
			tracked_pos2 = left_hand_pos;
			break;
		case 2:
			tracked_pos2 = right_hand_pos;
			break;
		case 3:
			tracked_pos2 = left_foot_pos;
			break;
		case 4:
			tracked_pos2 = right_foot_pos;
			break;
		default:
			tracked_pos2 = left_hand_pos;
		}



		//set faux origin temporarily via hardcoding and scale to cm from raw m values
		float scalar = 100.0f;
		faux_origin2 = vec3(tracked_pos2.x * scalar, tracked_pos2.y * scalar, tracked_pos2.z * scalar);

		if (!faux_origin2_set) faux_origin2_set = true;

		return;
	}
}




//drawing some bones here

void KinectXRobotApp::draw3D_bone(NUI_SKELETON_DATA skeletonData, NUI_SKELETON_POSITION_INDEX joint1, NUI_SKELETON_POSITION_INDEX joint2) {
	NUI_SKELETON_POSITION_TRACKING_STATE q_joint1 = skeletonData.eSkeletonPositionTrackingState[joint1];
	NUI_SKELETON_POSITION_TRACKING_STATE q_joint2 = skeletonData.eSkeletonPositionTrackingState[joint2];

	vec3 joint1_pos = vec3(skeletonData.SkeletonPositions[joint1].x, skeletonData.SkeletonPositions[joint1].y, skeletonData.SkeletonPositions[joint1].z);
	vec3 joint2_pos = vec3(skeletonData.SkeletonPositions[joint2].x, skeletonData.SkeletonPositions[joint2].y, skeletonData.SkeletonPositions[joint2].z);


	//adjust scale from m to cm
	int scalar = 100;
	joint1_pos = vec3(joint1_pos.x * scalar, joint1_pos.y * scalar, joint1_pos.z * scalar);
	joint2_pos = vec3(joint2_pos.x * scalar, joint2_pos.y * scalar, joint2_pos.z * scalar);



	if (q_joint1 == NUI_SKELETON_POSITION_INFERRED || q_joint2 == NUI_SKELETON_POSITION_INFERRED) {
		gl::lineWidth(1);
		gl::drawLine(joint1_pos, joint2_pos);
	}
	else if (q_joint1 == NUI_SKELETON_POSITION_TRACKED && q_joint2 == NUI_SKELETON_POSITION_TRACKED) {
		gl::lineWidth(10);
		gl::drawLine(joint1_pos, joint2_pos);
		//reset line width
		gl::lineWidth(1);
	}



	return;
}

void KinectXRobotApp::draw_skeleton() {


	//Idea: cycle through all skeleton data, might help stabalize tracking
	//Transfer to proper variable
	NUI_SKELETON_DATA skeletonData = c_skeletonFrame.SkeletonData[0];

	for (int i = 0; skeletonData.eTrackingState == NUI_SKELETON_NOT_TRACKED; i++) {
		if (i >= 6) {
			OutputDebugStringA("No skeleton tracked \n");
			return;
		}
		else
			skeletonData = c_skeletonFrame.SkeletonData[i];

	}

	//Extracting skeleton data to use for drawing displacement vectors
	Vector4 head_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HEAD];
	Vector4 left_hand_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT];
	Vector4 right_hand_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT];
	Vector4 left_foot_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_FOOT_LEFT];
	Vector4 right_foot_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_FOOT_RIGHT];




	//--Drawing displacement vector for first tracking target
	//draw vector from faux_origin to tracked target (temporarily left hand) if faux origin has been set
	if (faux_origin_set) {
		//get trackig target

		Vector4 tracked_pos = Vector4();


		switch (tracking_target) {
		case 0:
			tracked_pos = head_pos;
			break;
		case 1:
			tracked_pos = left_hand_pos;
			break;
		case 2:
			tracked_pos = right_hand_pos;
			break;
		case 3:
			tracked_pos = left_foot_pos;
			break;
		case 4:
			tracked_pos = right_foot_pos;
			break;
		default:
			tracked_pos = left_hand_pos;
		}

		float scalar = 100.0f;	//convert from m to cm
		vec3 target_pos = vec3(tracked_pos.x * scalar, tracked_pos.y * scalar , tracked_pos.z * scalar);
		

		//assume faux_origin has been set and scaled properly
		gl::color(Color(1, 0, 0));
		gl::lineWidth(8);
		gl::drawLine(faux_origin, target_pos);
		gl::color(Color(1, 1, 1));

		//calculate and record displacement to be applied to robot
		if (apply_displacement) {
			displacement = target_pos - faux_origin;
		}
		
	}

	//--Drawing displacement vector for second tracking target
	//draw vector from faux_origin to tracked target (temporarily left hand) if faux origin has been set
	if (faux_origin2_set) {
		//get trackig target

		Vector4 tracked_pos2 = Vector4();

		switch (tracking_target2) {
		case 0:
			tracked_pos2 = head_pos;
			break;
		case 1:
			tracked_pos2 = left_hand_pos;
			break;
		case 2:
			tracked_pos2 = right_hand_pos;
			break;
		case 3:
			tracked_pos2 = left_foot_pos;
			break;
		case 4:
			tracked_pos2 = right_foot_pos;
			break;
		default:
			tracked_pos2 = left_hand_pos;
		}

		float scalar = 100.0f;	//convert from m to cm
		vec3 target_pos2 = vec3(tracked_pos2.x * scalar, tracked_pos2.y * scalar, tracked_pos2.z * scalar);


		//assume faux_origin2 has been set and scaled properly
		gl::color(Color(0, 1, 0));
		gl::lineWidth(8);
		gl::drawLine(faux_origin2, target_pos2);
		gl::color(Color(1, 1, 1));

		//calculate and record displacement to be applied to robot
		if (apply_displacement) {
			displacement2 = target_pos2 - faux_origin2;
		}

	}

	//PROCEED TO DRAW THE TRACKED SKELETON

	//3D render mode
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SHOULDER_CENTER);
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT);
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT);
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SPINE);
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_HIP_CENTER);
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_LEFT);
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_RIGHT);

	// Left Arm
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT);
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT);
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT);

	// Right Arm
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT);
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT);
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT);

	// Left Leg
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_HIP_LEFT, NUI_SKELETON_POSITION_KNEE_LEFT);
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT);
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT);

	// Right Leg
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_HIP_RIGHT, NUI_SKELETON_POSITION_KNEE_RIGHT);
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT);
	draw3D_bone(skeletonData, NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT);

}


//handle resources
void KinectXRobotApp::cleanup() {
	//Clean up
	if (!FAILED(this->hr)) {
		sensor->Release();			//Release sensor (might cause some detection errors if not)
		CloseHandle(Skel_dready);	//Close handle
		OutputDebugStringA("Kinect cleaned up\n");
	}

	//Close port1 if opened
	if (port_opened == true)
		s1.close();

	//Close port2 if opened
	if (port_opened2 == true)
		s2.close();
}


CINDER_APP( KinectXRobotApp, RendererGl )
