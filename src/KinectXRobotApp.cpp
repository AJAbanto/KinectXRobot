#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/CinderImGui.h"

#include <Windows.h>
#include <iostream>
#include <string>
#include <queue>

#include "NuiApi.h"
#include "robot_manipulator.h"
#include "serial.h"

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
	vec4 robot_dest;	//vector to be passed to robot model  
	vec3 robot_home_point; //home position or initial position of model 

	//robot serial communication
	queue <vec4> gcode_queue;	//queue of vector positions to pass to robot model
	vector <string> gcode_string_vector;	//vector of strings of points in the queue

	SerialPort s1;
	bool port_opened;			//port status bool
	int gcode_queue_len;		//queue length
	bool send_gcode;			//serial communication flag

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

	int tracking_target;	//int for choosing which target to track for displacement
	bool apply_displacement; //flag for applying displacement
	vec3 displacement;		//displacement vector calculated relative to origin




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
	tracking_target = 0;
	apply_displacement = false;

	//initialize robot model position to home point
	
	robot_home_point = vec3(250, 250, 0);		//x=250, y=250, z=0;
	robot_dest = vec4(robot_home_point, 0);		//home point , gamma = 0;
	r1.set_dest(robot_dest);					//move robot to home point


	//initialize serial port status
	gcode_queue_len = 10;						//store 10 points at a time
	port_opened = false;
	s1 = SerialPort();						   //intantiate port object

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
	//Modify camera stuff
	ImGui::Begin("Camera controls");
	ImGui::DragFloat3("Left eye point", &left_eye_point, 0.01f);
	ImGui::DragFloat3("Left look at", &left_look_at, 0.01f);
	ImGui::DragFloat("Left farplane", &left_fp, 0.01f);
	ImGui::DragFloat3("Right eye point", &right_eye_point, 1.0f);
	ImGui::DragFloat3("Right look at", &right_look_at, 1.0f);
	ImGui::DragFloat("Right far plane", &right_fp, 1.0f);
	ImGui::Separator();
	ImGui::Checkbox("Seated Tracking", &seated_tracking);
	
	if(ImGui::Button("View orthogonal z=0")) right_eye_point = vec3(300.0f, 1500.0f, 0.0f);
	if (ImGui::Button("View orthogonal y=0")) right_eye_point = vec3(100.0f, 100.0f, 1500.0f);
	if (ImGui::Button("Reset Defaults")) set_cam_def();
	ImGui::End();

	//update camera parameters
	left_cam.setEyePoint(left_eye_point);
	left_cam.lookAt(left_look_at);

	right_cam.setEyePoint(right_eye_point);
	right_cam.lookAt(right_look_at);
	right_cam.setFarClip(right_fp);
	

	//Get robot model information
	vec4 angles = r1.get_angles();
	string alpha_str = "Alpha:   " + std::to_string(angles.x);
	string beta_str = "Beta:    " + std::to_string(angles.y);
	string theta_str = "Theta:   " + std::to_string(angles.z);
	string theta_0_str = "Theta_0: " + std::to_string(angles.w);
	

	//Update robot end effector destination and reset to home point
	ImGui::Begin("Robot model control panel");
	ImGui::DragFloat4("End effector pos: ", &robot_dest, 1.0f, 0.0f, 400.0f);
	ImGui::Text(alpha_str.c_str());
	ImGui::Text(beta_str.c_str());
	ImGui::Text(theta_str.c_str());
	ImGui::Text(theta_0_str.c_str());
	if (ImGui::Button("Home robotmodel")) robot_dest = vec4(robot_home_point,0);	//reset to home point
	ImGui::Checkbox("Apply displacement vector", &apply_displacement);
	

	
	//Starting communication Port
	
	if (!port_opened) {								//Port not yet opened
		
		static char buff[32] = "";
		ImGui::InputText("port", buff,32, ImGuiInputTextFlags_CharsUppercase | ImGuiInputTextFlags_CharsNoBlank);
		
		//Attempt to open port
		if (ImGui::Button("Open port\n")) {
			
			if (s1.open(buff) == -1) ImGui::OpenPopup("Error COM port");	//throw error and prepare error modal window
			else {
				port_opened = true;										//else raise flag to indicate port is opened
				s1.write("Starting Transmission;");
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
		if(ImGui::Button("Close port connection")) { //If port already opened give show button to close
			s1.close();								//close port
			port_opened = false;					//reset flag to allow reconnection
		}
	}

	int dummy = 0;	//dummy vairable
	ImGui::ListBox("Tracking queue", &dummy, gcode_string_vector, gcode_queue_len);	//view gcode queue
	ImGui::Checkbox("Send gcode", &send_gcode);	//flag to start serial communication

	

	ImGui::End();



	//robot displacement to apply to robot model
	vec4 robot_displacement = vec4(robot_home_point + displacement, 0);
	gcode_queue.push(robot_displacement);	//queue the displacement
	
	//convert coordinate to string
	string robot_displacement_str = "X" + to_string(robot_displacement.x) + " Y" + to_string(robot_displacement.y) + " Z" + to_string(robot_displacement.z);
	gcode_string_vector.push_back(robot_displacement_str);

	//if apply displacement flag is set then update robot using displacement vector
	//also check the queue if it's has more than 3 otherwise points wait
	if (apply_displacement && gcode_queue.size() > 3) {
		
		robot_dest = gcode_queue.front() ;
		gcode_queue.pop();


		//move vector of string list forward
		if (!gcode_string_vector.empty()) {
			for (int i = 0; i < gcode_string_vector.size() - 1; i++)
				gcode_string_vector[i] = gcode_string_vector[i+1];
		}
	}
	
	//trim vector
	while (gcode_string_vector.size() > gcode_queue_len)
		gcode_string_vector.pop_back();
	
	//trim queue to ensure that command queue is at appropriate length
	while (gcode_queue.size() > gcode_queue_len)
		gcode_queue.pop();


	



	if (send_gcode) {
		//send gcode here

		//------Serial communication here -------//


		//--------------------------------------//
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

	//Displacement vector info
	string displacement_str = "displacement: (" + std::to_string(displacement.x) + " , " + std::to_string(displacement.y) + " , "
		+ std::to_string(displacement.z) + " )";


	//Kinect tracking control window
	ImGui::Begin("Kinect output");
	ImGui::Combo("Tracking target", &tracking_target, tracking_targets);
	if (ImGui::Button("Set faux_origin")) set_faux_origin();

	string faux_origin_string = "Faux origin ( " + std::to_string(faux_origin.x) + ", " 
		+ std::to_string(faux_origin.y) + ", " + std::to_string(faux_origin.z) + ")";

	ImGui::Text(faux_origin_string.c_str());
	ImGui::Text(displacement_str.c_str());

	
	
	ImGui::End();

	

	ImGui::ShowDemoWindow();
	
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

	//draw vector from faux_origin to tracked target (temporarily left hand) if faux origin has been set
	if (faux_origin_set) {
		//get trackig target

		Vector4 tracked_pos = Vector4();

		Vector4 head_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HEAD];
		Vector4 left_hand_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT];
		Vector4 right_hand_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT];
		Vector4 left_foot_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_FOOT_LEFT];
		Vector4 right_foot_pos = skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_FOOT_RIGHT];

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

		float scalar = 100.0f;	//convert to cm from m
		vec3 target_pos = vec3(tracked_pos.x * scalar, tracked_pos.y * scalar , tracked_pos.z * scalar);
		

		//assume faux_origin has been set and scaled properly
		gl::color(Color(1, 0, 0));
		gl::lineWidth(8);
		gl::drawLine(faux_origin, target_pos);
		gl::color(Color(1, 1, 1));

		//calculate and record displacement to be applied to robot
		if(apply_displacement) displacement = target_pos - faux_origin;	
	}
	//Draw bones here

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
}


CINDER_APP( KinectXRobotApp, RendererGl )
