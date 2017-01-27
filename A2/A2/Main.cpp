/*
 *	Includes
 */
#include <assimp/cimport.h>		// C importer
#include <assimp/scene.h>		// Collects data
#include <assimp/postprocess.h> // Various extra operations
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>
#include <math.h>
#include <mmsystem.h>
#include <sstream>
#include <stdio.h>
#include <vector>				// STL dynamic memory
#include <windows.h>

#include "Antons_maths_funcs.h" // Anton's maths functions
#include "Camera.h"
#include "Mesh.h"
#include "RigidBody.h"
#include "Shader_Functions.h"
#include "text.h"
#include "time.h"

using namespace std;

/*
 *	Globally defined variables and constants
 */
#define BUFFER_OFFSET(i) ((char *)NULL + (i))  // Macro for indexing vertex buffer

#define NUM_MESHES   1
#define NUM_SHADERS	 3
#define NUM_TEXTURES 1

bool firstMouse = true;
bool keys[1024];
Camera camera(vec3(0.0f, -0.5f, 3.0f));
enum Meshes { OBJECT_MESH };
enum Shaders { SKYBOX, PARTICLE_SHADER, BASIC_TEXTURE_SHADER };
enum Textures { OBJECT_TEXTURE };
GLfloat cameraSpeed = 0.005f;
GLfloat deltaTime = 1.0f / 60.0f;
GLfloat friction = 0.98f;
GLfloat lastX = 400, lastY = 300;
GLfloat resilience = 0.01f;
GLuint shaderProgramID[NUM_SHADERS];
int screenWidth = 1000;
int screenHeight = 800;
int stringIDs[13];
Mesh skyboxMesh, objectMesh;
RigidBody rigidBody;
vec3 gravity = vec3(0.0f, -9.81f, 0.0f);
//vec3 groundVector = vec3(0.0f, -1.0f, 0.0f);
//vec3 groundNormal = vec3(0.0f, 1.0f, 0.0f);

// | Resource Locations
const char * meshFiles[NUM_MESHES] = { "../Meshes/particle_reduced.dae" };
const char * skyboxTextureFiles[6] = { "../Textures/DSposx.png", "../Textures/DSnegx.png", "../Textures/DSposy.png", "../Textures/DSnegy.png", "../Textures/DSposz.png", "../Textures/DSnegz.png"};
const char * textureFiles[NUM_TEXTURES] = { "../Textures/asphalt.jpg" };

const char * vertexShaderNames[NUM_SHADERS] = { "../Shaders/SkyboxVertexShader.txt", "../Shaders/ParticleVertexShader.txt", "../Shaders/BasicTextureVertexShader.txt" };
const char * fragmentShaderNames[NUM_SHADERS] = { "../Shaders/SkyboxFragmentShader.txt", "../Shaders/ParticleFragmentShader.txt", "../Shaders/BasicTextureFragmentShader.txt" };

void draw_text()
{
	ostringstream massOSS, ibodyOSS, ibodyIOSS, posOSS, orientOSS, linMomOSS, angMomOSS, iInvOSS, rotOSS, velOSS, angVelOSS, torOSS, forOSS;
	string mass, ibody, ibodyI, pos, orient, linMom, angMom, iInv, rot, vel, angVel, tor, force;
	massOSS << "Mass ( " << rigidBody.mass << " )";
	//
	//
	posOSS << "Position ( " << rigidBody.position.v[0] << ", " << rigidBody.position.v[1] << ", " << rigidBody.position.v[2] << ")";
	orientOSS << "Orientation ( " << rigidBody.orientation.q[0] << ", " << rigidBody.orientation.q[1] << ", " << rigidBody.orientation.q[2] << ", " << rigidBody.orientation.q[3] << ")";
	linMomOSS << "Linear Momentum ( " << rigidBody.linearMomentum.v[0] << ", " << rigidBody.linearMomentum.v[1] << ", " << rigidBody.linearMomentum.v[2] << ")";
	angMomOSS << "Angular Momentum ( " << rigidBody.angularMomentum.v[0] << ", " << rigidBody.angularMomentum.v[1] << ", " << rigidBody.angularMomentum.v[2] << ")";
	//
	//
	velOSS << "Velocity ( " << rigidBody.velocity.v[0] << ", " << rigidBody.velocity.v[1] << ", " << rigidBody.velocity.v[2] << ")";
	angVelOSS << "Angular Velocity ( " << rigidBody.angularVelocity.v[0] << ", " << rigidBody.angularVelocity.v[1] << ", " << rigidBody.angularVelocity.v[2] << ")";
	torOSS << "Torque ( " << rigidBody.torque.v[0] << ", " << rigidBody.torque.v[1] << ", " << rigidBody.torque.v[2] << ")";
	forOSS << "Force ( " << rigidBody.force.v[0] << ", " << rigidBody.force.v[1] << ", " << rigidBody.force.v[2] << ")";
	
	mass = massOSS.str();
	//ibody = ibodyOSS.str();
	//ibodyI = ibodyIOSS.str();
	pos = posOSS.str();
	orient = orientOSS.str();
	linMom = linMomOSS.str();
	angMom = angMomOSS.str();
	//iInv = iInvOSS.str();
	//rot = rotOSS.str();
	vel = velOSS.str();
	angVel = angVelOSS.str();
	tor = torOSS.str();
	force = forOSS.str();
	
	update_text(stringIDs[0], mass.c_str());
	//update_text(stringIDs[1], ibody.c_str());
	//update_text(stringIDs[2], ibodyI.c_str());
	update_text(stringIDs[3], pos.c_str());
	update_text(stringIDs[4], orient.c_str());
	update_text(stringIDs[5], linMom.c_str());
	update_text(stringIDs[6], angMom.c_str());
	//update_text(stringIDs[7], iInv.c_str());
	//update_text(stringIDs[8], rot.c_str());
	update_text(stringIDs[9], vel.c_str());
	update_text(stringIDs[10], angVel.c_str());
	update_text(stringIDs[11], tor.c_str());
	update_text(stringIDs[12], force.c_str());

	draw_texts();
}

void init_text()
{
	stringIDs[0] = add_text("Mass ( )", -0.95f, 0.95f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[1] = add_text("Ibody (,,,)", -0.95f, 0.9f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[2] = add_text("Ibody-1 (,,,)", -0.95f, 0.85f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[3] = add_text("Position (,,)",-0.95f, 0.8f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[4] = add_text("Orientation (,,,)", -0.95f, 0.75f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[5] = add_text("Linear Momentum (,,,)", -0.95f, 0.7f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[6] = add_text("Angular Momentum (,,,)", -0.95f, 0.65f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[7] = add_text("I-1 (,,,)", -0.95f, 0.6f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[8] = add_text("Rotation (,,,)", -0.95f, 0.55f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[9] = add_text("Velocity (,,,)", -0.95f, 0.5f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[10] = add_text("AngularVelocity (,,,)", -0.95f, 0.45f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[11] = add_text("Torque (,,,)", -0.95f, 0.4f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[12] = add_text("Force (,,,)", -0.95f, 0.35f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
}

void display() 
{
	// Tell GL to only draw onto a pixel if the shape is closer to the viewer
	glEnable(GL_DEPTH_TEST);	// Enable depth-testing
	glDepthFunc(GL_LESS);		// Depth-testing interprets a smaller value as "closer"
	glClearColor(5.0f/255.0f, 1.0f/255.0f, 15.0f/255.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Draw skybox first
	mat4 view = camera.GetViewMatrix(); 
	mat4 projection = perspective(camera.Zoom, (float)screenWidth / (float)screenHeight, 0.1f, 100.0f);

	//skyboxMesh.drawSkybox(view, projection);

	mat4 objectModel = identity_mat4();
	objectModel = rigidBody.rotation * objectModel;
	objectModel = translate(objectModel, rigidBody.position);

	//objectMesh.drawMesh(view, projection, objectModel);

	draw_text();
	
	glutSwapBuffers();
}

void computeForcesAndTorque()
{
	// Clear Forces
	rigidBody.force = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	rigidBody.torque = vec4(0.0f, 0.0f, 0.0f, 0.0f);
}

void updateRigidBody()
{
	// Might change this to user input
	//computeForcesAndTorque();

	//vec3 xdot = rigidBody.velocity;
	rigidBody.position += rigidBody.velocity * deltaTime;

	versor omega = quat_from_axis_deg(0.0f, rigidBody.angularVelocity.v[0], rigidBody.angularVelocity.v[1], rigidBody.angularVelocity.v[2]);
	cout << "Omega = " << omega.q[0] << ", " << omega.q[1] << ", " << omega.q[2] << ", " << omega.q[3] << endl;
	
	versor qdot = omega * rigidBody.orientation;
	qdot = qdot * 0.5 * deltaTime;
	
	cout << "Qdot = " << qdot.q[0] << ", " << qdot.q[1] << ", " << qdot.q[2] << ", " << qdot.q[3] << endl;
	normalise(qdot);
	cout << "Qdot = " << qdot.q[0] << ", " << qdot.q[1] << ", " << qdot.q[2] << ", " << qdot.q[3] << endl;

	// Not sure if this will be correct
	multiplyQuat(rigidBody.orientation, qdot, rigidBody.orientation);

	//rigidBody.linearMomentum += rigidBody.force * deltaTime;
	//rigidBody.angularMomentum += rigidBody.torque * deltaTime;

	//vec3 pdot = rigidBody.force;
	//vec3 ldot = rigidBody.torque;

	//rigidBody.velocity = rigidBody.linearMomentum / rigidBody.mass;
	rigidBody.rotation = quat_to_mat4(normalise(rigidBody.orientation));
	//rigidBody.Iinv = rigidBody.rotation * rigidBody.IbodyInv * transpose(rigidBody.rotation);
	//rigidBody.angularVelocity = rigidBody.Iinv * rigidBody.angularMomentum;
}

void processInput()
{
	if (keys[GLUT_KEY_UP])
		camera.ProcessKeyboard(FORWARD, cameraSpeed);
	if(keys[GLUT_KEY_DOWN])
		camera.ProcessKeyboard(BACKWARD, cameraSpeed);
	if (keys[GLUT_KEY_LEFT])
		camera.ProcessKeyboard(LEFT, cameraSpeed);
	if (keys[GLUT_KEY_RIGHT])
		camera.ProcessKeyboard(RIGHT, cameraSpeed);
	if (keys['q'])
	{
		rigidBody.velocity.v[0] -= 0.01f;
	}
	if (keys['w'])
	{
		rigidBody.velocity.v[0] += 0.01f;
	}
	if (keys[(char)27])
		exit(0);
}

void updateScene()
{
	processInput();
	updateRigidBody();
	// Draw the next frame
	glutPostRedisplay();
}

void init()
{
	if (!init_text_rendering("../Textures/freemono.png", "../Textures/freemono.meta", screenWidth, screenHeight))
	{
		fprintf(stderr, "ERROR init text rendering\n");
		exit(1);
	}
	init_text();

	// Compile the shaders
	for (int i = 0; i < NUM_SHADERS; i++)
	{
		shaderProgramID[i] = CompileShaders(vertexShaderNames[i], fragmentShaderNames[i]);
	}

	skyboxMesh = Mesh(&shaderProgramID[SKYBOX]);
	skyboxMesh.setupSkybox(skyboxTextureFiles);

	objectMesh = Mesh(&shaderProgramID[BASIC_TEXTURE_SHADER]);
	objectMesh.generateObjectBufferMesh(meshFiles[OBJECT_MESH]);
	objectMesh.loadTexture(textureFiles[OBJECT_TEXTURE]);

	rigidBody = RigidBody();
}

/*
 *	User Input Functions
 */
#pragma region USER_INPUT_FUNCTIONS
void pressNormalKeys(unsigned char key, int x, int y)
{
	keys[key] = true;
	if (keys['p'])
	{
	}
}

void releaseNormalKeys(unsigned char key, int x, int y)
{
	keys[key] = false;
}

void pressSpecialKeys(int key, int x, int y)
{
	keys[key] = true;
}

void releaseSpecialKeys(int key, int x, int y)
{
	keys[key] = false;
}

void mouseClick(int button, int state, int x, int y)
{}

void processMouse(int x, int y)
{
	if (firstMouse)
	{
		lastX = x;
		lastY = y;
		firstMouse = false;
	}

	GLfloat xoffset = x - lastX;
	GLfloat yoffset = lastY - y;

	lastX = x;
	lastY = y;

	camera.ProcessMouseMovement(xoffset, yoffset);
}

void mouseWheel(int button, int dir, int x, int y)
{}
#pragma endregion

/*
 *	Main
 */
int main(int argc, char** argv) 
{
	srand(time(NULL));

	// Set up the window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(screenWidth, screenHeight);
	glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH) - screenWidth) / 2, (glutGet(GLUT_SCREEN_HEIGHT) - screenHeight) / 4);
	glutCreateWindow("Particle System");

	// Glut display and update functions
	glutDisplayFunc(display);
	glutIdleFunc(updateScene);

	// User input functions
	glutKeyboardFunc(pressNormalKeys);
	glutKeyboardUpFunc(releaseNormalKeys);
	glutSpecialFunc(pressSpecialKeys);
	glutSpecialUpFunc(releaseSpecialKeys);
	glutMouseFunc(mouseClick);
	glutPassiveMotionFunc(processMouse);
	glutMouseWheelFunc(mouseWheel);


	glewExperimental = GL_TRUE; //for non-lab machines, this line gives better modern GL support
	
	// A call to glewInit() must be done after glut is initialized!
	GLenum res = glewInit();
	// Check for any errors
	if (res != GLEW_OK) {
		fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
		return 1;
	}

	// Set up meshes and shaders
	init();
	// Begin infinite event loop
	glutMainLoop();
	return 0;
}