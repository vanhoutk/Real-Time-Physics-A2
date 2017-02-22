/*
 *	Includes
 */
#include <assimp/cimport.h>		// C importer
#include <assimp/scene.h>		// Collects data
#include <assimp/postprocess.h> // Various extra operations
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iomanip>				// setprecision
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
Camera camera(vec3(0.0f, 0.0f, 10.0f));
enum Meshes { OBJECT_MESH };
enum Shaders { SKYBOX, PARTICLE_SHADER, BASIC_TEXTURE_SHADER };
enum Textures { OBJECT_TEXTURE };
GLfloat cameraSpeed = 0.005f;
GLfloat friction = 0.98f;
GLfloat resilience = 0.01f;
GLuint lastX = 400, lastY = 300;
GLuint shaderProgramID[NUM_SHADERS];
int screenWidth = 1000;
int screenHeight = 800;
int stringIDs[13];
Mesh skyboxMesh, objectMesh;
RigidBody rigidBody;

// | Resource Locations
const char * meshFiles[NUM_MESHES] = { "../Meshes/particle_reduced.dae" };
const char * skyboxTextureFiles[6] = { "../Textures/DSposx.png", "../Textures/DSnegx.png", "../Textures/DSposy.png", "../Textures/DSnegy.png", "../Textures/DSposz.png", "../Textures/DSnegz.png"};
const char * textureFiles[NUM_TEXTURES] = { "../Textures/asphalt.jpg" };

const char * vertexShaderNames[NUM_SHADERS] = { "../Shaders/SkyboxVertexShader.txt", "../Shaders/ParticleVertexShader.txt", "../Shaders/BasicTextureVertexShader.txt" };
const char * fragmentShaderNames[NUM_SHADERS] = { "../Shaders/SkyboxFragmentShader.txt", "../Shaders/ParticleFragmentShader.txt", "../Shaders/BasicTextureFragmentShader.txt" };

string frf(const float &f)
{
	ostringstream ss;
	ss << setfill(' ') << std::setw(6) << fixed << setprecision(3) << f;
	string s(ss.str());
	return s;
}

void draw_text()
{
	ostringstream oss[13]; // massOSS, ibodyOSS, ibodyIOSS, posOSS, orientOSS, linMomOSS, angMomOSS, iInvOSS, rotOSS, velOSS, angVelOSS, torOSS, forOSS;
	string strings[13]; // mass, ibody, ibodyI, pos, orient, linMom, angMom, iInv, rot, vel, angVel, tor, force;
	oss[0] << "Mass ( " << fixed << setprecision(3) << rigidBody.mass << " )";
	oss[1] << "Ibody \n |" << frf(rigidBody.Ibody.m[0]) << " " << frf(rigidBody.Ibody.m[4]) << " " << frf(rigidBody.Ibody.m[8]) << " |\n" <<
		" |" << frf(rigidBody.Ibody.m[1]) << " " << frf(rigidBody.Ibody.m[5]) << " " << frf(rigidBody.Ibody.m[9]) << " |\n" <<
		" |" << frf(rigidBody.Ibody.m[2]) << " " << frf(rigidBody.Ibody.m[6]) << " " << frf(rigidBody.Ibody.m[10]) << " |";
	oss[2] << "Ibody-1 \n |" << frf(rigidBody.IbodyInv.m[0]) << " " << frf(rigidBody.IbodyInv.m[4]) << " " << frf(rigidBody.IbodyInv.m[8]) << " |\n" <<
		" |" << frf(rigidBody.IbodyInv.m[1]) << " " << frf(rigidBody.IbodyInv.m[5]) << " " << frf(rigidBody.IbodyInv.m[9]) << " |\n" <<
		" |" << frf(rigidBody.IbodyInv.m[2]) << " " << frf(rigidBody.IbodyInv.m[6]) << " " << frf(rigidBody.IbodyInv.m[10]) << " |";
	oss[3] << "Position ( " << frf(rigidBody.position.v[0]) << ", " << frf(rigidBody.position.v[1]) << ", " << frf(rigidBody.position.v[2]) << ")";
	oss[4] << "Orientation ( " << frf(rigidBody.orientation.q[0]) << ", " << frf(rigidBody.orientation.q[1]) << ", " << frf(rigidBody.orientation.q[2]) << ", " << frf(rigidBody.orientation.q[3]) << ")";
	oss[5] << "Linear Momentum ( " << frf(rigidBody.linearMomentum.v[0]) << ", " << frf(rigidBody.linearMomentum.v[1]) << ", " << frf(rigidBody.linearMomentum.v[2]) << ")";
	oss[6] << "Angular Momentum ( " << frf(rigidBody.angularMomentum.v[0]) << ", " << frf(rigidBody.angularMomentum.v[1]) << ", " << frf(rigidBody.angularMomentum.v[2]) << ")";
	oss[7] << "I-1 \n |" << frf(rigidBody.Iinv.m[0]) << " " << frf(rigidBody.Iinv.m[4]) << " " << frf(rigidBody.Iinv.m[8]) << " |\n" <<
		" |" << frf(rigidBody.Iinv.m[1]) << " " << frf(rigidBody.Iinv.m[5]) << " " << frf(rigidBody.Iinv.m[9]) << " |\n" <<
		" |" << frf(rigidBody.Iinv.m[2]) << " " << frf(rigidBody.Iinv.m[6]) << " " << frf(rigidBody.Iinv.m[10]) << " |";
	oss[8] << "Rotation \n |" << frf(rigidBody.rotation.m[0]) << " " << frf(rigidBody.rotation.m[4]) << " " << frf(rigidBody.rotation.m[8]) << " |\n" <<
		" |" << frf(rigidBody.rotation.m[1]) << " " << frf(rigidBody.rotation.m[5]) << " " << frf(rigidBody.rotation.m[9]) << " |\n" <<
		" |" << frf(rigidBody.rotation.m[2]) << " " << frf(rigidBody.rotation.m[6]) << " " << frf(rigidBody.rotation.m[10]) << " |";
	oss[9] << "Velocity ( " << fixed << setprecision(3) << rigidBody.velocity.v[0] << ", " << rigidBody.velocity.v[1] << ", " << rigidBody.velocity.v[2] << ")";
	oss[10] << "Angular Velocity ( " << frf(rigidBody.angularVelocity.v[0]) << ", " << frf(rigidBody.angularVelocity.v[1]) << ", " << frf(rigidBody.angularVelocity.v[2]) << ")";
	oss[11] << "Torque ( " << frf(rigidBody.torque.v[0]) << ", " << frf(rigidBody.torque.v[1]) << ", " << frf(rigidBody.torque.v[2]) << ")";
	oss[12] << "Force ( " << frf(rigidBody.force.v[0]) << ", " << frf(rigidBody.force.v[1]) << ", " << frf(rigidBody.force.v[2]) << ")";
	
	for (int i = 0; i < 13; i++)
	{
		strings[i] = oss[i].str();
		update_text(stringIDs[i], strings[i].c_str());
	}

	draw_texts();
}

void init_text()
{
	stringIDs[0] = add_text("Mass ( )", -0.95f, 0.95f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[1] = add_text("Ibody (,,,)", -0.95f, 0.9f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[2] = add_text("Ibody-1 (,,,)", -0.95f, 0.7f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[3] = add_text("Position (,,)",-0.95f, 0.5f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[4] = add_text("Orientation (,,,)", -0.95f, 0.45f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[5] = add_text("Linear Momentum (,,,)", -0.95f, 0.4f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[6] = add_text("Angular Momentum (,,,)", -0.95f, 0.35f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[7] = add_text("I-1 (,,,)", -0.95f, 0.3f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[8] = add_text("Rotation (,,,)", -0.95f, 0.1f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[9] = add_text("Velocity (,,,)", -0.95f, -0.1f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[10] = add_text("AngularVelocity (,,,)", -0.95f, -0.15f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[11] = add_text("Torque (,,,)", -0.95f, -0.2f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[12] = add_text("Force (,,,)", -0.95f, -0.25f, 20.0f, 1.0f, 1.0f, 1.0f, 1.0f);
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

	skyboxMesh.drawSkybox(view, projection);

	rigidBody.drawMesh(view, projection);

	draw_text();
	
	glutSwapBuffers();
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
	if (keys[(char)27])
		exit(0);
}

void updateScene()
{
	processInput();
	updateRigidBody(rigidBody);
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

	rigidBody = RigidBody(objectMesh);
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
		rigidBody.force = vec4(0.0f, 0.01f, 0.0f, 0.0f);
		rigidBody.torque = getTorque(rigidBody.force, rigidBody.position, rigidBody.worldVertices[0]);
	}
	if (keys['o'])
	{
		rigidBody.force = vec4(0.0f, -1.0f, 0.0f, 0.0f);
		rigidBody.torque = getTorque(rigidBody.force, rigidBody.position, vec4(10.0f, 0.0f, 0.0f, 0.0f));
	}
	if (keys['q'])
		rigidBody.force = vec4(-0.1f, 0.0f, 0.0f, 0.0f);
	if (keys['w'])
		rigidBody.force = vec4(0.1f, 0.0f, 0.0f, 0.0f);
	if (keys['a'])
		rigidBody.force = vec4(0.0f, -0.1f, 0.0f, 0.0f);
	if (keys['s'])
		rigidBody.force = vec4(0.0f, 0.1f, 0.0f, 0.0f);
	if (keys['z'])
		rigidBody.force = vec4(0.0f, 0.0f, -0.1f, 0.0f);
	if (keys['x'])
		rigidBody.force = vec4(0.0f, 0.0f, 0.1f, 0.0f);

	if (keys['t'])
		rigidBody.torque = vec4(-0.1f, 0.0f, 0.0f, 0.0f);
	if (keys['y'])
		rigidBody.torque = vec4(0.1f, 0.0f, 0.0f, 0.0f);
	if (keys['g'])
		rigidBody.torque = vec4(0.0f, -0.1f, 0.0f, 0.0f);
	if (keys['h'])
		rigidBody.torque = vec4(0.0f, 0.1f, 0.0f, 0.0f);
	if (keys['b'])
		rigidBody.torque = vec4(0.0f, 0.0f, -0.1f, 0.0f);
	if (keys['n'])
		rigidBody.torque = vec4(0.0f, 0.0f, 0.1f, 0.0f);
	if (keys['0'])
	{
		rigidBody.angularMomentum = vec4(0.0f, 0.0f, 0.0f, 0.0f);
		rigidBody.angularVelocity = vec4(0.0f, 0.0f, 0.0f, 0.0f);
		rigidBody.position = vec4(0.0f, 0.0f, 0.0f, 0.0f);
		rigidBody.velocity = vec4(0.0f, 0.0f, 0.0f, 0.0f);
		rigidBody.linearMomentum = vec4(0.0f, 0.0f, 0.0f, 0.0f);
		rigidBody.orientation.q[0] = 0.0f;
		rigidBody.orientation.q[1] = 0.0f;
		rigidBody.orientation.q[2] = 1.0f;
		rigidBody.orientation.q[3] = 0.0f;
	}
}

void releaseNormalKeys(unsigned char key, int x, int y)
{
	keys[key] = false;
	if (!keys['p'])
	{
		rigidBody.force = vec4(0.0f, 0.0f, 0.0f, 0.0f);
		rigidBody.torque = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	}
	if (!keys['o'])
	{
		rigidBody.force = vec4(0.0f, 0.0f, 0.0f, 0.0f);
		rigidBody.torque = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	}

	if (!keys['q'])
		rigidBody.force = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	if (!keys['w'])
		rigidBody.force = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	if (!keys['a'])
		rigidBody.force = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	if (!keys['s'])
		rigidBody.force = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	if (!keys['z'])
		rigidBody.force = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	if (!keys['x'])
		rigidBody.force = vec4(0.0f, 0.0f, 0.0f, 0.0f);

	if (!keys['t'])
		rigidBody.torque = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	if (!keys['y'])
		rigidBody.torque = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	if (!keys['g'])
		rigidBody.torque = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	if (!keys['h'])
		rigidBody.torque = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	if (!keys['b'])
		rigidBody.torque = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	if (!keys['n'])
		rigidBody.torque = vec4(0.0f, 0.0f, 0.0f, 0.0f);
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

	int xoffset = x - lastX;
	int yoffset = lastY - y;

	lastX = x;
	lastY = y;

	//camera.ProcessMouseMovement((GLfloat)xoffset, (GLfloat)yoffset);
}

void mouseWheel(int button, int dir, int x, int y)
{}
#pragma endregion

/*
 *	Main
 */
int main(int argc, char** argv) 
{
	srand((unsigned int)time(NULL));

	// Set up the window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(screenWidth, screenHeight);
	glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH) - screenWidth) / 2, (glutGet(GLUT_SCREEN_HEIGHT) - screenHeight) / 4);
	glutCreateWindow("Rigid Body Unconstrained Motion");

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