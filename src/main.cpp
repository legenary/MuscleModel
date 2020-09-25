// App_Example.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "Simulation.h"
#include <stdio.h>
#include <iostream>
// #include <vector>
#include <signal.h>
#include <stdlib.h>
#include <string>

//src
#include "CommonInterfaces/CommonExampleInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
//#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
//#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "Bullet3Common/b3Quaternion.h"

//examples
#include "Utils/b3Clock.h"
#include "ExampleBrowser/OpenGLGuiHelper.h"


volatile sig_atomic_t exitFlag = 0;

void exit_function(int sigint)
{
	exitFlag = 1;
}

Simulation *simulation;
int gSharedMemoryKey = -1;

b3MouseMoveCallback prevMouseMoveCallback = 0;
static void OnMouseMove(float x, float y)
{
	bool handled = false;
	handled = simulation->mouseMoveCallback(x, y);
	if (!handled)
	{
		if (prevMouseMoveCallback)
			prevMouseMoveCallback(x, y);
	}
}

b3MouseButtonCallback prevMouseButtonCallback = 0;
static void OnMouseDown(int button, int state, float x, float y) {
	bool handled = false;

	handled = simulation->mouseButtonCallback(button, state, x, y);
	if (!handled)
	{
		if (prevMouseButtonCallback)
			prevMouseButtonCallback(button, state, x, y);
	}
}

class LessDummyGuiHelper : public DummyGUIHelper
{
	CommonGraphicsApp* m_app;
public:
	virtual CommonGraphicsApp* getAppInterface()
	{
		return m_app;
	}

	LessDummyGuiHelper(CommonGraphicsApp* app)
		:m_app(app)
	{
	}
};

int main(int argc, char** argv)
{
	std::cout << "main starts." << std::endl;
	SimpleOpenGL3App *app = new SimpleOpenGL3App("Simulation test", 1024, 768, true);

	prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
	prevMouseMoveCallback = app->m_window->getMouseMoveCallback();

	app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
	app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);

	OpenGLGuiHelper gui(app, false);
	CommonExampleOptions options(&gui);

	simulation = new Simulation(options.m_guiHelper);
	simulation->processCommandLineArgs(argc, argv);

	simulation->initPhysics();
	simulation->resetCamera();

	int textureWidth = 128;
	int textureHeight = 128;

	unsigned char*	image = new unsigned char[textureWidth*textureHeight * 4];
	int textureHandle = app->m_renderer->registerTexture(image, textureWidth, textureHeight);
	do {
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera(app->getUpAxis());

		simulation->stepSimulation();
	}while (!app->m_window->requestedExit() && !(exitFlag || simulation->exitSim));


	simulation->exitPhysics();
	std::cout << "Simulation terminated..." << std::endl;

	delete simulation;
	delete app;
	delete[] image;
	std::cout << "Done." << std::endl;


	return 0;
}