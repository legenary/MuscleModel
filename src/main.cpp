// App_Example.cpp : This file contains the 'main' function. 
// Program execution begins and ends there.
//
#include "my_pch.h"
#include "Simulation.h"
#include "Parameter.h"

#include "CommonInterfaces/CommonExampleInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"

//examples
#include "Utils/b3Clock.h"
#include "ExampleBrowser/OpenGLGuiHelper.h"


volatile sig_atomic_t exitFlag = 0;

void exit_function(int sigint)
{
	exitFlag = 1;
}

Simulation *simulation;

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
	std::cout << "main starts.\n";
	SimpleOpenGL3App *app = new SimpleOpenGL3App("Simulation test", 1024, 768, true);

	prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
	prevMouseMoveCallback = app->m_window->getMouseMoveCallback();

	app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
	app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);

	OpenGLGuiHelper gui(app, false);
	CommonExampleOptions options(&gui);

	Parameter *param = new Parameter();
	simulation = new Simulation(options.m_guiHelper);
	simulation->processCommandLineArgs(argc, argv);

	simulation->initParameter(param);
	simulation->initPhysics();
	simulation->resetCamera();

	char bla[32];
	float clr_black[4] = { 1, 1, 1, 1 };
	const char* videoFileName = "../output/output_video.mp4";
	app->dumpFramesToVideo(videoFileName);

	app->m_renderer->writeTransforms();

	do {
		static int frameCount = 0;
		frameCount++;

		// initializing rendered
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera(app->getUpAxis());

		// step simualtion
		//simulation->stepSimulation(param->m_time_step);
		simulation->stepSimulation(param->m_time_step);

		// render instances
		simulation->renderScene();
		app->m_renderer->renderScene();

		// draw grid
		DrawGridData dg;
		dg.gridSize = 15;
		dg.upAxis = app->getUpAxis();
		app->setBackgroundColor(1, 1, 1);
		app->drawGrid(dg);

		// draw Text
		sprintf(bla, "Frame: %d", frameCount);
		app->drawText(bla, 10, 10, 0.5, clr_black);

		// end rendering this frame
		app->swapBuffer();

	}while (!app->m_window->requestedExit() && !(exitFlag || simulation->exitSim));


	simulation->exitPhysics();
	std::cout << "Simulation terminated...\n";

	delete simulation;
	delete param;
	delete app;
	std::cout << "Done. Exit Safe.\n";


	return 0;
}