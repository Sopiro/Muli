#include "common.h"
#include "application.h"

int main(int argc, char** argv)
{
	SPDLOG_INFO("Start program");

	spe::Application* app = spe::Application::Create(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE);

	app->Run();

	delete app;

	return 0;
}