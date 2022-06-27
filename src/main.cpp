#pragma warning(disable:4819)

#include "application.h"

int main(int argc, char** argv)
{
	SPDLOG_INFO("Start program");

    std::srand(std::time(0));
	spe::Application* app = spe::Application::Create(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE);
	
	app->Run();

	delete app;

	return 0;
}