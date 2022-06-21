#include "common.h"
#include "engine.h"

int main(int argc, char** argv)
{
	SPDLOG_INFO("Start program");

	spe::Engine* engine = new spe::Engine(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE);

	engine->Run();

	delete engine;

	return 0;
}
