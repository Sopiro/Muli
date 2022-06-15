#include "common.h"
#include "engine.h"

int main(int argc, char** argv)
{
	SPDLOG_INFO("Start program");

	Engine engine(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE);

	engine.run();

	return 0;
}
