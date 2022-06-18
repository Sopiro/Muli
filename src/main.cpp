#include "common.h"
#include "engine.h"

class A
{
public:
	A(int a)
	{
		aa = a;
	}
private:
	int aa;
};

int main(int argc, char** argv)
{
	SPDLOG_INFO("Start program");

	Engine* engine = new Engine(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE);

	engine->Run();

	delete engine;

	return 0;
}
