#include "spe/box.h"

namespace spe
{

Box::Box(float _width, float _height, BodyType _type, float _density) :
    Polygon{ {{0, 0}, {_width, 0}, {_width, _height}, {0, _height}}, std::move(_type), true, std::move(_density) },
    width{ std::move(_width) },
    height{ std::move(_height) }
{

}

Box::Box(float _width, BodyType _type, float _density) :
    Box{ _width, _width, _type, _density }
{

}

}