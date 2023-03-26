#include "Vector.hpp"

namespace logic
{
namespace data_structures
{
float Vector::norm()
{
    float mag = sqrt(x * x + y * y + z * z);
    return mag;
}

void Vector::toArray(float* arr)
{
    arr[0] = x;
    arr[1] = y;
    arr[2] = z;
}

void Vector::toFourComponentArray(float* arr, float fourthComponent)
{
    toArray(arr);
    arr[3] = fourthComponent;
}

bool Vector::isNanOrInfinity()
{
    return (x - x != 0) || (y - y != 0) || (z - z != 0);
}

float Vector::dot(const Vector &lhs, const Vector &rhs)
{
    float product = lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    return product;
}

bool Vector::is_string_vec(const std::string &str)
{
    bool res = false;
    std::regex rx("^[0-9]+,[0-9]+,[0-9]+(?:\\r\\n|\\r|\\n){1}$");
    std::smatch m;

    if (std::regex_match(str, m, rx) && m.size() == 1)
    {
        res = true;
    }

    return res;
}

Vector Vector::zero()
{
    return Vector(0, 0, 0);
}

Vector Vector::vforward()
{
    return Vector(0, 0, 1);
}

Vector Vector::up()
{
    return Vector(0, 1, 0);
}

Vector Vector::right()
{
    return Vector(1, 0, 0);
}
}
}
