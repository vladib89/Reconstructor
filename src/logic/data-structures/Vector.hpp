#ifndef VECTOR_H
#define VECTOR_H

#include <iomanip>
#include <cmath>
#include <iostream>
#include <regex>
#include <vector>


namespace logic
{
namespace data_structures
{
#define vector_len 3

struct Vector
{
public:
	float x;
	float y;
	float z;
	bool is_valid = true;
	bool is_final = false;

	Vector(): x(0), y(0), z(0) {}

	Vector(float x, float y, float z):
		x(x),
		y(y),
		z(z)
	{
	}

	Vector(float x, float y): x(x), y(y) {}
	Vector(float *arr): Vector(arr[0], arr[1], arr[2]) {}

	Vector(std::string& s)
	{
		if (s.length() == 0) throw 0;

		char cr = '\r';
		char lf = '\n';
		char eol = s[s.length() - 1];
		char eol1  = s[s.length() - 2];

		if(eol == lf)
		{
			if (eol1 == cr)
			{
				s.erase(s.length() - 2);
			}
			else
			{
				s.erase(s.length() - 1);
			}
		}

		std::vector<std::string> out;
		std::string token;
		std::string delim = ",";

		size_t start;
		size_t end = 0;

		while ((start = s.find_first_not_of(delim, end)) != std::string::npos)
		{
			end = s.find(delim, start);
			out.push_back(s.substr(start, end - start));
		}

		x = std::stof(out[0]);
		y = std::stof(out[1]);
		z = std::stof(out[2]);
	}

	float norm();
	void toArray(float*);
	void toFourComponentArray(float*, float);
	bool isNanOrInfinity();
	static Vector zero();
	static Vector up();
	static Vector right();
	static Vector vforward();
	static float dot(const Vector& lhs, const Vector& rhs);
	static bool is_string_vec(const std::string &str);

	Vector& operator =(const Vector &rhs)
	{
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		is_valid = rhs.is_valid;
		is_final = rhs.is_final;
		return *this;
	}

	std::string to_string()
	{
        std::string res = std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
        return res;
	}

	friend bool operator ==(const Vector &lhs, const Vector &rhs)
	{
		bool res = false;

		res = lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;

		return res;
	}

	friend bool operator !=(const Vector &lhs, const Vector &rhs)
	{
		bool res = false;

		res = lhs.x != rhs.x || lhs.y != rhs.y || lhs.z != rhs.z;

		return res;
	}

	friend std::ostream& operator <<(std::ostream &os, const Vector &v)
	{
		return os << std::setprecision(3) << std::fixed << v.x << "," << v.y << "," << v.z;
	}

	friend Vector operator *(const Vector &lhs, const Vector &rhs)
	{
		Vector cross_p;
		cross_p.x = lhs.y * rhs.z - lhs.z * rhs.y;
		cross_p.y = lhs.z * rhs.x - lhs.x * rhs.z;
		cross_p.z = lhs.x * rhs.y - lhs.y * rhs.x;
		return cross_p;
	}

	friend Vector operator *(const Vector &lhs, float multiplier)
	{
		Vector res;
		res.x = lhs.x * multiplier;
		res.y = lhs.y * multiplier;
		res.z = lhs.z * multiplier;
		return res;
	}

	friend Vector operator *(const int multiplier, const Vector &rhs)
	{
		Vector res;
		res.x = multiplier * rhs.x;
		res.y = multiplier * rhs.y;
		res.z = multiplier * rhs.z;
		return res;
	}

	friend Vector operator /(const Vector &lhs, float divisor)
	{
		Vector res;
		res.x = lhs.x / divisor;
		res.y = lhs.y / divisor;
		res.z = lhs.z / divisor;
		return res;
	}

	friend Vector operator +(const Vector &lhs, const Vector &rhs)
	{
		Vector res;
		res.x = lhs.x + rhs.x;
		res.y = lhs.y + rhs.y;
		res.z = lhs.z + rhs.z;
		return res;
	}

	friend Vector operator -(const Vector &lhs, const Vector &rhs)
	{
		Vector res;
		res.x = lhs.x - rhs.x;
		res.y = lhs.y - rhs.y;
		res.z = lhs.z - rhs.z;
		return res;
	}

	friend Vector operator -(const Vector &rhs)
	{
		Vector res;
		res.x = -rhs.x;
		res.y = -rhs.y;
		res.z = -rhs.z;
		return res;
	}
};
}
}

#endif // VECTOR_H




