#pragma once

template <int T>
class Vector
{
public:
	Vector();
	Vector(std::vector<T> data);
	Vector(Vector<T> v);

	// various operators

private:
	double[T] vec;
};