#include "Vector2f.h"

namespace Liar
{
	Vector2f::Vector2f(Liar::NAVDTYPE x, Liar::NAVDTYPE y):m_x(x),m_y(y)
	{}

	Vector2f::Vector2f(const Liar::Vector2f& rhs):m_x(rhs.m_x),m_y(rhs.m_y)
	{}

	Vector2f::~Vector2f()
	{}

	void Vector2f::Set(Liar::NAVDTYPE x, Liar::NAVDTYPE y)
	{
		m_x = x;
		m_y = y;
	}

	void Vector2f::Set(const Vector2f& rhs)
	{
		Set(rhs.m_x, rhs.m_y);
	}

	Liar::NAVDTYPE Vector2f::Length() const
	{
		return sqrtf(LengthSquared());
	}

	Liar::NAVDTYPE Vector2f::LengthSquared() const
	{
		return m_x * m_x + m_y * m_y;
	}

	Liar::NAVDTYPE Vector2f::Distance(const Vector2f& rhs) const
	{
		return Distance(rhs.m_x, rhs.m_y);
	}

	Liar::NAVDTYPE Vector2f::Distance(Liar::NAVDTYPE x, Liar::NAVDTYPE y) const
	{

		return sqrt(DistanceSqured(x, y));
	}

	Liar::NAVDTYPE Vector2f::DistanceSqured(const Liar::Vector2f& rhs) const
	{
		return DistanceSqured(rhs.m_x, rhs.m_y);
	}

	Liar::NAVDTYPE Vector2f::DistanceSqured(Liar::NAVDTYPE x, Liar::NAVDTYPE y) const
	{
		Liar::NAVDTYPE dx = x - m_x;
		Liar::NAVDTYPE dy = y - m_y;
		return dx * dx + dy * dy;
	}

	void Vector2f::Zero()
	{
		m_x = m_y = Liar::ZERO;
	}

	bool Vector2f::Equals(const Vector2f& rhs, Liar::EPSILONTYPE e) const
	{
		return Equals(rhs.m_x, rhs.m_y, e);
	}

	bool Vector2f::Equals(Liar::NAVDTYPE x, Liar::NAVDTYPE y, Liar::EPSILONTYPE e) const
	{
		return fabs(m_x - x) < e && fabs(m_y - y) < e;
	}

	Vector2f Vector2f::operator-() const
	{
		return Liar::Vector2f(-m_x, -m_y);
	}

	Liar::Vector2f Vector2f::operator-(const Vector2f& rhs) const
	{
		return Liar::Vector2f(m_x - rhs.m_x, m_y - rhs.m_y);
	}

	Liar::Vector2f Vector2f::operator+(const Vector2f& rhs) const
	{
		return Liar::Vector2f(m_x + rhs.m_x, m_y + rhs.m_y);
	}

	Liar::Vector2f& Vector2f::operator-=(const Vector2f& rhs)
	{
		m_x -= rhs.m_x;
		m_y -= rhs.m_y;
		return *this;
	}

	Liar::Vector2f& Vector2f::operator+=(const Vector2f& rhs)
	{
		m_x += rhs.m_x;
		m_y += rhs.m_y;
		return *this;
	}

	Liar::Vector2f Vector2f::operator*(const Liar::NAVDTYPE scale) const
	{
		return Liar::Vector2f(m_x*scale, m_y*scale);
	}

	Liar::Vector2f& Vector2f::operator*=(const Liar::NAVDTYPE scale)
	{
		m_x *= scale;
		m_y *= scale;
		return *this;
	}

	Liar::Vector2f Vector2f::operator*(const Vector2f& rhs) const
	{
		return Liar::Vector2f(rhs.m_x*m_x, rhs.m_y*m_y);
	}

	Liar::Vector2f& Vector2f::operator*=(const Vector2f& rhs)
	{
		m_x *= rhs.m_x;
		m_y *= rhs.m_y;
		return *this;
	}

	Liar::Vector2f Vector2f::operator/(const Liar::NAVDTYPE scale) const
	{
		return Liar::Vector2f(m_x/scale, m_y/scale);
	}

	Liar::Vector2f& Vector2f::operator/=(const Liar::NAVDTYPE scale)
	{
		m_x /= scale;
		m_y /= scale;
		return *this;
	}

	bool Vector2f::operator==(const Liar::Vector2f& rhs) const
	{
		return m_x == rhs.m_x && m_y == rhs.m_y;
	}

	bool Vector2f::operator==(const Liar::NAVDTYPE value) const
	{
		return m_x == value && m_y == value;
	}

	bool Vector2f::operator!=(const Liar::Vector2f& rhs) const
	{
		return m_x != rhs.m_x || m_y != rhs.m_y;
	}

	bool Vector2f::operator!=(const Liar::NAVDTYPE value)
	{
		return m_x != value || m_y != value;
	}

	bool Vector2f::operator<=(const Liar::Vector2f& rhs) const
	{
		return m_x <= rhs.m_x && m_y <= rhs.m_y;
	}

	bool Vector2f::operator>=(const Liar::Vector2f& rhs) const
	{
		return m_x >= rhs.m_x && m_y >= rhs.m_y;
	}

	bool Vector2f::operator<(const Liar::Vector2f& rhs) const
	{
		return m_x < rhs.m_x && m_y < rhs.m_y;
	}

	bool Vector2f::operator>(const Liar::Vector2f& rhs) const
	{
		return m_x > rhs.m_x && m_y > rhs.m_y;
	}

	Liar::NAVDTYPE Vector2f::operator[](size_t index) const
	{
		return (&m_x)[index];
	}

	Liar::NAVDTYPE & Vector2f::operator[](size_t index)
	{
		return (&m_x)[index];
	}

	Liar::Vector2f operator*(const Liar::NAVDTYPE scale, const Liar::Vector2f& rhs)
	{
		return Liar::Vector2f(scale*rhs.m_x, scale*rhs.m_y);
	}

	std::ostream & operator<<(std::ostream & os, const Liar::Vector2f& rhs)
	{
		os << "(" << rhs.m_x << ", " << rhs.m_y << ")";
		return os;
	}
}
