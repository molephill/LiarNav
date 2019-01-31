#include "Vector2D.h"

namespace Liar
{
	template<typename T>
	Vector2D<T>::Vector2D(T x, T y)
	{
		Set(x, y);
	}

	template<typename T>
	Vector2D<T>::Vector2D(const Liar::Vector2D<T>& rhs)
	{
		Set(rhs);
	}

	template<typename T>
	Vector2D<T>::~Vector2D()
	{
	}

	template<typename T>
	void Vector2D<T>::Set(T x, T y)
	{
		m_x = y;
		m_y = y;
	}

	template<typename T>
	void Vector2D<T>::Set(const Liar::Vector2D<T>& rhs)
	{
		Set(rhs.m_x, rhs.m_y);
	}

	template<typename T>
	float Vector2D<T>::Length() const 
	{
		return sqrf(LengthSquared());
	}

	template<typename T>
	T Vector2D<T>::LengthSquared() const
	{
		return m_x * m_x + m_y * m_y;
	}

	template<typename T>
	template<typename T2>
	T2 Vector2D<T>::Distance(const Liar::Vector2D<T>& rhs) const
	{
		return Distance(rhs.m_x, rhs.m_y);
	}

	template<typename T>
	template<typename T2>
	T2 Vector2D<T>::Distance(T x, T y) const
	{
		
		return static_cast<T2>(sqrt(DistanceSqured(x, y)));
	}

	template<typename T>
	T Vector2D<T>::DistanceSqured(const Liar::Vector2D<T>& rhs) const
	{
		return DistanceSqured(rhs.m_x, rhs.m_y);
	}

	template<typename T>
	T Vector2D<T>::DistanceSqured(T x, T y) const
	{
		T dx = x - m_x;
		T dy = y - my;
		return dx * dx + dy * dy;
	}

	template<typename T>
	void Vector2D<T>::Zero()
	{
		m_x = m_y = static_cast<T>(0);
	}

	template<typename T>
	bool Vector2D<T>::Equal(const Liar::Vector2D<T>& rhs, Liar::NAVDTYPE e) const
	{
		return Equal(rhs.m_x, rhs.m_y, e);
	}

	template<typename T>
	bool Vector2D<T>::Equal(T x, T y, Liar::NAVDTYPE) const
	{
		return fabs(m_x - x) < e && fabs(m_y - y) < e;
	}

	template<typename T>
	Liar::Vector2D<T> Vector2D<T>::operator-() const
	{
		return Liar::Vector2D<T>(-m_x, -m_y);
	}

	template<typename T>
	Liar::Vector2D<T> Vector2D<T>::operator-(const Liar::Vector2D<T>& rhs) const
	{
		return Liar::Vector2D<T>(m_x - rhs.m_x, m_y - rhs.m_y);
	}

	template<typename T>
	Liar::Vector2D<T> Vector2D<T>::operator+(const Liar::Vector2D<T>& rhs) const
	{
		return Liar::Vector2D<T>(m_x + rhs.m_x, m_y + rhs.m_y);
	}

	template<typename T>
	Liar::Vector2D<T>& Vector2D<T>::operator-=(const Liar::Vector2D<T>& rhs)
	{
		m_x -= rhs.m_x;
		m_y -= rhs.m_y;
		return *this;
	}

	template<typename T>
	Liar::Vector2D<T>& Vector2D<T>::operator+=(const Liar::Vector2D<T>& rhs)
	{
		m_x += rhs.m_x;
		m_y += rhs.m_y;
		return *this;
	}

	template<typename T>
	Liar::Vector2D<T> Vector2D<T>::operator*(const T scale) const
	{
		return Liar::Vector2D<T>(m_x*scale, m_y*scale);
	}

	template<typename T>
	Liar::Vector2D<T>& Vector2D<T>::operator*=(const T scale)
	{
		m_x *= scale;
		m_y *= scale;
		return *this;
	}

	template<typename T>
	Liar::Vector2D<T> Vector2D<T>::operator*(const Liar::Vector2D<T>& rhs) const
	{
		return Liar::Vector2D<T>(rhs.m_x*m_x, rhs.m_y*m_y);
	}

	template<typename T>
	Liar::Vector2D<T>& Vector2D<T>::operator*=(const Liar::Vector2D<T>& rhs)
	{
		m_x *= rhs.m_x;
		m_y *= rhs.m_y;
		return *this;
	}

	//template<typename T>
	//Liar::Vector2D<T> Vector2D<T>::operator/(const float scale) const
	//{
	//	return Liar::Vector2D<T>(m_x);
	//}

	//template<typename T>
	//Liar::Vector2D<T>& Vector2D<T>::operator/=(const float)
	//{
	//	// TODO: 在此处插入 return 语句
	//}

	template<typename T>
	bool Vector2D<T>::operator==(const Liar::Vector2D<T>& rhs) const
	{
		return m_x == rhs.m_x && m_y == rhs.m_y;
	}

	template<typename T>
	bool Vector2D<T>::operator==(const T value) const
	{
		return m_x == value && m_y == value;
	}

	template<typename T>
	bool Vector2D<T>::operator!=(const Liar::Vector2D<T>& rhs) const
	{
		return m_x != rhs.m_x || m_y != rhs.m_y;
	}

	template<typename T>
	bool Vector2D<T>::operator!=(const T value)
	{
		return m_x != value || m_y != value;
	}

	template<typename T>
	bool Vector2D<T>::operator<=(const Liar::Vector2D<T>& rhs) const
	{
		return m_x <= rhs.m_x && m_y <= rhs.m_y;
	}

	template<typename T>
	bool Vector2D<T>::operator>=(const Liar::Vector2D<T>& rhs) const
	{
		return m_x >= rhs.m_x && m_y >= rhs.m_y;
	}

	template<typename T>
	bool Vector2D<T>::operator<(const Liar::Vector2D<T>& rhs) const
	{
		return m_x < rhs.m_x && m_y < rhs.m_y;
	}

	template<typename T>
	bool Vector2D<T>::operator>(const Liar::Vector2D<T>& rhs) const
	{
		return m_x > rhs.m_x && m_y > rhs.m_y;
	}

	template<typename T>
	T Vector2D<T>::operator[](size_t index) const
	{
		return (&m_x)[index];
	}

	template<typename T>
	T & Vector2D<T>::operator[](size_t index)
	{
		return (&m_x)[index];
	}

	/*template<typename T>
	Liar::Vector2D<T> operator*(const float, const Liar::Vector2D<T>&)
	{
		return Liar::Vector2D<T>();
	}*/

	template<typename T>
	std::ostream & operator<<(std::ostream & os, const Liar::Vector2D<T>& rhs)
	{
		os << "(" << rhs.m_x << ", " << rhs.m_y << ")";
		return os;
	}

}
