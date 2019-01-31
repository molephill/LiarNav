
#ifndef  __VECTOR2F_H__
#define  __VECTOR2F_H__

#ifdef  EditorMod  
#define  VECTOR2F_API _declspec(dllexport)  
#else  
#define  VECTOR2F_API _declspec(dllimport)  
#endif

#include <iostream>
#include <math.h>

#include "Define.h"

namespace Liar
{
#ifdef EditorMod
	class VECTOR2F_API Vector2f
#else
	class Vector2f
#endif // EditorMod
	{
	public:
		Vector2f(Liar::NAVDTYPE = 0.0f, Liar::NAVDTYPE = 0.0f);
		Vector2f(const Liar::Vector2f&);
		~Vector2f();

	protected:
		Liar::NAVDTYPE m_x;
		Liar::NAVDTYPE m_y;

	public:
		void Set(Liar::NAVDTYPE, Liar::NAVDTYPE);
		void Set(const Liar::Vector2f&);
		Liar::NAVDTYPE Length() const;
		Liar::NAVDTYPE LengthSquared() const;
		Liar::NAVDTYPE Distance(const Liar::Vector2f&) const;
		Liar::NAVDTYPE Distance(Liar::NAVDTYPE, Liar::NAVDTYPE) const;
		Liar::NAVDTYPE DistanceSqured(const Liar::Vector2f&) const;
		Liar::NAVDTYPE DistanceSqured(Liar::NAVDTYPE, Liar::NAVDTYPE) const;
		void Zero();
		bool Equals(const Liar::Vector2f&, Liar::NAVDTYPE = Liar::EPSILON) const;
		bool Equals(Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE = Liar::EPSILON) const;
		Liar::Vector2f operator-() const;
		Liar::Vector2f operator-(const Liar::Vector2f&) const;
		Liar::Vector2f operator+(const Liar::Vector2f&) const;
		Liar::Vector2f& operator-=(const Liar::Vector2f&);
		Liar::Vector2f& operator+=(const Liar::Vector2f&);
		Liar::Vector2f operator*(const Liar::NAVDTYPE) const;
		Liar::Vector2f& operator*=(const Liar::NAVDTYPE);
		Liar::Vector2f operator*(const Liar::Vector2f&) const;
		Liar::Vector2f& operator*=(const Liar::Vector2f&);
		Liar::Vector2f operator/(const Liar::NAVDTYPE) const;
		Liar::Vector2f& operator/=(const Liar::NAVDTYPE);
		bool operator==(const Liar::Vector2f&) const;
		bool operator==(const Liar::NAVDTYPE) const;
		bool operator!=(const Liar::Vector2f&) const;
		bool operator!=(const Liar::NAVDTYPE);
		bool operator<=(const Liar::Vector2f&) const;
		bool operator>=(const Liar::Vector2f&) const;
		bool operator<(const Liar::Vector2f&) const;
		bool operator>(const Liar::Vector2f&) const;
		Liar::NAVDTYPE operator[](size_t) const;
		Liar::NAVDTYPE& operator[](size_t);

		Liar::NAVDTYPE GetX() const { return m_x; };
		Liar::NAVDTYPE GetY() const { return m_y; };

		friend Liar::Vector2f operator*(const Liar::NAVDTYPE, const Liar::Vector2f&);
		friend std::ostream& operator<<(std::ostream&, const Liar::Vector2f&);

	};
}

#endif // ! __VECTOR2D_H__