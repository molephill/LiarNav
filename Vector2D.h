
#ifndef  __VECTOR2D_H__
#define  __VECTOR2D_H__

#ifdef  EditorMod  
#define  VECTOR2D_API _declspec(dllexport)  
#else  
#define  VECTOR2D_API _declspec(dllimport)  
#endif 

#include <iostream>
#include <math.h>

#include "Define.h"

namespace Liar
{
	template<typename T>
	class Vector2D
	{
	public:
		Vector2D(T, T);
		Vector2D(const Liar::Vector2D<T>&);
		~Vector2D();

	protected:
		T m_x;
		T m_y;

	public:
		void Set(T, T);
		void Set(const Liar::Vector2D<T>&);
		float Length() const;
		T LengthSquared() const;
		template<typename T2>
		T2 Distance(const Liar::Vector2D<T>&) const;
		template<typename T2>
		T2 Distance(T, T) const;
		T DistanceSqured(const Liar::Vector2D<T>&) const;
		T DistanceSqured(T, T) const;
		void Zero();
		bool Equal(const Liar::Vector2D<T>&, Liar::NAVDTYPE = Liar::EPSILON) const;
		bool Equal(T, T, Liar::NAVDTYPE = Liar::EPSILON) const;
		Liar::Vector2D<T> operator-() const;
		Liar::Vector2D<T> operator-(const Liar::Vector2D<T>&) const;
		Liar::Vector2D<T> operator+(const Liar::Vector2D<T>&) const;
		Liar::Vector2D<T>& operator-=(const Liar::Vector2D<T>&);
		Liar::Vector2D<T>& operator+=(const Liar::Vector2D<T>&);
		Liar::Vector2D<T> operator*(const T) const;
		Liar::Vector2D<T>& operator*=(const T);
		Liar::Vector2D<T> operator*(const Liar::Vector2D<T>&) const;
		Liar::Vector2D<T>& operator*=(const Liar::Vector2D<T>&);
		/*Liar::Vector2D<T> operator/(const T) const;
		Liar::Vector2D<T>& operator/=(const float);*/
		bool operator==(const Liar::Vector2D<T>&) const;
		bool operator==(const T) const;
		bool operator!=(const Liar::Vector2D<T>&) const;
		bool operator!=(const T);
		bool operator<=(const Liar::Vector2D<T>&) const;
		bool operator>=(const Liar::Vector2D<T>&) const;
		bool operator<(const Liar::Vector2D<T>&) const;
		bool operator>(const Liar::Vector2D<T>&) const;
		T operator[](size_t) const;
		T& operator[](size_t);

		T GetX() const { return m_x; };
		T GetY() const { return m_y; };

	public:
		//friend Liar::Vector2D operator*(const float, const Liar::Vector2D<T>&);
		friend std::ostream& operator<<(std::ostream&, const Liar::Vector2D<T>&);
	};
}

#endif // ! __VECTOR2D_H__
