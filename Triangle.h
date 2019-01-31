
#ifndef __TRIANGLE_H__
#define __TRIANGLE_H__

#include "Line2d.h"

#ifdef  EditorMod  
#define  TRIANGLE_API _declspec(dllexport)  
#else  
#define  TRIANGLE_API _declspec(dllimport)  
#endif 

namespace Liar
{
#ifdef EditorMod
	class TRIANGLE_API Triangle :
		public Liar::MapSource
#else
	class Triangle :
		public Liar::MapSource
#endif // EditorMod
	{
	public:
		Triangle(Liar::Map const*);
		~Triangle();

	protected:
		Liar::Uint m_pointIndexA;
		Liar::Uint m_pointIndexB;
		Liar::Uint m_pointIndexC;

		Liar::Vector2f* m_center;
		Liar::Line2d** m_sides;

		bool m_centerCalculated;

	public:
		void Set(Liar::Map const*);
		bool Set(Liar::Uint, Liar::Uint, Liar::Uint);
		bool Set(Liar::Map const*, Liar::Uint, Liar::Uint, Liar::Uint);
		void Set(const Liar::Triangle&);

		bool IsPointIn(const Liar::Vector2f&, bool = true);
		bool IsPointIn(Liar::NAVDTYPE, Liar::NAVDTYPE, bool = true);
		
		Liar::Line2d& GetSide(Liar::Uint) const;

		Liar::Vector2f& GetPointA() const;
		Liar::Vector2f& GetPointB() const;
		Liar::Vector2f& GetPointC() const;

	protected:
		bool CalcInit();
	};
}

#endif // !__TRIANGLE_H__