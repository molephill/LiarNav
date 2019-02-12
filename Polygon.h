
#ifndef __POLYGON_H__
#define __POLYGON_H__

#include "MapSource.h"

#ifdef  EditorMod  
#define  POLYGON_API _declspec(dllexport)  
#else  
#define  POLYGON_API _declspec(dllimport)  
#endif 

namespace Liar
{
#ifdef EditorMod
	class POLYGON_API Polygon :public Liar::MapSource
#else
	class Polygon:public Liar::MapSource
#endif // EditorMod
	{
	public:
		Polygon(Liar::Map const*);
		~Polygon();

	private:
		Liar::NAVDTYPE* m_rect;
		Liar::Uint m_isClockWise;

	private:
		Liar::NAVDTYPE Multiply(const Vector2f& sp, const Vector2f& ep, const Vector2f& op);

	public:
		void Set(Liar::Map const*);
		void Set(const Liar::Polygon&);
		bool IsCW();
		void AutoCW(bool = true);
		Liar::NAVDTYPE* Rectangle(bool = false);
		Liar::NAVDTYPE* GetRect() const { return m_rect; };
	};
}

#endif // !__POLYGON_H__