
#ifndef __DELAUNAY_H__
#define __DELAUNAY_H__

#include "Map.h"
#include "Cell.h"

#ifdef  EditorMod  
#define  DELAUNAY_API _declspec(dllexport)  
#else  
#define  DELAUNAY_API _declspec(dllimport)  
#endif 

namespace Liar
{
#ifdef EditorMod
	class DELAUNAY_API Delaunay
#else
	class Delaunay
#endif // EditorMod
	{
	private:
		class Circle;

	public:
		Delaunay();
		~Delaunay();

	private:
		Liar::Line2d** m_line2ds;
		Liar::Uint m_totalLines;
		Liar::Uint m_curNumLines;

	public:
		void Set(Liar::Map&, bool = true, Liar::Uint = 0);

	private:
		void BuildEdges(Liar::Map&);
		void BuildTrianges(Liar::Map&, bool = true, Liar::Uint = 0);
		Liar::Line2d& GetBoundEdage(Liar::Map&, bool = true, Liar::Uint = 0);

		Liar::Int FindDT(Liar::Map&, const Liar::Line2d&, bool = true);

		bool IsVisiblePointOfLine(Liar::Map&, Liar::Uint, const Liar::Line2d&, bool = true);
		bool IsVisibleIn2Point(Liar::Map&, Liar::Uint, Liar::Uint);

		void CircumCircle(const Liar::Vector2f&, const Liar::Vector2f&, const Liar::Vector2f&, Liar::Delaunay::Circle& cir);
		void CircleBounds(const Liar::Delaunay::Circle&, Liar::NAVDTYPE*);

		Liar::NAVDTYPE LineAngle(const Liar::Vector2f&, const Liar::Vector2f&, const Liar::Vector2f&);

		void RemovePosLine(Liar::Line2d** lv, int&, int pos);
		Liar::Int FindLinePos(const Liar::Vector2f&, const Liar::Vector2f&, const Liar::Line2d**, Liar::Uint);
		void LinkCells(Liar::Map&, bool = true);
	};

	/**
	* Բ
	* @author liar
	*/
	class Delaunay::Circle
	{
	public:
		Circle();
		~Circle();

	public:
		void Set(const Liar::Vector2f&, Liar::NAVDTYPE);
		void Set(Liar::NAVDTYPE, Liar::NAVDTYPE, Liar::NAVDTYPE);

	public:
		Liar::Vector2f* center;
		Liar::NAVDTYPE r;
	};
}

#endif  // !__DELAUNAY_H__