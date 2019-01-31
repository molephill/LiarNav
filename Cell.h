
#ifndef __CELL_H__
#define __CELL_H__

#include "Triangle.h"

#ifdef  EditorMod  
#define  CELL_API _declspec(dllexport)  
#else  
#define  CELL_API _declspec(dllimport)  
#endif 

namespace Liar
{
#ifdef EditorMod
	class CELL_API Cell :
		public Triangle
#else
	class Cell :
		public Triangle
#endif // EditorMod
	{
	public:
		Cell(Liar::Map const*);
		~Cell();

	private:
		Liar::Int m_index;
		Liar::Int* m_links;
		Liar::NAVDTYPE* m_wallDistance;

	public:
		Liar::Int sessionId;
		Liar::NAVDTYPE f;
		Liar::NAVDTYPE h;
		bool isOpen;
		Liar::Cell* parent;
		Liar::Int arrivalWall;
		Liar::Int checkLinkCount;

	public:
		void Set(Liar::Map const*);
		bool Set(Liar::Uint, Liar::Uint, Liar::Uint);
		void Set(const Liar::Cell&);
		bool Set(const Liar::Map*, Liar::Uint, Liar::Uint, Liar::Uint);
		void SetIndex(Liar::Int);
		void CheckAndLink(Liar::Cell&);
		bool CheckAllLink() const;
		Liar::Int GetLink(Liar::Uint) const;

		void ComputeHeuristic(const Liar::Vector2f&);
		void ComputeHeuristic(Liar::NAVDTYPE, Liar::NAVDTYPE);

		Liar::NAVDTYPE GetWallDistance(int side) { return m_wallDistance[side]; };

		Liar::Int GetIndex() const { return m_index; };
		int SetAndGetArrivalWall(int);

	protected:
		bool CalcInit();

	private:
		bool RequestList(const Liar::Vector2f&, const Liar::Vector2f&, Liar::Cell&);
		void SetLink(Liar::TriangleSide, const Liar::Cell&);
		Liar::NAVDTYPE GetWallDistatnce(Liar::TriangleSide) const;
		int SetAndGetArrivalWall(int);
		void ComputeHeuristic(const Liar::Vector2f&);
	};
}

#endif // !__CELL_H__