
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
		bool Set(Liar::Map const*, Liar::Uint, Liar::Uint, Liar::Uint);
		void Set(const Liar::Cell&);
		void SetIndex(Liar::Int);
		void CheckAndLink(Liar::Cell&);
		bool CheckAllLink() const;
		Liar::Int GetLink(Liar::Uint) const;
		Liar::NAVDTYPE GetWallDistance(int side) { return m_wallDistance[side]; };
		int SetAndGetArrivalWall(int);
		void ComputeHeuristic(const Liar::Vector2f&);
		void ComputeHeuristic(Liar::NAVDTYPE, Liar::NAVDTYPE);
		Liar::Int GetIndex() const { return m_index; };
		bool Equals(const Liar::Cell&);
		void Dispose();

#if defined(DEBUG_NIF) || defined(EditorMod)
		void WriteErlang(std::ofstream&);
#endif

	protected:
		bool CalcSides();

	private:
		bool RequestList(const Liar::Vector2f&, const Liar::Vector2f&, Liar::Cell&);
		void SetLink(Liar::TriangleSide, const Liar::Cell&);
		Liar::NAVDTYPE GetWallDistatnce(Liar::TriangleSide) const;
	};
}

#endif // !__CELL_H__