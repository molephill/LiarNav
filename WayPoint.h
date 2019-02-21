
#ifndef __WAYPOINT_H__
#define __WAYPOINT_H__

#ifdef  EditorMod  
#define  WAYPOINT_API _declspec(dllexport)  
#else  
#define  WAYPOINT_API _declspec(dllimport)  
#endif 

#include "Cell.h"

namespace Liar
{
#ifdef EditorMod
	class CELL_API WayPoint
#else
	class WayPoint
#endif // EditorMod
	{
	public:
		WayPoint();
		~WayPoint();

	private:
		Liar::Vector2f* m_position;
		Liar::Vector2f* m_lineAPointA;
		Liar::Vector2f* m_lineAPointB;
		Liar::Vector2f* m_lineBPointA;
		Liar::Vector2f* m_lineBPointB;
		Liar::Cell* m_caller;

	public:
		void Init();
		void Set(Liar::Cell*, const Liar::Vector2f&);
		Liar::Vector2f& GetPos() const { return *m_position; };
		Liar::Cell* GetCaller() const { return m_caller; };
		void SetPos(const Liar::Vector2f&) const;
		void SetPos(Liar::NAVDTYPE, Liar::NAVDTYPE) const;

		void GetFurthestWayPoint(Liar::Cell**, int, Liar::NAVDTYPE, Liar::NAVDTYPE, bool = true, Liar::NAVDTYPE = Liar::EPSILON);

	private:
		void SetLineA(const Liar::Vector2f&, const Liar::Vector2f&);
		void SetLineB(const Liar::Vector2f&, const Liar::Vector2f&);
	};
}

#endif //!__WAYPOINT_H__