
#ifndef __WAYPOINT_H__
#define __WAYPOINT_H__

#ifdef  EditorMod  
#define  WAYPOINT_API _declspec(dllexport)  
#else  
#define  WAYPOINT_API _declspec(dllimport)  
#endif 

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

	public:
		const Liar::Cell* caller;
	};
}

#endif //!__WAYPOINT_H__