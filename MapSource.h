
#ifndef __MAPSOURCE_H__
#define __MAPSOURCE_H__

#include "Vector2f.h"

#ifdef  EditorMod  
#define  MAPSOURCE_API _declspec(dllexport)  
#else  
#define  MAPSOURCE_API _declspec(dllimport)  
#endif 

namespace Liar
{
	class Map;

#ifdef EditorMod
	class MAPSOURCE_API MapSource
#else
	class MapSource
#endif // EditorMod
	{
	public:
		MapSource(const Liar::Map*);
		~MapSource();

	protected:
		const Liar::Map* m_map;
		Liar::Uint* m_pointIndices;
		Liar::Uint m_numberPoints;

	public:
		void Set(const Liar::Map*);
		void Set(const Liar::MapSource&);
		void AddPointIndex(Liar::Uint);
		Liar::Uint GetPointIndex(Liar::Uint) const;
		Liar::Vector2f* GetVertex(Liar::Uint) const;
		Liar::Uint GetNumPoints() const { return m_numberPoints; };
	};
}

#endif // !__MAPSOURCE_API__