
#ifndef __MAPSOURCE_H__
#define __MAPSOURCE_H__

#include "Vector2f.h"

#if defined(DEBUG_NIF) || defined(EditorMod)
#include <fstream>
#include <iostream>
#endif // DEBUG_NIF

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

	public:
		void Set(const Liar::Map*);
		void Set(const Liar::MapSource&);
		Liar::Vector2f* GetVertex(Liar::Uint) const;
	};
}

#endif // !__MAPSOURCE_API__