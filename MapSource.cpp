#include "MapSource.h"
#include "Map.h"

namespace Liar
{
	MapSource::MapSource(const Liar::Map* map) :
		m_map(map)
	{
	}

	MapSource::~MapSource()
	{
		Dispose();
	}

	void MapSource::Set(const Liar::Map * map)
	{
		m_map = map;
	}

	void MapSource::Set(const Liar::MapSource& source)
	{
		m_map = source.m_map;
	}

	Liar::Vector2f* MapSource::GetVertex(Liar::Uint index) const
	{
		return m_map->GetVertex(index);
	}

	void MapSource::Dispose()
	{
		m_map = nullptr;
	}

#if defined(DEBUG_NIF) || defined(EditorMod)
	void MapSource::WriteLog(const char* log, bool add, const char* logPath)
	{
		if (add)
		{
			std::ofstream out(logPath, std::ios::app);
			out << log << std::endl;
			out.close();
		}
		else
		{
			std::ofstream out(logPath);
			out << log << std::endl;
			out.close();
		}
	}
#endif
}
