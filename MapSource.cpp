#include "MapSource.h"
#include "Map.h"

namespace Liar
{
	MapSource::MapSource(const Liar::Map* map):
		m_map(map)
	{
	}

	MapSource::~MapSource()
	{
		m_map = nullptr;
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
}
