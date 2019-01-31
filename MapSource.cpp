#include "MapSource.h"
#include "Map.h"

namespace Liar
{
	MapSource::MapSource(const Liar::Map* map):
		m_map(map),
		m_pointIndices(nullptr), m_numberPoints(0)
	{
	}


	MapSource::~MapSource()
	{
		m_map = nullptr;

		if (m_pointIndices)
		{
			free(m_pointIndices);
			m_pointIndices = nullptr;
		}
		m_numberPoints = 0;
	}

	void MapSource::Set(const Liar::Map * map)
	{
		m_map = map;
		m_pointIndices = nullptr;
		m_numberPoints = 0;
	}

	void MapSource::Set(const Liar::MapSource& source)
	{
		if (m_pointIndices)
		{
			free(m_pointIndices);
			m_pointIndices = nullptr;
		}

		m_map = source.m_map;
		m_numberPoints = source.m_numberPoints;
		if (m_numberPoints > 0)
		{
			size_t size = sizeof(Liar::Uint)*m_numberPoints;
			m_pointIndices = (Liar::Uint*)malloc(size);
			memcpy(m_pointIndices, source.m_pointIndices, size);
		}
	}

	void MapSource::AddPointIndex(Liar::Uint index)
	{
		m_numberPoints++;
		size_t size = sizeof(Liar::Uint)*m_numberPoints;
		if (!m_pointIndices) m_pointIndices = (Liar::Uint*)malloc(size);
		else m_pointIndices = (Liar::Uint*)realloc(m_pointIndices, size);
		m_pointIndices[m_numberPoints - 1] = index;
	}

	Liar::Vector2f* MapSource::GetVertex(Liar::Uint index) const
	{
		if (index >= m_numberPoints) return nullptr;
		return m_map->GetVertex(m_pointIndices[index]);
	}

	Liar::Uint MapSource::GetPointIndex(Liar::Uint index) const
	{
		if (index >= m_numberPoints) return m_numberPoints;
		return m_pointIndices[index];
	}
}
