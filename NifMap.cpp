#include "NifMap.h"

namespace Liar
{
	NifMap::NifMap() :
		m_bid(0),
		m_mapList(nullptr), m_mapcount(0),
		m_isLocked(false),
		m_navMesh(nullptr)
	{
	}


	NifMap::~NifMap()
	{
		Liar::Uint i = 0;

		if (m_mapList)
		{
			for (i = 0; i < m_mapcount; ++i)
			{
				m_mapList[i]->~Map();
				free(m_mapList[i]);
				m_mapList[i] = nullptr;
			}
			free(m_mapList);
			m_mapList = nullptr;
			m_mapcount = 0;
		}

		DisposeNavMesh();
	}

	void NifMap::DisposeNavMesh()
	{
		if (m_navMesh)
		{
			m_navMesh->~NavMesh();
			free(m_navMesh);
			m_navMesh = nullptr;
		}
	}

	// from erlang to nav_type
	bool NifMap::ReadErlangCPType(ErlNifEnv* env, ERL_NIF_TERM term, NAVDTYPE& out)
	{
		double tmpD = 0.0;
		if (enif_get_double(env, term, &tmpD))
		{
			out = static_cast<Liar::NAVDTYPE>(tmpD);
			return true;
		}
		int tmpInt = 0;
		if (enif_get_int(env, term, &tmpInt))
		{
			out = static_cast<NAVDTYPE>(tmpInt);
			return true;
		}
		return false;
	}

	bool NifMap::ParseErlangTerm(ErlNifEnv* env, ERL_NIF_TERM Info, bool isWall)
	{
		unsigned int len = 0;
		enif_get_list_length(env, Info, &len);
		if (len <= 0) return false;

		ERL_NIF_TERM head, tail;
		ERL_NIF_TERM subHead, subTail;
		int arity = 0;
		NAVDTYPE x = 0.0, y = 0.0;
		const ERL_NIF_TERM* termArray;

		Liar::Map& map = AutoAddMap();
		Liar::Uint pointIndex = 0;
		Liar::Polygon* polygon = nullptr;

		while (enif_get_list_cell(env, Info, &head, &tail))
		{
			if (isWall) polygon = &(map.AutoAddPolygon());
			else polygon = nullptr;

			while (enif_get_list_cell(env, head, &subHead, &subTail))
			{
				if (enif_get_tuple(env, subHead, &arity, &termArray))
				{
					if (!NifMap::ReadErlangCPType(env, termArray[0], x) || !NifMap::ReadErlangCPType(env, termArray[1], y)) 
					{
						map.DisposePolygon(polygon);
						return false;
					}

					if (!isWall && !polygon) polygon = CheckAutoAddPolygon(x, y);

					if (polygon)
					{
						pointIndex = map.AddVertex(x, y);
						polygon->AddPointIndex(pointIndex);
					}
				}
				else
				{
					map.DisposePolygon(polygon);
					return false;
				}

				head = subTail;
			}

			Info = tail;
		}

		return true;
	}

	void NifMap::Set(Liar::Int bid)
	{
		m_mapcount = 0;
		m_bid = bid;
		m_mapList = nullptr;
		m_isLocked = false;

		m_navMesh = (Liar::NavMesh*)malloc(sizeof(Liar::NavMesh));
		m_navMesh->Init(nullptr);
	}

	bool NifMap::CanWalk(Liar::NAVDTYPE x, Liar::NAVDTYPE y)
	{
		for (Liar::Uint i = 0; i < m_mapcount; ++i)
		{
			Liar::Map* map = m_mapList[i];
			if (map->InMap(x, y))
			{
				return map->CanWalk(x, y);
			}
		}

		return false;
	}

	Liar::Vector2f** NifMap::FindPath(NAVDTYPE startX, NAVDTYPE startY, NAVDTYPE endX, NAVDTYPE endY, Liar::Uint& count)
	{
#ifndef EditorMod
		if (m_isLocked) return nullptr;
		m_isLocked = true;
#endif // !EditorMod

		Liar::Map* map = nullptr;
		for (Liar::Uint i = 0; i < m_mapcount; ++i)
		{
			map = m_mapList[i];
			if (map->InMap(startX, startY) && map->InMap(endX, endY))
			{
				m_navMesh->Set(map);
				Liar::Vector2f** out = m_navMesh->FindPath(startX, startY, endX, endY, count);
				return out;
			}
		}

		return nullptr;
	}

	void NifMap::DestoryLast()
	{
		--m_mapcount;
		if (m_mapcount >= 0)
		{
			m_mapList[m_mapcount]->~Map();
			free(m_mapList[m_mapcount]);
			m_mapList[m_mapcount] = nullptr;
		}
		else
		{
			m_mapcount = 0;
		}
	}

	// 自动增加一块场景数据
	Liar::Map& NifMap::AutoAddMap()
	{
		++m_mapcount;
		size_t blockSize = sizeof(Liar::Map*)*m_mapcount;
		if (m_mapList) m_mapList = (Liar::Map**)realloc(m_mapList, blockSize);
		else m_mapList = (Liar::Map**)malloc(blockSize);
		Liar::Map* map = (Liar::Map*)malloc(sizeof(Liar::Map));
		map->Init();
		m_mapList[m_mapcount - 1] = map;
		return *map;
	}

	void NifMap::CheckAddPolygon(Vector2f* v, int vecIndex)
	{
		for (Liar::Uint i = 0; i < m_mapcount; ++i)
		{
			if (m_mapList[i]->InMap(v[0].GetX(), v[0].GetY()))
			{
				m_mapList[i]->AddPolygon(v, vecIndex);
				return;
			}
		}
	}

	Liar::Polygon* NifMap::CheckAutoAddPolygon(Liar::NAVDTYPE x, Liar::NAVDTYPE y)
	{
		for (Liar::Uint i = 0; i < m_mapcount; ++i)
		{
			if (m_mapList[i]->InMap(x, y))
			{
				return &(m_mapList[i]->AutoAddPolygon());
			}
		}
		return nullptr;
	}

	void NifMap::CalcAllMapBound()
	{
		for (Liar::Uint i = 0; i < m_mapcount; ++i)
		{
			m_mapList[i]->CalcBound();
		}
	}

	int NifMap::BuildByList(ErlNifEnv* env, ERL_NIF_TERM wall, ERL_NIF_TERM block, bool isCW)
	{
		if(!ParseErlangTerm(env, wall)) return -1;
		CalcAllMapBound();
		ParseErlangTerm(env, block, false);
		return BuildAll(isCW);
	}

	int NifMap::BuildByList(ErlNifEnv* env, ERL_NIF_TERM Info, bool isCW)
	{
		if(!ParseErlangTerm(env, Info)) return -1;
		CalcAllMapBound();
		return Liar::Delaunay::Set(*(m_mapList[m_mapcount - 1]), isCW);
	}

	int NifMap::BuildAll(bool cw)
	{
		int cellCount = 0;
		for (Liar::Uint i = 0; i < m_mapcount; ++i)
		{
			cellCount += BuildMapByIndex(i, cw);
		}
		return cellCount;
	}

	// 构建场景
	int NifMap::BuildMapByIndex(Liar::Uint index, bool cw)
	{
		if (index >= m_mapcount)
		{
			return 0;
		}
		return Liar::Delaunay::Set(*(m_mapList[index]), cw);
	}

	// 给场景加数据
	bool NifMap::AddPologyMapByIndex(int index, Vector2f* v, Liar::Uint count)
	{
		if (static_cast<Liar::Uint>(index) >= m_mapcount) return false;
		m_mapList[index]->AddPolygon(v, count);
		return true;
	}

#ifdef EditorMod
	void NifMap::GetBound(Liar::NAVDTYPE& minX, Liar::NAVDTYPE& minY, Liar::NAVDTYPE& maxX, Liar::NAVDTYPE& maxY)
	{
		for (Liar::Uint i = 0; i < m_mapcount; ++i)
		{
			Liar::Map* map = m_mapList[i];
			Liar::NAVDTYPE curMinX = map->GetMinX();
			Liar::NAVDTYPE curMinY = map->GetMinY();
			Liar::NAVDTYPE curMaxX = map->GetMaxX();
			Liar::NAVDTYPE curMaxY = map->GetMaxY();
			if (minX > curMinX) minX = curMinX;
			if (minY > curMinY) minY = curMinY;
			if (maxX < curMaxX) maxX = curMaxX;
			if (maxY < curMaxY) maxY = curMaxY;
		}
	}
#endif // EditorMod

#if defined(DEBUG_NIF) || defined(EditorMod)
	void NifMap::WriteErlang(const char* base)
	{
		char str[512];
		if (!base)
		{
			sprintf_s(str, "%d.txt", m_bid);
		}
		else
		{
			std::string tmp(base);
			tmp += "\\%d.txt";
			sprintf_s(str, tmp.c_str(), m_bid);
		}
		std::ofstream outfile(str, std::ios::ate);
		if (!outfile)
		{
			printf("can not open file to write\n");
			return;
		}

		for (Liar::Uint i = 0; i < m_mapcount; ++i)
		{
			Liar::Map& mapInfo = *(m_mapList[i]);
			mapInfo.WriteErlang(outfile);
		}
		outfile.close();
	}
#endif
}
