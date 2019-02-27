
#ifndef __NIFMAP_H__
#define __NIFMAP_H__

#include "Map.h"
#include "Delaunay.h"
#include "erl_nif.h"

#ifdef  EditorMod  
#define  NIFMAP_API _declspec(dllexport)  
#else  
#define  NIFMAP_API _declspec(dllimport)  
#endif 

namespace Liar
{
#ifdef  EditorMod
	class NIFMAP_API NifMap
#else
	class NifMap
#endif
	{
	public:
		NifMap();
		~NifMap();

	private:
		Liar::Int m_bid;
		Liar::Map** m_mapList;
		Liar::Uint m_mapcount;
		bool m_isLocked;
		// navMesh
		Liar::NavMesh* m_navMesh;

	private:
		bool ParseErlangTerm(ErlNifEnv*, ERL_NIF_TERM, bool = true);
		void CheckAddPolygon(Liar::Vector2f*, int);
		int BuildMapByIndex(Liar::Uint, bool = true);
		void CalcAllMapBound();
		void DisposeNavMesh();

	public:
		static bool ReadErlangCPType(ErlNifEnv*, ERL_NIF_TERM, NAVDTYPE&);

		void Set(Liar::Int);
		bool AddPologyMapByIndex(int, Liar::Vector2f*, Liar::Uint);
		Liar::Map* GetInMap(Liar::NAVDTYPE, Liar::NAVDTYPE);
		Liar::Polygon* CheckAutoAddPolygon(Liar::NAVDTYPE, Liar::NAVDTYPE);
		int BuildAll(bool = true);

		Liar::Vector2f** FindPath(NAVDTYPE startx, NAVDTYPE startY, NAVDTYPE endX, NAVDTYPE endY, Liar::Uint&);
		bool CanWalk(NAVDTYPE, NAVDTYPE);

		int BuildByList(ErlNifEnv*, ERL_NIF_TERM, bool = false);
		int BuildByList(ErlNifEnv*, ERL_NIF_TERM, ERL_NIF_TERM, bool = false);

		Liar::Map& AutoAddMap();
		void DestoryLast();

		Liar::Int GetBid() const { return m_bid; };
		bool Equals(const Liar::NifMap& map) { return map.m_bid == m_bid; };
		bool Equals(Liar::Int bid) { return bid == m_bid; };
		void SetLocked(bool locked) { m_isLocked = locked; };
		bool GetLocked() const { return m_isLocked; };

#ifdef EditorMod
		void GetBound(Liar::NAVDTYPE&, Liar::NAVDTYPE&, Liar::NAVDTYPE&, Liar::NAVDTYPE&);
		Liar::Uint GetMapCount() const { return m_mapcount; };
		Liar::Map* GetMap(size_t index) { return m_mapList[index]; };
		Liar::Cell** GetCrossList() const { return m_navMesh ? m_navMesh->GetCrossCells() : nullptr; };
		Liar::Uint GetCrossCount() const { return m_navMesh ? m_navMesh->GetNumCrossCells() : 0; };
#endif // EditorMod

#if defined(DEBUG_NIF) || defined(EditorMod)
		void WriteErlang(const char* = nullptr);
#endif

	};
}

#endif // !__NIFMAP_H__