
#include "NifMap.h"
#include<stdlib.h>

static Liar::NifMap* maps;
static int mapcount = 0;

#ifdef DEBUG_NIF
static int num_log = 0;
static int log_max = 50;
#endif // DEBUG_NIF


static Liar::NifMap* GetMap(int bid)
{
	if (maps)
	{
		for (int i = 0; i < mapcount; ++i)
		{
			if (maps[i].GetBid() == bid)
			{
				return &maps[i];
			}
		}
	}

	return nullptr;
}

static void DestoryMap(int bid)
{
	if (maps)
	{
		int findIndex = mapcount;
		for (int i = 0; i < mapcount; ++i)
		{
			if (maps[i].GetBid() == bid)
			{
				findIndex = i;
				maps[i].~NifMap();
			}

			if (i > findIndex)
			{
				maps[i - 1] = maps[i];
			}
		}

		if (findIndex < mapcount)
		{
			--mapcount;
			if (mapcount <= 0)
			{
				free(maps);
				maps = nullptr;
			}
			//free(&maps[mapcount]);
		}
	}
}

// 项2创建场景的方式
/**
最外层信息：
[
[x,y.... 最外层],
[x,y.... 最外层],
[x,y.... 最外层]...
]

障碍层：
[
[x,y.... 障碍层],
[x,y.... 障碍层],
[x,y.... 障碍层]...
]
*/
static ERL_NIF_TERM mapinfo(ErlNifEnv* env, ERL_NIF_TERM bidTerm, ERL_NIF_TERM wallTerm, ERL_NIF_TERM blockTerm, ERL_NIF_TERM cwTerm)
{
	int bid = 0;
	if (!enif_get_int(env, bidTerm, &bid)) return enif_make_int(env, 0);

	int intCW = 0;
	if (!enif_get_int(env, cwTerm, &intCW)) return enif_make_int(env, 0);

	bool isCW = true;
	if (intCW <= 0) isCW = false;

	Liar::NifMap* map = GetMap(bid);

	if (!map)
	{
		int pre = mapcount++;
		if (maps) maps = (Liar::NifMap*)realloc(maps, sizeof(Liar::NifMap)*mapcount);
		else maps = (Liar::NifMap*)malloc(sizeof(Liar::NifMap)*mapcount);

		maps[pre].Set(bid);
		map = &maps[pre];
	}

	try
	{
		int cellCount = 0;
		if (blockTerm) cellCount = map->BuildByList(env, wallTerm, blockTerm, isCW);
		else cellCount = map->BuildByList(env, wallTerm, isCW);
		if (cellCount <= 0)
		{
			if (cellCount == 0) map->DestoryLast();
			return enif_make_int(env, 0);
		}
		return enif_make_int(env, cellCount);
	}
	catch (char *)
	{
		DestoryMap(bid);
		return enif_make_int(env, 0);
	}
}

// 项1创建场景方式
/** [
[x,y.... 最外层],
[x,y.... 障碍层],
[x,y.... 障碍层]...
]
*/
static ERL_NIF_TERM buildmap(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
{
	switch (Liar::NAV_VERSION)
	{
	case 1:
		return mapinfo(env, argv[0], argv[1], 0, argv[2]);
	case 2:
		return mapinfo(env, argv[0], argv[1], argv[2], argv[3]);
	default:
		return enif_make_int(env, 0);
	}
}

static ERL_NIF_TERM destory(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
{
	int bid = 0;
	if (!enif_get_int(env, argv[0], &bid)) return enif_make_string(env, "failed 1111!", ERL_NIF_LATIN1);

	DestoryMap(bid);

	return enif_make_string(env, "delete suc!!!", ERL_NIF_LATIN1);
}

static ERL_NIF_TERM findpath(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
{
	ERL_NIF_TERM res = enif_make_list(env, 0);

	int bid = 0;
	if (!enif_get_int(env, argv[0], &bid)) return res;

	Liar::NifMap* map = GetMap(bid);

	if (!map) return res;

#ifdef DEBUG_NIF
	++num_log;
	bool add = num_log > log_max ? false : true;
	Liar::MapSource::WriteLog("start_find_path", add);
	char str[512];
	sprintf_s(str, "bid:%d", bid);
	Liar::MapSource::WriteLog(str);
#endif // DEBUG_NIF

	Liar::NAVDTYPE startx = Liar::ZERO;
	if (!Liar::NifMap::ReadErlangCPType(env, argv[1], startx)) return res;
	Liar::NAVDTYPE starty = Liar::ZERO;
	if (!Liar::NifMap::ReadErlangCPType(env, argv[2], starty)) return res;
	Liar::NAVDTYPE endx = Liar::ZERO;
	if (!Liar::NifMap::ReadErlangCPType(env, argv[3], endx)) return res;
	Liar::NAVDTYPE endy = Liar::ZERO;
	if (!Liar::NifMap::ReadErlangCPType(env, argv[4], endy)) return res;

#ifdef DEBUG_NIF
	sprintf_s(str, "sx:%f,sy:%f,ex:%f,ey:%f", startx, starty, endx, endy);
	Liar::MapSource::WriteLog(str);
#endif // DEBUG_NIF
	try
	{
		Liar::NavMesh* navMesh = nullptr;
#ifndef ShareFind
		navMesh = (Liar::NavMesh*)malloc(sizeof(Liar::NavMesh));
		navMesh->Init(nullptr);
#endif // 

		Liar::Uint count = 0;
		Liar::Vector2f** path = map->FindPath(startx, starty, endx, endy, count, navMesh);
#ifdef DEBUG_NIF
		sprintf_s(str, "path_count:%d", count);
		Liar::MapSource::WriteLog(str);
#endif // DEBUG_NIF
		if (count > 0)
		{
			int lastIndex = 0;
#ifdef PASS_FIRST
			lastIndex = 1;
#endif // PASS_FIRST
			for (int i = count - 1; i >= lastIndex; --i)
			{
				res = enif_make_list_cell(env, enif_make_double(env, path[i]->GetY()), res);
				res = enif_make_list_cell(env, enif_make_double(env, path[i]->GetX()), res);
			}
		}

#ifndef ShareFind
		navMesh->~NavMesh();
		free(m_navMesh);
		navMesh = nullptr;
#endif // !ShareFind

	}
	catch (char *)
	{
#ifdef DEBUG_NIF
		Liar::MapSource::WriteLog("find_error");
#endif
	}

	return res;
}

static ERL_NIF_TERM can_walk(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
{
	int bid = 0;
	if (!enif_get_int(env, argv[0], &bid)) return enif_make_int(env, -1);

	Liar::NifMap* map = GetMap(bid);

	if (!map) return enif_make_int(env, -2);

	Liar::NAVDTYPE startx = Liar::ZERO;
	Liar::NAVDTYPE starty = Liar::ZERO;

	if (!Liar::NifMap::ReadErlangCPType(env, argv[1], startx)) return enif_make_int(env, -3);
	if (!Liar::NifMap::ReadErlangCPType(env, argv[2], starty)) return enif_make_int(env, -4);

	if (map->CanWalk(startx, starty))
	{
		return enif_make_int(env, 1);
	}
	else
	{
		return enif_make_double(env, startx);
	}
}

static ERL_NIF_TERM free(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
{
	Liar::Delaunay::Dispose();
	return enif_make_int(env, 0);
}

// { "createmap", 3, createmap },
static ErlNifFunc nif_funcs[] = {
	{ "buildmap", 3, buildmap },
	{ "buildmap", 4, buildmap },
	{ "destory", 1, destory },
	{ "findpath", 5, findpath },
	{ "can_walk", 3, can_walk },
	{ "free", 0, free }
};

ERL_NIF_INIT(nifmaps, nif_funcs, nullptr, nullptr, nullptr, nullptr)