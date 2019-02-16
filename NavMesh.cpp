#include "NavMesh.h"

namespace Liar
{
#ifdef ShareFind
#ifdef FindNearest
	NavMesh::NavMesh() :
		m_cells(nullptr), m_numberCell(0),
		m_openList((Heap*)malloc(sizeof(Heap))), m_closeList(nullptr), m_closeCount(0),
		m_path(nullptr), m_numPath(0),
		m_nearstCells(nullptr), m_nearstCount(0)
#else
	NavMesh::NavMesh() :
		m_cells(nullptr), m_numberCell(0),
		m_openList((Heap*)malloc(sizeof(Heap))), m_closeList(nullptr), m_closeCount(0),
		m_path(nullptr), m_numPath(0)
#endif // FindNearest
#else
#ifdef FindNearest
	NavMesh::NavMesh() :
		m_cells(nullptr), m_numberCell(0),
		m_openList((Heap*)malloc(sizeof(Heap))), m_closeList(nullptr), m_closeCount(0),
		m_path(nullptr), m_numPath(0),
		m_nearstCells(nullptr), m_nearstCount(0),
		m_testCells(nullptr), m_testCount(0),
		m_tmpClosetCell(nullptr), m_pathSessionId(0)
#else
	NavMesh::NavMesh() :
		m_cells(nullptr), m_numberCell(0),
		m_openList((Heap*)malloc(sizeof(Heap))), m_closeList(nullptr), m_closeCount(0),
		m_path(nullptr), m_numPath(0),
		m_testCells(nullptr), m_testCount(0),
		m_tmpClosetCell(nullptr), m_pathSessionId(0)
#endif // FindNearest
#endif // ShareFind
	{
	}

	NavMesh::~NavMesh()
	{
		Dispose();
	}

	void NavMesh::Dispose()
	{
		m_cells = nullptr;
		m_numberCell = 0;

		m_openList->~Heap();
		free(m_openList);
		m_openList = nullptr;
		if (m_closeList)
		{
			free(m_closeList);
			m_closeList = nullptr;
		}

		DisposePath();

#ifndef ShareFind
		if (m_testCells)
		{
			for (Liar::Uint i = 0; i < m_testCount; ++i)
			{
				m_testCells[i]->~Cell();
				free(m_testCells[i]);
				m_testCells[i] = nullptr;
			}
			free(m_testCells);
		}

		if (m_tmpClosetCell)
		{
			m_tmpClosetCell->~Cell();
			free(m_tmpClosetCell);
			m_tmpClosetCell = nullptr;
		}

#endif // !ShareFind


#ifdef EditorMod
		DisposeFindCtr();
#endif // EditorMod
	}

	void NavMesh::Set(Liar::Cell** cells, Liar::Uint numCell)
	{
		m_cells = cells;
		m_numberCell = numCell;

		m_openList = (Heap*)malloc(sizeof(Heap));
		m_openList->Set(numCell);

		m_closeList = nullptr;
		m_closeCount = 0;

		m_path = nullptr;
		m_numPath = 0;

#ifdef FindNearest
		m_nearstCells = nullptr;
		m_nearstCount = 0;
#endif // FindNearest

#ifdef EditorMod
		m_crossCount = 0;
		m_crossList = nullptr;
#endif // EdtorMod

#ifndef ShareFind
		m_testCells = nullptr;
		m_testCount = 0;
		m_tmpClosetCell = nullptr;
#endif // !ShareFind
	}

#ifdef EditorMod
	void NavMesh::DisposeFindCtr()
	{
		if (m_crossList)
		{
			free(m_crossList);
			m_crossList = nullptr;
		}
		m_crossCount = 0;
	}

	void NavMesh::GetCrossInfo(Liar::Cell** crossList, Liar::Uint& crossCount)
	{
		crossList = m_crossList;
		crossCount = m_crossCount;
	}
#endif // EditorMod

	Liar::Vector2f** NavMesh::FindPath(Liar::NAVDTYPE startX, Liar::NAVDTYPE startY, Liar::NAVDTYPE endX, Liar::NAVDTYPE endY, Liar::Uint& outLen, bool rw)
	{

		if (m_numPath >= Liar::NavMesh::PATHMAX) DisposePath();

#ifdef EditorMod
		DisposeFindCtr();
#endif // EditorMod

#ifdef ShareFind
		++NavMesh::PATHSESSIONID;
#else
		++m_pathSessionId;
#endif // ShareFind

		Liar::Cell* startCell = FindClosestCell(startX, startY, rw);
#ifndef OutFind
		if (!startCell) return nullptr;
#endif // !OutFind
		Liar::Cell* endCell = FindClosestCell(endX, endY, rw);
		if (!endCell) return nullptr;

#ifdef OutFind
		if (!startCell)
		{
			AddPathPoint(startX, startY, outLen);
			AddPathPoint(endX, endY, outLen);
			return m_path;
		}
#endif // OutFind

#ifdef ShareFind
		if (startCell == endCell || TestOneLine2D(startX, startY, endX, endY, startCell, endCell))
#else
		if (startCell->Equals(*endCell) || TestOneLine2D(startX, startY, endX, endY, startCell, endCell))
#endif // ShareFind
		{
			AddPathPoint(startX, startY, outLen);
			AddPathPoint(endX, endY, outLen);
			return m_path;
		}
		else
		{
			return BuildPath(startCell, startX, startY, endCell, endX, endY, outLen, rw);
		}
	}

	/**
	* 构建路径
	* @param startCell
	* @param startPos
	* @param endCell
	* @param endPos
	* @return Point路径数组
	*/
	Liar::Vector2f** NavMesh::BuildPath(
		Liar::Cell* startCell, const Liar::Vector2f& startPos,
		Liar::Cell* endCell, const Liar::Vector2f& endPos, 
		Liar::Uint& outLen, bool rw)
	{
		return BuildPath(startCell, startPos.GetX(), startPos.GetY(), endCell, endPos.GetX(), endPos.GetY(), outLen, rw);
	}

	Liar::Vector2f** NavMesh::BuildPath(
		Liar::Cell* startCell, Liar::NAVDTYPE startX, Liar::NAVDTYPE startY,
		Liar::Cell* endCell, Liar::NAVDTYPE endX, Liar::NAVDTYPE endY, 
		Liar::Uint& outLen, bool rw)
	{
		m_openList->Clear();
		m_openList->Push(endCell);
		endCell->f = 0;
		endCell->h = 0;
		endCell->isOpen = false;
		endCell->parent = nullptr;

#ifdef ShareFind
		endCell->sessionId = Liar::NavMesh::PATHSESSIONID;
#else
		endCell->sessionId = m_pathSessionId;
#endif // ShareFind

		bool foundPath = false;			//是否找到路径
		Liar::Cell* currNode = nullptr;		//当前节点
		Liar::Cell* adjacentTmp = nullptr;	//当前节点的邻接三角型
		Liar::Cell* sourceAdjacentTmp = nullptr;

		while (m_openList->Size())
		{
			// 1. 把当前节点从开放列表删除, 加入到封闭列表
			currNode = m_openList->Pop();
			++m_closeCount;
			size_t blockSize = sizeof(Liar::Cell*)*m_closeCount;
			if (m_closeList) m_closeList = (Liar::Cell**)realloc(m_closeList, blockSize);
			else m_closeList = (Liar::Cell**)malloc(blockSize);
			m_closeList[m_closeCount - 1] = currNode;

#ifdef ShareFind
			if (currNode == startCell)
#else
			if (currNode->Equals(*startCell))
#endif // ShareFind
			{
				foundPath = true;
				break;
			}

			// 2. 对当前节点相邻的每一个节点依次执行以下步骤:
			//所有邻接三角型
			int adjacentId = 0;
			for (int i = 0; i < 3; ++i)
			{
				adjacentId = currNode->GetLink(i);
				// 3. 如果该相邻节点不可通行或者该相邻节点已经在封闭列表中,
				//    则什么操作也不执行,继续检验下一个节点;

				if (adjacentId < 0) continue; //不能通过
				else sourceAdjacentTmp = m_cells[adjacentId];

				if (sourceAdjacentTmp)
				{

#ifdef ShareFind
					adjacentTmp = sourceAdjacentTmp;
					if (adjacentTmp->sessionId != NavMesh::PATHSESSIONID)
					{
						// 4. 如果该相邻节点不在开放列表中,则将该节点添加到开放列表中, 
						//    并将该相邻节点的父节点设为当前节点,同时保存该相邻节点的G和F值;
						adjacentTmp->sessionId = NavMesh::PATHSESSIONID;
#else
					adjacentTmp = AddTestCell(sourceAdjacentTmp);
					if (adjacentTmp->sessionId != m_pathSessionId)
					{
						// 4. 如果该相邻节点不在开放列表中,则将该节点添加到开放列表中, 
						//    并将该相邻节点的父节点设为当前节点,同时保存该相邻节点的G和F值;
						adjacentTmp->sessionId = m_pathSessionId;
#endif // ShareFind
						adjacentTmp->parent = currNode;
						adjacentTmp->isOpen = true;

						//H和F值
						adjacentTmp->ComputeHeuristic(startX, startY);
						adjacentTmp->f = currNode->f + adjacentTmp->GetWallDistance(abs(i - currNode->arrivalWall));

						//放入开放列表并排序
						m_openList->Push(adjacentTmp);

						// remember the side this caller is entering from
						adjacentTmp->SetAndGetArrivalWall(currNode->GetIndex());
						if (currNode->arrivalWall < 0)
						{
							currNode->SetAndGetArrivalWall(adjacentTmp->GetIndex());
						}
					}
					else
					{
						// 5. 如果该相邻节点在开放列表中, 
						//    则判断若经由当前节点到达该相邻节点的G值是否小于原来保存的G值,
						//    若小于,则将该相邻节点的父节点设为当前节点,并重新设置该相邻节点的G和F值
						if (adjacentTmp->isOpen)	//已经在openList中
						{
							if (currNode->f + adjacentTmp->GetWallDistance(abs(i - currNode->arrivalWall)) < adjacentTmp->f)
							{
								adjacentTmp->f = currNode->f;
								adjacentTmp->parent = currNode;

								// remember the side this caller is entering from
								adjacentTmp->SetAndGetArrivalWall(currNode->GetIndex());
								if (currNode->arrivalWall < 0)
								{
									currNode->SetAndGetArrivalWall(adjacentTmp->GetIndex());
								}
							}
						}
						else
						{
							//已在closeList中
							//sourceAdjacentTmp = nullptr;
							continue;
						}
					}
				}
			}
		}

		//由网格路径生成Point数组路径
		if (foundPath)
		{
			return GetPath(startX, startY, endX, endY, outLen, rw);
		}
		else
		{
			return nullptr;
		}
	}

	/**
	* 根据经过的三角形返回路径点(下一个拐角点法)
	* @param start
	* @param end
	* @return Point数组
	*/
	Liar::Vector2f** NavMesh::GetPath(const Liar::Vector2f& start, const Liar::Vector2f& end, Liar::Uint& outLen, bool rw)
	{
		return GetPath(start.GetX(), start.GetY(), end.GetX(), end.GetY(), outLen, rw);
	}

	Liar::Vector2f** NavMesh::GetPath(Liar::NAVDTYPE startX, Liar::NAVDTYPE startY, Liar::NAVDTYPE endX, Liar::NAVDTYPE endY, Liar::Uint& outLen, bool rw)
	{
		//经过的三角形
		int cellPathSize = 0;
		Liar::Cell** cellPath = GetCellPath(cellPathSize);

		if (cellPathSize > 0)
		{
			//开始点
			AddPathPoint(startX, startY, outLen);
			//起点与终点在同一三角形中
			if (cellPathSize == 1)
			{
				AddPathPoint(endX, endY, outLen);
			}
			else
			{
				Liar::WayPoint* w = (Liar::WayPoint*)malloc(sizeof(Liar::WayPoint));
				w->Set(cellPath[0], *(m_path[0]));

#ifdef FindNearest
				int preNCount = m_nearstCount++;
				m_nearstCells = (Liar::Cell**)malloc(sizeof(Liar::Cell*)*m_nearstCount);
				m_nearstCells[preNCount] = w->GetCaller();
#endif // FindNearest

				while (!(w->GetPos().Equals(endX, endY)))
				{
					w->GetFurthestWayPoint(cellPath, cellPathSize, endX, endY, rw);
					AddPathPoint(w->GetPos(), outLen);

#ifdef FindNearest
					preNCount = m_nearstCount++;
					m_nearstCells = (Liar::Cell**)realloc(m_nearstCells, sizeof(Liar::Cell*)*m_nearstCount);
					m_nearstCells[preNCount] = w->GetCaller();
#endif // FindNearest

				}
				w->~WayPoint();
				free(w);
				w = nullptr;

#ifdef FindNearest
				FindNearestPath(0, m_path, outLen);
#ifdef ShareFind
				for (Liar::Uint i = 0; i < m_nearstCount; ++i) m_nearstCells[i]->checkLinkCount = 0;
#endif
				free(m_nearstCells);
				m_nearstCells = nullptr;
				m_nearstCount = 0;
#endif // FindNearest

			}

			free(cellPath);
			m_closeCount = 0;
			free(m_closeList);
			m_closeList = nullptr;
		}
		return m_path;

	}

#ifdef FindNearest
	void NavMesh::FindNearestPath(Liar::Uint startIndex, Liar::Vector2f** path, Liar::Uint& pathCount)
	{
		if (startIndex >= pathCount)
		{
			return;
		}

		int revertIndex = pathCount - startIndex - 1;
		Liar::Vector2f* startPos = path[revertIndex];
		Liar::Cell* startCell = m_nearstCells[revertIndex];

		int farIndex = -1;

		for (int i = 0; i < revertIndex; ++i)
		{
			Liar::Vector2f* checkPos = path[i];
			Liar::Cell* checkCell = m_nearstCells[i];

#ifdef ShareFind
			if (startCell == checkCell) continue;
#else
			if (startCell->Equals(*checkCell)) continue;
#endif // ShareFind

			if (abs(i - revertIndex) == 2)
			{
				Liar::Vector2f* nextPos = path[i + 1];
				Liar::Cell* nextCell = m_nearstCells[i + 1];
				if (
					checkCell->IsSide(*checkPos, *startPos) || 
					checkCell->IsSide(*checkPos, *nextPos) || 
					nextCell->IsSide(*checkPos, *startPos) || 
					nextCell->IsSide(*nextPos, *startPos)
					)
				{
					break;
				}
			}

			if (TestOneLine2D(*startPos, *checkPos, startCell, checkCell))
			{
				if (TestOneLine2D(*checkPos, *startPos, checkCell, startCell))
				{
					farIndex = i;
					break;
				}
			}
		}

		Liar::Uint setStartIndex = farIndex + 1;
		Liar::Uint resetCount = revertIndex - setStartIndex;
		Liar::Uint resetIndex = 0;
		Liar::Uint i = 0;
		if (farIndex >= 0 && resetCount > 0)
		{
			for (i = setStartIndex; i < static_cast<Liar::Uint>(revertIndex); ++i)
			{
				/*path[i]->~Vector2f();
				free(path[i]);
				path[i] = nullptr;*/
				m_nearstCells[i]->checkLinkCount = 0;
				m_nearstCells[i] = nullptr;
			}

			resetIndex = setStartIndex;
			Liar::Vector2f* tmp = nullptr;
			for (i = revertIndex; i < pathCount; ++i)
			{
				tmp = path[resetIndex];
				path[resetIndex++] = path[i];
				path[i] = tmp;
			}

			resetIndex = setStartIndex;
			for (i = revertIndex; i < m_nearstCount; ++i)
			{
				m_nearstCells[resetIndex++] = m_nearstCells[i];
			}

			pathCount = pathCount - resetCount;
			m_nearstCount -= resetCount;
		}

		FindNearestPath(startIndex + 1, path, pathCount);
	}
#endif // FindNearest

	/**
	* 路径经过的网格
	* @return
	*/
	Liar::Cell** NavMesh::GetCellPath(int& len)
	{
		Liar::Cell* st = m_closeList[m_closeCount - 1];
		int pre = len;
		++len;
		Liar::Cell** out = (Liar::Cell**)malloc(sizeof(Liar::Cell*)*len);
		out[pre] = st;

		while (st->parent)
		{
			pre = len;
			++len;
			out = (Liar::Cell**)realloc(out, sizeof(Liar::Cell*)*len);
			out[pre] = st->parent;
			AddCrossCell(st);
			st = st->parent;
		}
		AddCrossCell(st);
		return out;
	}

	void NavMesh::AddCrossCell(Cell* cell)
	{
#ifdef EditorMod
		int preCross = m_crossCount++;
		size_t blockSize = sizeof(Liar::Cell*)*m_crossCount;
		if (m_crossList) m_crossList = (Liar::Cell**)realloc(m_crossList, blockSize);
		else m_crossList = (Liar::Cell**)malloc(blockSize);
		m_crossList[preCross] = cell;
#endif // EditorMod
	}

	/*
	*	add point to path
	*/
	Liar::Vector2f** NavMesh::AddPathPoint(const Liar::Vector2f& vec, Liar::Uint& len)
	{
		return AddPathPoint(vec.GetX(), vec.GetY(), len);
	}

	Liar::Vector2f** NavMesh::AddPathPoint(Liar::NAVDTYPE x, Liar::NAVDTYPE y, Liar::Uint& len)
	{
		++len;
		Liar::Vector2f* addPoint = nullptr;
		if (len > m_numPath)
		{
			size_t blockSize = sizeof(Liar::Vector2f*)*len;
			if (m_path) m_path = (Liar::Vector2f**)realloc(m_path, blockSize);
			else m_path = (Liar::Vector2f**)malloc(blockSize);
			m_numPath = len;
			addPoint = (Liar::Vector2f*)malloc(sizeof(Liar::Vector2f));
		}
		else
		{
			addPoint = m_path[len - 1];
		}
		addPoint->Set(x, y);
		m_path[len - 1] = addPoint;
		return m_path;
	}

	/**
	* 找出给定点所在的三角型
	* @param Point
	* @return
	*/
	Liar::Cell* NavMesh::FindClosestCell(const Vector2f& pt, bool rw)
	{
		return FindClosestCell(pt.GetX(), pt.GetY(), rw);
	}

	Liar::Cell* NavMesh::FindClosestCell(Liar::NAVDTYPE x, Liar::NAVDTYPE y, bool rw)
	{

#ifndef ShareFind
		bool initTmp = false;
		if (!m_tmpClosetCell)
		{
			m_tmpClosetCell = (Liar::Cell*)malloc(sizeof(Liar::Cell));
			initTmp = true;
		}
#endif // !ShareFind

		for (Liar::Uint i = 0; i < m_numberCell; ++i)
		{
			Liar::Cell* it = m_cells[i];
#ifdef ShareFind
			if (it->IsPointIn(x, y, rw))
			{
				return it;
			}
#else
			m_tmpClosetCell->Set(*it, initTmp);
			initTmp = false;
			if(m_tmpClosetCell->IsPointIn(x, y, rw))
			{
				return AddTestCell(m_tmpClosetCell);
			}
#endif
		}
		return nullptr;
	}

	// 检测两点间是否能直达
	bool NavMesh::TestOneLine2D(const Vector2f& start, const Vector2f& end, Cell* startCell, Cell* endCell)
	{
		return TestOneLine2D(start.GetX(), start.GetY(), end.GetX(), end.GetY(), startCell, endCell);
	}

	bool NavMesh::TestOneLine2D(
		Liar::NAVDTYPE startX, Liar::NAVDTYPE startY,
		Liar::NAVDTYPE endX, Liar::NAVDTYPE endY,
		Liar::Cell* startCell, Liar::Cell* endCell
	)
	{
		// 记录已经找过的路径
		Liar::Cell** crossCells = (Liar::Cell**)malloc(sizeof(Liar::Cell*));
		int crossCount = 1;
		int pre = 0;
		Liar::Cell* cell = endCell;
		crossCells[0] = cell;

		bool findCross = true;

#ifdef ShareFind
		while (cell && cell != startCell)
#else
		while (cell && cell->Equals(*startCell))
#endif // ShareFind
		{
			cell = GetCrossCell(startX, startY, endX, endY, *cell, crossCells, crossCount, findCross);
			findCross = false;
			if (cell)
			{
				pre = crossCount++;
				crossCells = (Liar::Cell**)realloc(crossCells, sizeof(Liar::Cell*)*crossCount);
				crossCells[pre] = cell;
			}
			else
			{
				if (crossCount - 2 >= 0)
				{
					Liar::Cell* preCell = crossCells[crossCount - 2];
					if (!preCell->CheckAllLink())
					{
						cell = preCell;
					}
				}
			}
		}
		for (int i = 0; i < crossCount; ++i)
		{
			crossCells[i]->checkLinkCount = 0;
		}
		free(crossCells);

#ifdef ShareFind
		return (cell && cell == startCell);
#else
		return (cell && cell->Equals(*startCell));
#endif // ShareFind
	}

	Cell* NavMesh::GetCrossCell(const Vector2f& start, const Vector2f& end, Liar::Cell& testCell, Liar::Cell** crossList, int count, bool findCross)
	{
		return GetCrossCell(start.GetX(), start.GetY(), end.GetX(), end.GetY(), testCell, crossList, count, findCross);
	}

	Liar::Cell* NavMesh::GetCrossCell(
		Liar::NAVDTYPE startX, Liar::NAVDTYPE startY,
		Liar::NAVDTYPE endX, Liar::NAVDTYPE endY,
		Liar::Cell& testCell, Liar::Cell** crossList, int count, bool findCross
	)
	{
		int needCount = 2;
		int interCount = 0;

		int adjacentId = 0;
		Liar::Cell* sourceAdjacentTmp = nullptr;
		int startCrossCount = 0;
		Liar::Cell* outTmp = nullptr;
		for (Liar::Uint i = 0; i < 3; ++i)
		{
			adjacentId = testCell.GetLink(i);
			if (adjacentId < 0)
			{
				continue;
			}
			else
			{

				sourceAdjacentTmp = m_cells[adjacentId];

				bool crossed = false;
				for (int f = 0; f < count; ++f)
				{
#ifdef ShareFind
					if (crossList[f] == sourceAdjacentTmp)
#else
					if(crossList[f]->Equals(*sourceAdjacentTmp))
#endif // ShareFind
					{
						crossed = true;
						break;
					}
				}

				testCell.checkLinkCount += 1;

				if (!crossed)
				{
					interCount = TestLineCell(*sourceAdjacentTmp, startX, startY, endX, endY);
					if (interCount >= needCount)
					{
						//return sourceAdjacentTmp;
						if (interCount > startCrossCount)
						{
							outTmp = sourceAdjacentTmp;
							startCrossCount = interCount;
							if (!findCross || interCount > needCount) break;
						}
					}
				}
			}
		}
		return outTmp;
	}

	int NavMesh::TestLineCell(const Cell& sourceAdjacentTmp, const Vector2f& start, const Vector2f& end)
	{
		return TestLineCell(sourceAdjacentTmp, start.GetX(), start.GetY(), end.GetX(), end.GetY());
	}

	int NavMesh::TestLineCell(
		const Liar::Cell& sourceAdjacentTmp, 
		Liar::NAVDTYPE startX, Liar::NAVDTYPE startY, 
		Liar::NAVDTYPE endX, Liar::NAVDTYPE endY)
	{
		int crossCount = 0;
		for (int j = 0; j < 3; ++j)
		{
			Liar::Line2d* testline = sourceAdjacentTmp.GetSide(j);
			Liar::Vector2f& pa = testline->GetPointA();
			Liar::Vector2f& pb = testline->GetPointB();
			int intersect1 = LineIntersectSide(endX, endY, startX, startY, pa, pb);
			int intersect2 = LineIntersectSide(pa, pb, endX, endY, startX, startY);
			int intersect = intersect1 < intersect2 ? intersect1 : intersect2;
			if (intersect == 0) // 如果是在共用边上的交点，也是可以忽略的
			{
				/*if (sourceAdjacentTmp.arrivalWall < 0)
				{
					++crossCount;
				}
				else
				{
					if (_crossCount <= 0) continue;
					NAVDTYPE tx = 0.0;
					NAVDTYPE ty = 0.0;
					GetCrossVector2f(end, start, *pa, *pb, tx, ty);
					if (pa->Equals(tx, ty, EPSILON) || pb->Equals(tx, ty, EPSILON))
					{
						++crossCount;
					}
				}*/
				++crossCount;
			}
			else if (intersect > 0)
			{
				if (sourceAdjacentTmp.GetLink(j) > 0)
				{
					crossCount += 2;
				}
				else
				{
					return 0;
				}
			}
		}

		return crossCount;
	}

	int NavMesh::LineIntersectSide(const Vector2f& A, const Vector2f& B, const Vector2f& C, const Vector2f& D)
	{
		// A(x1, y1), B(x2, y2)的直线方程为：  
		// f(x, y) =  (y - y1) * (x1 - x2) - (x - x1) * (y1 - y2) = 0  

		Liar::NAVDTYPE AX = A.GetX();
		Liar::NAVDTYPE AY = A.GetY();

		Liar::NAVDTYPE BX = B.GetX();
		Liar::NAVDTYPE BY = B.GetY();

		Liar::NAVDTYPE CX = C.GetX();
		Liar::NAVDTYPE CY = C.GetY();

		Liar::NAVDTYPE DX = D.GetX();
		Liar::NAVDTYPE DY = D.GetY();

		return LineIntersectSide(AX, AY, BX, BY, CX, CY, DX, DY);
	}

	int NavMesh::LineIntersectSide(
		const Liar::Vector2f& A,
		const Liar::Vector2f& B,
		Liar::NAVDTYPE CX, Liar::NAVDTYPE CY,
		Liar::NAVDTYPE DX, Liar::NAVDTYPE DY
	)
	{
		return LineIntersectSide(A.GetX(), A.GetY(), B.GetX(), B.GetY(), CX, CY, DX, DY);
	}

	int NavMesh::LineIntersectSide(
		Liar::NAVDTYPE AX, Liar::NAVDTYPE AY,
		Liar::NAVDTYPE BX, Liar::NAVDTYPE BY,
		const Liar::Vector2f& C,
		const Liar::Vector2f& D
	)
	{
		return LineIntersectSide(AX, AY, BX, BY, C.GetX(), C.GetY(), D.GetX(), D.GetY());
	}

	int NavMesh::LineIntersectSide(
		Liar::NAVDTYPE AX, Liar::NAVDTYPE AY, 
		Liar::NAVDTYPE BX, Liar::NAVDTYPE BY, 
		Liar::NAVDTYPE CX, Liar::NAVDTYPE CY, 
		Liar::NAVDTYPE DX, Liar::NAVDTYPE DY)
	{
		NAVDTYPE fC = (CY - AY) * (AX - BX) - (CX - AX) * (AY - BY);
		NAVDTYPE fD = (DY - AY) * (AX - BX) - (DX - AX) * (AY - BY);

		double val = fC * fD;

		if (val > 0)
		{
			return -1;	// 没交点
		}
		else if (val == 0)
		{
			return 0;	// 在线上
		}
		else
		{
			return 1;	// 相交
		}
	}

	// 获得交点
	bool NavMesh::GetCrossVector2f(const Vector2f& ss, const Vector2f& sd, const Vector2f& ts, const Vector2f& td, NAVDTYPE& x, NAVDTYPE& y)
	{
		Liar::NAVDTYPE ssX = ss.GetX();
		Liar::NAVDTYPE ssY = ss.GetY();

		Liar::NAVDTYPE sdX = sd.GetX();
		Liar::NAVDTYPE sdY = sd.GetY();

		Liar::NAVDTYPE tsX = ts.GetX();
		Liar::NAVDTYPE tsY = ts.GetY();

		Liar::NAVDTYPE tdX = td.GetX();
		Liar::NAVDTYPE tdY = td.GetY();

		NAVDTYPE sa = ssY - sdY;
		NAVDTYPE sb = sdX - ssX;
		NAVDTYPE sc = ssX * sdY - sdX * ssY;

		NAVDTYPE ta = tsY - tdY;
		NAVDTYPE tb = tdX - tsX;
		NAVDTYPE tc = tsX * tdY - tdX * tsY;

		NAVDTYPE d = sa * tb - ta * sb;
		if (d != 0)
		{
			x = (sb*tc - tb * sc) / d;
			y = (sc*ta - tc * sa) / d;
			return true;
		}
		else
		{
			return false;
		}
	}

	void NavMesh::DisposePath()
	{
		if (m_path)
		{
			for (Liar::Uint i = 0; i < m_numPath; ++i)
			{
				m_path[i]->~Vector2f();
				free(m_path[i]);
				m_path[i] = nullptr;
			}
			free(m_path);
			m_path = nullptr;
		}
	}

#ifndef ShareFind
	Liar::Cell* NavMesh::AddTestCell(Liar::Cell* source)
	{
		if (!source) return nullptr;
		Liar::Cell* out = nullptr;
		for (Liar::Uint i = 0; i < m_testCount; ++i)
		{
			if (m_testCells[i]->Equals(*source))
			{
				return m_testCells[i];
			}
		}

		out = (Liar::Cell*)malloc(sizeof(Liar::Cell));
		out->Set(*source);

		++m_testCount;
		size_t blockSize = sizeof(Liar::Cell*)*m_testCount;
		if (m_testCells) m_testCells = (Liar::Cell**)realloc(m_testCells, blockSize);
		else m_testCells = (Liar::Cell**)malloc(blockSize);
		m_testCells[m_testCount - 1] = out;
		return out;
	}
#else
	int NavMesh::PATHSESSIONID = 0;
#endif // !ShareFind

#if defined(DEBUG_NIF) || defined(EditorMod)
	void NavMesh::WriteErlang(std::ofstream& outfile)
	{
		for (Liar::Uint i = 0; i < m_numberCell; ++i)
		{
			outfile << "\n";
			m_cells[i]->WriteErlang(outfile);
		}
	}
#endif

	Liar::Uint NavMesh::PATHMAX = 50;
}
