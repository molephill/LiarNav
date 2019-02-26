#include "SingleFindPath.h"

#include<stdlib.h> 

namespace Liar
{
	SingleFindPath::SingleFindPath()
	{
	}


	SingleFindPath::~SingleFindPath()
	{
		_openList->~Heap();
		free(_openList);
		_openList = nullptr;

#ifdef EditorMod
		DestoryTestCell();
		if (_crossList)
		{
			free(_crossList);
			_crossList = nullptr;
		}
		_crossCount = 0;
#endif // EditorMod

#ifdef FindNearest
		if (_nearstCells)
		{
			free(_nearstCells);
		}
		_nearstCount = 0;
#endif // FindNearest

	}

	void SingleFindPath::Init(Map* map)
	{
		_map = map;
		_pathsession = 0;

		_openList = (Liar::Heap*)malloc(sizeof(Liar::Heap));
		_openList->Init();
		int size = _map->GetCellCount();
		_openList->Set(size);
		_closeList = nullptr;
		_closeCount = 0;

#ifdef EditorMod
		_crossCount = 0;
		_crossList = nullptr;
#endif // EditorMod

#ifdef FindNearest
		_nearstCells = nullptr;
		_nearstCount = 0;
#endif // FindNearest

	}

	Vector2f* SingleFindPath::Path(NAVDTYPE startX, NAVDTYPE startY, NAVDTYPE endX, NAVDTYPE endY, int& outLen, bool rw)
	{
		++_pathsession;

		Cell* startCell = FindClosestCell(startX, startY, rw);
#ifndef OutFind
		if (!startCell)
		{
			return nullptr;
		}
#endif // OutFind
		Cell* endCell = FindClosestCell(endX, endY, rw);
		if (!endCell)
		{
			return nullptr;
		}

#ifdef OutFind
		if (!startCell)
		{
			outLen = 2;
			Vector2f* out = (Vector2f*)malloc(sizeof(Vector2f)*outLen);
			out[0].Set(startX, startY);
			out[1].Set(endX, endY);
			return out;
		}
#endif // OutFind

		if (startCell == endCell || TestOneLine2D(startX, startY, endX, endY, startCell, endCell))
		//if (startCell == endCell)
		{
			outLen = 2;
			Vector2f* out = (Vector2f*)malloc(sizeof(Vector2f)*outLen);
			out[0].Set(startX, startY);
			out[1].Set(endX, endY);

			return out;
		}
		else
		{
			return BuildPath(startCell, startX, startY, endCell, endX, endY, outLen, rw);
		}
	}

	/**
	* 找出给定点所在的三角型
	* @param Point
	* @return
	*/
	Cell* SingleFindPath::FindClosestCell(const Vector2f& pt, bool rw)
	{
		return FindClosestCell(pt.GetX(), pt.GetY(), rw);
	}

	Cell* SingleFindPath::FindClosestCell(NAVDTYPE x, NAVDTYPE y, bool rw)
	{
		int cellCount = _map->GetCellCount();
		for (int i = 0; i < cellCount; ++i)
		{
			Liar::Cell* it = _map->GetCell(i);
			if (it->IsPointIn(x, y, rw))
			{
				return it;
			}
		}
		return nullptr;
	}

	/**
	* 构建路径
	* @param startCell
	* @param startPos
	* @param endCell
	* @param endPos
	* @return Point路径数组
	*/
	Vector2f* SingleFindPath::BuildPath(Cell* startCell, const Liar::Vector2f& startPos
		, Cell* endCell, const Liar::Vector2f& endPos, int& outLen, bool rw)
	{
		return BuildPath(startCell, startPos.GetX(), startPos.GetY(), endCell, endPos.GetX(), endPos.GetY(), outLen, rw);
	}

	Vector2f* SingleFindPath::BuildPath(Cell* startCell, NAVDTYPE startX, NAVDTYPE startY
		, Cell* endCell, NAVDTYPE endX, NAVDTYPE endY, int& outLen, bool rw)
	{
		_openList->Clear();
		_openList->Push(endCell);
		endCell->f = 0;
		endCell->h = 0;
		endCell->isOpen = false;
		endCell->parent = nullptr;
		endCell->sessionId = _pathsession;

		bool foundPath = false;			//是否找到路径
		Cell* currNode = nullptr;		//当前节点
		Cell* adjacentTmp = nullptr;	//当前节点的邻接三角型
		Cell* sourceAdjacentTmp = nullptr;

		while (_openList->Size())
		{
			// 1. 把当前节点从开放列表删除, 加入到封闭列表
			currNode = _openList->Pop();
			++_closeCount;
			if (_closeList)
			{
				_closeList = (Cell**)realloc(_closeList, sizeof(Cell*)*_closeCount);
			}
			else
			{
				_closeList = (Cell**)malloc(sizeof(Cell*)*_closeCount);
			}
			_closeList[_closeCount - 1] = currNode;

			if (currNode == startCell)
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
				if (adjacentId < 0)		//不能通过
				{
					continue;
				}
				else
				{
					sourceAdjacentTmp = _map->GetCell(adjacentId);
				}

				if (sourceAdjacentTmp)
				{
					adjacentTmp = sourceAdjacentTmp;

					if (adjacentTmp->sessionId != _pathsession)
					{
						// 4. 如果该相邻节点不在开放列表中,则将该节点添加到开放列表中, 
						//    并将该相邻节点的父节点设为当前节点,同时保存该相邻节点的G和F值;
						adjacentTmp->sessionId = _pathsession;
						adjacentTmp->parent = currNode;
						adjacentTmp->isOpen = true;

						//H和F值
						adjacentTmp->ComputeHeuristic(startX, startY);
						adjacentTmp->f = currNode->f + adjacentTmp->GetWallDistance(abs(i - currNode->arrivalWall));

						//放入开放列表并排序
						_openList->Push(adjacentTmp);

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
							//adjacentTmp = nullptr;
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
			DestoryTestCell();
			return nullptr;
		}
	}

	/**
	* 根据经过的三角形返回路径点(下一个拐角点法)
	* @param start
	* @param end
	* @return Point数组
	*/
	Vector2f* SingleFindPath::GetPath(const Vector2f& start, const Vector2f& end, int& outLen, bool rw)
	{
		return GetPath(start.GetX(), start.GetY(), end.GetX(), end.GetY(), outLen, rw);
	}

	Vector2f* SingleFindPath::GetPath(NAVDTYPE startX, NAVDTYPE startY, NAVDTYPE endX, NAVDTYPE endY, int& outLen, bool rw)
	{
		//经过的三角形
		int cellPathSize = 0;
		Cell** cellPath = GetCellPath(cellPathSize);

		if (cellPathSize > 0)
		{
			//开始点
			int pre = outLen;
			++outLen;
			Vector2f* pathArr = (Vector2f*)malloc(sizeof(Vector2f));
			pathArr[0].Set(startX, startY);

			pre = outLen;
			//起点与终点在同一三角形中
			if (cellPathSize == 1)
			{
				++outLen;
				pathArr = (Vector2f*)realloc(pathArr, sizeof(Vector2f)*outLen);
				pathArr[pre].Set(endX, endY);
			}
			else
			{
				Line2d* lastLineA = (Line2d*)malloc(sizeof(Line2d));
				Line2d* lastLineB = (Line2d*)malloc(sizeof(Line2d));
				lastLineA->Init();
				lastLineB->Init();

				WayPoint* w = (WayPoint*)malloc(sizeof(WayPoint));
				w->Init(cellPath[0], *pathArr);

#ifdef FindNearest
				int preNCount = _nearstCount++;
				_nearstCells = (Cell**)malloc(sizeof(Cell*)*_nearstCount);
				_nearstCells[preNCount] = w->caller;
#endif // FindNearest


				while (!(w->GetPos()->Equals(endX, endY, EPSILON)))
				{
					GetFurthestWayPoint(w, cellPath, cellPathSize, endX, endY, lastLineA, lastLineB, rw);
					pre = outLen;
					++outLen;
					pathArr = (Vector2f*)realloc(pathArr, sizeof(Vector2f)*outLen);
					pathArr[pre].SetVector2f(*(w->GetPos()));

#ifdef FindNearest
					preNCount = _nearstCount++;
					_nearstCells = (Cell**)realloc(_nearstCells, sizeof(Cell*)*_nearstCount);
					_nearstCells[preNCount] = w->caller;
#endif // FindNearest

				}
				w->~WayPoint();
				free(w);
				w = nullptr;

				lastLineA->~Line2d();
				free(lastLineA);
				lastLineA = nullptr;
				lastLineB->~Line2d();
				free(lastLineB);
				lastLineB = nullptr;

#ifdef FindNearest
				FindNearestPath(0, pathArr, outLen);
				free(_nearstCells);
				_nearstCells = nullptr;
				_nearstCount = 0;
#endif // FindNearest

			}

			free(cellPath);
			free(_closeList);
			_closeList = nullptr;

#ifndef EditorMod
			DestoryTestCell();
#endif // !EditorMod


			return pathArr;
		}
		else
		{

			DestoryTestCell();

			return nullptr;
		}
	}

#ifdef FindNearest
	void SingleFindPath::FindNearestPath(int startIndex, Vector2f* path, int& pathCount)
	{
		if (startIndex >= pathCount)
		{
			return;
		}

		int revertIndex = pathCount - startIndex - 1;
		Vector2f& startPos = path[revertIndex];
		Cell* startCell = _nearstCells[revertIndex];

		int farIndex = -1;

		for (int i = 0; i < revertIndex; ++i)
		{
			Vector2f& checkPos = path[i];
			Cell* checkCell = _nearstCells[i];

			if (startCell->GetIndex() == checkCell->GetIndex())
			{
				continue;
			}

			if (abs(i - revertIndex) == 2)
			{
				Vector2f& nextPos = path[i + 1];
				Cell* nextCell = _nearstCells[i + 1];
				if (checkCell->IsSide(checkPos, startPos) || checkCell->IsSide(checkPos, nextPos) || nextCell->IsSide(checkPos, startPos) || nextCell->IsSide(nextPos, startPos))
				{
					break;
				}
			}

			if (TestOneLine2D(startPos, checkPos, startCell, checkCell))
			{
				if (TestOneLine2D(checkPos, startPos, checkCell, startCell))
				{
					farIndex = i;
					break;
				}
			}
		}

		int setStartIndex = farIndex + 1;
		int resetCount = revertIndex - setStartIndex;
		int resetIndex = 0;
		if (farIndex >= 0 && resetCount > 0)
		{
			for (int i = setStartIndex; i < revertIndex; ++i)
			{
				path[i].~Vector2f();
				_nearstCells[i]->checkLinkCount = 0;
				_nearstCells[i] = nullptr;
			}

			resetIndex = setStartIndex;
			for (int i = revertIndex; i < pathCount; ++i)
			{
				path[resetIndex++] = path[i];
			}

			resetIndex = setStartIndex;
			for (int i = revertIndex; i < _nearstCount; ++i)
			{
				_nearstCells[resetIndex++] = _nearstCells[i];
			}

			pathCount = pathCount - resetCount;
			_nearstCount -= resetCount;
		}

		FindNearestPath(startIndex + 1, path, pathCount);
	}
#endif // FindNearest


	/**
	* 路径经过的网格
	* @return
	*/
	Cell** SingleFindPath::GetCellPath(int& len)
	{
		Cell* st = _closeList[_closeCount - 1];
		int pre = len;
		++len;
		Cell** out = (Cell**)malloc(sizeof(Cell*)*len);
		out[pre] = st;

		while (st->parent)
		{
			pre = len;
			++len;
			out = (Cell**)realloc(out, sizeof(Cell*)*len);
			out[pre] = st->parent;
			AddCrossCell(st);
			st = st->parent;
		}
		AddCrossCell(st);
		return out;
	}

	void SingleFindPath::AddCrossCell(Cell* cell)
	{
#ifdef EditorMod
		int preCross = _crossCount++;
		if (_crossList)
		{
			_crossList = (Cell**)realloc(_crossList, sizeof(Cell*)*_crossCount);
		}
		else
		{
		_crossList = (Cell**)malloc(sizeof(Cell*)*_crossCount);
		}
		_crossList[preCross] = cell;
#endif // EditorMod
	}


	/**
	* 下一个拐点
	* @param wayPoint 当前所在路点
	* @param cellPath 网格路径
	* @param end 终点
	* @return
	*/
	void SingleFindPath::GetFurthestWayPoint(WayPoint* waypoint, Cell* cellPath[], int cellCount, const Vector2f& end, Line2d* lastLineA, Line2d* lastLineB, bool rw)
	{
		GetFurthestWayPoint(waypoint, cellPath, cellCount, end.GetX(), end.GetY(), lastLineA, lastLineB, rw);
	}
	void SingleFindPath::GetFurthestWayPoint(WayPoint* waypoint, Cell* cellPath[], int cellCount, NAVDTYPE endX, NAVDTYPE endY, Line2d* lastLineA, Line2d* lastLineB, bool rw)
	{
		Vector2f* end = (Vector2f*)malloc(sizeof(Vector2f));
		end->Set(endX, endY);

		Vector2f* startPt = waypoint->GetPos();
		Cell* caller = waypoint->caller;
		//Cell* lastCell = caller;

		Line2d* outSide = caller->GetSide(caller->arrivalWall);	//路径线在网格中的穿出边
		Vector2f* lastPtA = outSide->GetA();
		Vector2f* lastPtB = outSide->GetB();
		lastLineA->SetPoints(*startPt, *lastPtA);
		lastLineB->SetPoints(*startPt, *lastPtB);
		Vector2f* testPtA = nullptr;
		Vector2f* testPtB = nullptr;

		int startIndex = 0, i = 0;
		for (i = 0; i < cellCount; ++i)
		{
			if (caller == cellPath[i])
			{
				startIndex = i;
				break;
			}
		}

		int closeA = -1;
		Vector2f* closeAPoint = nullptr;
		int closeB = -1;
		Vector2f* closeBPoint = nullptr;

		// 当是同一条直线时
		/*int setLineB = -1;
		int setLineA = -1;*/

		for (i = startIndex + 1; i < cellCount; ++i)
		{
			caller = cellPath[i];
			outSide = caller->GetSide(caller->arrivalWall);

			if (i == cellCount - 1)
			{
				testPtA = end;
				testPtB = end;
			}
			else
			{
				testPtA = outSide->GetA();
				testPtB = outSide->GetB();
			}

			PointClassification typeA_A = lastLineA->ClassifyPoint(*testPtA, EPSILON, rw);
			PointClassification typeA_B = lastLineA->ClassifyPoint(*testPtB, EPSILON, rw);
			PointClassification typeB_B = lastLineB->ClassifyPoint(*testPtB, EPSILON, rw);
			PointClassification typeB_A = lastLineB->ClassifyPoint(*testPtA, EPSILON, rw);

			if (lastLineA->Length() > EPSILON && typeB_A == PointClassification::RIGHT_SIDE)
			{
				//路点
				// 优化，当前的拐点和起点之间可能可以直达，那么就不要经过前面遇到过的拐点 2011
				//if (closeB > 0 && !TestOneLine2D(*startPt, *testPtB, waypoint->caller, caller))
				if (closeB > 0)
				{
					waypoint->caller = cellPath[closeB];
					waypoint->SetPos(*closeBPoint);
				}
				else
				{
					waypoint->caller = caller;
					waypoint->SetPos(*lastPtB);
				}
				return;
			}
			else if (lastLineB->Length() > EPSILON && typeA_B == PointClassification::LEFT_SIDE)
			{
				//路点
				// 优化，当前的拐点和起点之间可能可以直达，那么就不要经过前面遇到过的拐点
				//if (closeA > 0 && !TestOneLine2D(*startPt, *testPtA, waypoint->caller, caller))
				if (closeA > 0)
				{
					waypoint->caller = cellPath[closeA];
					waypoint->SetPos(*closeAPoint);
				}
				else
				{
					waypoint->caller = caller;
					waypoint->SetPos(*lastPtA);
				}
				return;
			}
			else if (typeA_A == PointClassification::ON_LINE && typeA_B == PointClassification::ON_LINE)	// 在同一条直线上(一条直线的起点和终点一样)
			{
				if (lastLineA->Length() <= EPSILON)
				{
					//lastCell = caller;
					// 这里表明lastLineB是一条线，lastLineA是个点
					if (typeB_A == PointClassification::LEFT_SIDE)
					{
						lastLineA->SetB(*testPtA);
						lastPtA = testPtA;
						//setLineA = i;
					}
					else
					{
						lastLineB->SetB(*testPtB);
						lastPtB = testPtB;
						//setLineB = i;
					}
				}
				else if (lastLineB->Length() <= EPSILON)
				{
					//lastCell = caller;
					// 这里表明lastLineA是一条线，lastLineB是个点
					if (typeA_B == PointClassification::RIGHT_SIDE)
					{
						lastLineB->SetB(*testPtB);
						lastPtB = testPtB;
						//setLineB = i;
					}
					else
					{
						lastLineA->SetB(*testPtA);
						lastPtA = testPtA;
						//setLineA = i;
					}
				}
			}
			else if ((typeA_A != PointClassification::ON_LINE && typeA_A == PointClassification::RIGHT_SIDE) || (typeA_A == PointClassification::ON_LINE && lastLineA->Length() <= EPSILON))
			{
				lastPtA = testPtA;
				//lastCell = caller;
				//重设直线
				lastLineA->SetB(*lastPtA);
				closeA = -1;
				closeAPoint = nullptr;
			}
			else if ((typeB_B != PointClassification::ON_LINE && typeB_B == PointClassification::LEFT_SIDE) || (typeB_B == PointClassification::ON_LINE && lastLineB->Length() <= EPSILON))
			{
				lastPtB = testPtB;
				//lastCell = caller;
				//重设直线
				lastLineB->SetB(*lastPtB);
				closeB = -1;
				closeBPoint = nullptr;

			}
			else if (typeB_B == PointClassification::RIGHT_SIDE || typeA_A == PointClassification::LEFT_SIDE)
			{
				if (typeB_B == PointClassification::RIGHT_SIDE)
				{
					// B两次遇到过障碍物了,第一次遇到的才是拐点
					if (closeB > 0)
					{
						/*waypoint->caller = cellPath[closeB];
						waypoint->SetPos(*closeBPoint);
						return;*/
					} // A 已遇到过障碍物了，取两者最短的路径
					//else if (closeA > 0)
					//{
					//	lastLineA->SetB(*closeAPoint);
					//	NAVDTYPE aLen = lastLineA->Length();
					//	NAVDTYPE bLen = lastLineB->Length();

					//	if (aLen < bLen)
					//	{
					//		waypoint->caller = cellPath[closeA];
					//		waypoint->SetPos(*closeAPoint);
					//	}
					//	else
					//	{
					//		waypoint->caller = caller;
					//		waypoint->SetPos(*lastLineB->GetB());
					//	}
					//	return;
					//}
					else
					{
						closeB = i;
						closeBPoint = lastLineB->GetB();
						//lastCell = caller;
					}
				}

				if (typeA_A == PointClassification::LEFT_SIDE)
				{
					// A两次遇到过障碍物了,第一次遇到的才是拐点
					if (closeA > 0)
					{
						/*waypoint->caller = cellPath[closeA];
						waypoint->SetPos(*closeAPoint);
						return;*/
					} // B 已遇到过障碍物了，取两者最短的路径
					//else if (closeB > 0)
					//{
					//	//lastCell = cellPath[i - 1];

					//	lastLineB->SetB(*closeBPoint);
					//	NAVDTYPE aLen = lastLineA->Length();
					//	NAVDTYPE bLen = lastLineB->Length();
					//	if (aLen < bLen)
					//	{
					//		waypoint->caller = caller;
					//		waypoint->SetPos(*lastLineA->GetB());
					//	}
					//	else
					//	{
					//		waypoint->caller = cellPath[closeB];
					//		waypoint->SetPos(*closeBPoint);
					//	}
					//	return;
					//}
					else
					{
						closeA = i;
						closeAPoint = lastLineA->GetB();
						//lastCell = caller;
					}
				}
			}

		}

		waypoint->caller = cellPath[cellCount - 1];
		waypoint->SetPos(*end);
		end->~Vector2f();
		free(end);
	}

	// 检测两点间是否能直达
	bool SingleFindPath::TestOneLine2D(const Vector2f& start, const Vector2f& end, Cell* startCell, Cell* endCell)
	{
		return TestOneLine2D(start.x, start.y, end.x, end.y, startCell, endCell);
	}

	bool SingleFindPath::TestOneLine2D(NAVDTYPE startX, NAVDTYPE startY, NAVDTYPE endX, NAVDTYPE endY, Cell* startCell, Cell* endCell)
	{
		// 记录已经找过的路径
		Cell** crossCells = (Cell**)malloc(sizeof(Cell*));
		int crossCount = 1;
		int pre = 0;
		Cell* cell = endCell;
		crossCells[0] = cell;

		bool findCross = true;
		while (cell && cell != startCell)
		{
			cell = GetCrossCell(startX, startY, endX, endY, *cell, crossCells, crossCount, *endCell, findCross);
			findCross = false;
			if (cell)
			{
				pre = crossCount++;
				crossCells = (Cell**)realloc(crossCells, sizeof(Cell*)*crossCount);
				crossCells[pre] = cell;
			}
			else
			{
				if (crossCount - 2 >= 0)
				{
					Cell* preCell = crossCells[crossCount - 2];
					if (!preCell->CheckAllLink())
					{
						cell = preCell;
						findCross = true;
					}
				}
			}
		}
		for (int i = 0; i < crossCount; ++i)
		{
			crossCells[i]->checkLinkCount = 0;
		}
		free(crossCells);
		return (cell && cell == startCell);
	}

	Cell* SingleFindPath::GetCrossCell(const Vector2f& start, const Vector2f& end, Cell& testCell, Cell** crossList, int count, const Cell& finalCell, bool findCross)
	{
		return GetCrossCell(start.x, start.y, end.x, end.y, testCell, crossList, count, finalCell, findCross);
	}

	Cell* SingleFindPath::GetCrossCell(NAVDTYPE startX, NAVDTYPE startY, NAVDTYPE endX, NAVDTYPE endY, 
		Cell& testCell, Cell** crossList, int count, const Cell& finalCell, bool findCross)
	{
		int needCount = 2;
		int interCount = 0;
		int tmpIndex = 0;

		int adjacentId = 0;
		Cell* sourceAdjacentTmp = nullptr;
		int startCrossCount = 0;
		Cell* outTmp = nullptr;
		for (int i = 0; i < 3; ++i)
		{
			adjacentId = testCell.GetLink(i);
			if (adjacentId < 0)
			{
				continue;
			}
			else
			{
				sourceAdjacentTmp = _map->GetNavMesh()->GetCellVector() + adjacentId;
				tmpIndex = sourceAdjacentTmp->GetIndex();
				bool crossed = false;
				for (int f = 0; f < count; ++f)
				{
					if (crossList[f]->GetIndex() == tmpIndex)
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

	int SingleFindPath::TestLineCell(const Cell& sourceAdjacentTmp, const Vector2f& start, const Vector2f& end)
	{
		return TestLineCell(sourceAdjacentTmp, start.x, start.y, end.x, end.y);
	}

	int SingleFindPath::TestLineCell(const Cell& sourceAdjacentTmp, NAVDTYPE startX, NAVDTYPE startY, NAVDTYPE endX, NAVDTYPE endY)
	{
		int crossCount = 0;
		for (int j = 0; j < 3; ++j)
		{
			Line2d* testline = const_cast<Cell&>(sourceAdjacentTmp).GetSide(j);
			Vector2f* pa = testline->GetA();
			Vector2f* pb = testline->GetB();
			int intersect1 = LineIntersectSide(endX, endY, startX, startY, *pa, *pb);
			int intersect2 = LineIntersectSide(*pa, *pb, endX, endY, startX, startY);
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


	int SingleFindPath::LineIntersectSide(const Vector2f& A, const Vector2f& B, const Vector2f& C, const Vector2f& D)
	{
		return LineIntersectSide(A.x, A.y, B.x, B.y, C.x, C.y, D.x, D.y);
	}

	int SingleFindPath::LineIntersectSide(NAVDTYPE AX, NAVDTYPE AY, NAVDTYPE BX, NAVDTYPE BY, const Vector2f& C, const Vector2f& D)
	{
		return LineIntersectSide(AX, AY, BX, BY, C.x, C.y, D.x, D.y);
	}

	int SingleFindPath::LineIntersectSide(const Vector2f& C, const Vector2f& D, NAVDTYPE AX, NAVDTYPE AY, NAVDTYPE BX, NAVDTYPE BY)
	{
		return LineIntersectSide(C.x, C.y, D.x, D.y, AX, AY, BX, BY);
	}

	int SingleFindPath::LineIntersectSide(NAVDTYPE AX, NAVDTYPE AY, NAVDTYPE BX, NAVDTYPE BY, 
										  NAVDTYPE CX, NAVDTYPE CY, NAVDTYPE DX, NAVDTYPE DY)
	{
		// A(x1, y1), B(x2, y2)的直线方程为：  
		// f(x, y) =  (y - y1) * (x1 - x2) - (x - x1) * (y1 - y2) = 0  
		NAVDTYPE fC = (CY - AY) * (AX - BX) - (CX - AX) * (AY - BY);
		NAVDTYPE fD = (DY - AY) * (AX - BX) - (DX - AX) * (AY - BY);

		double val = fC * fD;

		if (val > 0)
		{
			return -1;	// 没交点
		}
		else if(val == 0)
		{
			return 0;	// 在线上
		}
		else
		{
			return 1;	// 相交
		}
	}

	// 获得交点
	bool SingleFindPath::GetCrossVector2f(const Vector2f& ss, const Vector2f& sd, const Vector2f& ts, const Vector2f& td, NAVDTYPE& x, NAVDTYPE& y)
	{
		NAVDTYPE sa = ss.y - sd.y;
		NAVDTYPE sb = sd.x - ss.x;
		NAVDTYPE sc = ss.x*sd.y - sd.x*ss.y;

		NAVDTYPE ta = ts.y - td.y;
		NAVDTYPE tb = td.x - ts.x;
		NAVDTYPE tc = ts.x*td.y - td.x*ts.y;

		NAVDTYPE d = sa*tb - ta*sb;
		if (d != 0)
		{
			x = (sb*tc - tb*sc) / d;
			y = (sc*ta - tc*sa) / d;
			return true;
		}
		else
		{
			return false;
		}
	}
}
