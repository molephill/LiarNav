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
	* �ҳ����������ڵ�������
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
	* ����·��
	* @param startCell
	* @param startPos
	* @param endCell
	* @param endPos
	* @return Point·������
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

		bool foundPath = false;			//�Ƿ��ҵ�·��
		Cell* currNode = nullptr;		//��ǰ�ڵ�
		Cell* adjacentTmp = nullptr;	//��ǰ�ڵ���ڽ�������
		Cell* sourceAdjacentTmp = nullptr;

		while (_openList->Size())
		{
			// 1. �ѵ�ǰ�ڵ�ӿ����б�ɾ��, ���뵽����б�
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

			// 2. �Ե�ǰ�ڵ����ڵ�ÿһ���ڵ�����ִ�����²���:
			//�����ڽ�������
			int adjacentId = 0;
			for (int i = 0; i < 3; ++i)
			{
				adjacentId = currNode->GetLink(i);
				// 3. ��������ڽڵ㲻��ͨ�л��߸����ڽڵ��Ѿ��ڷ���б���,
				//    ��ʲô����Ҳ��ִ��,����������һ���ڵ�;
				if (adjacentId < 0)		//����ͨ��
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
						// 4. ��������ڽڵ㲻�ڿ����б���,�򽫸ýڵ���ӵ������б���, 
						//    ���������ڽڵ�ĸ��ڵ���Ϊ��ǰ�ڵ�,ͬʱ��������ڽڵ��G��Fֵ;
						adjacentTmp->sessionId = _pathsession;
						adjacentTmp->parent = currNode;
						adjacentTmp->isOpen = true;

						//H��Fֵ
						adjacentTmp->ComputeHeuristic(startX, startY);
						adjacentTmp->f = currNode->f + adjacentTmp->GetWallDistance(abs(i - currNode->arrivalWall));

						//���뿪���б�����
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
						// 5. ��������ڽڵ��ڿ����б���, 
						//    ���ж������ɵ�ǰ�ڵ㵽������ڽڵ��Gֵ�Ƿ�С��ԭ�������Gֵ,
						//    ��С��,�򽫸����ڽڵ�ĸ��ڵ���Ϊ��ǰ�ڵ�,���������ø����ڽڵ��G��Fֵ
						if (adjacentTmp->isOpen)	//�Ѿ���openList��
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
							//����closeList��
							//adjacentTmp = nullptr;
							continue;
						}
					}
				}
			}
		}

		//������·������Point����·��
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
	* ���ݾ����������η���·����(��һ���սǵ㷨)
	* @param start
	* @param end
	* @return Point����
	*/
	Vector2f* SingleFindPath::GetPath(const Vector2f& start, const Vector2f& end, int& outLen, bool rw)
	{
		return GetPath(start.GetX(), start.GetY(), end.GetX(), end.GetY(), outLen, rw);
	}

	Vector2f* SingleFindPath::GetPath(NAVDTYPE startX, NAVDTYPE startY, NAVDTYPE endX, NAVDTYPE endY, int& outLen, bool rw)
	{
		//������������
		int cellPathSize = 0;
		Cell** cellPath = GetCellPath(cellPathSize);

		if (cellPathSize > 0)
		{
			//��ʼ��
			int pre = outLen;
			++outLen;
			Vector2f* pathArr = (Vector2f*)malloc(sizeof(Vector2f));
			pathArr[0].Set(startX, startY);

			pre = outLen;
			//������յ���ͬһ��������
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
	* ·������������
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
	* ��һ���յ�
	* @param wayPoint ��ǰ����·��
	* @param cellPath ����·��
	* @param end �յ�
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

		Line2d* outSide = caller->GetSide(caller->arrivalWall);	//·�����������еĴ�����
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

		// ����ͬһ��ֱ��ʱ
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
				//·��
				// �Ż�����ǰ�Ĺյ�����֮����ܿ���ֱ���ô�Ͳ�Ҫ����ǰ���������Ĺյ� 2011
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
				//·��
				// �Ż�����ǰ�Ĺյ�����֮����ܿ���ֱ���ô�Ͳ�Ҫ����ǰ���������Ĺյ�
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
			else if (typeA_A == PointClassification::ON_LINE && typeA_B == PointClassification::ON_LINE)	// ��ͬһ��ֱ����(һ��ֱ�ߵ������յ�һ��)
			{
				if (lastLineA->Length() <= EPSILON)
				{
					//lastCell = caller;
					// �������lastLineB��һ���ߣ�lastLineA�Ǹ���
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
					// �������lastLineA��һ���ߣ�lastLineB�Ǹ���
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
				//����ֱ��
				lastLineA->SetB(*lastPtA);
				closeA = -1;
				closeAPoint = nullptr;
			}
			else if ((typeB_B != PointClassification::ON_LINE && typeB_B == PointClassification::LEFT_SIDE) || (typeB_B == PointClassification::ON_LINE && lastLineB->Length() <= EPSILON))
			{
				lastPtB = testPtB;
				//lastCell = caller;
				//����ֱ��
				lastLineB->SetB(*lastPtB);
				closeB = -1;
				closeBPoint = nullptr;

			}
			else if (typeB_B == PointClassification::RIGHT_SIDE || typeA_A == PointClassification::LEFT_SIDE)
			{
				if (typeB_B == PointClassification::RIGHT_SIDE)
				{
					// B�����������ϰ�����,��һ�������Ĳ��ǹյ�
					if (closeB > 0)
					{
						/*waypoint->caller = cellPath[closeB];
						waypoint->SetPos(*closeBPoint);
						return;*/
					} // A ���������ϰ����ˣ�ȡ������̵�·��
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
					// A�����������ϰ�����,��һ�������Ĳ��ǹյ�
					if (closeA > 0)
					{
						/*waypoint->caller = cellPath[closeA];
						waypoint->SetPos(*closeAPoint);
						return;*/
					} // B ���������ϰ����ˣ�ȡ������̵�·��
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

	// ���������Ƿ���ֱ��
	bool SingleFindPath::TestOneLine2D(const Vector2f& start, const Vector2f& end, Cell* startCell, Cell* endCell)
	{
		return TestOneLine2D(start.x, start.y, end.x, end.y, startCell, endCell);
	}

	bool SingleFindPath::TestOneLine2D(NAVDTYPE startX, NAVDTYPE startY, NAVDTYPE endX, NAVDTYPE endY, Cell* startCell, Cell* endCell)
	{
		// ��¼�Ѿ��ҹ���·��
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
			if (intersect == 0) // ������ڹ��ñ��ϵĽ��㣬Ҳ�ǿ��Ժ��Ե�
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
		// A(x1, y1), B(x2, y2)��ֱ�߷���Ϊ��  
		// f(x, y) =  (y - y1) * (x1 - x2) - (x - x1) * (y1 - y2) = 0  
		NAVDTYPE fC = (CY - AY) * (AX - BX) - (CX - AX) * (AY - BY);
		NAVDTYPE fD = (DY - AY) * (AX - BX) - (DX - AX) * (AY - BY);

		double val = fC * fD;

		if (val > 0)
		{
			return -1;	// û����
		}
		else if(val == 0)
		{
			return 0;	// ������
		}
		else
		{
			return 1;	// �ཻ
		}
	}

	// ��ý���
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
