#include "Heap.h"

namespace Liar
{
	Heap::Heap():
		m_heap(nullptr), m_numCells(0), m_curNum(0)
	{
	}


	Heap::~Heap()
	{
		if (m_heap)
		{
			free(m_heap);
			m_heap = nullptr;
			m_numCells = 0;
			m_curNum = 0;
		}
	}

	void Heap::Set(Liar::Uint size)
	{
		if (m_numCells < size + 1)
		{
			size_t blockSize = sizeof(sizeof(Liar::Cell*)) *(size + 1);
			if (m_heap) m_heap = (Liar::Cell**)realloc(m_heap, blockSize);
			else m_heap = (Liar::Cell**)malloc(blockSize);
		}
		m_numCells = size + 1;
		for (Liar::Uint i = 0; i < m_numCells; ++i) m_heap[i] = nullptr;
		m_curNum = 0;
	}

	bool Heap::Push(Liar::Cell* ce)
	{
		if (m_curNum + 1 < m_numCells)
		{
			m_heap[++m_curNum] = ce;

			/*printf("=========put in heap======count:%d\n", _count);
			ce->Print();*/

			int i = m_curNum;
			int parent = i >> 1;
			Cell* tmp = m_heap[i];

			while (parent > 0)
			{
				Cell* v = m_heap[parent];
				if (Compare(tmp, v) > 0)
				{
					//printf("\n=====swap:%d\n", i);
					m_heap[i] = v;
					i = parent;
					parent >>= 1;
				}
				else
				{
					break;
				}
			}
			/*printf("\n=====after:%d\n", i);
			tmp->Print();
			printf("\n");*/
			m_heap[i] = tmp;
			return true;
		}

		return false;
	}

	/**
	* Dequeues and returns the front item.
	*
	* @return The Heap's front item or nullptr if it is empty.
	*/
	Cell* Heap::Pop()
	{
		if (m_curNum > 0)
		{
			Liar::Cell* o = m_heap[1];

			m_heap[1] = m_heap[m_curNum];
			m_heap[m_curNum] = nullptr;
			//delCellPosVector(_heap, _count);

			int i = 1;
			Liar::Uint child = i << 1;
			Liar::Cell* tmp = m_heap[i];

			while (child < m_curNum)
			{
				if (child < m_curNum - 1)
				{
					if (Compare(m_heap[child], m_heap[child + 1]) < 0) ++child;
				}

				Liar::Cell* v = m_heap[child];
				if (Compare(tmp, v) < 0)
				{
					m_heap[i] = v;
					i = child;
					child <<= 1;
				}
				else
				{
					break;
				}
			}

			m_heap[i] = tmp;
			--m_curNum;
			return o;
		}
		return nullptr;
	}

	Liar::NAVDTYPE Heap::Compare(const Liar::Cell* const c1, const Liar::Cell* const c2)
	{
		return c2->f - c1->f;
	}

	void Heap::Clear()
	{
		for (Liar::Uint i = 0; i < m_numCells; ++i) m_heap[i] = nullptr;
		m_curNum = 0;
	}
}
