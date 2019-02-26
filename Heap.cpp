
#include "Heap.h"

#include<stdlib.h> 

namespace Liar
{

	Heap::Heap()
	{
	}


	Heap::~Heap()
	{
		free(m_heap);
		m_heap = nullptr;
		m_count = 0;
	}

	void Heap::Set(int size)
	{
		m_size = size + 1;
		size_t blockSize = sizeof(Liar::Cell*)*m_size;
		m_heap = (Liar::Cell**)malloc(blockSize);
		m_count = 0;
	}

	bool Heap::Push(Cell * ce)
	{
		if (m_count + 1 < m_size)
		{
			m_heap[++m_count] = ce;

			/*printf("=========put in heap======count:%d\n", _count);
			ce->Print();*/

			int i = m_count;
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
		if (m_count > 0)
		{
			Cell* o = m_heap[1];

			m_heap[1] = m_heap[m_count];
			m_heap[m_count] = nullptr;
			//delCellPosVector(_heap, _count);

			int i = 1;
			int child = i << 1;
			Cell* tmp = m_heap[i];

			while (child < m_count)
			{
				if (child < m_count - 1)
				{
					if (Compare(m_heap[child], m_heap[child + 1]) < 0)
					{
						++child;
					}
				}

				Cell* v = m_heap[child];
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
			--m_count;
			return o;
		}
		return nullptr;
	}

	int Heap::Size()
	{
		return m_count;
	}


	// ==================== Ë½ÓÐº¯Êý ===============
	NAVDTYPE Heap::Compare(Cell* const c1, Cell* const c2)
	{
		return c2->f - c1->f;
	}

	void Heap::Clear()
	{
		for (int i = 0; i < m_size; ++i) m_heap[i] = nullptr;
		m_count = 0;
	}
}
