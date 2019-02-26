#ifndef __HEAP_H__
#define __HEAP_H__

#include "Cell.h"

namespace Liar
{

	class Heap
	{
	public:
		Heap();
		~Heap();

	private:
		Liar::Cell** m_heap;
		int m_size;
		int m_count;

	public:
		bool Push(Liar::Cell* ce);
		Liar::Cell* Pop();
		void Clear();
		int Size();
		void Set(int size);

	private:
		Liar::NAVDTYPE Compare(Liar::Cell* const, Liar::Cell* const);
	};

}

#endif //!__HEAP_H__