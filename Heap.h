
#ifndef __HEAP_H__
#define __HEAP_H__

#include "Cell.h"

#ifdef  EditorMod  
#define  HEAPL_API _declspec(dllexport)  
#else  
#define  HEAPL_API _declspec(dllimport)  
#endif 

namespace Liar
{
#ifdef EditorMod
	class CELL_API Heap
#else
	class Heap
#endif // EditorMod
	{
	public:
		Heap();
		~Heap();

	private:
		Liar::Cell** m_heap;
		Liar::Uint m_numCells;
		Liar::Uint m_curNum;

	public:
		void Set(Liar::Uint);
		bool Push(Liar::Cell *);
		Liar::Cell* Pop();
		void Clear();

		Liar::Uint Size() const { return m_curNum; };

	private:
		Liar::NAVDTYPE Compare(const Liar::Cell* const, const Liar::Cell* const);
	};
}

#endif //!__HEAP_H__