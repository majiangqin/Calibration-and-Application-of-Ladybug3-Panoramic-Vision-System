// DataAlgorithm.h: data strcuture algorithm

#ifndef DATA_ALGORITHM_H
#define DATA_ALGORITHM_H

#pragma once

template<typename T>
class HeapSort
{
public:
	static void sort(T data[], int size)
	{
		for (int ii = size/2-1; ii >= 0; --ii)	// Create the heap
		{
			moveDown(data,ii,size-1);
		}
		for (int ii = size-1; ii >= 1; --ii)
		{
			swap<T>(data[0],data[ii]); // move the largest item to data[ii]
			moveDown(data,0,ii-1); // restore the heap property
		}
	}

protected:
	static void moveDown(T data[], int index, int endIndex)
	{
		int lChild = 2*index+1, rChild = 2*index+2;
		if (lChild > endIndex)
			return; // Child doesn't exist
		if ( (rChild > endIndex) && (data[index] < data[lChild]) )
		{
			swap<T>(data[index],data[lChild]);
		}
		else if ( (rChild <= endIndex) && (data[lChild] < data[rChild]) && (data[index] < data[rChild]) )
		{
			swap<T>(data[index],data[rChild]);
			moveDown(data,rChild,endIndex);
		}
		else if ( (rChild <= endIndex) && (data[rChild] < data[lChild]) && (data[index] < data[lChild]) )
		{
			swap<T>(data[index],data[lChild]);
			moveDown(data,lChild,endIndex);
		}
		return;
	}
};

template<typename T>
int BinarySearch(T data[], const T& item, int size)
{
	// If finding the item, returns item's index. Otherwise returns -1.
	int li = 0, hi = size-1;
	while (hi >= li)
	{
		int mi = (li+hi)/2;
		if (data[mi] < item)
			li = mi+1;
		else if (data[mi] > item)
			hi = mi-1;
		else
			return mi;
	}
	return -1;
}

#endif