#pragma once

/**
 * \class Queue
 */

template<class Type>
Queue<Type>::Queue(unsigned int buf_size)
{
  buffer = new Type[buffer_size];
  head = buffer;
  tail = buffer;
  buffer_size = buf_size;
  isEmpty = true;
  isFull = false;
  data_count = 0;
}

template<class Type>
Queue<Type>::~Queue()
{
  head = NULL;
  tail = NULL;
  isEmpty = true;
  isFull = false;
  delete [] buffer;
}

template<class Type>
void Queue<Type>::enq(const Type& in)
{
  if (!isEmpty)
  {
    if (isFull)
    {
      next(head);
    }
    else
    {
      data_count++;
    }
    next(tail);
    if (head == tail)
    {
      isFull = true;
    }
  }
  *tail = in;
  if (isEmpty)
  {
    isEmpty = false;
  }
  //checkBuffer();
  return;
}

template<class Type>
int Queue<Type>::deq(Type& out)
{
  if (isEmpty)
  {
    return -1;
  }
  out = *head;
  if (head == tail && !isFull)
  {
    isEmpty = true;
  }
  else
  {
    next(head);
  }
  if (isFull)
  {
    isFull = false;
  }
  //checkBuffer();
  data_count--;
  return 0;
}

template<class Type>
inline void Queue<Type>::next(Type*& pointer)
{
  pointer++;
  if (pointer == buffer + buffer_size)
  {
    pointer = buffer;
  }
}

template<class Type>
inline void Queue<Type>::checkBuffer()
{
  if (isEmpty)
  {
    std::cout << "E ";
  }
  else
  {
    std::cout << "  ";
  }
  if (isFull)
  {
    std::cout << "F ";
  }
  else
  {
    std::cout << "  ";
  }
  for (unsigned int i = 0; i < buffer_size; i++)
  {
    if (head == &buffer[i])
    {
      std::cout << "(" << buffer[i] << ")" << " ";
    }
    else if (tail == &buffer[i])
    {
      std::cout << "[" << buffer[i] << "]" << " ";
    }
    else
    {
      std::cout << buffer[i] << " ";
    }
  }
  std::cout << std::endl;
  return;
}
