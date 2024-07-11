
#include "ByteQueue.h"

//---------------------------------------------
//  Insert a Chunk of Bytes into the Queue
//---------------------------------------------
void ByteQueue::enqueue(uint8_t* _data, size_t len)
{
  if(this->free_space() >= len)
  {
    for(size_t i = 0 ; i < len ; i++)
    {
      size_t avail = (this->current + this->length) % this->capacity;
      this->data[avail] = _data[i];
      this->length++;
    }
  }
}

//---------------------------------------------
//  Insert a single Byte into the Queue
//---------------------------------------------
void ByteQueue::enqueue(uint8_t byte)
{
  if(this->free_space() != 0)
  {
    size_t avail = (this->current + this->length) % this->capacity;
    this->data[avail] = byte;
    this->length++;
  }
}

//-------------------------------------------------
//  Get / Remove a Chunk of Bytes into the Queue
//-------------------------------------------------
void ByteQueue::dequeue(uint8_t* _data, size_t len)
{
  if(this->size() >= len)
  {
    for(size_t i = 0 ; i < len ; i++)
    {
      _data[i] = this->data[this->current];
      this->current = (this->current + 1) % this->capacity;
      this->length--;
    }
  }
}

//---------------------------------------------------
//  Get / Remove a single Byte Bytes into the Queue
//---------------------------------------------------
uint8_t ByteQueue::dequeue()
{
  if(this->size() != 0)
  {
    uint8_t tmp = this->data[this->current];
    this->current = (this->current + 1) % this->capacity;
    this->length--;
    return tmp;
  }

  return 0;
}

//-----------------------------------
// number of elemnts in the Queue
//-----------------------------------
size_t ByteQueue::size()
{
  return this->length;
}

//-----------------------------------------------
// available space left for inserting elemnts
//-----------------------------------------------
size_t ByteQueue::free_space()
{
  return this->capacity - this->length;
}
