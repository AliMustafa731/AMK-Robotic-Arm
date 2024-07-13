
#include "ByteQueue.h"

//---------------------------------------------
//  Insert a Chunk of Bytes into the Queue
//---------------------------------------------
void ByteQueue::write(uint8_t* src, size_t len)
{
  if(this->free_space() >= len)
  {
    for(size_t i = 0 ; i < len ; i++)
    {
      size_t avail = (this->current + this->length) % this->capacity;
      this->data[avail] = src[i];
      this->length++;
    }
  }
}

//---------------------------------------------
//  Insert a single Byte into the Queue
//---------------------------------------------
void ByteQueue::write(uint8_t byte)
{
  if(this->free_space() != 0)
  {
    size_t avail = (this->current + this->length) % this->capacity;
    this->data[avail] = byte;
    this->length++;
  }
}

//-------------------------------------------------
//  Get / Remove a Chunk of Bytes from the Queue
//-------------------------------------------------
void ByteQueue::read(uint8_t* dest, size_t len)
{
  if(this->size() >= len)
  {
    for(size_t i = 0 ; i < len ; i++)
    {
      dest[i] = this->data[this->current];
      this->current = (this->current + 1) % this->capacity;
      this->length--;
    }
  }
}

//---------------------------------------------------
//  Get / Remove a single Byte from the Queue
//---------------------------------------------------
uint8_t ByteQueue::read()
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

//----------------------------------------
//  Get Common Data Types from the Queue
//----------------------------------------
int8_t ByteQueue::nextByte()
{
  int8_t result;

  this->read((uint8_t*) &result, 1);

  return result;
}

int16_t ByteQueue::nextInt_2_Bytes()
{
  int16_t result;

  this->read((uint8_t*) &result, 2);

  return result;
}

int32_t ByteQueue::nextInt_4_Bytes()
{
  int32_t result;

  this->read((uint8_t*) &result, 4);

  return result;
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
