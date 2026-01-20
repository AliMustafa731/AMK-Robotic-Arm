/*
  Robotic Arm using Arduino & Bluetooth
  By : Ali Mustafa Kamel, 2024 Feb-July
*/


#include "ByteQueue.h"

//---------------------------------------------
//  Insert a Chunk of Bytes into the Queue
//---------------------------------------------
void ByteQueue::write(uint8_t* src, size_t len)
{
  if(this->freeSpace() >= len)
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
  if(this->freeSpace() != 0)
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
  return this->read();
}

int16_t ByteQueue::nextInt16()
{
  int16_t result;
  this->read((uint8_t*) &result, sizeof(int16_t));

  return result;
}

int32_t ByteQueue::nextInt32()
{
  int32_t result;
  this->read((uint8_t*) &result, sizeof(int32_t));

  return result;
}

float ByteQueue::nextFloat()
{
  float result;
  this->read((uint8_t*) &result, sizeof(float));

  return result;
}

//--------------------------------------------
//  Insert Common Data Types into the Queue
//--------------------------------------------
void ByteQueue::putByte(uint8_t data)
{
  this->write(data);
}

void ByteQueue::putInt16(int16_t data)
{
  this->write((uint8_t*) &data, sizeof(int16_t));
}

void ByteQueue::putInt32(int32_t data)
{
  this->write((uint8_t*) &data, sizeof(int32_t));
}

void ByteQueue::putFloat(float data)
{
  this->write((uint8_t*) &data, sizeof(float));
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
size_t ByteQueue::freeSpace()
{
  return this->capacity - this->length;
}
