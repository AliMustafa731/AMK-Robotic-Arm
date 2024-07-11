#ifndef BYTE_QUEUE_H
#define BYTE_QUEUE_H

#include <Arduino.h>

//---------------------------------------------
//  Queue Data Strcuture that works on Bytes
//---------------------------------------------
class ByteQueue
{
private:

  uint8_t* data;   // pointer for the Bytes Array
  size_t length;     // number of elements (Bytes) inserted to the Queue
  size_t capacity; // the size of the Array pointed by (data)
  size_t current;  // current index of the last inserted element (Byte)

public:

  ByteQueue(){}
  ByteQueue(uint8_t* _data, size_t _capacity)
  {
    this->data = _data;
    this->capacity = _capacity;
    this->current = 0;
    this->length = 0;
  }

  //---------------------------------------------
  //  Insert a Chunk of Bytes into the Queue
  //---------------------------------------------
  void enqueue(uint8_t* src, size_t len);

  //---------------------------------------------
  //  Insert a single Byte into the Queue
  //---------------------------------------------
  void enqueue(uint8_t byte);

  //-------------------------------------------------
  //  Get / Remove a Chunk of Bytes into the Queue
  //-------------------------------------------------
  void dequeue(uint8_t* dest, size_t len);

  //---------------------------------------------------
  //  Get / Remove a single Byte Bytes into the Queue
  //---------------------------------------------------
  uint8_t dequeue();

  //-----------------------------------
  // number of elemnts in the Queue
  //-----------------------------------
  size_t size();

  //-----------------------------------------------
  // available space left for inserting elemnts
  //-----------------------------------------------
  size_t free_space();
};

#endif  // BYTE_QUEUE_H
