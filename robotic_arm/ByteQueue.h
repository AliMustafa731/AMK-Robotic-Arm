/*
  Robotic Arm using Arduino & Bluetooth
  By : Ali Mustafa Kamel, 2024 Feb-July
*/


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
  size_t length;   // number of elements (Bytes) inserted to the Queue
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
  void write(uint8_t* src, size_t len);

  //---------------------------------------------
  //  Insert a single Byte into the Queue
  //---------------------------------------------
  void write(uint8_t byte);

  //-------------------------------------------------
  //  Get / Remove a Chunk of Bytes from the Queue
  //-------------------------------------------------
  void read(uint8_t* dest, size_t len);

  //---------------------------------------------------
  //  Get / Remove a single Byte from the Queue
  //---------------------------------------------------
  uint8_t read();

  //----------------------------------------
  //  Get Common Data Types from the Queue
  //----------------------------------------
  int8_t nextByte();
  int16_t nextInt_2_Bytes();
  int32_t nextInt_4_Bytes();

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
