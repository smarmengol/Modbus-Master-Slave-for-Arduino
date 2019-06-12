#pragma once

#include <stdint.h>
#include <Stream.h>


class Loopback: public Stream
{
private:
  uint8_t* const  buffer;
  const uint8_t   size; ///< Number of bytes in buffer.
  uint8_t         head;
  uint8_t         tail;
  Loopback*       other;

public:
  Loopback(const uint8_t size_=64):
      buffer( new uint8_t[size_] ),
      size( size_ ),
      head( 0 ),
      tail( 0 ),
      other( NULL )
    {}

  void connect(Loopback& obj)
    {
      other = &obj;
      obj.other = this;
    }

  size_t queue_length() const
    {
      int result = head;
      result -= tail;
      if(result < 0)
         result += size;
      return result;
    }

  void print_status(Stream& outstream, const char* label = "?") const
    {
      size_t len = queue_length();
      outstream.print(F("  Loopback \""));
      outstream.print(label);
      outstream.print(F("\": "));
      outstream.print(len);
      outstream.print(F(" bytes"));
      if(len)
      {
        outstream.print(F(", next byte = "));
        outstream.print(buffer[tail], HEX);
      }
      outstream.println("");
    }

public:
  // Print
  using Print::write;

  virtual size_t write(uint8_t c)
    {
      uint8_t i = (head+1) % size;
      if(i!=tail)
      {
        buffer[head] = c;
        head = i;
        return 1;
      }
      return 0;
    }

  virtual int availableForWrite()
    {
      return ((head+1) % size != tail);
    }

  // Stream
  virtual int available()
    {
      return (other && other->head != other->tail);
    }

  virtual int read()
    {
      if( available() )
      {
        int c = other->buffer[ other->tail++ ];
        other->tail %= other->size;
        return c;
      }
      return -1;
    }

  virtual int peek()
    {
      if( available() )
          return other->buffer[ other->tail ];
      else
          return -1;
    }
};

