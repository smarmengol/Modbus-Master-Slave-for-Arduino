/**
 * Uses a loopback stream to set up communication between a master and slave on the
 * same machine. Exercises various MODBUS functions, and tests that the results
 * are correct.
 *
 * Test failures are reported to Serial, labelled "FAIL".
 */

#include <ModbusRtu.h>
#include "src/loopback.h"

// Set to 1, to report PASSes as well as FAILures.
// Set to higher numbers for increasing levels of detail.
#define VERBOSE_RESULTS 0

//
// Master

const uint16_t master_data_count = 16;
uint16_t master_data[master_data_count+1];
modbus_t telegram;
int8_t master_poll_result;

Loopback master_stream(MAX_BUFFER+1);
Modbus master(0,master_stream,0);


//
// Slave

const uint8_t slave_id = 1;
const uint16_t slave_data_count = 9;
uint16_t slave_data[slave_data_count+1]; ///< Include extra OOB register
int8_t slave_poll_result;

Loopback slave_stream(MAX_BUFFER+1);
Modbus slave(slave_id,slave_stream,0);


/** Calculate MODBUS CRC of data. */
uint16_t calcCRC(const void* data, uint8_t len)
{
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    unsigned int temp, temp2, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < len; i++)
    {
        temp = temp ^ bytes[i];
        for (unsigned char j = 1; j <= 8; j++)
        {
            flag = temp & 0x0001;
            temp >>=1;
            if (flag)
                temp ^= 0xA001;
        }
    }
    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;
    // the returned value is already swapped
    // crcLo byte is first & crcHi byte is last
    return temp;
}

void report(const char* type, uint16_t addr, uint16_t val, uint16_t expected)
{
  Serial.print(type);
  Serial.print(F(": "));
  Serial.print(addr);
  Serial.print(F("="));
  Serial.print(val);
  Serial.print(F(" ?"));
  Serial.println(expected);
}
void pass(const char* type, uint16_t addr, uint16_t val, uint16_t expected)
{
  Serial.print(F("PASS "));
  report(type, addr, val, expected);
}
void fail(const char* type, uint16_t addr, uint16_t val, uint16_t expected)
{
  Serial.print(F("FAIL "));
  report(type, addr, val, expected);
}

void test_equal(const char* type, uint16_t addr, uint16_t val, uint16_t expected)
{
  if(val!=expected)
      fail(type, addr, val, expected);
#if defined(VERBOSE_RESULTS) && (VERBOSE_RESULTS>=1)
  else
      pass(type, addr, val, expected);
#endif
}

uint16_t addr2word(uint16_t addr)
{
  return calcCRC(&addr, sizeof(addr));
}

bool addr2bool(uint16_t addr)
{
  uint16_t index = addr / 16;
  uint16_t crc = calcCRC(&index, sizeof(index));
  return bitRead(crc, addr % 16);
}

void init_master()
{
  telegram.u8id = slave_id;
  telegram.u8fct = 0;
  telegram.u16RegAdd = 0;
  telegram.u16CoilsNo = 0;
  telegram.au16reg = master_data; // pointer to a memory array in the Arduino
}

void init_slave()
{
  // Also initialise the *extra* register, at the end of the array.
  for(size_t i=0; i<=slave_data_count; ++i)
  {
    slave_data[i] = addr2word(i);
  }
}


/** Poll both master and slave. */
void poll()
{
#if defined(VERBOSE_RESULTS) && (VERBOSE_RESULTS>=4)
      master_stream.print_status(Serial, "master");
      slave_stream.print_status(Serial, "slave");
#endif

  slave_poll_result = slave.poll(slave_data, sizeof(slave_data));
#if defined(VERBOSE_RESULTS) && (VERBOSE_RESULTS>=3)
      master_stream.print_status(Serial, "master");
      slave_stream.print_status(Serial, "slave");
#endif

  master_poll_result = master.poll();
#if defined(VERBOSE_RESULTS) && (VERBOSE_RESULTS>=3)
      master_stream.print_status(Serial, "master");
      slave_stream.print_status(Serial, "slave");
#endif

#if defined(VERBOSE_RESULTS) && (VERBOSE_RESULTS>=2)
      Serial.print(F("  poll: slave=")); Serial.print(slave_poll_result);
      Serial.print(F(" master="));     Serial.print(master_poll_result);
      Serial.print(F(" st:"));   Serial.print(master.getState());
      Serial.print(F(" out:"));  Serial.print(master.getOutCnt());
      Serial.print(F(" in:"));   Serial.print(master.getInCnt());
      Serial.print(F(" err:"));  Serial.print(master.getErrCnt());
      Serial.print(F(" last:")); Serial.print(master.getLastError());
      Serial.println("");
#endif
}


//
// HOLDING REGISTERS

uint16_t read_holding_register(uint16_t addr)
{
  telegram.u8fct = MB_FC_READ_REGISTERS;
  telegram.u16RegAdd = addr;
  telegram.u16CoilsNo = 1;
  master.query( telegram );
  while(master.getState()==COM_WAITING)
  {
    poll();
  }
  return telegram.au16reg[0];
}

void write_holding_register(uint16_t addr, uint16_t val)
{
  telegram.u8fct = MB_FC_WRITE_REGISTER;
  telegram.u16RegAdd = addr;
  telegram.u16CoilsNo = 0;
  telegram.au16reg[0] = val;
  master.query( telegram );
  while(master.getState()==COM_WAITING)
  {
    poll();
  }
}

/** Holding registers are read/write. */
void test_holding_register(uint16_t reg_addr)
{
  // First check the starting value.
  uint16_t val = read_holding_register(reg_addr);
  uint16_t expected = addr2word(reg_addr);
  test_equal("test_holding_register, starting", reg_addr, val, expected);

  // Test write & read-back.
  uint16_t new_val = addr2word(~expected+321);

  // Write
  write_holding_register(reg_addr, new_val);
  test_equal("test_holding_register, write", reg_addr, slave_data[reg_addr], new_val);

  // Read
  val = read_holding_register(reg_addr);
  test_equal("test_holding_register, read", reg_addr, val, new_val);
}

void test_oob_holding_register()
{
  uint16_t errcnt0 = master.getErrCnt();

  // Currently FAILS, because the slave does not check its bounds.
  uint16_t val = read_holding_register(slave_data_count);
  test_equal(
      "test_oob_holding_register, non-existant",
      slave_data_count,
      master.getErrCnt(),
      errcnt0+1
    );

  // Check that the slave does not read beyond the bounds of its data array.
  test_equal(
      "test_oob_holding_register, OOB read check",
      slave_data_count,
      val == slave_data[slave_data_count],
      0 ///< Should be false
    );

  // Check that the slave will not write to arbitrary memory.
  uint16_t new_val = 0xBAD;
  write_holding_register(slave_data_count, new_val);
  test_equal(
      "test_oob_holding_register, OOB write check",
      slave_data_count,
      new_val == slave_data[slave_data_count],
      0 ///< Should be false
    );
}

void test_holding_registers()
{
  uint16_t errcnt0 = master.getErrCnt();
  for(uint16_t reg_addr=0; reg_addr<slave_data_count; ++reg_addr)
  {
    test_holding_register(reg_addr);
  }
  test_equal("test_holding_registers, errcnt", 0, master.getErrCnt(), errcnt0);

  // Test for correct handling of an out-of-bounds register.
  test_oob_holding_register();
}


//
// COILS

bool read_coil(uint16_t addr)
{
  telegram.u8fct = MB_FC_READ_COILS;
  telegram.u16RegAdd = addr;
  telegram.u16CoilsNo = 1;
  master.query( telegram );
  while(master.getState()==COM_WAITING)
  {
    poll();
  }
  return telegram.au16reg[0];
}

void write_coil(uint16_t addr, bool val)
{
  telegram.u8fct = MB_FC_WRITE_COIL;
  telegram.u16RegAdd = addr;
  telegram.u16CoilsNo = 0;
  telegram.au16reg[0] = (val? 1: 0);
  master.query( telegram );
  while(master.getState()==COM_WAITING)
  {
    poll();
  }
}

/** Coils are read/write. */
void test_coil(uint16_t reg_addr)
{
  // First check the starting value.
  bool val = read_coil(reg_addr);
  bool expected = addr2bool(reg_addr);
  test_equal("test_coil, starting", reg_addr, val, expected);

  // Test write & read-back.
  bool new_val = reg_addr % 2; // ?? This pattern is too regular - could
                               // ?? be correct by accident.

  // Write
  write_coil(reg_addr, new_val);
  test_equal(
      "test_coil, write",
      reg_addr,
      bitRead(slave_data[reg_addr/16], reg_addr%16),
      new_val
    );

  // Read
  val = read_coil(reg_addr);
  test_equal("test_coil, read", reg_addr, val, new_val);
}

void test_coils()
{
  uint16_t errcnt0 = master.getErrCnt();
  for(uint16_t reg_addr=0; reg_addr<(slave_data_count*16); ++reg_addr)
  {
    test_coil(reg_addr);
  }
  test_equal("test_coils, errcnt", 0, master.getErrCnt(), errcnt0);

  // Test for correct handling of a non-existant register.
  // Currently FAILS, because the slave does not check its bounds.
  read_coil(slave_data_count*16);
  test_equal("test_coils, non-existant",0,master.getErrCnt(),errcnt0+1);
}


void fill_array_with_test_data(uint16_t* arr, uint16_t len)
{
  const char test_data[] = "abcdefghijklmnopqrstuvwxyz0123456789";
  uint8_t* begin = (uint8_t*)arr;
  uint8_t* end = (uint8_t*)(arr + len);
  while(begin < end)
  {
    size_t run = end - begin;
    if(run > sizeof(test_data))
        run = sizeof(test_data);
    memcpy((void*)begin, (void*)test_data, run);
    begin += run;
  }
}


/** Read/write multiple registers. */
void test_multiple_registers()
{
  const uint16_t errcnt0 = master.getErrCnt();
  const uint16_t max_num = min(slave_data_count, master_data_count);
  for(uint16_t num=2; num<max_num; ++num)
  {
    for(uint16_t reg_addr=0; (reg_addr+num)<slave_data_count; ++reg_addr)
    {

      telegram.u8fct = MB_FC_WRITE_MULTIPLE_REGISTERS;
      telegram.u16RegAdd = reg_addr;
      telegram.u16CoilsNo = num;
      fill_array_with_test_data(telegram.au16reg, num);
      const uint16_t crc0 = calcCRC(telegram.au16reg, num*2);

      master.query( telegram );
      while(master.getState()==COM_WAITING)
      {
        poll();
      }

      // Check that the slave data has been set correctly.
      const uint16_t test_id = num*100 + reg_addr;
      const uint16_t crc1 = calcCRC(slave_data+reg_addr, num*2);
      test_equal("test_multiple_registers, write", test_id, crc1, crc0);
    }
  }
  test_equal("test_multiple_registers",0,master.getErrCnt(),errcnt0);
}


void test_loopback()
{
  const char* test_str = "Hello";
  master_stream.write(test_str, (size_t)5);
  size_t i = 0;
  while(slave_stream.available())
  {
    int c = slave_stream.read();
    test_equal("test_loopback", i, c, test_str[i]);
    ++i;
  }
}



void setup()
{
  Serial.begin(115200);
  Serial.println(F(__FILE__ "  Build: " __DATE__ ", " __TIME__));

  master_stream.connect(slave_stream);
}

void loop()
{
  init_master();

  // Reset counters
  master.start();
  slave.start();

  test_loopback();

  init_slave();
  test_holding_registers();

  init_slave();
  test_coils();

  test_multiple_registers();

  test_equal("master error count", 0, master.getErrCnt(), 0);

  Serial.println(F("OK"));
  delay(10000);
}

