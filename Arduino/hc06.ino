
#include <SPI.h>
#include <STBLE.h>
#if defined (ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif
/*
 * AT -> OK
 * AT+NAME(藍牙名稱)
 * AT+PIN (4位數字)
 * 
 * example:
 * 點開右上角放大鏡
 * AT 顯示OK
 * AT+NAMETA (藍牙名稱設定為TA)
 * AT+PIN1111 (PIN設定為1111)
 */
void setup()
{
  SerialMonitorInterface.begin(115200);
  SerialMonitorInterface.println("Enter AT commands:");
}

void loop()
{
  if (SerialMonitorInterface.available())
  SerialMonitorInterface.write(SerialMonitorInterface.read());

}
