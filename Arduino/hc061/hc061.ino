#include <SoftwareSerial.h>
SoftwareSerial BTSerial(8, 7);

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
  Serial.begin(9600);
  Serial.println("Enter AT commands:");
  BTSerial.begin(9600);
}

void loop()
{
  if (BTSerial.available())
  Serial.write(BTSerial.read());
  if (Serial.available())
  BTSerial.write(Serial.read());
}
