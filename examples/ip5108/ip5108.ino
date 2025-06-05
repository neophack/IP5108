/*
  示例：IP5108 Arduino Sketch
  - 初始化 IP5108
  - 读取并打印电池电压、电流、开路电压
  - 监测按键短按/长按事件

  注意：
    * 请根据实际开发板修改 SDA_PIN 和 SCL_PIN
    * 如果使用默认的 Wire（如 UNO/A4-A5），也可将 SDA_PIN/SCL_PIN 改为 0, 0
    * 本示例假设 IP5108 的 I2C 地址为 0x75，如果你实际焊接时更改了 Address，请同步修改
*/

#include <Arduino.h>
#include <Wire.h>
#include "IP5108.h"

// ------------------------ 用户可根据板子修改 ------------------------
static const int SDA_PIN = 21;        // ESP32 默认为 21
static const int SCL_PIN = 22;        // ESP32 默认为 22
static const uint32_t I2C_FREQUENCY = 400000;  // 400kHz
static const uint8_t IP5108_I2C_ADDRESS = 0x75; // 默认地址，若有变化请修改
// ---------------------------------------------------------------------

// 全局 IP5108 对象
// 构造函数：IP5108(TwoWire *i, int sdaPin, int sclPin, uint32_t frequency);
IP5108 ip5108(&Wire, SDA_PIN, SCL_PIN, I2C_FREQUENCY);

void setup() {
  // 串口初始化，用于打印调试信息
  Serial.begin(115200);
  while (!Serial) {
    ; // 等待串口建立
  }
  Serial.println();
  Serial.println("=== IP5108 示例程序 ===");

  // 初始化 I2C（Wire）并传给 IP5108 库
  // 如果 SDA_PIN/SCL_PIN 为 0，则默认使用 Wire.begin()
  Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQUENCY);

  // 设置 IP5108 的 I2C 地址（如果库内部没有自动设置的话）
  ip5108.Address = IP5108_I2C_ADDRESS;

  // 调用库中的 setup（内部会配置各寄存器默认值）
  ip5108.setup();

  Serial.println("IP5108 初始化完成");
  Serial.println("开始循环读取电池参数并监测按键");
  Serial.println("---------------------------------");
}

void loop() {
  // 更新内部状态（例如读取寄存器、刷新缓存）
  ip5108.update();

  // 1. 读取并打印电池电压（单位：伏特）
  float batVolt = ip5108.getBattVoltage();
  Serial.print("Battery Voltage: ");
  Serial.print(batVolt, 3);
  Serial.println(" V");

  // 2. 读取并打印电池电流（单位：毫安/安，根据库内部换算，这里假设返回值以 mA 为单位）
  float batCurr = ip5108.getBattCurrent();
  Serial.print("Battery Current: ");
  Serial.print(batCurr, 2);
  Serial.println(" mA");

  // 3. 读取并打印电池开路电压（OCV，单位：伏特）
  float batOcV = ip5108.getBattOcVoltage();
  Serial.print("Battery OCV: ");
  Serial.print(batOcV, 3);
  Serial.println(" V");

  // 4. 读取芯片状态（可选，库内部会保存到 ip5108.State）
  Serial.print("CHG State Reg: 0x");
  Serial.println(ip5108.State, HEX);

  // 5. 检测按键事件
  //    - readBtn()  读取当前是否有按键按下（按下后需再判断长按或短按）
  //    - isLongPress()  检测是否产生“长按”事件
  //    - isClickPress() 检测是否产生“短按”事件
  if (ip5108.readBtn()) {
    delay(50); // 简单去抖
    if (ip5108.isLongPress()) {
      Serial.println(">> 检测到按键长按");
      // 在此处加入长按要执行的逻辑
    } else if (ip5108.isClickPress()) {
      Serial.println(">> 检测到按键短按");
      // 在此处加入短按要执行的逻辑
    }
  }

  // 6. 如果需要扫描 I2C 总线上的其他设备，可调用 scan_i2c()
  //    ip5108.scan_i2c();
  //    // scan_i2c() 函数内部会打印扫描到的地址

  Serial.println("---------------------------------");

  // 延时 1 秒再循环
  delay(1000);
}