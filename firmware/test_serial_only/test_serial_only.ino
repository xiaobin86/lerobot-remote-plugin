/**
 * 最小串口测试 - 用于验证 ESP32 硬件和 USB 串口是否正常
 *
 * 烧录后打开串口监视器(115200)，应该每秒看到一行 "Alive: X"
 * 如果什么都看不到，说明是硬件/USB线/驱动问题，和项目代码无关。
 */

void setup() {
  // 尝试多种波特率，确保至少有一种能匹配串口监视器
  Serial.begin(115200);
  while (!Serial) { ; }  // 等待串口连接 (仅 Leonardo/Micro 需要，ESP32 会立即通过)
  delay(500);

  Serial.println("=================================");
  Serial.println("ESP32 Serial Test OK!");
  Serial.println("If you see this, USB serial works.");
  Serial.println("=================================");
}

int counter = 0;

void loop() {
  Serial.print("Alive: ");
  Serial.println(counter++);
  delay(1000);  // 每秒输出一次
}
