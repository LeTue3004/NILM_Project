#include <TFT_eSPI.h>
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI(); // Khởi tạo đối tượng TFT

void setup() {
  tft.init(); // Khởi động màn hình
  tft.setRotation(1); // Đặt độ xoay
  tft.fillScreen(TFT_BLACK); // Đặt nền màu đen
}

void loop() {
  // Vẽ một ô trắng lấp đầy toàn bộ màn hình
  tft.fillScreen(TFT_WHITE);
  delay(2000); // Hiển thị trong 2 giây

  // Vẽ một ô đen lấp đầy toàn bộ màn hình
  tft.fillScreen(TFT_BLACK);
  delay(2000); // Hiển thị trong 2 giây
}