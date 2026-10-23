/**
  *  Беспроводной COM-порт для STM32 телеметрии через ESP8266 (D1 mini).
  *
  *  Заливается в Arduino IDE (плата Lolin(Wemos) D1 mini).
  *  Роль: прозрачный TCP-сервер на порту 23. Всё, что STM32 шлёт по
  *  USART3, уходит клиенту (ПК/смартфон) и наоборот.
  *
  *  СВЯЗЬ С STM32:
  *    D1 mini GPIO13(RX) <- STM32 PB10(USART3_TX)
  *    D1 mini GPIO15(TX) -> STM32 PB11(USART3_RX)
  *    сделать Serial.swap() — ОБЯЗАТЕЛЬНО, см. ниже.
  *
  *  ПИТАНИЕ: +5V на пин 5V модуля (у него СВОЙ регулятор, НЕ вешать на 3.3V
  *  Blue Pill — там регулятор уже отдаёт ток на МК и USB). C 470 мкФ у модуля
  *  обязателен. +5V источник ~700 мА.
  */

#include <ESP8266WiFi.h>

const char* SSID = "ваша_сеть";
const char* PASS = "пароль";
const uint32_t BAUD = 460800;          // должна совпадать с WIFI_BAUD в mc_wifi.h

WiFiServer server(23);
WiFiClient client;

void setup() {
  Serial.begin(BAUD);
  // ОБЯЗАТЕЛЬНО. Штатные пины UART0 (D9/RX, D10/TX) заняты загрузчиком и
  // отладочным выводом: при старте туда сыплется мусор на 74880 бод,
  // который пошёл бы в STM32. swap() переводит аппаратный UART0 на
  // GPIO13(RX) / GPIO15(TX), а загрузочный мусор остаётся на штатных.
  Serial.swap();

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  while (WiFi.status() != WL_CONNECTED) delay(200);

  server.begin();
  server.setNoDelay(true);   // без этого Nagle склеит кадры и добавит 40 мс
}

void loop() {
  if (server.hasClient()) {
    if (!client || !client.connected()) client = server.available();
    else server.available().stop();     // один клиент за раз
  }

  // UART -> Wi-Fi (телеметрия)
  static uint8_t buf[1024];
  int n = (client && client.connected()) ? Serial.available() : 0;
  if (n > 0) {
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    Serial.readBytes(buf, n);
    client.write(buf, n);
  }

  // Wi-Fi -> UART (команды управления от ПК/телефона)
  while (client && client.available()) Serial.write(client.read());
}