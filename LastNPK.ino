#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <SPI.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define GPS_RX 4
#define GPS_TX 2
HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

#define RO 33
#define DI 32
#define RE 26
#define DE 25

#define BUTTON_SAVE 14
#define BUTTON_NEW 27
#define BUTTON_REFRESH 12
#define SD_CS 5
#define DEBOUNCE 200

volatile uint8_t screenState = 0;
volatile bool flags[3] = {false, false, false};
unsigned long lastInterrupt = 0;
String currentFile = "";

// >>> CAMBIO: estructura con campos de fecha
struct SensorData {
  double lat = 0.0;
  double lng = 0.0;
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  uint8_t day = 0;
  uint8_t month = 0;
  uint16_t year = 0;
  bool gpsValid = false;
  float temp = -1.0;
  float humidity = -1.0;
  unsigned int ec = 0;
  float ph = -1.0;
  unsigned int n = 0;
  unsigned int p = 0;
  unsigned int k = 0;
  unsigned int sal = 0;
  bool sensorValid = false;
};

SensorData datos;

class SoilSensor {
private:
  const byte cmd[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x08, 0x44, 0x0C};
  byte response[21];

  uint16_t combinarBytes(uint8_t high, uint8_t low) {
    return (high << 8) | low;
  }

public:
  void iniciar() {
    Serial2.begin(9600, SERIAL_8N1, RO, DI);
    pinMode(RE, OUTPUT);
    pinMode(DE, OUTPUT);
    digitalWrite(RE, LOW);
    digitalWrite(DE, LOW);
  }

  bool leer(SensorData &data) {
    digitalWrite(DE, HIGH);
    digitalWrite(RE, HIGH);
    delay(10);
    Serial2.write(cmd, 8);
    Serial2.flush();
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);

    uint8_t idx = 0;
    unsigned long inicio = millis();
    while((millis() - inicio < 1000) && (idx < 21)) {
      if(Serial2.available()) {
        response[idx++] = Serial2.read();
      }
    }

    if(idx == 21 && response[0] == 0x01 && response[1] == 0x03) {
      data.temp = combinarBytes(response[3], response[4]) / 10.0;
      data.humidity = combinarBytes(response[5], response[6]) / 10.0;
      data.ec = combinarBytes(response[7], response[8]);
      data.ph = combinarBytes(response[9], response[10]) / 100.0;
      data.n = combinarBytes(response[11], response[12]);
      data.p = combinarBytes(response[13], response[14]);
      data.k = combinarBytes(response[15], response[16]);
      data.sal = combinarBytes(response[17], response[18]);
      data.sensorValid = true;
      return true;
    }
    data.sensorValid = false;
    return false;
  }
};

SoilSensor sensorSuelo;

void IRAM_ATTR handleRefresh() {
  if(millis() - lastInterrupt > DEBOUNCE) {
    flags[0] = true;
    lastInterrupt = millis();
  }
}

void IRAM_ATTR handleSave() {
  if(millis() - lastInterrupt > DEBOUNCE) {
    flags[1] = true;
    lastInterrupt = millis();
  }
}

void IRAM_ATTR handleNewFile() {
  if(millis() - lastInterrupt > DEBOUNCE) {
    flags[2] = true;
    lastInterrupt = millis();
  }
}

void crearNuevoArchivo() {
  uint8_t numArchivo = 1;
  do {
    currentFile = "/DAT" + String(numArchivo < 10 ? "0" : "") + String(numArchivo) + ".CSV";
    numArchivo++;
  } while(SD.exists(currentFile));

  File archivo = SD.open(currentFile, FILE_WRITE);
  if(archivo) {
    // >>> CAMBIO: Se agrega fecha a los encabezados
    archivo.println("Fecha,Latitud,Longitud,Hora,Min,Seg,Temp,Humedad,CE,pH,N,P,K,Sal");
    archivo.close();
  }
}

void guardarEnSD() {
  File archivo = SD.open(currentFile, FILE_APPEND);
  if(archivo) {
    // >>> CAMBIO: incluye fecha en formato DD/MM/AAAA
    archivo.printf("%02d/%02d/%04d,%.6f,%.6f,%02d,%02d,%02d,%.1f,%.1f,%d,%.2f,%d,%d,%d,%d\n",
                  datos.day, datos.month, datos.year,
                  datos.lat, datos.lng, datos.hour, datos.minute, datos.second,
                  datos.temp, datos.humidity, datos.ec, datos.ph,
                  datos.n, datos.p, datos.k, datos.sal);
    archivo.close();
  }
}

void actualizarPantalla() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  Serial.println("---------------");

  if (screenState == 0) {
    display.println("== SISTEMA ==");
    Serial.println("== SISTEMA ==");

    if (datos.gpsValid) {
      display.printf("Lat:%.6f\n", datos.lat);
      display.printf("Lon:%.6f\n", datos.lng);
      display.printf("%02d/%02d/%04d\n", datos.day, datos.month, datos.year); // >>> CAMBIO
      Serial.printf("Fecha: %02d/%02d/%04d\n", datos.day, datos.month, datos.year);
    } else {
      display.println("GPS: Sin senal");
      Serial.println("GPS: Sin senal");
    }

    display.printf("Hora: %02d:%02d:%02d\n", datos.hour, datos.minute, datos.second);
    Serial.printf("Hora: %02d:%02d:%02d\n", datos.hour, datos.minute, datos.second);

    String nombreCorto = currentFile.substring(1);
    if (nombreCorto.length() > 16) {
      display.println(nombreCorto.substring(0, 16));
      display.println(nombreCorto.substring(16));
    } else {
      display.println(nombreCorto);
    }
  } else {
    display.println("== DATOS SUELO ==");
    if (datos.sensorValid) {
      display.printf("T:%.1fC Hm:%.1f%%\n", datos.temp, datos.humidity);
      display.printf("CE:%d  pH:%.2f\n", datos.ec, datos.ph);
      display.printf("N:%d P:%d K:%d\n", datos.n, datos.p, datos.k);
      display.printf("Sal:%d", datos.sal);
    } else {
      display.println("Error en sensor!");
    }
  }

  display.display();
}

void mostrarSplash() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(15, 20);
  display.print("NPK-CISNA");
  display.setTextSize(1);
  display.setCursor(30, 45);
  display.print("INICIANDO...");
  display.display();
  delay(2000);
  display.clearDisplay();
}

void mostrarMensajeTemporal(const char* msg) {
  display.clearDisplay();
  display.setCursor(10, 28);
  display.print(msg);
  display.display();
  delay(800);
}

void mostrarError(const char* msg) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("ERROR:");
  display.setCursor(0, 20);
  display.print(msg);
  display.display();
}

void actualizarSensorSuelo() {
  static unsigned long ultimaLectura = 0;
  if(millis() - ultimaLectura > 2000) {
    sensorSuelo.leer(datos);
    ultimaLectura = millis();
  }
}

void manejarEventos() {
  if(flags[0]) {
    screenState = (screenState + 1) % 2;
    flags[0] = false;
  }
  if(flags[1]) {
    guardarEnSD();
    flags[1] = false;
    mostrarMensajeTemporal("Dato Guardado");
  }
  if(flags[2]) {
    crearNuevoArchivo();
    flags[2] = false;
    mostrarMensajeTemporal("Nuevo Archivo");
  }
}

void taskGPS(void *pvParameters) {
  while (true) {
    while(GPSSerial.available()) {
      char c = GPSSerial.read();
      gps.encode(c);
      if(gps.location.isValid() && gps.time.isValid() && gps.date.isValid()) { // >>> CAMBIO
        datos.gpsValid = true;
        datos.lat = gps.location.lat();
        datos.lng = gps.location.lng();
        datos.hour = (gps.time.hour() + 19) % 24; // Ajuste UTC -5
        datos.minute = gps.time.minute();
        datos.second = gps.time.second();
        datos.day = gps.date.day();
        datos.month = gps.date.month();
        datos.year = gps.date.year();
      } else {
        datos.gpsValid = false;
      }
    }
    delay(10);
  }
}

void taskMain(void *pvParameters) {
  while (true) {
    actualizarSensorSuelo();
    manejarEventos();
    actualizarPantalla();
    delay(100);
  }
}

void setup() {
  Serial.begin(115200);
  display.begin(0x3C, true);
  display.setTextColor(SH110X_WHITE);
  mostrarSplash();

  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  sensorSuelo.iniciar();

  pinMode(BUTTON_REFRESH, INPUT_PULLUP);
  pinMode(BUTTON_SAVE, INPUT_PULLUP);
  pinMode(BUTTON_NEW, INPUT_PULLUP);
  attachInterrupt(BUTTON_REFRESH, handleRefresh, FALLING);
  attachInterrupt(BUTTON_SAVE, handleSave, FALLING);
  attachInterrupt(BUTTON_NEW, handleNewFile, FALLING);

  if(!SD.begin(SD_CS)) {
    mostrarError("Error en SD!");
    while(true);
  }
  crearNuevoArchivo();

  xTaskCreatePinnedToCore(taskGPS, "TaskGPS", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskMain, "TaskMain", 8192, NULL, 1, NULL, 1);
}

void loop() {
  // No se usa porque trabajamos con tareas en los dos núcleos
}