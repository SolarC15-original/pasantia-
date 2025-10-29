#include <Wire.h>              // Librería para la comunicación I2C, usada por el OLED
#include <Adafruit_GFX.h>      // Biblioteca gráfica base para las pantallas Adafruit
#include <Adafruit_SH110X.h>   // Biblioteca específica para pantallas SH1106 (OLED)
#include <TinyGPS++.h>         // Biblioteca para interpretar datos del GPS
#include <SD.h>                // Biblioteca para manejo de tarjetas SD
#include <SPI.h>               // Biblioteca para la comunicación SPI (necesaria para la SD)

// ================== Configuración OLED ==================
#define SCREEN_WIDTH 128       // Ancho en píxeles de la pantalla OLED
#define SCREEN_HEIGHT 64       // Altura en píxeles de la pantalla OLED
#define OLED_RESET -1          // Pin de reset (no se utiliza un pin físico en este caso)
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define GPS_RX 4               // Pin para recibir datos del GPS
#define GPS_TX 2               // Pin para enviar datos al GPS (aunque realmente solo se recibe)
HardwareSerial GPSSerial(1);   // Se crea un puerto serial adicional para el GPS
TinyGPSPlus gps;               // Objeto para procesar la información del GPS

#define RO 33                  // Pin RX para el sensor de suelo vía RS485
#define DI 32                  // Pin TX para el sensor de suelo
#define RE 26                  // Pin para activar la recepción
#define DE 25                  // Pin para activar la transmisión

#define BUTTON_SAVE 14         // Pin asignado para el botón de guardar datos
#define BUTTON_NEW 27          // Pin asignado para el botón de crear un nuevo archivo
#define BUTTON_REFRESH 12      // Pin asignado para el botón de refrescar pantalla
#define SD_CS 5                // Pin Chip Select para la tarjeta SD
#define DEBOUNCE 200           // Tiempo de "debounce" para evitar múltiples activaciones

volatile uint8_t screenState = 0; // Estado de la pantalla: 0 = Sistema, 1 = Suelo
volatile bool flags[3] = {false, false, false}; // Flags para eventos: [refresh, save, newFile]
unsigned long lastInterrupt = 0;  // Controla el tiempo entre interrupciones
String currentFile = "";          // Guarda el nombre del archivo actual en la SD

// Mutex para proteger el acceso a la estructura de datos compartida entre tareas
SemaphoreHandle_t xSensorDataMutex;

struct SensorData {
  // Datos del GPS
  double lat = 0.0;
  double lng = 0.0;
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  bool gpsValid = false;
  
  // Datos del sensor de suelo
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
// La variable 'datos' ahora debe ser global para ser accesible por ambas tareas
SensorData datos;

class SoilSensor {
private:
  // Comando para solicitar datos del sensor (según protocolo RS485)
  const byte cmd[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x08, 0x44, 0x0C};
  byte response[21]; // Buffer donde se almacena la respuesta del sensor
  
  // Función para combinar dos bytes y formar un entero de 16 bits
  uint16_t combinarBytes(uint8_t high, uint8_t low) {
    return (high << 8) | low;
  }

public:
  // Inicializa el sensor configurando el puerto serie y los pines de control
  void iniciar() {
    Serial2.begin(9600, SERIAL_8N1, RO, DI);
    pinMode(RE, OUTPUT);
    pinMode(DE, OUTPUT);
    digitalWrite(RE, LOW);
    digitalWrite(DE, LOW);
  }

  // Envía el comando al sensor y procesa la respuesta para actualizar los datos
  bool leer(SensorData &data) {
    // Activa la transmisión para enviar el comando
    digitalWrite(DE, HIGH);
    digitalWrite(RE, HIGH);
    delay(10);
    Serial2.write(cmd, 8);
    Serial2.flush();
    // Desactiva la transmisión para empezar a leer la respuesta
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);

    uint8_t idx = 0;
    unsigned long inicio = millis();
    // Espera hasta 1 segundo o hasta recibir los 21 bytes esperados
    while((millis() - inicio < 1000) && (idx < 21)) {
      if(Serial2.available()) {
        response[idx++] = Serial2.read();
      }
    }
    
    // Verifica la integridad de la respuesta y asigna los valores a 'data'
    if(idx == 21 && response[0] == 0x01 && response[1] == 0x03) {
      data.temp = combinarBytes(response[3], response[4]) / 10.0;
      data.humidity = combinarBytes(response[5], response[6]) / 10.0;
      data.ec = combinarBytes(response[7], response[8]);
      data.ph = combinarBytes(response[9], response[10]) / 100.0;
      data.n = combinarBytes(response[11], response[12]);
      data.p = combinarBytes(response[13], response[14]);
      data.k = combinarBytes(response[15], response[16]);
      data.sal = combinarBytes(response[17], response[18]);
      data.sensorValid = true; // Los datos recibidos son válidos
      return true;
    }
    data.sensorValid = false; // Si la respuesta falla, se marca el sensor como inválido
    return false;
  }
};

SoilSensor sensorSuelo;  // Instancia para manejar el sensor de suelo


// --- Función para la Tarea del GPS ---
void taskGPS(void * parameter) {
  for (;;) { // Bucle infinito para la tarea
    while(GPSSerial.available()) {
      if(gps.encode(GPSSerial.read())) {
        // Usar un Mutex para proteger el acceso a la estructura 'datos'
        if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
          datos.gpsValid = gps.location.isValid() && gps.time.isValid();
          if(datos.gpsValid) {
            datos.lat = gps.location.lat();
            datos.lng = gps.location.lng();
            // Ajuste de zona horaria a Colombia
            // Asegúrate de que la hora nunca sea negativa después del ajuste
            datos.hour = (gps.time.hour() - 5 + 24) % 24; 
            datos.minute = gps.time.minute();
            datos.second = gps.time.second();
          }
          xSemaphoreGive(xSensorDataMutex); // Liberar el Mutex
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Pequeña pausa para no monopolizar el núcleo
  }
}

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

// ================== Función setup() ==================
void setup() {
  digitalWrite(22, LOW);  // Inicia el reinicio del sistema
  delay(1200); 
  Serial.begin(115200);   // Inicia la comunicación serial para debug

  // Inicializa la pantalla OLED y muestra la pantalla de inicio
  display.begin(0x3C, true);
  display.setTextColor(SH110X_WHITE);
  mostrarSplash();
  
  // Inicializa el GPS y el sensor de suelo
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  sensorSuelo.iniciar();
  
  // Configura los botones con resistencia interna pull-up
  pinMode(BUTTON_REFRESH, INPUT_PULLUP);
  pinMode(BUTTON_SAVE, INPUT_PULLUP);
  pinMode(BUTTON_NEW, INPUT_PULLUP);
  // Asocia cada botón a su función de interrupción
  attachInterrupt(BUTTON_REFRESH, handleRefresh, FALLING);
  attachInterrupt(BUTTON_SAVE, handleSave, FALLING);
  attachInterrupt(BUTTON_NEW, handleNewFile, FALLING);
  
  // Inicializa la tarjeta SD y crea un archivo nuevo para almacenar datos
  if(!SD.begin(SD_CS)) {
    mostrarError("Error en SD!");
    while(true);
  }
  crearNuevoArchivo();

  // Crear el Mutex antes de crear las tareas
  xSensorDataMutex = xSemaphoreCreateMutex();
  if (xSensorDataMutex == NULL) {
    Serial.println("Error al crear el Mutex");
    mostrarError("Error Mutex!");
    while(true);
  }

  // Crear la tarea para el GPS en el Core 0
  xTaskCreatePinnedToCore(
    taskGPS,         // Función que implementa la tarea
    "GPS_Task",      // Nombre de la tarea
    2048,            // Tamaño de la pila de la tarea (bytes)
    NULL,            // Parámetro a pasar a la tarea (NULL en este caso)
    1,               // Prioridad de la tarea (0 es la más baja, configMAX_PRIORITIES-1 es la más alta)
    NULL,            // Handle de la tarea (NULL si no se necesita)
    0                // Número del núcleo donde se ejecutará (0 para el segundo núcleo)
  );
}

void loop() {
  // El GPS ahora se actualiza en su propia tarea (taskGPS)
  actualizarSensorSuelo(); // Lee los datos del sensor de suelo
  manejarEventos();        // Maneja eventos generados por los botones
  actualizarPantalla();    // Refresca la pantalla OLED
  delay(100);
}


// La función actualizarGPS ya no se llama directamente en loop,
// su lógica ahora está en taskGPS.
// void actualizarGPS() { /* ... */ }

// Realiza una lectura del sensor cada 2 segundos
void actualizarSensorSuelo() {
  static unsigned long ultimaLectura = 0;
  if(millis() - ultimaLectura > 2000) {
    // Tomar el Mutex antes de escribir en 'datos'
    if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
      sensorSuelo.leer(datos);
      xSemaphoreGive(xSensorDataMutex); // Liberar el Mutex
    }
    ultimaLectura = millis();
  }
}

//  Función para Manejar Eventos  
void manejarEventos() {
  if(flags[0]) { // Si se presionó el botón de refrescar
    screenState = (screenState + 1) % 2;
    flags[0] = false;
  }
  
  if(flags[1]) { // Si se presionó el botón de guardar datos
    guardarEnSD();
    flags[1] = false;
    mostrarMensajeTemporal("Dato Guardado");
  }
  
  if(flags[2]) { // Si se presionó el botón para nuevo archivo
    crearNuevoArchivo();
    flags[2] = false;
    mostrarMensajeTemporal("Nuevo Archivo");
  }
}

// Función para Crear un Nuevo Archivo en la SD 
void crearNuevoArchivo() {
  uint8_t numArchivo = 1;
  do {
    currentFile = "/DAT" + String(numArchivo < 10 ? "0" : "") + String(numArchivo) + ".CSV";
    numArchivo++;
  } while(SD.exists(currentFile));
  
  // Abre el archivo y designa los nombres de cada columna
  File archivo = SD.open(currentFile, FILE_WRITE);
  if(archivo) {
    archivo.println("Latitud,Longitud,Hora,Min,Seg,Temp,Humedad,CE,pH,N,P,K,Sal");
    archivo.close();
  }
}

// Función para Guardar Datos en la SD 
void guardarEnSD() {
  // Tomar el Mutex antes de leer de 'datos'
  if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
    File archivo = SD.open(currentFile, FILE_APPEND);
    if(archivo) {
      archivo.printf("%.6f,%.6f,%02d,%02d,%02d,%.1f,%.1f,%d,%.2f,%d,%d,%d,%d\n",
                    datos.lat,
                    datos.lng,
                    datos.hour,
                    datos.minute,
                    datos.second,
                    datos.temp,
                    datos.humidity,
                    datos.ec,
                    datos.ph,
                    datos.n,
                    datos.p,
                    datos.k,
                    datos.sal);
      archivo.close();
    }
    xSemaphoreGive(xSensorDataMutex); // Liberar el Mutex
  }
}

// Función para Actualizar la Pantalla 
void actualizarPantalla() {
  display.clearDisplay();
  display.setCursor(0, 0);
  
  // Tomar el Mutex antes de leer de 'datos' para mostrar
  if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
    if(screenState == 0) { // Muestra la información del sistema
      display.println("== SISTEMA ==");
      
      if(datos.gpsValid) {
        display.printf("Lat:%.6f\n", datos.lat);
        display.printf("Lon:%.6f\n", datos.lng);
      } else {
        display.println("GPS: Sin senal");
      }
      
      display.printf("Hora: %02d:%02d:%02d\n", 
                    datos.hour, 
                    datos.minute, 
                    datos.second);
      
      // Muestra el nombre del archivo actual d
      String nombreCorto = currentFile.substring(1);
      if(nombreCorto.length() > 16) {
        display.println(nombreCorto.substring(0, 16));
        display.println(nombreCorto.substring(16));
      } else {
        display.println(nombreCorto);
      }
    }
    else { // Muestra los datos del sensor de suelo
      display.println("== DATOS SUELO ==");
      if(datos.sensorValid) {
        display.printf("T:%.1fC H:%.1f%%\n", datos.temp, datos.humidity);
        display.printf("CE:%d pH:%.2f\n", datos.ec, datos.ph);
        display.printf("N:%d P:%d K:%d\n", datos.n, datos.p, datos.k);
        display.printf("Sal:%d", datos.sal);
      } else {
        display.println("Error en sensor!");
      }
    }
    xSemaphoreGive(xSensorDataMutex); // Liberar el Mutex
  }
  display.display();  // Envía la información actualizada a la pantalla
}

//  Función para Mostrar la Pantalla de Inicio 
void mostrarSplash() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(15, 20);
  display.print("AGROTECH");
  display.setTextSize(1);
  display.setCursor(30, 45);
  display.print("INICIANDO...");
  display.display();
  delay(2000);
  display.setTextSize(1);
  display.clearDisplay();
}

// Función para Mostrar un Mensaje 
void mostrarMensajeTemporal(const char* msg) {
  display.clearDisplay();
  display.setCursor(10, 28);
  display.print(msg);
  display.display();
  delay(800);
}

//  Función para Mostrar un Mensaje de Error
void mostrarError(const char* msg) {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("ERROR:");
  display.setCursor(0, 20);
  display.print(msg);
  display.display();
}
