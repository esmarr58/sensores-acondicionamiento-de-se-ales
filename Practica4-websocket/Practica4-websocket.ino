#include <Arduino.h>
#include <WiFi.h>
//#include <DHT.h>  //By adafruit 1.4.6 mas dependencias
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>  //By lacemra 3.1.0 con modificaciones
#include <AsyncWebSocket.h>
#include <ArduinoJson.h>  //BenoitBlanchon
#include <time.h>


#define ADC_PIN 4  // Cambia este valor según el pin del ADC que estés usando
#define ADC_RESOLUTION 4095  // Resolución del ADC de 12 bits (0-4095)
#define ADC_VREF 3.3  // Voltaje de referencia del ADC
#define MEASUREMENT_TIME 1000  // Tiempo de medición en milisegundos (1 segundo)
const char* ssqid = "GWN571D04";
const char* password = "ESP32CUCEI$$";
const float R0 = 100.0;  // Resistencia a 0°C
const float alpha = 0.00385;  // Coeficiente de temperatura del platino
bool heartbeatReceived = false;  // Indica si ya se ha recibido el primer heartbeat
unsigned long lastAdcSendTime = 0;  // Tiempo de la última lectura del ADC
int tiempoMuestreo = 1000;
int comandoNumerico = 0;
const char* comando;


// Configuración del servidor y WebSocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void blinkTask(void *param) {
    while(1){
    unsigned long startTime = millis();  // Capturar tiempo de inicio
    unsigned int sampleCount = 0;  // Contador de muestras
    float voltageSum = 0.0;  // Suma de voltajes

    // Tomar mediciones durante 1 segundo
    while (millis() - startTime < MEASUREMENT_TIME) {
        int raw_adc = analogRead(ADC_PIN);  // Leer ADC
        float voltage = (raw_adc / (float)ADC_RESOLUTION) * ADC_VREF;  // Convertir a voltaje
        
        voltageSum += voltage;  // Acumular voltaje
        sampleCount++;  // Contar muestras
    }

    // Calcular el voltaje promedio
    float voltajePromedio = voltageSum / sampleCount;

    // Calcular la distancia con la ecuación dada
    float resistencia = 76.09*voltajePromedio+5.75;
    float temperatura = (resistencia - R0) / (alpha * R0);

        DynamicJsonDocument doc(256);
        doc["type"] = "adc_reading";
        doc["temperatura"] = temperatura;
        doc["resistencia"] = resistencia;
        doc["voltaje"] = voltajePromedio;

        // Serializar el JSON y enviarlo
        String jsonString;
        serializeJson(doc, jsonString);

    // Imprimir los resultados
    Serial.print("Muestras: ");
    Serial.print(sampleCount);
    Serial.print(" | Voltaje Promedio: ");
    Serial.print(voltajePromedio, 3);  // Mostrar con 3 decimales
    Serial.print(" Resistencia: ");
    Serial.print(resistencia, 2);  // Mostrar con 2 decimales
    Serial.print(" ohms   |  ");
    Serial.print(" Temperatura: ");
    Serial.print(temperatura, 2);  // Mostrar con 2 decimales
    Serial.println(" °C");
            ws.cleanupClients();


            for (AsyncWebSocketClient *client : ws.getClients()) {
                if (client->status() == WS_CONNECTED) {
                    client->text(jsonString);
                }
            }


    }
  
}



void wsAdcTask(void *param) {
    while (1) {
        // Mantén limpios los clientes inactivos del WebSocket
        ws.cleanupClients();

        // Verificar si ya se ha recibido el primer heartbeat y enviar lecturas cada 100 ms
        if (heartbeatReceived && (millis() - lastAdcSendTime >= tiempoMuestreo)) {
            lastAdcSendTime = millis();  // Actualizar el tiempo de la última lectura
            // Enviar lectura a los clientes conectados
            for (AsyncWebSocketClient *client : ws.getClients()) {
                if (client->status() == WS_CONNECTED) {
                    //sendAdcReading(client);
                }
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Pequeño retardo para no saturar el procesador
    }
}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    analogReadResolution(12);  // Configurar la resolución del ADC a 12 bits
     // Conéctate a la red WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Conectando a WiFi...");
    }
    Serial.println("Conectado a WiFi");

    // Imprime la IP asignada por el router
    Serial.print("IP asignada: ");
    Serial.println(WiFi.localIP());

    // Configura el WebSocket y asigna el manejador de eventos
    ws.onEvent(onWebSocketMessage);
    server.addHandler(&ws);

    // Inicia el servidor
    server.begin();
    Serial.println("Servidor WebSocket iniciado");

     xTaskCreatePinnedToCore(
        wsAdcTask,       // Función de la tarea
        "WS ADC Task",   // Nombre de la tarea
        4096,            // Tamaño de la pila
        NULL,            // Parámetro de entrada (null en este caso)
        1,               // Prioridad
        NULL,            // Puntero a la tarea (no lo usamos)
        0);              // Ejecutar en el core 0
  
    // Crear la tarea para el blink en Core 1
    xTaskCreatePinnedToCore(
        blinkTask,       // Función de la tarea
        "Blink Task",    // Nombre de la tarea
        2048,            // Tamaño de la pila
        NULL,            // Parámetro de entrada (null en este caso)
        1,               // Prioridad
        NULL,            // Puntero a la tarea (no lo usamos)
        1);              // Ejecutar en el core 1

  

}

void updateEsp32Time(unsigned long timestamp) {
    timeval tv;
    tv.tv_sec = timestamp;
    tv.tv_usec = 0;
    settimeofday(&tv, nullptr);  // Actualizar el tiempo de la ESP32
    Serial.println("Tiempo de la ESP32 actualizado.");
}

// Función para manejar mensajes JSON
void onWebSocketMessage(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        Serial.print("Message length: ");
        Serial.println(len);  // Mostrar la longitud del mensaje

        if (len > 0) {
            // Convertir los datos recibidos en una cadena
            String message = String((char*)data);
            Serial.println("Mensaje recibido: " + message);

            // Crear un objeto DynamicJsonDocument para almacenar el JSON
            DynamicJsonDocument doc(1024);

            // Intentar deserializar el mensaje como JSON
            DeserializationError error = deserializeJson(doc, message);

            // Si el mensaje es un JSON válido
            if (!error) {
                Serial.println("Mensaje JSON válido recibido");

                // Comprobar si es un heartbeat
                const char* tipo = doc["type"];
                if (strcmp(tipo, "heartbeat") == 0) {
                    // Obtener el timestamp del heartbeat
                    unsigned long timestamp = doc["timestamp"];
                    Serial.print("Heartbeat recibido con timestamp: ");
                    Serial.println(timestamp);

                    // Actualizar el tiempo de la ESP32
                    updateEsp32Time(timestamp);

                    // Marcar que se ha recibido el primer heartbeat
                    heartbeatReceived = true;
                }
                else if (strcmp(tipo, "sensor") == 0) {

                  
                }
                     
               
        
               
            } else {
            // Si el mensaje no es un JSON válido
             Serial.println("Mensaje no es un JSON válido, ignorando.");
             client->text("Error: Mensaje no es un JSON válido.");
           }
        } else {
            Serial.println("No data received.");
        }
    } else if (type == WS_EVT_CONNECT) {
        Serial.println("Client connected");
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.println("Client disconnected");
    }
}



void loop() {


}
