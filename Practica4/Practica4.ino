#include <Arduino.h>

#define ADC_PIN 4  // Cambia este valor según el pin del ADC que estés usando
#define ADC_RESOLUTION 4095  // Resolución del ADC de 12 bits (0-4095)
#define ADC_VREF 3.3  // Voltaje de referencia del ADC
#define MEASUREMENT_TIME 1000  // Tiempo de medición en milisegundos (1 segundo)
const float R0 = 100.0;  // Resistencia a 0°C
const float alpha = 0.00385;  // Coeficiente de temperatura del platino




void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    analogReadResolution(12);  // Configurar la resolución del ADC a 12 bits

}

void loop() {
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

    delay(1000);  // Esperar 1 segundo antes de la siguiente medición
}
