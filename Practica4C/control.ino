// control_horno_pid.ino

#include <PID_v1.h> //Brett Beauregard 1.2.0

// --- Definiciones de pines y constantes ---
#define SSR_PIN 13

// --- Variables del PID ---
double setpoint = 150.0;    // Temperatura objetivo inicial
double input = 0.0;         // Temperatura leída
double output = 0.0;        // Salida del PID

// Valores PID iniciales (pueden ajustarse en tiempo de ejecución)
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// --- Estructura para zonas de temperatura ---
struct Zona {
  float tempObjetivo;
  unsigned long tiempoMs;
};

Zona perfil[] = {
  {150, 60000},  // Precalentamiento
  {180, 30000},  // Remojo
  {240, 20000},  // Reflujo
  {50, 60000},   // Enfriamiento
};

int zonaActual = 0;
unsigned long tiempoInicioZona = 0;
bool perfilActivo = false;

// --- Funciones públicas ---
void iniciarPerfil() {
  zonaActual = 0;
  setpoint = perfil[zonaActual].tempObjetivo;
  tiempoInicioZona = millis();
  perfilActivo = true;
  digitalWrite(SSR_PIN, LOW);
  myPID.SetMode(AUTOMATIC);
}

void actualizarPID(double nuevaTemp) {
  input = nuevaTemp;
  myPID.Compute();
  digitalWrite(SSR_PIN, output > 0.5 ? HIGH : LOW);
}

void actualizarZona() {
  if (!perfilActivo) return;

  if (millis() - tiempoInicioZona > perfil[zonaActual].tiempoMs) {
    zonaActual++;
    if (zonaActual < sizeof(perfil) / sizeof(perfil[0])) {
      setpoint = perfil[zonaActual].tempObjetivo;
      tiempoInicioZona = millis();
    } else {
      perfilActivo = false;
      digitalWrite(SSR_PIN, LOW);  // Apagar horno
    }
  }
}

void configurarPID(double p, double i, double d) {
  Kp = p; Ki = i; Kd = d;
  myPID.SetTunings(Kp, Ki, Kd);
}

// --- Setup del módulo ---
void setupControlHorno() {
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);
  myPID.SetOutputLimits(0, 1);
  myPID.SetMode(MANUAL);
}
