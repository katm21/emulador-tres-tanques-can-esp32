/* ============================================================================
   TANQUE 2  -  nodo con bomba
   Emulación en tiempo real de un sistema de tres tanques acoplados (ESP32 + bus CAN)
   ============================================================================

   ¿Qué hace este nodo?
     - Emula el tanque 2: calcula su nivel con el modelo discreto (Tm = 0,5 s)
       y lo publica por CAN cada 200 ms.
     - Recibe por CAN los niveles de los tanques 1 y 3.
     - Mueve su propia bomba con una salida DAC (GPIO 25).
     - No decide las referencias: las recibe del Tanque 1, y si se le pide un
       cambio (por consola) se lo pide al Tanque 1 por CAN.

   Hardware
     ESP32 DevKit (38 pines) + MCP2515 por SPI (SCK 18, MISO 19, MOSI 23, CS 5,
     cristal de 8 MHz) + pantalla TFT ILI9341 en paralelo de 8 bits (pines en el
     User_Setup.h de TFT_eSPI) + bomba por DAC (GPIO 25) + botón (GPIO 34, con
     10 kΩ externa a 3V3).

   Modelo y control (el mismo en los tres nodos; cada uno calcula solo su fila)
     x[k+1] = Ad·x[k] + Bd·u[k]            modelo discreto linealizado (Tm = 0,5 s)
     z[k+1] = z[k] + Tm·(r - x)            integradores del error de los tanques 1 y 2
     u[k]   = -(K·x[k] + Ki·z[k])          saturada a ±q_bar (anti-windup por
                                           integración condicional)
   Las matrices y las ganancias vienen de MATLAB (ubicación de polos con place).

   Mensajes CAN (500 kbit/s, ID estándar de 11 bits; menor ID = mayor prioridad)
     0x070  SYS      1 byte    orden del sistema: 1 = START, 2 = RESET, 3 = OFF
     0x080  REF      4 bytes   referencias r1 y r2 (int16 x 10000), las difunde el Tanque 1
     0x081  REF_SET  4 bytes   pedido de cambio de referencia hacia el Tanque 1
     0x100  T1       2 bytes   nivel del tanque 1 (int16 x 10000, es decir 0,1 mm)
     0x101  T2       2 bytes   nivel del tanque 2
     0x102  T3       2 bytes   nivel del tanque 3

   Botón de sistema (GPIO 34, el mismo pulsador llega a los tres nodos)
     Pulsación corta con el sistema detenido  -> INICIAR
     Pulsación corta con el sistema en marcha -> RESET
     Pulsación de 3 s                         -> APAGAR (deep sleep)

   Monitor serie (115200 baud)
     REF1(0.25)   cambia la referencia del tanque 1 (en metros)
     REF2(0.08)   cambia la referencia del tanque 2 (en metros)
     REF?         muestra las referencias actuales
     Gráfica: Herramientas > Serial Plotter

   Trabajo de grado: «Infraestructura de emulación en tiempo real basada en sistemas
   ciber-físicos de bajo costo para la validación de leyes de control digital»
   Autor: Miguel Agudelo
   ============================================================================ */

#include <TFT_eSPI.h>
#include <SPI.h>
#include <mcp2515.h>
#include <esp_sleep.h>
#include <driver/gpio.h>

// ============================================================================
//  PANTALLA TFT  -  dibujo del tanque
// ============================================================================
TFT_eSPI tft = TFT_eSPI();                    // objeto de la pantalla (pines en User_Setup.h de TFT_eSPI)
int tankX = 10, tankY = 15, tankW = 220, tankH = 290;  // posición y tamaño del tanque en pantalla (píxeles)

#define COLOR_FONDO   TFT_BLACK               // fondo de la pantalla en marcha
#define COLOR_PARADO  0x3151                  // azul índigo de la pantalla de espera
#define COLOR_SUAVE   0xCE7F                  // letra azul clarito de los textos secundarios
#define COLOR_AGUA    0x7E1F                  // celeste del agua
#define COLOR_TANQUE  0xCE79                  // gris azulado del tanque (parte vacía)
#define COLOR_BORDE   0x7BEF                  // gris del borde y de las líneas de la escala

const float H_FS = 0.6f;                      // altura máxima del tanque (m) = 100 % de la escala
unsigned long prevAnimMillis = 0;             // última vez que se redibujó la pantalla

// ============================================================================
//  BUS CAN  -  MCP2515 a 500 kbit/s
// ============================================================================
MCP2515 mcp2515(5);                           // módulo MCP2515 con el chip select en el GPIO 5
struct can_frame txMsg, rxMsg;                // trama que se envía y trama que se recibe

#define ID_SYS     0x070                      // orden del sistema: 1 = START, 2 = RESET, 3 = OFF
#define ID_REF     0x080                      // referencias r1 y r2 (las difunde el Tanque 1)
#define ID_REF_SET 0x081                      // pedido de cambio de referencia (de T2 o T3 hacia T1)
#define ID_T1      0x100                      // nivel del tanque 1
#define ID_T2      0x101                      // nivel del tanque 2
#define ID_T3      0x102                      // nivel del tanque 3

const float K_LEVEL = 10000.0f;               // el nivel viaja como int16 = metros x 10000 (0,1 mm)

unsigned long lastRx100 = 0, lastRx102 = 0;   // última trama recibida de cada vecino
bool linkT1 = false, linkT3 = false;          // true mientras ese vecino siga enviando datos
const unsigned long CAN_TIMEOUT = 2000;       // 2 s sin datos de un vecino = enlace caído («CAN?»)
unsigned long lastSend = 0;                   // instante del último envío de mi nivel (cada 200 ms)

// Vigilancia del módulo CAN: si el MCP2515 se bloquea (bus-off) se reinicia solo.
unsigned long ultChequeoCan = 0;              // última revisión del vigilante del CAN (cada 250 ms)

// Reinicia el MCP2515 y lo deja otra vez en modo normal a 500 kbit/s.
void canReinicializar() {
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println(F("!! CAN reiniciado"));
}

// Se llama en cada vuelta del loop pero solo revisa cada 250 ms:
//   - bus-off                          -> reinicia el módulo
//   - buffers de llegada desbordados   -> los limpia
//   - muchos errores de envío (TEC)    -> reinicia el módulo
void canWatchdog(unsigned long now) {
  if (now - ultChequeoCan < 250) return;      // solo revisa cada 250 ms
  ultChequeoCan = now;

  uint8_t eflg = mcp2515.getErrorFlags();     // banderas de error del MCP2515 (registro EFLG)

  if (eflg & 0x20) {                          // TXBO: el nodo quedó fuera del bus (bus-off)
    canReinicializar();
    return;
  }
  if (eflg & 0xC0) {                          // RX1OVR / RX0OVR: se desbordó un buffer de recepción
    mcp2515.clearRXnOVR();
    mcp2515.clearInterrupts();
  }
  if (mcp2515.errorCountTX() > 200) canReinicializar();  // TEC muy alto: mejor reiniciar el módulo
}

// Un entero de 16 bits viaja en 2 bytes (byte alto primero): cada nivel ocupa
// solo 2 de los 8 bytes que admite una trama CAN.
void put16(uint8_t *p, int16_t v) {
  p[0] = v >> 8;                              // byte alto
  p[1] = v & 0xFF;                            // byte bajo
}
int16_t get16(const uint8_t *p) {
  return (int16_t)((p[0] << 8) | p[1]);       // junta los dos bytes y recupera el signo
}

// ============================================================================
//  REFERENCIAS  -  las difunde el Tanque 1; este nodo las recibe o las pide
// ============================================================================
float r[2] = {0.2f, 0.1f};                    // referencias [r1, r2] en metros (por defecto 0,20 y 0,10 m)

// Este nodo no decide las referencias: se las pide al Tanque 1 por CAN (ID 0x081)
// y el Tanque 1 las valida y las difunde de vuelta a todos.
void pedirRef(float r1, float r2) {
  struct can_frame f;
  f.can_id  = ID_REF_SET;
  f.can_dlc = 4;
  put16(&f.data[0], lroundf(r1 * K_LEVEL));
  put16(&f.data[2], lroundf(r2 * K_LEVEL));
  mcp2515.sendMessage(&f);
  Serial.println(F(">> Pedido enviado al tanque 1, esperando respuesta..."));
}

// ============================================================================
//  MODELO DE LA PLANTA Y GANANCIAS DEL CONTROLADOR
// ============================================================================
const int   salidaPin = 25;                   // salida analógica (DAC1) que maneja la bomba de este tanque
const float Tm    = 0.5f;                     // periodo de muestreo del modelo discreto (s)
const float q_max = 1.2e-4f;                  // caudal máximo de la bomba (m³/s)
const float q_bar = 0.5f * q_max;             // caudal del punto de operación, la mitad del máximo (m³/s)
const float u_max = +q_bar;                   // saturación superior de la acción de control (m³/s)
const float u_min = -q_bar;                   // saturación inferior de la acción de control (m³/s)

float x[3] = {0, 0, 0};                       // niveles de los tres tanques [h1, h2, h3] en metros
float z[2] = {0, 0};                          // estados de los dos integradores del error
float u[2] = {0, 0};                          // acciones de control [u1, u2] (caudal respecto a q_bar)

const float Ad[3] = {0.0000f, 0.9937f, 0.0033f};  // fila del tanque 2 de Ad (modelo discreto, Tm = 0,5 s)
const float Bd[2] = {0.0001f, 32.3649f};      // fila del tanque 2 de Bd (efecto de las bombas 1 y 2)

// Ganancias de realimentación de estados K (2x3), calculadas en MATLAB por ubicación de polos.
const float Kd[2][3] = {
  { 0.0024f, -0.0006f,  0.0017f},
  {-0.0005f,  0.0022f, -0.0024f}
};
// Ganancias de los integradores Ki (2x2), también calculadas en MATLAB.
const float Kid[2][2] = {
  {-0.0000821f,  0.0000626f},
  { 0.0000307f, -0.0001060f}
};

unsigned long prevControlMillis = 0;          // instante de la última ejecución del control

// ============================================================================
//  LEY DE CONTROL  -  un paso cada 0,5 s
// ============================================================================
// Un paso del controlador (cada Tm = 0,5 s):
//   1) actualiza el nivel del tanque 2 con el modelo discreto
//   2) integra el error de los tanques 1 y 2 (con anti-windup)
//   3) calcula u = -(K·x + Ki·z) saturada a ±q_bar
//   4) saca el caudal por la bomba 2 (DAC)
void leyDeControl() {
  // nivel del tanque 2 con el modelo
  x[1] = (Ad[0] * x[0]) + (Ad[1] * x[1]) + (Ad[2] * x[2])
       + (Bd[0] * u[0]) + (Bd[1] * u[1]);

  float e1 = r[0] - x[0];                     // error del tanque 1 (referencia - nivel)
  float e2 = r[1] - x[1];                     // error del tanque 2 (referencia - nivel)

  // Anti-windup (integración condicional): si la bomba ya está en el tope y el error
  // empuja hacia ese mismo tope, el integrador deja de acumular.
  bool tope1 = (u[0] <= u_min && e1 <= 0) || (u[0] >= u_max && e1 >= 0);
  bool tope2 = (u[1] <= u_min && e2 <= 0) || (u[1] >= u_max && e2 >= 0);
  if (!tope1) z[0] += Tm * e1;                // integrador 1: acumula Tm·e1 salvo que esté saturado
  if (!tope2) z[1] += Tm * e2;                // integrador 2: acumula Tm·e2 salvo que esté saturado

  // Parte proporcional (K·x) y parte integral (Ki·z) de cada acción de control.
  float up1 = (Kd[0][0]*x[0]) + (Kd[0][1]*x[1]) + (Kd[0][2]*x[2]);
  float up2 = (Kd[1][0]*x[0]) + (Kd[1][1]*x[1]) + (Kd[1][2]*x[2]);
  float ui1 = (Kid[0][0]*z[0]) + (Kid[0][1]*z[1]);
  float ui2 = (Kid[1][0]*z[0]) + (Kid[1][1]*z[1]);

  // Ley de control u = -(K·x + Ki·z), limitada a [u_min, u_max].
  u[0] = constrain(-(up1 + ui1), u_min, u_max);
  u[1] = constrain(-(up2 + ui2), u_min, u_max);

  // Salida a la bomba 2: caudal q_bar + u, escalado a 0-255 para el DAC.
  int dacVal = ((q_bar + u[1]) / q_max) * 255.0f;  // caudal pedido como fracción del máximo, escalado a 8 bits
  dacWrite(salidaPin, constrain(dacVal, 0, 255));  // tensión de control hacia la bomba
}

// ============================================================================
//  PANTALLAS  -  espera y tanque en marcha
// ============================================================================
// Pantalla de espera: nombre del tanque e instrucciones del botón.
void pantallaParado() {
  int cx = tft.width() / 2;
  tft.fillScreen(COLOR_PARADO);
  tft.setTextDatum(MC_DATUM);                 // texto centrado

  tft.setTextColor(TFT_WHITE, COLOR_PARADO);
  tft.setTextSize(4);
  tft.drawString("TANQUE 2", cx, 90);
  tft.drawFastHLine(30, 125, tft.width() - 60, TFT_WHITE);

  tft.setTextSize(2);
  tft.drawString("Sistema detenido", cx, 155);

  tft.setTextColor(COLOR_SUAVE, COLOR_PARADO);
  tft.drawString("Pulse el boton", cx, 210);
  tft.drawString("para iniciar", cx, 232);
  tft.drawString("3 s = apagar", cx, 280);

  tft.setTextDatum(TL_DATUM);
}

// Dibuja el contorno del tanque y las líneas que separan textos y escala.
// Se llama una sola vez, al iniciar.
void dibujarMarco() {
  tft.fillScreen(COLOR_FONDO);
  tft.fillRoundRect(tankX, tankY, tankW, tankH, 10, COLOR_TANQUE);
  tft.drawRoundRect(tankX, tankY, tankW, tankH, 10, COLOR_BORDE);
  tft.drawRoundRect(tankX - 1, tankY - 1, tankW + 2, tankH + 2, 10, TFT_WHITE);
  tft.drawFastHLine(tankX, tankY + 45, tankW, COLOR_BORDE);
  tft.drawFastHLine(tankX, tankY + tankH - 20, tankW, COLOR_BORDE);
}

// Redibuja el agua según el nivel propio, la escala lateral y los textos
// (nivel, referencia y estado del enlace CAN). Se llama cada 250 ms.
void dibujarNivel() {
  float nivel = constrain(x[1] / H_FS, 0.0f, 1.0f);  // nivel normalizado (0 a 1) respecto a la altura máxima

  int innerX = tankX + 2, innerW = tankW - 4;
  int arriba = tankY + 46;                    // debajo de los textos
  int fondo  = (tankY + tankH) - 21;
  int altoMax = fondo - arriba;
  int aguaY = fondo - (int)(nivel * altoMax);

  tft.fillRect(innerX, arriba, innerW, aguaY - arriba, COLOR_TANQUE);
  tft.fillRect(innerX, aguaY, innerW, fondo - aguaY, COLOR_AGUA);

  // Marcas de la escala cada 20 % de la altura (0 a 0,6 m).
  tft.setTextSize(1);
  for (int i = 0; i <= 100; i += 20) {
    int markY = fondo - map(i, 0, 100, 0, altoMax);
    tft.drawFastHLine(tankX + 4, markY, 12, TFT_BLACK);
    tft.setTextColor(TFT_BLACK, (markY > aguaY) ? COLOR_AGUA : COLOR_TANQUE);
    tft.setCursor(tankX + 20, markY - 3);
    tft.print((i / 100.0f) * H_FS, 1);
  }

  tft.setTextSize(2);
  tft.setTextColor(TFT_BLACK, COLOR_TANQUE);
  tft.setCursor(tankX + 8, tankY + 8);
  tft.print("T2: "); tft.print(x[1], 3); tft.print("   ");
  tft.setCursor(tankX + 8, tankY + 26);
  tft.print("Ref:"); tft.print(r[1], 3);
  tft.print((linkT1 && linkT3) ? " OK " : " CAN?");
}

// ============================================================================
//  BOTÓN DE SISTEMA  -  GPIO 34, compartido por los tres nodos
// ============================================================================
// Corto parado = iniciar | corto en marcha = reset | 3 s = apagar
// GPIO 34 no tiene pull-up interno: lleva 10k externa a 3V3
#define PIN_BOTON 34                          // pulsador compartido; GPIO solo de entrada, con pull-up externo
const unsigned long BTN_DEBOUNCE_MS = 40;     // tiempo para ignorar los rebotes (ms)
const unsigned long BTN_LARGA_MS    = 3000;   // duración de la pulsación larga = apagar (ms)

bool sysRun = false;                          // false = detenido, true = en marcha

// Como los 3 nodos ven el mismo botón, estas esperas evitan que un nodo
// tome la soltura del botón como un RESET que no es.
unsigned long sysCmdMs = 0;       // ultima orden que llego por CAN
unsigned long sysRunMs = 0;       // cuando arranco
const unsigned long BTN_GUARDA_MS = 1500;     // tras una orden por CAN se ignora el botón durante 1,5 s
const unsigned long RUN_MIN_MS    = 2500;     // el RESET por botón solo vale pasados 2,5 s del arranque

bool btnCrudoPrev = false, btnEstable = false, btnLargaHecha = false;  // lectura previa, estado sin rebotes y «larga ya atendida»
unsigned long btnUltMuestreo = 0, btnCambioMs = 0, btnPulsadoMs = 0;

#define SYS_START 1                           // arrancar el sistema
#define SYS_RESET 2                           // reiniciar los nodos (vuelven a la pantalla de espera)
#define SYS_OFF   3                           // apagar los nodos (deep sleep)

// Manda una orden de sistema (START, RESET u OFF) a los otros nodos por CAN.
void enviarSys(uint8_t cmd) {
  struct can_frame f;
  f.can_id  = ID_SYS;
  f.can_dlc = 1;
  f.data[0] = cmd;
  mcp2515.sendMessage(&f);
  delay(30);                      // que alcance a salir antes de reiniciar
}

// Arranca el control y dibuja el tanque. Se ejecuta con el botón local o con la orden START.
void iniciarSistema() {
  if (sysRun) return;
  sysRun = true;
  sysRunMs = millis();
  dibujarMarco();
  prevControlMillis = millis();
  prevAnimMillis    = millis();
  Serial.println(F(">> START"));
}

// Apaga el nodo: muestra «APAGADO», duerme la pantalla y entra en deep sleep.
// Se despierta al pulsar el botón (GPIO 34).
void apagarSistema() {
  Serial.println(F(">> APAGANDO. Pulse el boton para encender."));
  tft.fillScreen(COLOR_PARADO);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, COLOR_PARADO);
  tft.setTextSize(3);
  tft.drawString("APAGADO", tft.width() / 2, tft.height() / 2);
  delay(600);
  tft.fillScreen(TFT_BLACK);
  tft.writecommand(0x10);         // pantalla a dormir

  // esperar a que suelten el boton para que no se despierte de una
  unsigned long t0 = millis();
  while (digitalRead(PIN_BOTON) == LOW && millis() - t0 < 8000) delay(10);
  delay(100);

  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BOTON, 0);   // despierta con el boton
  esp_deep_sleep_start();
}

// Reinicia el ESP32; al arrancar vuelve a la pantalla de espera.
void reiniciarSistema() {
  Serial.println(F(">> RESET"));
  delay(50);
  ESP.restart();
}

// Pulsación corta: si está detenido ARRANCA; si está en marcha hace RESET.
void pulsacionCorta() {
  if (millis() - sysCmdMs < BTN_GUARDA_MS) return;   // ya llego por CAN

  if (!sysRun) {
    enviarSys(SYS_START);
    iniciarSistema();
  }
  else if (millis() - sysRunMs >= RUN_MIN_MS) {
    enviarSys(SYS_RESET);
    reiniciarSistema();
  }
}

// Pulsación larga (3 s): apaga los tres nodos.
void pulsacionLarga() {
  enviarSys(SYS_OFF);
  apagarSistema();
}

void atenderSysCan(uint8_t cmd) {             // orden de otro nodo: no se reenvía, para no entrar en bucle
  sysCmdMs = millis();
  if      (cmd == SYS_START) iniciarSistema();
  else if (cmd == SYS_RESET) reiniciarSistema();
  else if (cmd == SYS_OFF)   apagarSistema();
}

// Configura el pulsador y espera a que lo suelten si venimos de un apagado.
void configurarBoton() {
  pinMode(PIN_BOTON, INPUT);
  // si viene de apagado el boton puede seguir presionado
  unsigned long t0 = millis();
  while (digitalRead(PIN_BOTON) == LOW && millis() - t0 < 8000) delay(10);
  delay(BTN_DEBOUNCE_MS);
  btnCrudoPrev = btnEstable = false;
  btnCambioMs = millis();
}

// Lee el botón cada 5 ms, filtra rebotes y distingue la pulsación corta (al soltar)
// de la larga (3 s).
void botonTask() {
  unsigned long now = millis();
  if (now - btnUltMuestreo < 5) return;     // se lee cada 5 ms
  btnUltMuestreo = now;

  bool crudo = (digitalRead(PIN_BOTON) == LOW);   // LOW = presionado

  if (crudo != btnCrudoPrev) {               // rebote, vuelve a contar
    btnCrudoPrev = crudo;
    btnCambioMs  = now;
    return;
  }
  if (now - btnCambioMs < BTN_DEBOUNCE_MS) return;

  if (crudo != btnEstable) {
    btnEstable = crudo;
    if (btnEstable) { btnPulsadoMs = now; btnLargaHecha = false; }
    else if (!btnLargaHecha) pulsacionCorta();     // actua al soltar
  }
  else if (btnEstable && !btnLargaHecha && (now - btnPulsadoMs >= BTN_LARGA_MS)) {
    btnLargaHecha = true;
    pulsacionLarga();
  }
}

// ============================================================================
//  MONITOR SERIE Y GRÁFICA
// ============================================================================
String lineaSerial;                           // línea que se va armando con lo que llega por el monitor serie

// Imprime niveles y referencias con el formato «nombre:valor,...» del Serial Plotter.
void enviarGrafica() {
  Serial.print(F("Nivel1:")); Serial.print(x[0], 4);
  Serial.print(F(",Nivel2:")); Serial.print(x[1], 4);
  Serial.print(F(",Nivel3:")); Serial.print(x[2], 4);
  Serial.print(F(",Ref1:"));   Serial.print(r[0], 4);
  Serial.print(F(",Ref2:"));   Serial.println(r[1], 4);
}

// Extrae el número entre paréntesis de un comando, por ejemplo REF1(0.25).
bool extraerValor(const String &s, const char *clave, float &v) {
  int i = s.indexOf(clave);
  if (i < 0) return false;
  int a = s.indexOf('(', i);
  int b = s.indexOf(')', a);
  if (a < 0 || b < 0 || b <= a + 1) return false;
  v = s.substring(a + 1, b).toFloat();
  return true;
}

// Comandos del monitor serie: REF1(valor), REF2(valor) y REF?.
void manejarComando(String s) {
  s.trim(); s.toUpperCase();
  float v;
  if (extraerValor(s, "REF1", v))      pedirRef(v, r[1]);
  else if (extraerValor(s, "REF2", v)) pedirRef(r[0], v);
  else if (s.indexOf("REF?") >= 0) {
    Serial.print(F(">> r1=")); Serial.print(r[0], 3);
    Serial.print(F(" r2="));   Serial.println(r[1], 3);
  }
  else Serial.println(F(">> Comandos: REF1(valor)  REF2(valor)  REF?"));
}

// Arma la línea carácter a carácter y la ejecuta al llegar el salto de línea.
void leerSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineaSerial.length()) manejarComando(lineaSerial);
      lineaSerial = "";
    } else if (lineaSerial.length() < 40) lineaSerial += c;
  }
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("TANQUE 2"));
  Serial.println(F("Comandos: REF1(valor)  REF2(valor)  REF?"));

  // soltar los pines retenidos si venimos de apagado (deep sleep)
  gpio_deep_sleep_hold_dis();

  SPI.begin(18, 19, 23, 5);                   // SCK = 18, MISO = 19, MOSI = 23, CS = 5 hacia el MCP2515
  if (mcp2515.reset() != MCP2515::ERROR_OK) Serial.println(F("!! reset CAN fallo"));
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK)
    Serial.println(F("!! setBitrate fallo (cristal 16MHz?)"));
  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) Serial.println(F("!! normalMode fallo"));

  configurarBoton();

  tft.init();
  tft.setRotation(0);                         // pantalla en vertical
  pantallaParado();                           // se arranca en la pantalla de espera

  dacWrite(salidaPin, (int)((q_bar / q_max) * 255.0f));  // mientras se espera el START la bomba queda en q_bar (u = 0)
}

// ============================================================================
//  LOOP PRINCIPAL
// ============================================================================
void loop() {
  unsigned long now = millis();

  leerSerial();                               // atiende los comandos del monitor serie
  botonTask();                                // atiende el pulsador

  // --- 1. Recibir todo lo que haya llegado por CAN ---
  while (mcp2515.readMessage(&rxMsg) == MCP2515::ERROR_OK) {
    if (rxMsg.can_id == ID_SYS && rxMsg.can_dlc >= 1) {  // orden de sistema (START / RESET / OFF)
      atenderSysCan(rxMsg.data[0]);
      continue;
    }
    if (rxMsg.can_id == ID_REF && rxMsg.can_dlc >= 4) {     // referencias del tanque 1
      float nr1 = get16(&rxMsg.data[0]) / K_LEVEL;
      float nr2 = get16(&rxMsg.data[2]) / K_LEVEL;
      if (nr1 != r[0] || nr2 != r[1]) {
        r[0] = nr1;  r[1] = nr2;
        Serial.print(F(">> REF recibida: r1=")); Serial.print(r[0], 3);
        Serial.print(F(" r2="));                 Serial.println(r[1], 3);
      }
      continue;
    }
    if (rxMsg.can_dlc < 2) continue;          // un nivel necesita al menos 2 bytes

    if (rxMsg.can_id == ID_T1) {
      x[0] = get16(&rxMsg.data[0]) / K_LEVEL;  // de entero a metros
      lastRx100 = now;  linkT1 = true;
    }
    else if (rxMsg.can_id == ID_T3) {
      x[2] = get16(&rxMsg.data[0]) / K_LEVEL;  // de entero a metros
      lastRx102 = now;  linkT3 = true;
    }
  }

  canWatchdog(now);                           // revisa la salud del módulo CAN
  if (linkT1 && (now - lastRx100 > CAN_TIMEOUT)) linkT1 = false;  // más de 2 s sin datos del tanque 1: enlace caído
  if (linkT3 && (now - lastRx102 > CAN_TIMEOUT)) linkT3 = false;  // más de 2 s sin datos del tanque 3: enlace caído

  // --- 2. Control: un paso cada 0,5 s ---
  if (sysRun && now - prevControlMillis >= 500) {
    prevControlMillis = now;
    leyDeControl();
  }

  // --- 3. Publicar mi nivel por CAN cada 200 ms ---
  if (now - lastSend >= 200) {
    lastSend = now;
    txMsg.can_id  = ID_T2;
    txMsg.can_dlc = 2;                        // 2 bytes de datos (un int16)
    put16(&txMsg.data[0], lroundf(x[1] * K_LEVEL));  // nivel en metros x 10000, redondeado
    mcp2515.sendMessage(&txMsg);
  }

  // --- 5. Gráfica serie (cada 100 ms) y pantalla (cada 250 ms) ---
  static unsigned long prevGrafica = 0;       // instante de la última línea enviada al Serial Plotter
  if (sysRun && now - prevGrafica >= 100) {
    prevGrafica = now;
    enviarGrafica();
  }
  if (sysRun && now - prevAnimMillis >= 250) {
    prevAnimMillis = now;
    dibujarNivel();
  }
}