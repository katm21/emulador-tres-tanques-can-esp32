# Firmware de los nodos (ESP32)

Código de los tres nodos de la infraestructura de emulación en tiempo real de tres tanques acoplados. Cada tanque es un ESP32 que calcula el nivel de su tanque con el modelo discreto (Tm = 0,5 s), lo publica por el bus CAN cada 200 ms y recibe los niveles de los otros dos.

```
firmware/
├── Tanque1/Tanque1.ino   Nodo maestro: decide las referencias y mueve la bomba 1
├── Tanque2/Tanque2.ino   Mueve la bomba 2
└── Tanque3/Tanque3.ino   Sin bomba ni referencia propia, solo calcula su nivel
```

Cada carpeta tiene el mismo nombre que su `.ino`, como pide el Arduino IDE: se abre el archivo y se carga a su ESP32.

## Qué hace cada nodo

| Nodo | Bomba (DAC GPIO 25) | Referencias |
|---|---|---|
| Tanque 1 | Sí | Las decide y las difunde por CAN |
| Tanque 2 | Sí | Las recibe del Tanque 1 |
| Tanque 3 | No | Las recibe del Tanque 1 |

Los tres nodos comparten el mismo pulsador (GPIO 34) y las mismas ganancias del controlador; cada uno resuelve solo la fila del modelo que le corresponde.

## Conexiones comunes

| Función | Pin del ESP32 |
|---|---|
| MCP2515: SCK / MISO / MOSI / CS | 18 / 19 / 23 / 5 |
| Pulsador de sistema | GPIO 34 (con resistencia externa de 10 kΩ a 3V3) |
| Salida a la bomba (DAC) | GPIO 25 (Tanques 1 y 2) |
| Pantalla TFT (ILI9341, paralelo de 8 bits) | Definida en el `User_Setup.h` de TFT_eSPI |

## Mensajes CAN (500 kbit/s)

| ID | Contenido | Quién lo envía |
|---|---|---|
| `0x070` | Orden del sistema: 1 = START, 2 = RESET, 3 = OFF | Cualquier nodo (el que reciba la pulsación) |
| `0x080` | Referencias r1 y r2 (int16 × 10000) | Tanque 1 |
| `0x081` | Pedido de cambio de referencia | Tanques 2 y 3, hacia el Tanque 1 |
| `0x100` / `0x101` / `0x102` | Nivel del tanque 1 / 2 / 3 (int16 × 10000, es decir 0,1 mm) | Cada tanque, cada 200 ms |

## Cómo compilar y cargar

1. Instalar el núcleo **esp32** de Espressif en el Arduino IDE y elegir la placa *ESP32 Dev Module*.
2. Instalar las librerías de la tabla siguiente.
3. Copiar el `User_Setup.h` de este repositorio (carpeta `TFT_eSPI_config/`) a la carpeta de la librería TFT_eSPI, reemplazando el original.
4. Abrir cada `.ino`, cargarlo en su ESP32 y conectar los tres al mismo bus CAN.

| Librería | Uso | Versión usada |
|---|---|---|
| TFT_eSPI | Pantalla TFT | _completar_ |
| arduino-mcp2515 (autowp) | Módulo CAN MCP2515 | _completar_ |
| Núcleo esp32 (Espressif) | Placa y DAC | _completar_ |

## Uso

- **Pulsador:** pulsación corta con el sistema detenido inicia; pulsación corta con el sistema en marcha reinicia; 3 s apaga los tres nodos.
- **Monitor serie (115200 baud):** `REF1(0.25)`, `REF2(0.08)` y `REF?`. El gráfico se ve en *Herramientas > Serial Plotter*.
- **Sin supervisión:** si se enciende el sistema sin el computador, los nodos arrancan con las referencias por defecto r1 = 0,20 m y r2 = 0,10 m.
- **Límites de las referencias:** el Tanque 1 las limita a 0–0,50 m (`R_MIN` y `R_MAX` en `Tanque1.ino`).
