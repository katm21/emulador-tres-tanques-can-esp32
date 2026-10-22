# Emulador de tres tanques acoplados con ESP32 y bus CAN

Infraestructura de emulación en tiempo real de un proceso de tres tanques acoplados, basada en sistemas ciber-físicos de bajo costo, para validar leyes de control digital. Cada tanque es un nodo con un ESP32, un módulo CAN MCP2515 y una pantalla TFT, y los nodos intercambian sus niveles por un bus CAN a 500 kbit/s.

Trabajo de grado: «Infraestructura de emulación en tiempo real basada en sistemas ciber-físicos de bajo costo para la validación de leyes de control digital».
Autores: Katherine Mora y Miguel Agudelo.

## Contenido

- `firmware/`: código de los tres nodos (ESP32), con su propio README.

