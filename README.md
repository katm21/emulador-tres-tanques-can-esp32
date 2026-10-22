# Emulador de tres tanques acoplados con ESP32 y bus CAN

Infraestructura de emulación en tiempo real de un proceso de tres tanques acoplados, basada en sistemas ciber-físicos de bajo costo, para validar leyes de control digital. Cada tanque es un nodo con un ESP32, un módulo CAN MCP2515 y una pantalla TFT, y los nodos intercambian sus niveles por un bus CAN a 500 kbit/s.

Trabajo de grado: «Infraestructura de emulación en tiempo real basada en sistemas ciber-físicos de bajo costo para la validación de leyes de control digital».
Autores: Katherine Mora y Miguel Agudelo.

## Contenido

- `firmware/`: código de los tres nodos (ESP32), con su propio README.
- `supervision/`: programa de supervisión en Python (`comparador_gui.py`) que lee el bus CAN, compara contra el modelo y guarda métricas, CSV y gráficas.
- `PCB/`: archivos Gerber de las placas, listos para fabricar. `Gerber_T1_AND_T2.zip` sirve para los tanques 1 y 2, y `Gerber_T3.zip` para el tanque 3.
- `Cajas_Tanques_corte/`: archivos de corte láser (DXF y SVG) de las cajas, en MDF de 3 mm y acrílico de 3 mm.
