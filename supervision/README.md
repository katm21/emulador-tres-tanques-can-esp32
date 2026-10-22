# Programa de supervisión (Python)

`comparador_gui.py` es el programa de supervisión de los tres tanques acoplados. Se conecta al bus CAN por un adaptador USB-CAN, envía las órdenes y las referencias a los nodos ESP32 y grafica en vivo los tres niveles junto con el modelo, que sigue las mismas referencias. Además calcula las métricas de cada prueba y guarda los resultados.

## Qué necesitas

- Python 3 con `pyserial`, `matplotlib` y `numpy` (Tkinter ya viene con Python). En Thonny: *Herramientas > Gestionar paquetes*.
- Un adaptador USB-CAN con protocolo SLCAN, conectado al bus. El programa le habla por puerto serie con los comandos `C`, `Z0`, `S6` (500 kbit/s) y `O`.
- Los tres nodos del [firmware](../firmware) encendidos y conectados al mismo bus.

## Cómo se usa

1. Ejecuta `comparador_gui.py` (F5 en Thonny).
2. Elige el puerto COM y pulsa **Conectar**.
3. Pulsa **RESET** si ya había una prueba y luego **START**.
4. Cambia las referencias cuando quieras: el modelo cambia en el mismo instante.
5. Pulsa **Guardar prueba** para generar los archivos.

Si arrancas el sistema con el botón físico en lugar de START, usa **Marcar t = 0** para que la gráfica empiece desde cero.

## Qué ves en la ventana

| Zona | Contenido |
|---|---|
| Izquierda | Conexión, órdenes (START, RESET, OFF), referencias, modelo, estado de cada tanque y registro |
| Pestañas | *Comparación* (modelo y sistema real), *Sistema real* y *Modelo MATLAB* |
| Abajo | Tabla de los tanques 1 y 2 por tramo (establecimiento, sobreimpulso, valor final y error estacionario) y RMSE |

Cada cambio de referencia inicia un tramo nuevo, y los tiempos de cada tramo se miden desde su inicio.

## Modelo de comparación

| Opción | Qué es |
|---|---|
| Ideal (MATLAB lineal) | Modelo lineal discretizado sin límites |
| Con saturación y antiwindup | El mismo modelo con las bombas limitadas, igual que el firmware |

## Qué se guarda

Al pulsar *Guardar prueba* se crean, en la carpeta desde la que se ejecuta el programa:

- `prueba_<fecha>.csv`: tiempo, niveles reales y niveles del modelo.
- `prueba_<fecha>_comparacion.png`, `_real.png` y `_modelo.png`: las tres gráficas.
- `informe_<fecha>.txt`: métricas de todos los tramos y errores frente al modelo (RMSE, IAE y máximo).

## Configuración

Al inicio del archivo, en la sección *CONFIGURACIÓN*, están el puerto por defecto, los identificadores CAN, el periodo de muestreo y las referencias por defecto. Si cambias algo en el firmware, cámbialo también aquí.
