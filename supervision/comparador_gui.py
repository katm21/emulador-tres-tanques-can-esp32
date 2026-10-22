"""
comparador_gui.py
=================
Programa de supervisión de los tres tanques acoplados (ESP32 + bus CAN).

Qué hace
--------
Compara en una sola ventana el MODELO (réplica del de MATLAB) con el SISTEMA REAL
(los tres nodos ESP32 hablando por CAN):
  - Izquierda: conexión, órdenes (START / RESET / OFF), referencias, modelo, estado.
  - Derecha: gráficas en pestañas (Comparación, Sistema real y Modelo MATLAB) y,
    debajo, la tabla comparativa de los tanques 1 y 2 por tramo.
El modelo está integrado en este archivo y sigue las mismas referencias que se
envíen al sistema real, así que las curvas se pueden comparar en vivo.

Qué necesita
------------
  - Python 3 con:  pyserial  matplotlib  numpy   (tkinter ya viene con Python).
    En Thonny: Herramientas > Gestionar paquetes.
  - Un adaptador USB-CAN con protocolo SLCAN conectado al bus (el programa le habla
    por puerto serie con los comandos C, Z0, S6 y O) y los tres nodos encendidos.

Cómo se usa
-----------
  1) Ejecuta el archivo (F5 en Thonny).
  2) Elige el puerto COM y pulsa Conectar.
  3) RESET (si ya había una prueba) y luego START.
  4) Cambia las referencias cuando quieras: el modelo cambia en el mismo instante.
  5) Guardar prueba: crea el CSV, las 3 gráficas en PNG y el informe completo
     (todas las métricas) en un archivo de texto, en la carpeta desde la que se ejecuta.

Modelo de comparación
---------------------
  IDEAL = modelo lineal de MATLAB sin límites.
  SAT   = el mismo modelo con saturación de las bombas y anti-windup, como el firmware.

Cómo está organizado el archivo
-------------------------------
  CONFIGURACIÓN > CAN POR SERIE > MODELO > DATOS COMPARTIDOS > ANÁLISIS >
  COMANDOS > GRÁFICAS > INTERFAZ
"""

import threading
import time
import datetime
import queue

import numpy as np                              # cálculo numérico y matrices del modelo
import serial                                   # pyserial: puerto serie del adaptador USB-CAN
import serial.tools.list_ports
import tkinter as tk                            # ventana y controles (viene con Python)
from tkinter import ttk, messagebox

import matplotlib
matplotlib.use("TkAgg")                         # dibuja las gráficas dentro de la ventana de Tkinter
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

# ==========================================================================
#  CONFIGURACIÓN  -  lo único que normalmente hay que tocar
# ==========================================================================
PUERTO = "COM13"                                # puerto COM por defecto (en la ventana se puede elegir otro)
BAUD_SERIE = 115200                             # velocidad del puerto serie hacia el adaptador
BITRATE_CAN_CMD = b"S6"                         # comando SLCAN S6 = 500 kbit/s (debe coincidir con los nodos)

ID_SYS     = 0x070                              # órdenes del sistema: 1 = START, 2 = RESET, 3 = OFF
ID_REF     = 0x080                              # referencias r1 y r2 que difunde el Tanque 1
ID_REF_SET = 0x081                              # pedido de nueva referencia (lo recibe el Tanque 1)
ID_T1      = 0x100                              # nivel del tanque 1
ID_T2      = 0x101                              # nivel del tanque 2
ID_T3      = 0x102                              # nivel del tanque 3

K_LEVEL = 10000.0                               # los niveles viajan como int16 = metros x 10000 (0,1 mm)
TM = 0.5                                        # periodo de muestreo del controlador [s]
T_FIN_MODELO = 400.0                            # horizonte mínimo que se simula del modelo [s]
NIVEL_MAX = 0.6                                 # escala máxima de la gráfica y de la pantalla TFT [m]
REF_DEFECTO = (0.2, 0.1)                        # referencias por defecto del firmware (r1, r2) [m]
BANDA = 0.02                                    # banda del 2 % para alcance y establecimiento
INTERVALO_GRAFICA_MS = 500                      # cada cuánto se redibuja la ventana [ms]
MODELO_INICIAL = "IDEAL"                        # IDEAL = lineal sin límites | SAT = con saturación y anti-windup (como el firmware)
Q_BAR = 0.5 * 1.2e-4                            # límite de caudal de la bomba (±) [m³/s]
CAN_TIMEOUT = 2.0                               # segundos sin datos = tanque sin enlace

COLORES = {1: "tab:blue", 2: "tab:red", 3: "tab:green"}  # color de cada tanque en las gráficas

# Tipografía: Times New Roman en toda la interfaz y en las gráficas.
FUENTE = "Times New Roman"
matplotlib.rcParams["font.family"] = "serif"
matplotlib.rcParams["font.serif"] = [FUENTE, "Liberation Serif", "DejaVu Serif"]

# Ganancias del controlador de MATLAB (ubicación de polos): Kaug = [K  Ki], de 2x5.
# Son las mismas que usa el firmware (matrices Kd y Kid de cada nodo).
KAUG = np.array([
    [2.396457374406e-03, -5.510665673028e-04, 1.658890284590e-03, -8.207094682207e-05, 6.256840266197e-05],
    [-5.324950709357e-04, 2.232633796694e-03, -2.380764900836e-03, 3.072657820992e-05, -1.060090862987e-04],
])

# Cola de mensajes: los hilos escriben aquí y la ventana los muestra en «Registro».
mensajes = queue.Queue()


# Manda un mensaje al cuadro «Registro» de la ventana (seguro entre hilos).
def log(texto):
    mensajes.put(texto)


def coma(texto):
    """Formato espanol para mostrar: 0.1234 -> 0,1234."""
    return texto.replace(".", ",")


# ==========================================================================
#  CAN POR SERIE (SLCAN)  -  habla con el adaptador USB-CAN
# ==========================================================================
# Comunicación con el adaptador USB-CAN mediante el protocolo SLCAN: el adaptador
# aparece como un puerto serie y cada trama CAN es una línea de texto ("tIIILDD...").
class BusSLCAN:
    def __init__(self, puerto=PUERTO, baud=BAUD_SERIE):
        self.ser = serial.Serial(puerto, baud, timeout=0.1)
        self._buffer = b""
        self._lock = threading.Lock()
        self._abrir()

    # Configura el adaptador: cierra el canal, desactiva las marcas de tiempo, fija
    # los 500 kbit/s y lo abre.
    def _abrir(self):
        self.ser.write(b"C\r"); time.sleep(0.2)  # C: cierra el canal por si estaba abierto
        self.ser.reset_input_buffer()
        self.ser.write(b"Z0\r"); time.sleep(0.2)  # Z0: sin marcas de tiempo en las tramas
        self.ser.write(BITRATE_CAN_CMD + b"\r"); time.sleep(0.2)  # velocidad del bus (S6 = 500 kbit/s)
        self.ser.write(b"O\r"); time.sleep(0.2)  # O: abre el canal

    def cerrar(self):
        try:
            self.ser.write(b"C\r")
        except Exception:
            pass
        self.ser.close()

    # Envía una trama estándar: "t" + ID (3 hex) + longitud + datos (hex) + retorno de carro.
    def enviar(self, can_id, datos):
        dlc = len(datos)
        cuerpo = f"t{can_id:03X}{dlc:01X}" + "".join(f"{b:02X}" for b in datos)
        with self._lock:
            self.ser.write((cuerpo + "\r").encode())

    # Generador: lee el puerto sin parar y entrega cada trama completa como (id, [bytes]).
    def leer_tramas(self):
        while True:
            with self._lock:
                chunk = self.ser.read(256)
            if chunk:
                self._buffer += chunk
                while b"\r" in self._buffer:
                    linea, self._buffer = self._buffer.split(b"\r", 1)
                    frame = self._parsear(linea)
                    if frame:
                        yield frame
            else:
                time.sleep(0.01)

    # Convierte una línea SLCAN en (id, datos); devuelve None si no es una trama estándar válida.
    @staticmethod
    def _parsear(linea):
        try:
            texto = linea.decode(errors="ignore")
        except Exception:
            return None
        if not texto.startswith("t") or len(texto) < 5:
            return None
        try:
            can_id = int(texto[1:4], 16)
            dlc = int(texto[4], 16)
            datos_hex = texto[5:5 + dlc * 2]
            datos = [int(datos_hex[i:i + 2], 16) for i in range(0, len(datos_hex), 2)]
            return can_id, datos
        except ValueError:
            return None


# Dos bytes (alto, bajo) a entero de 16 bits con signo, igual que en el firmware.
def get16(b0, b1):
    v = (b0 << 8) | b1
    if v >= 0x8000:
        v -= 0x10000
    return v


# Metros a dos bytes: valor x 10000, redondeado y limitado al rango de un int16.
def put16(valor):
    iv = int(round(valor * K_LEVEL))
    iv = max(-32768, min(32767, iv))
    if iv < 0:
        iv += 0x10000
    return [(iv >> 8) & 0xFF, iv & 0xFF]


# ==========================================================================
#  MODELO DE LA PLANTA Y DEL CONTROLADOR  -  réplica del de MATLAB
# ==========================================================================
def _expm(M):
    """Exponencial de matriz (escalado y cuadrado + Taylor), solo numpy."""
    s = max(0, int(np.ceil(np.log2(max(np.linalg.norm(M, 1), 1e-30)))) + 1)
    X = M / (2.0 ** s)
    R = np.eye(M.shape[0])
    T = np.eye(M.shape[0])
    for i in range(1, 25):
        T = T @ X / i
        R = R + T
    for _ in range(s):
        R = R @ R
    return R


# Construye el modelo linealizado de los tres tanques en el punto de operación,
# lo discretiza con retenedor de orden cero (Tm) y arma el lazo cerrado con KAUG.
# Devuelve Ad, Bd (modelo discreto), Acl (lazo cerrado aumentado) y Br (entrada de la referencia).
def _construir_modelo():
    S, Sp = 0.0154, 5e-5                        # sección del tanque (S) y de las tuberías (Sp) [m²]
    mu13, mu32, mu20 = 0.5, 0.5, 0.675          # coeficientes de descarga de las tres tuberías
    g, q_max = 9.81, 1.2e-4                     # gravedad [m/s²] y caudal máximo de la bomba [m³/s]
    q1b = q2b = 0.5 * q_max                     # punto de operación: las bombas a la mitad del caudal máximo
    # Niveles de equilibrio del punto de operación (de aguas abajo hacia arriba).
    l2b = (1 / (2 * g)) * ((q1b + q2b) / (mu20 * Sp)) ** 2
    l3b = l2b + (1 / (2 * g)) * (q1b / (mu32 * Sp)) ** 2
    l1b = l3b + (1 / (2 * g)) * (q1b / (mu13 * Sp)) ** 2
    # Coeficientes de la linealización de cada tubería (α, β y γ de la tesis).
    c13 = (mu13 * Sp * np.sqrt(2 * g)) / (2 * np.sqrt(l1b - l3b))
    c32 = (mu32 * Sp * np.sqrt(2 * g)) / (2 * np.sqrt(l3b - l2b))
    c20 = (mu20 * Sp * np.sqrt(2 * g)) / (2 * np.sqrt(l2b))
    # Matrices continuas del modelo linealizado: x' = A·x + B·u,  y = C·x.
    A = np.array([[-c13 / S, 0, c13 / S],
                  [0, -(c32 + c20) / S, c32 / S],
                  [c13 / S, c32 / S, -(c13 + c32) / S]])
    B = np.array([[1 / S, 0], [0, 1 / S], [0, 0]])
    C = np.array([[1.0, 0, 0], [0, 1.0, 0]])
    # Discretización exacta (retenedor de orden cero) con la exponencial de matriz:
    #   expm([[A·Tm, B·Tm], [0, 0]]) = [[Ad, Bd], [0, I]]
    M = np.zeros((5, 5))
    M[:3, :3] = A * TM
    M[:3, 3:] = B * TM
    E = _expm(M)
    Ad, Bd = E[:3, :3], E[:3, 3:]
    # Sistema aumentado con los integradores del error: estado [x; z], z[k+1] = z[k] + Tm·(r - y).
    Aaug = np.block([[Ad, np.zeros((3, 2))], [-TM * C, np.eye(2)]])
    Baug = np.vstack([Bd, np.zeros((2, 2))])
    Br = np.vstack([np.zeros((3, 2)), TM * np.eye(2)])
    Acl = Aaug - Baug @ KAUG                    # lazo cerrado con u = -KAUG·[x; z]
    return Ad, Bd, Acl, Br


AD, BD, ACL, BR = _construir_modelo()


def simular_modelo(t_fin, hist, saturado):
    """Simula el modelo con referencias que cambian en el tiempo.
    hist = [(t_cambio, r1, r2), ...] ordenado (el primero en t = 0).
    Devuelve (t, {1: x1, 2: x2, 3: x3})."""
    n = int(round(t_fin / TM)) + 1
    t = np.arange(n) * TM
    tc = np.array([h[0] for h in hist])
    rr = np.array([[h[1], h[2]] for h in hist])
    idx = np.clip(np.searchsorted(tc, t, side="right") - 1, 0, len(hist) - 1)
    r = rr[idx]                                   # referencia en cada muestra
    x = np.zeros((n, 3))
    if not saturado:                              # servo lineal de MATLAB
        xa = np.zeros((n, 5))
        for k in range(n - 1):
            xa[k + 1] = ACL @ xa[k] + BR @ r[k]
        x = xa[:, :3]
    else:                                         # igual que el firmware
        xk = np.zeros(3); z = np.zeros(2); u = np.zeros(2)
        for k in range(1, n):
            xk = AD @ xk + BD @ u
            e = r[k] - xk[:2]
            for j in range(2):
                cong = (u[j] <= -Q_BAR and e[j] <= 0) or (u[j] >= Q_BAR and e[j] >= 0)
                if not cong:
                    z[j] += TM * e[j]
            u = np.clip(-KAUG @ np.concatenate([xk, z]), -Q_BAR, Q_BAR)
            x[k] = xk
    return t, {1: x[:, 0], 2: x[:, 1], 3: x[:, 2]}


# ==========================================================================
#  DATOS COMPARTIDOS  -  entre el hilo que lee el CAN y la ventana
# ==========================================================================
lock = threading.Lock()                         # protege los datos que usan a la vez el hilo CAN y la ventana
real = {1: ([], []), 2: ([], []), 3: ([], [])}  # por tanque: (tiempos desde START [s], niveles [m])
ultimo_rx = {1: (0.0, None), 2: (0.0, None), 3: (0.0, None)}  # (hora de la última trama, último nivel) para el estado del enlace
# Estado de la prueba: referencias vistas en el bus, instante de START, historial de
# referencias (para que el modelo las siga) y modelo elegido.
estado = {"r1": None, "r2": None, "t_start": None, "ult_cmd": 0.0,
          "hist": [(0.0, REF_DEFECTO[0], REF_DEFECTO[1])],
          "modelo": MODELO_INICIAL}
cache = {"clave": None, "valor": None}          # evita recalcular el modelo si no cambió nada


# Referencias vigentes: las del bus o, si aún no llegó ninguna, las por defecto.
def refs_actuales():
    r1 = estado["r1"] if estado["r1"] is not None else REF_DEFECTO[0]
    r2 = estado["r2"] if estado["r2"] is not None else REF_DEFECTO[1]
    return r1, r2


def registrar_ref(r1, r2):
    """Anota un cambio de referencia (tiempo relativo a START). Con lock."""
    if estado["t_start"] is None:
        return
    ult = estado["hist"][-1]
    if abs(ult[1] - r1) > 1e-4 or abs(ult[2] - r2) > 1e-4:
        estado["hist"].append((time.time() - estado["t_start"], r1, r2))


def marcar_inicio():
    """Fija t = 0 y borra la prueba anterior."""
    with lock:
        for k in real:
            real[k][0].clear()
            real[k][1].clear()
        r1, r2 = refs_actuales()
        estado["t_start"] = time.time()
        estado["hist"] = [(0.0, r1, r2)]
        cache["clave"] = None
        modelo = estado["modelo"]
    log(f"[OK] t = 0 marcado. Ref inicial: r1={r1} r2={r2}. Modelo: {modelo}")


def modelo_actual(t_fin):
    """Curva del modelo con las referencias reales (con cache)."""
    with lock:
        hist = list(estado["hist"])
        sat = estado["modelo"] == "SAT"
    n = int(round(max(t_fin, T_FIN_MODELO) / TM / 20.0))     # crece de a 10 s
    clave = (tuple(hist), sat, n)
    if cache["clave"] != clave:
        t_h = (n + 1) * 20.0 * TM
        cache["valor"] = simular_modelo(t_h, hist, sat)
        cache["clave"] = clave
    return cache["valor"]


# Procesa cada trama que llega: guarda las referencias que difunde el Tanque 1
# y el nivel de cada tanque junto con su tiempo desde START.
def procesar_trama(can_id, datos):
    ahora = time.time()
    with lock:
        if can_id == ID_REF and len(datos) >= 4:
            estado["r1"] = get16(datos[0], datos[1]) / K_LEVEL
            estado["r2"] = get16(datos[2], datos[3]) / K_LEVEL
            if ahora - estado["ult_cmd"] > 1.5:     # evita eco de la ref anterior
                registrar_ref(estado["r1"], estado["r2"])
            return
        tanque = {ID_T1: 1, ID_T2: 2, ID_T3: 3}.get(can_id)
        if tanque and len(datos) >= 2:
            nivel = get16(datos[0], datos[1]) / K_LEVEL
            ultimo_rx[tanque] = (ahora, nivel)
            if estado["t_start"] is not None:
                real[tanque][0].append(ahora - estado["t_start"])
                real[tanque][1].append(nivel)


# Hilo de recepción: lee el bus sin parar y procesa cada trama.
def escuchar(bus):
    try:
        for can_id, datos in bus.leer_tramas():
            procesar_trama(can_id, datos)
    except Exception as e:
        if bus.ser.is_open:
            log(f"[ERROR] Se perdio la recepcion: {e!r}")


# ==========================================================================
#  ANÁLISIS  -  métricas de respuesta y error frente al modelo
# ==========================================================================
def _cruce(t, y, nivel):
    """Primer instante en que y >= nivel (interpolado). None si no ocurre."""
    idx = np.where(y >= nivel)[0]
    if len(idx) == 0:
        return None
    i = idx[0]
    if i == 0:
        return float(t[0])
    y0, y1 = y[i - 1], y[i]
    if y1 == y0:
        return float(t[i])
    return float(t[i - 1] + (nivel - y0) * (t[i] - t[i - 1]) / (y1 - y0))


def metricas(t, y, ref, y0=0.0, escalon=True):
    """Metricas de una curva en un tramo, frente a su referencia.
    y0 = nivel con el que empieza el tramo (el escalon va de y0 a ref).
    Si escalon=False (la referencia de ese tanque no cambio) solo se dan
    el valor final y el error estacionario."""
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    if len(t) < 3:
        return None
    ventana = max(10.0, 0.1 * (t[-1] - t[0]))   # ventana final sobre la que se promedia el valor final (mínimo 10 s)
    final = float(np.mean(y[t >= t[-1] - ventana]))
    res = {"t_subida": None, "t_alcance": None, "t_est": None, "sobre": None,
           "final": final, "ess": ref - final}
    salto = ref - y0
    if (not escalon) or abs(salto) < 1e-3:
        return res
    s = 1.0 if salto > 0 else -1.0
    yn, objetivo = s * (y - y0), abs(salto)

    t10 = _cruce(t, yn, 0.10 * objetivo)        # instantes en que la curva cruza el 10 % y el 90 % del escalón
    t90 = _cruce(t, yn, 0.90 * objetivo)
    res["t_alcance"] = _cruce(t, yn, (1 - BANDA) * objetivo)  # primer instante en que llega al 98 %
    res["t_subida"] = None if (t10 is None or t90 is None) else t90 - t10
    res["sobre"] = max(0.0, (yn.max() - objetivo) / objetivo * 100.0)  # sobreimpulso, en % del escalón

    fuera = np.where(np.abs(yn - objetivo) > BANDA * objetivo)[0]  # muestras fuera de la banda del 2 %: se establece tras la última
    if len(fuera) == 0:
        res["t_est"] = float(t[0])
    elif fuera[-1] >= len(t) - 1:
        res["t_est"] = None                # aun no se establece
    else:
        res["t_est"] = float(t[fuera[-1] + 1])
    return res


def error_entre(t_r, y_r, t_m, y_m):
    """RMSE, IAE y error maximo del real frente al modelo (tiempo comun)."""
    t_r, y_r = np.asarray(t_r, float), np.asarray(y_r, float)
    if len(t_r) < 3:
        return None
    lim = min(t_r[-1], t_m[-1])
    m = t_r <= lim
    if m.sum() < 3:
        return None
    dif = y_r[m] - np.interp(t_r[m], t_m, y_m)
    return {"rmse": float(np.sqrt(np.mean(dif ** 2))),
            "iae": float(np.trapezoid(np.abs(dif), t_r[m]) if hasattr(np, "trapezoid")
                         else np.trapz(np.abs(dif), t_r[m])),
            "max": float(np.max(np.abs(dif)))}


def _recorte(t, y, t_lim):
    t = np.asarray(t, float); y = np.asarray(y, float)
    m = t <= t_lim
    return t[m], y[m]


def tramos(hist, t_fin):
    """Parte la prueba en tramos: cada cambio de referencia inicia uno nuevo."""
    out = []
    for i, (t0, r1, r2) in enumerate(hist):
        t1 = hist[i + 1][0] if i + 1 < len(hist) else t_fin
        if t1 <= t0:
            continue
        if i == 0:
            nuevo = (True, True)
        else:
            nuevo = (abs(r1 - hist[i - 1][1]) > 1e-4, abs(r2 - hist[i - 1][2]) > 1e-4)
        out.append({"t0": t0, "t1": t1, "refs": (r1, r2), "nuevo": nuevo})
    return out


def analizar():
    """Analiza TODOS los tramos de la prueba (uno por cada cambio de referencia).
    Devuelve (texto del informe completo, resultados)."""
    with lock:
        copia = {k: (np.array(v[0]), np.array(v[1])) for k, v in real.items()}
        hist = list(estado["hist"])
        nombre_modelo = estado["modelo"]
    if len(copia[1][0]) < 3:
        return "Sin datos suficientes todavia (pulsa START y espera).", None

    t_fin = float(copia[1][0][-1])
    t_m, ym = modelo_actual(t_fin + 10)
    segs = tramos(hist, t_fin)
    res = {"tramos": [], "err_total": {}}
    for sg in segs:
        t0, t1 = sg["t0"], sg["t1"]
        d = {"t0": t0, "t1": t1, "refs": sg["refs"], "nuevo": sg["nuevo"], "m": {}, "err": {}}
        mmask = (t_m >= t0) & (t_m <= t1)
        for k in (1, 2):
            ref = sg["refs"][k - 1]
            y0 = float(np.interp(t0, t_m, ym[k]))          # mismo nivel inicial para ambos
            tr, yr = copia[k]
            rm = (tr >= t0) & (tr <= t1)
            mm = metricas(t_m[mmask] - t0, ym[k][mmask], ref, y0, sg["nuevo"][k - 1])
            mr = metricas(tr[rm] - t0, yr[rm], ref, y0, sg["nuevo"][k - 1])
            d["m"][k] = (mm, mr)
        for k in (1, 2, 3):
            tr, yr = copia[k]
            rm = (tr >= t0) & (tr <= t1)
            d["err"][k] = error_entre(tr[rm], yr[rm], t_m, ym[k])
        res["tramos"].append(d)
    for k in (1, 2, 3):
        res["err_total"][k] = error_entre(*copia[k], t_m, ym[k])

    f = lambda v, fmt="{:7.1f}": "   --  " if v is None else fmt.format(v)
    L = []
    L.append("INFORME DE LA PRUEBA - MODELO vs SISTEMA REAL")
    L.append(f"Duración real: {t_fin:.1f} s   |   Modelo: "
             f"{'MATLAB ideal (lineal)' if nombre_modelo == 'IDEAL' else 'con saturación y antiwindup'}")
    L.append(f"Tramos analizados: {len(res['tramos'])} (cada cambio de referencia inicia un tramo; "
             "los tiempos de cada tramo se miden desde su inicio)")
    for i, d in enumerate(res["tramos"], start=1):
        L.append("")
        L.append("=" * 62)
        L.append(f"TRAMO {i}:  t = {d['t0']:.1f} a {d['t1']:.1f} s   |   "
                 f"ref1 = {d['refs'][0]:.3f} m   ref2 = {d['refs'][1]:.3f} m")
        L.append("=" * 62)
        for k in (1, 2):
            mm, mr = d["m"][k]
            L.append(f"TANQUE {k}  (ref = {d['refs'][k - 1]:.3f} m)        MODELO    REAL   REAL-MOD")
            if mm is None or mr is None:
                L.append("   (datos insuficientes)")
                L.append("")
                continue
            if not d["nuevo"][k - 1]:
                L.append("   (la referencia de este tanque no cambió: solo valor final y error)")

            def fila(nombre, clave, fmt="{:7.1f}", unidad="s"):
                a, b = mm[clave], mr[clave]
                dif = None if (a is None or b is None) else b - a
                L.append(f"  {nombre:<24}{f(a, fmt)} {f(b, fmt)} "
                         f"{f(dif, '{:+7.1f}' if fmt == '{:7.1f}' else '{:+8.4f}')} {unidad}")
            if d["nuevo"][k - 1]:
                fila("t alcanza 98 % de ref", "t_alcance")
                fila("t subida (10-90 %)", "t_subida")
                fila("t establecimiento 2 %", "t_est")
                fila("sobreimpulso", "sobre", "{:7.2f}", "%")
            fila("valor final", "final", "{:7.4f}", "m")
            fila("error estacionario", "ess", "{:7.4f}", "m")
            L.append("")
        L.append("Real vs modelo en el tramo:")
        for k in (1, 2, 3):
            e = d["err"][k]
            if e:
                L.append(f"  Tanque {k}:  RMSE = {e['rmse']:.4f} m   IAE = {e['iae']:.3f} m*s"
                         f"   max = {e['max']:.4f} m")
    L.append("")
    L.append("=" * 62)
    L.append("DIFERENCIA REAL vs MODELO (toda la prueba):")
    for k in (1, 2, 3):
        e = res["err_total"][k]
        if e:
            L.append(f"  Tanque {k}:  RMSE = {e['rmse']:.4f} m   IAE = {e['iae']:.3f} m*s"
                     f"   max = {e['max']:.4f} m")
    return "\n".join(L), res


def guardar_csv():
    with lock:
        copia = {k: (np.array(v[0]), np.array(v[1])) for k, v in real.items()}
    if len(copia[1][0]) < 3:
        log("No hay datos para guardar.")
        return None
    t_m, ym = modelo_actual(copia[1][0][-1] + 10)
    sello = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    ruta = f"prueba_{sello}.csv"
    t_base = copia[1][0]
    cols = [t_base]
    for k in (1, 2, 3):
        tk_, yk = copia[k]
        cols.append(np.interp(t_base, tk_, yk) if len(tk_) > 1 else np.zeros_like(t_base))
    for k in (1, 2, 3):
        cols.append(np.interp(t_base, t_m, ym[k]))
    np.savetxt(ruta, np.column_stack(cols), delimiter=",", fmt="%.6f",
               header="t,real_h1,real_h2,real_h3,modelo_h1,modelo_h2,modelo_h3", comments="")
    log(f"[OK] Datos guardados en {ruta}")
    return sello


# ==========================================================================
#  COMANDOS HACIA LOS NODOS
# ==========================================================================
# Pide una nueva referencia (ID 0x081). El Tanque 1 la valida y la difunde,
# y el modelo cambia en el mismo instante.
def enviar_ref(bus, r1, r2):
    bus.enviar(ID_REF_SET, put16(r1) + put16(r2))
    with lock:
        estado["ult_cmd"] = time.time()
        registrar_ref(r1, r2)            # el modelo cambia en el mismo instante
    log(f">> REF_SET enviado: r1={r1}  r2={r2}")


# Envía una orden de sistema: 1 = START, 2 = RESET, 3 = OFF.
def enviar_sys(bus, cmd):
    nombres = {1: "START", 2: "RESET", 3: "OFF"}
    bus.enviar(ID_SYS, [cmd])
    log(f">> SYS_CMD enviado: {nombres.get(cmd, cmd)}")


# ==========================================================================
#  GRÁFICAS
# ==========================================================================
class Grafica:
    """Una pestana con su propia figura: comparacion, solo real o solo modelo."""

    def __init__(self, padre, titulo, con_modelo, con_real):
        self.con_modelo, self.con_real = con_modelo, con_real
        self.fig = Figure(figsize=(9, 5), constrained_layout=True)
        self.ax = self.fig.add_subplot(111)
        self.l_mod, self.l_real = {}, {}
        for k in (1, 2, 3):
            if con_modelo:
                self.l_mod[k], = self.ax.plot([], [], "--", color=COLORES[k], alpha=0.85,
                                              label=f"T{k} modelo" if con_real else f"Tanque {k}")
            if con_real:
                self.l_real[k], = self.ax.plot([], [], "-", color=COLORES[k], lw=1.6,
                                               label=f"T{k} real" if con_modelo else f"Tanque {k}")
        self.ref1, = self.ax.plot([], [], ":", color="gray", drawstyle="steps-post", label="Ref. 1")
        self.ref2, = self.ax.plot([], [], ":", color="black", alpha=0.6, drawstyle="steps-post", label="Ref. 2")
        self.ax.set_title(titulo, fontsize=13)
        self.ax.set_xlabel("Tiempo desde START [s]", fontsize=11)
        self.ax.set_ylabel("Nivel [m]", fontsize=11)
        self.ax.set_xlim(0, T_FIN_MODELO)
        self.ax.set_ylim(0, NIVEL_MAX)
        self.ax.set_yticks(np.arange(0, NIVEL_MAX + 1e-9, 0.1))
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc="upper right", ncol=4 if (con_modelo and con_real) else 5, fontsize=9)

        self.canvas = FigureCanvasTkAgg(self.fig, master=padre)
        NavigationToolbar2Tk(self.canvas, padre).update()
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def actualizar(self, copia, t_m, ym, hist, t_fin, auto):
        for k in (1, 2, 3):
            if self.con_modelo:
                self.l_mod[k].set_data(t_m, ym[k])
            if self.con_real:
                self.l_real[k].set_data(*copia[k])
        if self.con_modelo:
            x_max = max(T_FIN_MODELO, t_fin + 10)
        else:                                   # solo real: el eje sigue a la prueba
            x_max = max(60.0, t_fin + 10)
        th = [h[0] for h in hist] + [x_max]
        self.ref1.set_data(th, [h[1] for h in hist] + [hist[-1][1]])
        self.ref2.set_data(th, [h[2] for h in hist] + [hist[-1][2]])
        if auto:
            self.ax.set_xlim(0, x_max)
            minimo = min([0.0] + [h[1] for h in hist] + [h[2] for h in hist]
                         + [v for k in copia for v in copia[k][1]])
            self.ax.set_ylim(minimo - 0.02 if minimo < -0.005 else 0.0, NIVEL_MAX)


# ==========================================================================
#  INTERFAZ (Tkinter)
# ==========================================================================
# Filas de la tabla comparativa: (nombre en pantalla, clave de la métrica, formato).
FILAS_TABLA = (
    ("Tiempo de establecimiento (2 %) [s]", "t_est", "{:.1f}"),
    ("Sobreimpulso [%]", "sobre", "{:.2f}"),
    ("Valor final [m]", "final", "{:.4f}"),
    ("Error estacionario [m]", "ess", "{:.4f}"),
)


# Ventana principal: controles a la izquierda; gráficas, tabla comparativa y registro a la derecha.
class App:
    def __init__(self, root):
        self.root = root
        self.bus = None
        self.ult_analisis = 0.0

        root.title("Tres tanques - Modelo vs sistema real")
        root.geometry("1320x800")
        root.protocol("WM_DELETE_WINDOW", self.cerrar)

        est = ttk.Style()
        est.theme_use("clam")
        est.configure(".", font=(FUENTE, 11))
        est.configure("TLabelframe.Label", font=(FUENTE, 11, "bold"))      # subtitulos
        est.configure("Enc.TLabel", font=(FUENTE, 11, "bold"))
        est.configure("Peq.TLabel", font=(FUENTE, 10), foreground="gray")
        est.configure("TNotebook.Tab", font=(FUENTE, 11), padding=(12, 4))

        izq = ttk.Frame(root, padding=10)
        izq.pack(side="left", fill="y")
        der = ttk.Frame(root, padding=(0, 10, 10, 10))
        der.pack(side="right", fill="both", expand=True)

        # ---- Conexion ----
        f = ttk.LabelFrame(izq, text="Conexión", padding=8)
        f.pack(fill="x", pady=4)
        ttk.Label(f, text="Puerto:").grid(row=0, column=0, sticky="w")
        self.var_puerto = tk.StringVar(value=PUERTO)
        self.cmb_puerto = ttk.Combobox(f, textvariable=self.var_puerto, width=9, font=(FUENTE, 11))
        self.cmb_puerto.grid(row=0, column=1, padx=4)
        ttk.Button(f, text="Buscar", width=7, command=self.refrescar_puertos).grid(row=0, column=2)
        self.btn_con = ttk.Button(f, text="Conectar", command=self.conectar)
        self.btn_con.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(6, 0))
        self.lbl_con = tk.Label(f, text="Desconectado", fg="red", font=(FUENTE, 11))
        self.lbl_con.grid(row=2, column=0, columnspan=3, pady=(4, 0))
        self.refrescar_puertos()

        # ---- Sistema ----
        f = ttk.LabelFrame(izq, text="Sistema", padding=8)
        f.pack(fill="x", pady=4)
        ttk.Button(f, text="START", command=self.cmd_start).grid(row=0, column=0, sticky="ew", padx=2, pady=2)
        ttk.Button(f, text="RESET", command=lambda: self.cmd_sys(2)).grid(row=0, column=1, sticky="ew", padx=2, pady=2)
        ttk.Button(f, text="OFF", command=lambda: self.cmd_sys(3)).grid(row=1, column=0, sticky="ew", padx=2, pady=2)
        ttk.Button(f, text="Marcar t = 0", command=marcar_inicio).grid(row=1, column=1, sticky="ew", padx=2, pady=2)
        ttk.Label(f, text="«Marcar t = 0»: si arrancaste con el botón físico",
                  style="Peq.TLabel").grid(row=2, column=0, columnspan=2, sticky="w")
        f.columnconfigure(0, weight=1); f.columnconfigure(1, weight=1)

        # ---- Referencias ----
        f = ttk.LabelFrame(izq, text="Referencias [m]", padding=8)
        f.pack(fill="x", pady=4)
        ttk.Label(f, text="Ref. 1:").grid(row=0, column=0, sticky="w")
        self.var_r1 = tk.StringVar(value=str(REF_DEFECTO[0]))
        ttk.Entry(f, textvariable=self.var_r1, width=10, font=(FUENTE, 11)).grid(row=0, column=1, padx=4, pady=2)
        ttk.Label(f, text="Ref. 2:").grid(row=1, column=0, sticky="w")
        self.var_r2 = tk.StringVar(value=str(REF_DEFECTO[1]))
        ttk.Entry(f, textvariable=self.var_r2, width=10, font=(FUENTE, 11)).grid(row=1, column=1, padx=4, pady=2)
        ttk.Button(f, text="Enviar referencias", command=self.cmd_ref).grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        self.lbl_ref = ttk.Label(f, text="En el bus: --")
        self.lbl_ref.grid(row=3, column=0, columnspan=2, sticky="w", pady=(4, 0))

        # ---- Modelo ----
        f = ttk.LabelFrame(izq, text="Modelo", padding=8)
        f.pack(fill="x", pady=4)
        self.var_modelo = tk.StringVar(value=MODELO_INICIAL)
        ttk.Radiobutton(f, text="Ideal (MATLAB lineal)", value="IDEAL",
                        variable=self.var_modelo, command=self.cmd_modelo).pack(anchor="w")
        ttk.Radiobutton(f, text="Con saturación y antiwindup", value="SAT",
                        variable=self.var_modelo, command=self.cmd_modelo).pack(anchor="w")

        # ---- Estado de los tanques ----
        f = ttk.LabelFrame(izq, text="Estado", padding=8)
        f.pack(fill="x", pady=4)
        self.lbl_tiempo = ttk.Label(f, text="t = --")
        self.lbl_tiempo.pack(anchor="w")
        self.lbl_tanque = {}
        for k in (1, 2, 3):
            self.lbl_tanque[k] = tk.Label(f, text=f"T{k}: sin datos", fg="gray", anchor="w", font=(FUENTE, 11))
            self.lbl_tanque[k].pack(anchor="w", fill="x")

        # ---- Guardar ----
        f = ttk.LabelFrame(izq, text="Guardar", padding=8)
        f.pack(fill="x", pady=4)
        ttk.Button(f, text="Guardar prueba (CSV, PNG e informe)", command=self.cmd_guardar).pack(fill="x")
        self.var_auto = tk.BooleanVar(value=True)
        ttk.Checkbutton(f, text="Escala automática (quitar para zoom)",
                        variable=self.var_auto).pack(anchor="w", pady=(4, 0))

        # ---- Registro ----
        f = ttk.LabelFrame(izq, text="Registro", padding=4)
        f.pack(fill="both", expand=True, pady=4)
        self.txt_log = tk.Text(f, width=34, height=5, font=(FUENTE, 9), state="disabled")
        self.txt_log.pack(fill="both", expand=True)

        # ---- Comparacion de los tanques (abajo, siempre visible) ----
        cmp_ = ttk.Frame(der)
        cmp_.pack(side="bottom", fill="x", pady=(6, 0))
        sel = ttk.Frame(cmp_)
        sel.grid(row=0, column=0, columnspan=2, sticky="w", padx=6, pady=(0, 4))
        ttk.Label(sel, text="Comparación del tramo:", style="Enc.TLabel").pack(side="left")
        self.var_tramo = tk.StringVar(value="Último")
        self.cmb_tramo = ttk.Combobox(sel, textvariable=self.var_tramo, values=["Último"],
                                      state="readonly", width=12, font=(FUENTE, 11))
        self.cmb_tramo.pack(side="left", padx=6)
        self.cmb_tramo.bind("<<ComboboxSelected>>", lambda e: setattr(self, "ult_analisis", 0.0))
        self.var_nota = tk.StringVar(value="")
        ttk.Label(sel, textvariable=self.var_nota, style="Peq.TLabel").pack(side="left", padx=6)
        self.var_tabla = {}
        self.marco_tanque = {}
        for col, k in enumerate((1, 2)):
            lf = ttk.LabelFrame(cmp_, text=f"Tanque {k}", padding=6)
            lf.grid(row=1, column=col, sticky="nsew", padx=4)
            cmp_.columnconfigure(col, weight=1)
            self.marco_tanque[k] = lf
            ttk.Label(lf, text="", width=32).grid(row=0, column=0)
            ttk.Label(lf, text="Modelo", style="Enc.TLabel", anchor="e", width=9).grid(row=0, column=1)
            ttk.Label(lf, text="Real", style="Enc.TLabel", anchor="e", width=9).grid(row=0, column=2)
            for i, (nombre, clave, _fmt) in enumerate(FILAS_TABLA, start=1):
                ttk.Label(lf, text=nombre).grid(row=i, column=0, sticky="w")
                for c, cual in ((1, 0), (2, 1)):
                    var = tk.StringVar(value="--")
                    ttk.Label(lf, textvariable=var, anchor="e", width=9).grid(row=i, column=c, sticky="e")
                    self.var_tabla[(k, clave, cual)] = var
        self.var_rmse = tk.StringVar(value="RMSE del tramo [m]:  --")
        ttk.Label(cmp_, textvariable=self.var_rmse).grid(row=2, column=0, columnspan=2, sticky="w", padx=6, pady=(4, 0))
        self.var_rmse_tot = tk.StringVar(value="RMSE de toda la prueba [m]:  --")
        ttk.Label(cmp_, textvariable=self.var_rmse_tot).grid(row=3, column=0, columnspan=2, sticky="w", padx=6)

        # ---- Graficas (pestanas) ----
        self.libro = ttk.Notebook(der)
        self.libro.pack(side="top", fill="both", expand=True)
        marcos = []
        for titulo in ("Comparación", "Sistema real", "Modelo MATLAB"):
            m = ttk.Frame(self.libro)
            self.libro.add(m, text=titulo)
            marcos.append(m)
        self.graficas = [
            Grafica(marcos[0], "Modelo MATLAB vs sistema real (tres tanques)", True, True),
            Grafica(marcos[1], "Sistema real (tres tanques)", False, True),
            Grafica(marcos[2], "Modelo MATLAB (tres tanques)", True, False),
        ]
        self.nombres_png = ("comparacion", "real", "modelo")
        self.libro.bind("<<NotebookTabChanged>>",
                        lambda e: self.graficas[self.libro.index("current")].canvas.draw_idle())

        self.root.after(INTERVALO_GRAFICA_MS, self.actualizar)

    # ---------- utilidades ----------
    def escribir_log(self, texto):
        self.txt_log.configure(state="normal")
        self.txt_log.insert("end", texto + "\n")
        self.txt_log.see("end")
        self.txt_log.configure(state="disabled")

    def conectado(self):
        if self.bus is None:
            messagebox.showwarning("Sin conexión", "Primero conecta el adaptador CAN.")
            return False
        return True

    def refrescar_puertos(self):
        puertos = [p.device for p in serial.tools.list_ports.comports()]
        self.cmb_puerto["values"] = puertos
        if puertos and self.var_puerto.get() not in puertos:
            self.var_puerto.set(puertos[0])

    # ---------- botones ----------
    # Conecta o desconecta el adaptador CAN y arranca el hilo de recepción.
    def conectar(self):
        if self.bus is not None:                 # ya conectado: desconectar
            bus, self.bus = self.bus, None
            bus.cerrar()
            self.btn_con.configure(text="Conectar")
            self.lbl_con.configure(text="Desconectado", fg="red")
            log("Desconectado.")
            return
        puerto = self.var_puerto.get().strip()
        try:
            self.bus = BusSLCAN(puerto)
        except Exception as e:
            self.bus = None
            messagebox.showerror("Error de conexión", f"No se pudo abrir {puerto}:\n{e}")
            return
        threading.Thread(target=escuchar, args=(self.bus,), daemon=True).start()
        self.btn_con.configure(text="Desconectar")
        self.lbl_con.configure(text=f"Conectado a {puerto}", fg="green")
        log(f"Conectado a {puerto} @ {BAUD_SERIE} bps.")

    # START: espera 1,2 s a que llegue la difusión de referencias, marca t = 0 y manda la orden.
    def cmd_start(self):
        if not self.conectado():
            return
        log("Esperando la difusión de referencias...")
        # deja llegar la difusion de referencias antes de marcar t = 0
        self.root.after(1200, lambda: (marcar_inicio(), enviar_sys(self.bus, 1)) if self.bus else None)

    def cmd_sys(self, cmd):
        if self.conectado():
            enviar_sys(self.bus, cmd)

    # Lee r1 y r2 de las cajas de texto (acepta coma o punto) y las envía.
    def cmd_ref(self):
        if not self.conectado():
            return
        try:
            r1 = float(self.var_r1.get().replace(",", "."))
            r2 = float(self.var_r2.get().replace(",", "."))
        except ValueError:
            messagebox.showerror("Valor inválido", "Escribe números, por ejemplo 0.25")
            return
        enviar_ref(self.bus, r1, r2)

    # Cambia entre el modelo IDEAL y el SAT (con saturación y anti-windup).
    def cmd_modelo(self):
        with lock:
            estado["modelo"] = self.var_modelo.get()
            cache["clave"] = None
        log(f"[OK] Modelo: {self.var_modelo.get()}")

    def cmd_guardar(self):
        """Guarda CSV, las tres graficas y el informe completo (todas las metricas)."""
        texto, _ = analizar()
        sello = guardar_csv()
        if not sello:
            return
        ruta = f"informe_{sello}.txt"
        with open(ruta, "w", encoding="utf-8") as fh:
            fh.write(texto + "\n")
        log(f"[OK] Informe guardado en {ruta}")
        for g, nombre in zip(self.graficas, self.nombres_png):
            png = f"prueba_{sello}_{nombre}.png"
            g.fig.savefig(png, dpi=150)
            log(f"[OK] Gráfica guardada en {png}")

    def cerrar(self):
        if self.bus is not None:
            bus, self.bus = self.bus, None
            bus.cerrar()
        self.root.destroy()

    # ---------- comparacion resumida ----------
    def actualizar_tabla(self, res):
        """Muestra el tramo elegido (por defecto, el ultimo). Los tramos son los
        intervalos entre cambios de referencia."""
        tramos_ = res["tramos"] if res else []
        n = len(tramos_)
        valores = ["Último"] + [f"Tramo {i + 1}" for i in range(n)]
        if list(self.cmb_tramo["values"]) != valores:
            self.cmb_tramo["values"] = valores
        elegido = self.var_tramo.get()
        if n == 0:
            idx = None
        elif elegido.startswith("Tramo"):
            idx = min(int(elegido.split()[1]) - 1, n - 1)
        else:
            idx = n - 1
        d = tramos_[idx] if idx is not None else None

        for k in (1, 2):
            mm, mr = d["m"][k] if d else (None, None)
            for _n, clave, fmt in FILAS_TABLA:
                for cual, met in ((0, mm), (1, mr)):
                    v = None if met is None else met.get(clave)
                    self.var_tabla[(k, clave, cual)].set("--" if v is None else coma(fmt.format(v)))
            texto = f"Tanque {k}"
            if d:
                texto += "  (ref. = " + coma(f"{d['refs'][k - 1]:.3f}") + " m)"
                if not d["nuevo"][k - 1]:
                    texto += "  sin cambio de ref."
            self.marco_tanque[k].configure(text=texto)

        if d:
            def partes(errs):
                return "     ".join(
                    f"T{k} = " + ("--" if errs.get(k) is None else coma(f"{errs[k]['rmse']:.4f}"))
                    for k in (1, 2, 3))
            self.var_rmse.set("RMSE del tramo [m]:   " + partes(d["err"]))
            self.var_rmse_tot.set("RMSE de toda la prueba [m]:   " + partes(res["err_total"]))
            en_curso = "  (en curso)" if idx == n - 1 else ""
            t_a = coma("{:.1f}".format(d["t0"]))
            t_b = coma("{:.1f}".format(d["t1"]))
            self.var_nota.set("Tramo {} de {}:  t = {} a {} s{}".format(idx + 1, n, t_a, t_b, en_curso))
        else:
            self.var_rmse.set("RMSE del tramo [m]:  --")
            self.var_rmse_tot.set("RMSE de toda la prueba [m]:  --")
            self.var_nota.set("")

    # ---------- refresco periodico ----------
    # Se ejecuta cada 0,5 s: vuelca el registro, actualiza el estado de los tanques,
    # las gráficas y la tabla comparativa.
    def actualizar(self):
        while not mensajes.empty():
            self.escribir_log(mensajes.get())

        ahora = time.time()
        with lock:
            copia = {k: (list(v[0]), list(v[1])) for k, v in real.items()}
            hist = list(estado["hist"])
            t_start = estado["t_start"]
            r_bus = (estado["r1"], estado["r2"])
            ult = dict(ultimo_rx)

        # estado de los tanques
        for k in (1, 2, 3):
            t_rx, nivel = ult[k]
            if nivel is None:
                self.lbl_tanque[k].configure(text=f"T{k}: sin datos", fg="gray")
            elif ahora - t_rx > CAN_TIMEOUT:
                self.lbl_tanque[k].configure(text=f"T{k}: {coma(f'{nivel:.4f}')} m  (sin enlace)", fg="red")
            else:
                self.lbl_tanque[k].configure(text=f"T{k}: {coma(f'{nivel:.4f}')} m  OK", fg="green")
        if r_bus[0] is not None:
            self.lbl_ref.configure(text=f"En el bus: r1 = {coma(f'{r_bus[0]:.3f}')}   r2 = {coma(f'{r_bus[1]:.3f}')}")
        if t_start is not None:
            self.lbl_tiempo.configure(text=f"t = {coma(f'{ahora - t_start:.1f}')} s desde START")

        # graficas y comparacion
        if t_start is not None:
            t_fin = max(copia[1][0][-1] if copia[1][0] else 0, 1)
            t_m, ym = modelo_actual(t_fin + 10)
            auto = self.var_auto.get()
            for g in self.graficas:
                g.actualizar(copia, t_m, ym, hist, t_fin, auto)
            if ahora - self.ult_analisis > 1.0:
                self.ult_analisis = ahora
                _texto, res = analizar()
                self.actualizar_tabla(res)
            self.graficas[self.libro.index("current")].canvas.draw_idle()

        self.root.after(INTERVALO_GRAFICA_MS, self.actualizar)


# Punto de entrada: crea la ventana y arranca el bucle de Tkinter.
def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
