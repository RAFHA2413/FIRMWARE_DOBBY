"""
DOBBY 2026 - Panel de control por serial / Bluetooth
Autor: Alcides Ramos

Requiere: Python 3 + pyserial      (pip install pyserial)
Ejecutar: python dobby_panel.py
          python dobby_panel.py --demo     (mueve los indicadores con datos de prueba)

Organizacion:
  self.ctrl[NOMBRE]  -> controles (lo que el usuario escribe), StringVar
  self.ind[NOMBRE]   -> indicadores (lo que llega del robot), StringVar
  self.leds[i], self.bateria, self.vel_angular, self.posicion, self.graf_izq, self.graf_der
  Las acciones de cada boton estan en la seccion ACCIONES (por ahora solo registran en TRAMA).
"""

import re
import sys
import math
import queue
import random
import threading
import time
import tkinter as tk
from tkinter import ttk
from collections import deque

import serial
import serial.tools.list_ports

# ----------------------------------------------------------------- estilo
FONDO = "#b4b4b4"
PANEL = "#c4c4c4"
CAJA = "#d6d6d6"
ROSA = "#e000e0"
AZUL = "#0000c0"
ROJO = "#e00000"
NEGRO = "#000000"

F_TITULO = ("Segoe UI", 12, "bold")
F_ETIQ = ("Segoe UI", 9, "bold")
F_VALOR = ("Segoe UI", 12, "bold")
F_PEQ = ("Segoe UI", 8)

NUM_SENSORES = 16
POS_MAX = 8.0          # rango de la barra de posicion (+-)
BAUDIOS = 115200
PUERTO_DEFECTO = "COM9"
UMBRAL_LED = 0.5       # sensor normalizado (0..1) por encima de esto = LED encendido

# dato suelto entre asteriscos:  *j100.00*   *UPVELDER-3.20*   *mnan*
#                (tambien con = en medio:  *LINCO=0*)
DATO_AST = re.compile(r"\*([A-Za-z]+)\s*=?\s*(-?(?:\d+(?:[.,]\d+)?|nan|inf))\*")
# inicio de las tramas de los sensores: traen siempre NUM_SENSORES valores
INICIO_SENSORES = re.compile(r"(?:Nor|Cru)=")

# pruebas del modo TEST, MISMO orden que el enum de pruebas.h (TESTSEL=n/ manda el indice)
PRUEBAS_ROBOT = ("MOTORES", "ENCODER", "SENSORES", "IMU")
TX_ESPACIO_MS = 50
# estados de la carrera, MISMO orden que el enum de carrera.h (*EST n*)
ESTADOS_CARRERA = ("REPOSO", "ARRANQUE", "PID", "RECTA", "CURVA", "FRENO", "PERDIDO")     # tiempo entre comandos enviados (que no lleguen pegados en una trama)
# boton Luces: (texto, fondo, letra). El indice es n de LUZ=n/, MISMO orden que LUZ_COLOR de main.c
LUCES = (("OFF", "#a08040", NEGRO), ("ROJO", "#e02020", "white"), ("VERDE", "#20c020", NEGRO),
         ("AZUL", "#2040e0", "white"), ("AMARILLO", "#ffd000", NEGRO),
         ("MORADO", "#8000ff", "white"), ("BLANCO", "#f4f4f4", NEGRO))

# color de linea que reporta el robot (*LINCO=n*), segun IR_ANA_SEN.h: blanco_ 0, negro_ 1
COLOR_LINEA_ROBOT = {0: "BLANCO", 1: "NEGRO"}


def separa(buf):
    """saca del texto recibido los datos *etiqueta valor* y las lineas completas.
    devuelve (eventos, resto_sin_procesar)"""
    eventos = []
    while True:
        m = DATO_AST.search(buf)
        if not m:
            break
        eventos.append(("dato", (m.group(1), m.group(2).replace(",", "."))))
        buf = buf[:m.start()] + buf[m.end():]
    while "\n" in buf:
        linea, buf = buf.split("\n", 1)
        linea = linea.strip("\r")
        if linea.strip():
            eventos.append(("linea", linea))
    if len(buf) > 4000:            # basura sin fin de linea: no dejar crecer el buffer
        buf = buf[-200:]
    return eventos, buf


# ======================================================================
#                          WIDGETS PROPIOS
# ======================================================================
class Led(tk.Canvas):
    def __init__(self, master, d=20, on="#30ff30", off="#1f3f1f"):
        super().__init__(master, width=d, height=d, bg=PANEL, highlightthickness=0)
        self.on, self.off = on, off
        self.circ = self.create_oval(2, 2, d - 2, d - 2, fill=off, outline="#202020", width=2)

    def set(self, encendido):
        self.itemconfig(self.circ, fill=self.on if encendido else self.off)

    def set_valor(self, v, umbral=0.5):
        """brillo segun el valor 0..1 (se ven los sensores a medias, ej. ruido de 0.3).
        con valor > umbral el borde se pone blanco = 'encendido'"""
        v = max(0.0, min(1.0, v))
        a = [int(self.off[i:i + 2], 16) for i in (1, 3, 5)]
        b = [int(self.on[i:i + 2], 16) for i in (1, 3, 5)]
        color = "#" + "".join(f"{int(x + (y - x) * v):02x}" for x, y in zip(a, b))
        self.itemconfig(self.circ, fill=color,
                        outline="white" if v > umbral else "#202020")


class Interruptor(tk.Button):
    """boton que cambia entre encendido / apagado"""
    def __init__(self, master, txt_on, txt_off, col_on="#40c040", col_off="#808080",
                 comando=None, fg_on=None, fg_off=None, **kw):
        super().__init__(master, command=self._click, font=F_ETIQ, relief="raised", bd=3, **kw)
        self.txt_on, self.txt_off = txt_on, txt_off
        self.col_on, self.col_off = col_on, col_off
        self.fg_on, self.fg_off = fg_on, fg_off      #color de letra por estado (opcional)
        self.comando = comando
        self.estado = False
        self._pinta()

    def _pinta(self):
        self.config(text=self.txt_on if self.estado else self.txt_off,
                    bg=self.col_on if self.estado else self.col_off,
                    activebackground=self.col_on if self.estado else self.col_off,
                    relief="sunken" if self.estado else "raised")
        fg = self.fg_on if self.estado else self.fg_off
        if fg:
            self.config(fg=fg, activeforeground=fg)

    def _click(self):
        self.estado = not self.estado
        self._pinta()
        if self.comando:
            self.comando(self.estado)

    def set(self, estado):
        self.estado = bool(estado)
        self._pinta()


class Reloj(tk.Canvas):
    """indicador de aguja.  angulos en grados (0 = derecha, antihorario)"""
    def __init__(self, master, ancho, alto, cx, cy, r, vmin, vmax, a_ini, a_fin, paso,
                 banda=False, etiqueta_fuera=False, decimales=0):
        super().__init__(master, width=ancho, height=alto, bg=PANEL, highlightthickness=0)
        self.cx, self.cy, self.r = cx, cy, r
        self.vmin, self.vmax = vmin, vmax
        self.a_ini, self.a_fin = a_ini, a_fin
        self.dec = decimales
        self._fondo(paso, banda, etiqueta_fuera)
        self.aguja = self.create_line(cx, cy, cx, cy - r, width=3, fill=ROJO)
        self.create_oval(cx - 7, cy - 7, cx + 7, cy + 7, fill="#909090", outline="#404040")
        bx, by = cx, cy + (r * 0.32 if not banda else 26)
        self.create_rectangle(bx - 28, by - 11, bx + 28, by + 11, fill=CAJA, outline="#707070")
        self.txt = self.create_text(bx, by, text="0", font=F_ETIQ, fill=ROJO)
        self.set(0)

    def _ang(self, v):
        v = max(self.vmin, min(self.vmax, v))
        return self.a_ini + (v - self.vmin) / (self.vmax - self.vmin) * (self.a_fin - self.a_ini)

    def _xy(self, ang, rr):
        a = math.radians(ang)
        return self.cx + rr * math.cos(a), self.cy - rr * math.sin(a)

    def _fondo(self, paso, banda, fuera):
        cx, cy, r = self.cx, self.cy, self.r
        caja = (cx - r, cy - r, cx + r, cy + r)
        if banda:   # degradado verde -> amarillo -> rojo
            n = 40
            for i in range(n):
                v0 = self.vmin + (self.vmax - self.vmin) * i / n
                v1 = self.vmin + (self.vmax - self.vmin) * (i + 1) / n
                t = i / (n - 1)
                rr = int(min(1, 2 * t) * 255)
                gg = int(min(1, 2 * (1 - t)) * 200)
                self.create_arc(caja, start=self._ang(v0), extent=self._ang(v1) - self._ang(v0),
                                style="arc", width=12, outline=f"#{rr:02x}{gg:02x}00")
        else:
            self.create_oval(cx - r - 6, cy - r - 6, cx + r + 6, cy + r + 6,
                             fill="#e8e8e8", outline="#606060", width=3)
            self.create_arc(caja, start=self.a_ini, extent=self.a_fin - self.a_ini,
                            style="arc", width=2, outline="#404040")
        v = self.vmin
        while v <= self.vmax + 1e-9:
            a = self._ang(v)
            x0, y0 = self._xy(a, r - 8)
            x1, y1 = self._xy(a, r + (6 if banda else 0))
            self.create_line(x0, y0, x1, y1, width=2, fill="#303030")
            xt, yt = self._xy(a, r + 16 if fuera else r - 22)
            self.create_text(xt, yt, text=f"{v:g}", font=F_PEQ, fill=NEGRO)
            v += paso

    def set(self, v):
        x, y = self._xy(self._ang(v), self.r * 0.85)
        self.coords(self.aguja, self.cx, self.cy, x, y)
        self.itemconfig(self.txt, text=f"{v:.{self.dec}f}")


class BarraPosicion(tk.Canvas):
    """barra horizontal -max..+max, se llena desde la izquierda hasta el valor"""
    def __init__(self, master, ancho=290, vmax=POS_MAX):
        super().__init__(master, width=ancho, height=100, bg=PANEL, highlightthickness=0)
        self.vmax = vmax
        self.x0, self.x1 = 15, ancho - 15
        self.xc = (self.x0 + self.x1) / 2
        self.create_rectangle(self.x0 - 4, 6, self.x1 + 4, 44, fill="#e0e0e0", outline="#606060", width=2)
        self.create_rectangle(self.x0, 10, self.x1, 40, fill="#808080", outline="")
        self.barra = self.create_rectangle(self.xc, 10, self.xc, 40, fill="#ffb400", outline="")
        self.create_line(self.xc, 6, self.xc, 44, fill="#303030")
        paso = 2 if vmax > 6 else 1
        v = -vmax
        while v <= vmax + 1e-9:
            x = self._x(v)
            self.create_line(x, 44, x, 50, fill="#303030")
            self.create_text(x, 58, text=f"{v:g}", font=F_PEQ)
            v += paso
        self.create_rectangle(self.xc - 45, 70, self.xc + 45, 94, fill=CAJA, outline="#707070")
        self.txt = self.create_text(self.xc, 82, text="0", font=F_ETIQ, fill=ROJO)
        self.titulo = self.create_text(self.x0 - 4, 82, text="POSICION", font=F_ETIQ, anchor="w")
        self.hay_linea = True

    def _x(self, v):
        v = max(-self.vmax, min(self.vmax, v))
        return self.xc + v / self.vmax * (self.x1 - self.xc)

    def set(self, v):
        x = self._x(v)
        self.coords(self.barra, self.x0, 10, x, 40)     #relleno desde la izquierda hasta el valor
        self.itemconfig(self.txt, text=f"{v:.5f}")

    def set_linea(self, hay):
        """*L0*: el robot no ve la linea -> barra roja y SIN LINEA
        (la posicion que muestra es el extremo del ultimo lado donde la vio)"""
        if hay == self.hay_linea:
            return
        self.hay_linea = hay
        self.itemconfig(self.barra, fill="#ffb400" if hay else "#d03030")
        self.itemconfig(self.titulo, text="POSICION" if hay else "SIN LINEA",
                        fill=NEGRO if hay else "#d03030")


class Grafica(tk.Canvas):
    """grafica de tiempo con varias series.
    auto=True: la escala vertical se ajusta sola a los datos (m/s, rpm, lo que llegue)"""
    DIV = 10

    def __init__(self, master, ancho, alto, ymin, ymax, n=300,
                 series=(("SET", "#ffff00"), ("VEL", "#00ff00")), auto=True):
        super().__init__(master, width=ancho, height=alto, bg=PANEL, highlightthickness=0)
        self.ml, self.mr, self.mt, self.mb = 50, 10, 22, 28
        self.ancho, self.alto = ancho, alto
        self.ymin, self.ymax, self.n = ymin, ymax, n
        self.auto = auto
        self.x0, self.y0 = self.ml, self.mt
        self.x1, self.y1 = ancho - self.mr, alto - self.mb
        self.create_rectangle(self.x0, self.y0, self.x1, self.y1, fill="black", outline="#404040")
        self.txt_y = []
        for i in range(self.DIV + 1):
            y = self.y1 - (self.y1 - self.y0) * i / self.DIV
            self.create_line(self.x0, y, self.x1, y, fill="#202020")
            self.create_line(self.x0 - 4, y, self.x0, y)
            self.txt_y.append(self.create_text(self.x0 - 6, y, anchor="e", font=F_PEQ))
        self._pon_escala(ymin, ymax)
        self.create_text((self.x0 + self.x1) / 2, alto - 8, text="Tiempo", font=F_PEQ)
        self.datos = [deque(maxlen=n) for _ in series]
        self.lineas = []
        xl = self.x1
        for i, (nombre, color) in enumerate(reversed(series)):
            self.create_text(xl, 10, text=nombre, anchor="e", font=F_ETIQ, fill=color)
            self.create_rectangle(xl - 8 * len(nombre) - 22, 4, xl - 8 * len(nombre) - 8, 16,
                                  fill=color, outline="")
            xl -= 8 * len(nombre) + 34
        for _, color in series:
            self.lineas.append(self.create_line(0, 0, 0, 0, fill=color, width=2))
        self.sucio = False

    def _y(self, v):
        v = max(self.ymin, min(self.ymax, v))
        return self.y1 - (v - self.ymin) / (self.ymax - self.ymin) * (self.y1 - self.y0)

    def _pon_escala(self, ymin, ymax):
        self.ymin, self.ymax = ymin, ymax
        paso = (ymax - ymin) / self.DIV
        dec = 0 if paso >= 1 else (1 if paso >= 0.1 else (2 if paso >= 0.01 else 3))
        if dec and abs(paso * 10 ** dec - round(paso * 10 ** dec)) > 1e-6:
            dec += 1                                    # pasos como 0.25 necesitan un decimal mas
        for i, t in enumerate(self.txt_y):
            yv = ymin + paso * i
            self.itemconfig(t, text=f"{yv:.{dec}f}")

    def _escala_redonda(self, lo, hi):
        """escala con pasos 1, 2, 2.5, 5 x 10^k que contenga lo..hi"""
        span = max(hi - lo, 1e-3)
        base = 10 ** math.floor(math.log10(span / self.DIV))
        for m in (1, 2, 2.5, 5, 10, 20, 25, 50):
            paso = m * base
            ymin = math.floor(lo / paso) * paso
            if ymin + paso * self.DIV >= hi:
                return ymin, ymin + paso * self.DIV
        return lo, hi

    def _ajusta_escala(self):
        """cambia la escala si los datos se salen o si quedan muy apretados"""
        valores = [v for d in self.datos for v in d]
        if not valores:
            return
        lo, hi = min(valores), max(valores)
        rango = self.ymax - self.ymin
        sale = lo < self.ymin or hi > self.ymax
        apretado = (hi - lo) < 0.25 * rango
        if not (sale or apretado):
            return
        margen = max((hi - lo) * 0.1, abs(hi) * 0.05, 0.05)
        self._pon_escala(*self._escala_redonda(lo - margen, hi + margen))

    def agrega(self, *valores):
        for d, v in zip(self.datos, valores):
            d.append(v)
        self.sucio = True

    def limpia(self):
        for d in self.datos:
            d.clear()
        self.sucio = True

    def redibuja(self):
        if not self.sucio:
            return
        self.sucio = False
        if self.auto:
            self._ajusta_escala()
        dx = (self.x1 - self.x0) / (self.n - 1)
        for d, linea in zip(self.datos, self.lineas):
            if len(d) < 2:
                self.coords(linea, 0, 0, 0, 0)
                continue
            pts = []
            for i, v in enumerate(d):
                pts += (self.x0 + i * dx, self._y(v))
            self.coords(linea, *pts)


# ======================================================================
#                              PANEL
# ======================================================================
class Panel(tk.Tk):
    def __init__(self, demo=False):
        super().__init__()
        self.title("DOBBY 2026 - Panel de control")
        self.configure(bg=FONDO)
        self.geometry("1230x800")

        self.ctrl = {}      # entradas del usuario
        self.ind = {}       # indicadores
        self.enviado = {}   # ultimo texto enviado de cada cuadro (para no repetir al salir)
        self.editable = {}  # cuadros blancos que envian al robot: nombre -> (spinbox, formato)
        self.ser = None
        self.cola = queue.Queue()
        self.tx_cola = deque()      # comandos por enviar (uno cada TX_ESPACIO_MS)
        self.leyendo = False

        estilo = ttk.Style(self)
        estilo.configure("TNotebook", background=FONDO)
        estilo.configure("TNotebook.Tab", font=F_ETIQ, padding=(12, 4))

        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=4, pady=4)
        self.tab1 = tk.Frame(nb, bg=PANEL, width=1210, height=760)
        self.tab2 = tk.Frame(nb, bg=PANEL, width=1210, height=760)
        nb.add(self.tab1, text="PRINCIPAL")
        nb.add(self.tab2, text="VELO")

        self._crea_principal(self.tab1)
        self._crea_velo(self.tab2)
        self._pinta_run(False)

        #como LabVIEW: un clic en cualquier parte (boton, otro cuadro, fondo) le quita el foco
        #al cuadro que se estaba escribiendo -> <FocusOut> manda su valor
        self.bind_all("<Button-1>", self._click_foco, add="+")
        #SEGURIDAD: tecla Esc = STOP desde cualquier parte del panel
        self.bind_all("<Escape>", lambda e: self.stop())

        self.protocol("WM_DELETE_WINDOW", self.salir)
        self.after(30, self._ciclo)
        self.after(TX_ESPACIO_MS, self._tx_ciclo)
        if demo:
            self._t_demo = 0.0
            self.after(50, self._demo)

    # ------------------------------------------------ ayudas de dibujo
    def etiqueta(self, padre, x, y, texto, color=NEGRO, fuente=F_ETIQ):
        tk.Label(padre, text=texto, bg=PANEL, fg=color, font=fuente).place(x=x, y=y)

    def campo(self, padre, x, y, nombre, valor, desde, hasta, paso, formato="%.2f",
              ancho=6, lectura=True, titulo=None, envia=None):
        """control numerico (rosa) con su valor leido del robot debajo (rojo)
        envia: prefijo de la trama que se manda al cambiarlo, ej. envia="PROPOR" -> PROPOR=50.00/
               se manda con: flechas, Enter (siempre) o al salir del cuadro (solo si cambio)"""
        self.etiqueta(padre, x, y, titulo or nombre, ROSA, F_TITULO)
        var = tk.StringVar(value=formato % valor)
        sp = tk.Spinbox(padre, from_=desde, to=hasta, increment=paso, format=formato,
                        textvariable=var, width=ancho, font=F_VALOR, fg=AZUL,
                        justify="left", bd=2, relief="sunken")
        if envia:
            mandar = lambda *_: self.envia_campo(nombre, envia, formato)
            sp.config(command=mandar)            # flechas
            sp.bind("<Return>", mandar)          # escribir el valor y Enter
            sp.bind("<KP_Enter>", mandar)
            # salir del cuadro (Tab, clic en otro lado): manda solo si el valor cambio
            sp.bind("<FocusOut>",
                    lambda e: self.envia_campo(nombre, envia, formato, solo_si_cambia=True))
            self.enviado[nombre] = formato % valor   # el valor inicial cuenta como ya enviado
            self.editable[nombre] = (sp, formato)
        sp.place(x=x, y=y + 29)
        var.set(formato % valor)
        self.ctrl[nombre] = var
        if lectura:
            iv = tk.StringVar(value="0")
            tk.Label(padre, textvariable=iv, width=ancho + 1, anchor="w", font=F_VALOR, fg=ROJO,
                     bg=CAJA, bd=2, relief="sunken").place(x=x, y=y + 63)
            self.ind[nombre + "_R"] = iv
        return var

    def indicador(self, padre, x, y, nombre, ancho=8, titulo=None, color_titulo=NEGRO):
        if titulo != "":
            self.etiqueta(padre, x, y, titulo or nombre, color_titulo)
            y += 23
        iv = tk.StringVar(value="0")
        tk.Label(padre, textvariable=iv, width=ancho, anchor="w", font=F_VALOR, fg=ROJO,
                 bg=CAJA, bd=2, relief="sunken").place(x=x, y=y)
        self.ind[nombre] = iv
        return iv

    def boton(self, padre, x, y, texto, accion, ancho=10, alto=1, color="#d0d0d0", fg=NEGRO,
              fuente=F_ETIQ):
        b = tk.Button(padre, text=texto, command=accion, width=ancho, height=alto, bg=color,
                      fg=fg, font=fuente, bd=3, relief="raised", activebackground="#a0a0a0")
        b.place(x=x, y=y)
        return b

    # ------------------------------------------------ pestaña PRINCIPAL
    def _crea_principal(self, p):
        # puerto
        self.etiqueta(p, 10, 6, "PUERTO")
        self.cb_puerto = ttk.Combobox(p, width=14, state="readonly")
        self.cb_puerto.place(x=10, y=28)
        self.boton(p, 130, 25, "↻", self.actualiza_puertos, ancho=2)
        self.bt_conectar = Interruptor(p, "CONECTADO", "CONECTAR", "#40c040", "#d0d0d0",
                                       comando=self.conectar, width=11)
        self.bt_conectar.place(x=165, y=24)
        self.actualiza_puertos()

        # bateria
        self.etiqueta(p, 40, 62, "BATERIA")
        self.bateria = Reloj(p, 260, 160, 130, 120, 92, 0, 10, 150, 30, 2,
                             banda=True, etiqueta_fuera=True, decimales=2)
        self.bateria.place(x=20, y=80)

        # sensores de linea
        nombres = [f"I{i}" for i in range(NUM_SENSORES // 2, 0, -1)] + \
                  [f"D{i}" for i in range(1, NUM_SENSORES // 2 + 1)]
        self.leds = []
        self.lb_crudo = []
        x0 = 300
        for i, n in enumerate(nombres):
            x = x0 + i * 24
            self.etiqueta(p, x, 78, n, fuente=F_PEQ)
            led = Led(p)
            led.place(x=x, y=96)
            self.leds.append(led)
            #lectura cruda del ADC de este sensor (Cru=..., solo en REPOSO)
            lb = tk.Label(p, text="", font=("Segoe UI", 7), bg=PANEL, fg=AZUL)
            lb.place(x=x + 10, y=116 if i % 2 == 0 else 127, anchor="n")   #2 filas: no se pisan
            self.lb_crudo.append(lb)

        # posicion de la linea
        #(el titulo POSICION va dentro de la barra: arriba van los numeros crudos)
        self.posicion = BarraPosicion(p)
        self.posicion.place(x=355, y=146)

        # velocidades medidas
        self.indicador(p, 670, 148, "VELO_IZQ", 7)
        self.indicador(p, 790, 148, "VELO_DER", 7)

        # codigo ir, test
        self.indicador(p, 720, 50, "IR_CODIGO", 13)
        #TEST: LED = bandera que reporta el robot (*TST1*), ON/OFF y que prueba correr
        self.led_test = Led(p, d=18, on="#ffd000", off="#3f3a1f")
        self.led_test.place(x=700, y=112)
        self.etiqueta(p, 725, 112, "TEST")
        self.bt_test = Interruptor(p, "ON", "OFF", comando=self.cambia_test, width=4)
        self.bt_test.place(x=770, y=106)
        self.cb_test = ttk.Combobox(p, values=PRUEBAS_ROBOT, width=10, state="readonly")
        self.cb_test.current(0)
        self.cb_test.place(x=830, y=111)
        self.cb_test.bind("<<ComboboxSelected>>",
                          lambda e: self.enviar(f"TESTSEL={self.cb_test.current()}/"))

        # color de linea
        self.etiqueta(p, 1000, 6, "LINEA", fuente=F_TITULO)
        #activo = BLANCO (LINCO=0/), inactivo = NEGRO (LINCO=1/)   (IR_ANA_SEN.h: blanco_ 0, negro_ 1)
        self.bt_linea = Interruptor(p, "BLANCO", "NEGRO", "#f4f4f4", "#202020",
                                    comando=self.cambia_linea, width=9,
                                    fg_on=AZUL, fg_off="white")
        self.bt_linea.place(x=975, y=34)
        #color de linea que esta siguiendo el robot (*LINCO=n*)
        self.lb_linea = tk.Label(p, text="?", width=8, font=F_ETIQ, bg=CAJA, fg=ROJO,
                                 bd=2, relief="sunken")
        self.lb_linea.place(x=1095, y=38)

        # velocidad angular
        self.etiqueta(p, 1000, 82, "VEL_ANGULAR")
        self.vel_angular = Reloj(p, 210, 210, 105, 105, 82, -300, 300, 225, -45, 50)
        self.vel_angular.place(x=975, y=106)

        # velocidad basica y de giro
        self.boton(p, 15, 262, "LEE DATOS\nPID", lambda: self.enviar("PIDVAL"), ancho=10, alto=2)
        #velocidad base y maxima diferencia de giro, en PWM 0..1000 (carrera.h)
        self.campo(p, 130, 240, "V_BAS", 0, 0, 1000, 10, "%.0f", titulo="V_Bas", envia="VBASICA")
        self.campo(p, 240, 240, "V_GIR", 250, 0, 1000, 10, "%.0f", titulo="V_Gir", envia="VGIRO")

        # sd y flash
        self.etiqueta(p, 360, 250, "GUARDA\nSD+FLASH")
        self.boton(p, 360, 290, "GUARDAR", lambda: self.enviar("GUARDASD"), ancho=8)
        self.etiqueta(p, 470, 250, "  LEE\n  SD")
        self.boton(p, 460, 290, "LEER", lambda: self.enviar("LEESD"), ancho=8)
        self.etiqueta(p, 565, 250, "  LEE\nFLASH")
        self.boton(p, 560, 290, "LEER", lambda: self.enviar("LEEFLASH"), ancho=8)

        # PWM manual de cada motor (lazo abierto, paso 1 de la calibracion)
        #manda PWMIZQ=valor/  y  PWMDER=valor/   (-1000..1000)
        self.campo(p, 670, 228, "PWM_IZQ", 0, -1000, 1000, 10, "%.0f", lectura=False, envia="PWMIZQ")
        self.campo(p, 790, 228, "PWM_DER", 0, -1000, 1000, 10, "%.0f", lectura=False, envia="PWMDER")

        # pid de linea
        y = 340
        #salida del PID de linea en PWM (PID.h)
        self.campo(p, 25, y, "KP", 40, 0, 10000, 1, "%.2f", envia="PROPOR")
        self.campo(p, 125, y, "KI", 0, 0, 10000, 0.1, "%.2f", envia="INTEGRA")
        self.campo(p, 225, y, "KD", 1, 0, 10000, 0.1, "%.2f", envia="DERIVA")
        self.campo(p, 325, y, "TAU", 5, 1, 50, 0.5, "%.1f", envia="TAU")
        self.campo(p, 425, y, "KAW", 0.3, 0, 5, 0.05, "%.2f", envia="KAW")
        self.indicador(p, 545, y + 20, "UP", 9)
        self.indicador(p, 665, y + 20, "UI", 9)
        self.indicador(p, 785, y + 20, "UD", 9)
        self.indicador(p, 905, y + 20, "UT", 9)

        #KG: IMU en el PID de direccion: giro = PID - KG * giro_z (grados/s). 0 = sin IMU
        #rango util 0..1, empezar en 0.1 (amortigua: menos oscilacion)
        self.campo(p, 25, 450, "KG", 0, 0, 5, 0.05, "%.2f", envia="KGIRO")

        # estado de la maquina de carrera (*EST n*)
        self.indicador(p, 1050, 325, "ESTADO", 9, color_titulo=AZUL)
        #velocidad base que esta usando el robot (V_Bas menos lo que baja el IMU en curva)
        self.indicador(p, 1050, 381, "V_BASE", 9, titulo="V_BASE (IMU)", color_titulo=AZUL)

        # run / stop / salir
        self.corriendo = False
        self.t_run_local = 0.0      # ultima vez que el panel mando RUN o STOP
        self.bt_run = self.boton(p, 130, 470, "", self.run, ancho=20, alto=3,
                   color="#d8d8d8", fuente=("Segoe UI", 14, "bold"))
        self.boton(p, 600, 555, "STOP", self.stop, ancho=10, alto=2,
                   color="#d0d0d0", fg=ROJO, fuente=("Segoe UI", 11, "bold"))
        self.boton(p, 470, 560, "SALIR", self.salir, ancho=9, alto=2, color="#d06060",
                   fg="white")

        # musica y luces
        self.campo(p, 480, 455, "PLAY", 8, 1, 99, 1, "%.0f", ancho=3, lectura=False, titulo="Play")
        self.etiqueta(p, 590, 448, "VOLUMEN")
        self.boton(p, 580, 468, "▲  Up", lambda: self.accion("VOL_UP"), ancho=10)
        self.boton(p, 580, 505, "▼  Down", lambda: self.accion("VOL_DOWN"), ancho=10)
        self.boton(p, 700, 505, "◀  Previous", lambda: self.accion("PREVIOUS"), ancho=11)
        self.boton(p, 730, 565, "Next  ▶", lambda: self.accion("NEXT"), ancho=11)
        self.etiqueta(p, 720, 448, "Musica")
        self.bt_musica = Interruptor(p, "ON", "OFF", "#ffb000", "#a08040",
                                     comando=lambda e: self.accion("MUSICA", e), width=5)
        self.bt_musica.place(x=715, y=468)
        self.etiqueta(p, 820, 448, "Luces")
        self.luz = 0                # indice en LUCES (0 = apagadas)
        self.bt_luces = tk.Button(p, command=self.cambia_luces, font=F_ETIQ, bd=3, width=8)
        self.bt_luces.place(x=812, y=468)
        self._pinta_luces()

        # rectas (carrera.c, estado RECTA). Error_LIM, TRAMO o V_LINEA en 0 = rectas apagadas
        #es recta si |posicion| <= Error_LIM  y  |giro_z| <= GIRO_LIM, durante TRAMO cm seguidos
        #rango +- de la posicion de la linea (-7.5..7.5) para considerar que va en recta
        self.campo(p, 940, 440, "ERROR_LIM", 1.5, 0, 7.5, 0.1, "%.1f", titulo="Error_LIM",
                   envia="RECTERR")
        #rango +- de la velocidad angular del IMU (grados/s). 0 = no mira el IMU
        self.campo(p, 1060, 440, "GIRO_LIM", 40, 0, 500, 5, "%.0f", titulo="GIRO_LIM °/s",
                   envia="RECTGIR")
        #cm seguidos en recta (distancia del encoder) antes de pasar a V_LINEA
        self.campo(p, 940, 545, "TRAMO", 30, 0, 500, 5, "%.0f", titulo="TRAMO cm",
                   envia="RECTLARG")
        #velocidad base en recta en PWM: 0 .. 1000 (igual que V_Bas)
        self.campo(p, 1060, 545, "V_LINEA", 500, 0, 1000, 10, "%.0f", titulo="V_LINEA",
                   envia="RECTVEL")
        #rampa de V_Bas a V_LINEA en PWM/s. Mas baja: si detecta tarde gana poca velocidad
        self.campo(p, 840, 545, "ACEL_R", 300, 50, 5000, 50, "%.0f", titulo="ACEL_R",
                   envia="ACELR")
        # freno al salir de la recta (estado FRENO): base = V_FRENO hasta bajar a V_Bas
        #velocidad base mientras frena en PWM: 0 .. 1000 (0 = freno maximo)
        self.campo(p, 940, 650, "V_FRENO", 115, 0, 1000, 10, "%.0f", titulo="V_FRENO",
                   envia="FRENOV")
        #tiempo maximo frenando (ms). 0 = sin freno
        self.campo(p, 1060, 650, "T_FRENO", 200, 0, 2000, 10, "%.0f", titulo="T_FRENO ms",
                   envia="FRENOMS")
        #cm que lleva el robot en recta ahora mismo (*RD*): sirve para elegir TRAMO
        self.indicador(p, 25, 600, "RECTA_CM", 8, titulo="EN RECTA cm", color_titulo=AZUL)
        #LED verde: Error_LIM y GIRO_LIM se cumplen durante TRAMO cm seguidos (dato_recta_cm)
        self.etiqueta(p, 1165, 576, "RECTA", AZUL)
        self.led_recta = Led(p, d=28)
        self.led_recta.place(x=1172, y=600)

        # Competition V5: botones de aprendizaje, sin modificar la pista.
        # APRENDER inicia RUN y la grabacion. FIN VUELTA congela el mapa en RAM y frena.
        self.etiqueta(p, 25, 655, "PISTA RAM", AZUL)
        self.boton(p, 25, 680, "APRENDER", self.aprender_pista,
                   ancho=9, color="#a9dfd6")
        self.boton(p, 125, 680, "FIN VUELTA", self.fin_vuelta_pista,
                   ancho=10, color="#f5d98a")
        self.boton(p, 25, 718, "ESTADO MAPA",
                   lambda: self.enviar("MAPSTATUS/"), ancho=12)

        # trama recibida
        self.etiqueta(p, 225, 618, "TRAMA")
        #lineas que llegaron con bytes perdidos en el enlace, de las de sensores que llegaron
        self.tramas_ok, self.tramas_mal = 0, 0
        self.lb_tramas = tk.Label(p, text="", bg=PANEL, fg=NEGRO, font=F_ETIQ)
        self.lb_tramas.place(x=300, y=618)
        self._pinta_tramas()
        marco = tk.Frame(p, bg=PANEL)
        marco.place(x=225, y=640, width=700, height=110)
        self.trama = tk.Text(marco, font=("Consolas", 9), bg="#e4e4e4", bd=2, relief="sunken")
        sb = tk.Scrollbar(marco, command=self.trama.yview)
        self.trama.config(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.trama.pack(side="left", fill="both", expand=True)

    # ------------------------------------------------ pestaña VELO
    def _crea_velo(self, p):
        x = 25
        #             cuadro   valor paso  formato  trama que envia   (valores = PID.h)
        for nombre, val, paso, fmt, envia in (("KP_V", 763.72, 5, "%.2f", "VKP"),
                                              ("KI_V", 15497, 50, "%.0f", "VKI"),
                                              ("KD_V", 0, 0.5, "%.2f", "VKD"),
                                              ("TAU_V", 5, 0.5, "%.1f", "VTAU"),
                                              ("KAW_V", 0.5, 0.05, "%.2f", "VKAW")):
            self.campo(p, x, 30, nombre, val, 0, 100000, paso, fmt, envia=envia)
            x += 100

        # partes del pid de velocidad
        for i, n in enumerate(("UPVEL", "UIVEL", "UDVEL", "UTVEL")):
            self.etiqueta(p, 545 + i * 105, 40, n, ROJO, F_TITULO)
        for fila, (lado, y) in enumerate((("DER", 70), ("IZQ", 104))):
            for i, n in enumerate(("UPVEL", "UIVEL", "UDVEL", "UTVEL")):
                self.indicador(p, 545 + i * 105, y, f"{n}_{lado}", 7, titulo="")
        self.etiqueta(p, 970, 72, "DERECHA", "#2020e0", F_TITULO)
        self.etiqueta(p, 970, 106, "IZQUIERDA", "#2020e0", F_TITULO)

        self.indicador(p, 1100, 20, "BATERIA_V", 7, titulo="BATERIA")
        #modo del control de las llantas (*VMODO n*)
        self.indicador(p, 1100, 95, "MODO_V", 8, titulo="MODO")
        #bandera de TEST del robot (*TST n*), igual que en PRINCIPAL
        self.led_test_v = Led(p, d=18, on="#ffd000", off="#3f3a1f")
        self.led_test_v.place(x=1100, y=158)
        self.lb_test_v = tk.Label(p, text="TEST OFF", font=F_ETIQ, bg=PANEL, fg=NEGRO,
                                  justify="left")
        self.lb_test_v.place(x=1122, y=157)

        # motores
        self.sp_rx = {"IZQ": None, "DER": None}     # set point que reporta el robot (SETIZ/SETDE)
        self.sug = {}                               # Kp, Ki sugeridos por la identificacion
        self.graf_izq = self._motor(p, 25, 175, "IZQUIERDA", "IZQ", "SET_POINT_I")
        self.graf_der = self._motor(p, 620, 175, "DERECHA", "DER", "SET_POINT_D")

        # calibracion (ver PROCEDIMIENTO en llanta.h)
        y = 668
        self.campo(p, 25, y, "V_BAJA", 0.30, -5, 5, 0.1, "%.2f", ancho=5, lectura=False,
                   titulo="V_baja", envia="VBAJA")             # nivel bajo del escalon (m/s)
        self.campo(p, 120, y, "ESC_MS", 1000, 100, 10000, 100, "%.0f", ancho=5, lectura=False,
                   titulo="T_esc ms", envia="VESCMS")          # tiempo en cada nivel
        self.campo(p, 225, y, "PWM_ID", 500, 50, 1000, 10, "%.0f", ancho=5, lectura=False,
                   titulo="PWM_id")                            # escalon de la identificacion
        self.bt_pid = Interruptor(p, "PID ON", "PID OFF", width=8,
                                  comando=lambda e: self.modo_vel("VPID", e))
        self.bt_pid.place(x=330, y=692)
        self.bt_esc = Interruptor(p, "ESCALON ON", "ESCALON OFF", width=11,
                                  comando=lambda e: self.modo_vel("VESC", e))
        self.bt_esc.place(x=425, y=692)
        self.boton(p, 548, 692, "IDENTIFICAR", self.identificar, ancho=11, color="#ffd060")
        self.boton(p, 668, 692, "STOP MOTORES", self.stop_motores, ancho=12, fg=ROJO)
        self.boton(p, 798, 692, "LEE PID", lambda: self.enviar("PIDVAL"), ancho=8)
        self.indicador(p, 893, y, "KP_SUG", 7, color_titulo=ROSA)
        self.indicador(p, 993, y, "KI_SUG", 7, color_titulo=ROSA)
        self.boton(p, 1093, 692, "APLICAR", self.aplicar_sugeridos, ancho=9, color="#60c060")

    def _motor(self, p, x, y, titulo, lado, nombre_sp):
        self.indicador(p, x + 320, y - 30, f"SET_{lado}", 7)
        self.etiqueta(p, x, y + 10, f"MOTOR DE LA {titulo}", fuente=("Segoe UI", 12))
        marco = tk.Frame(p, bg=PANEL, bd=2, relief="groove")
        marco.place(x=x, y=y + 38, width=570, height=450)
        #SET_POINT_I manda VIZQ=valor/,  SET_POINT_D manda VDER=valor/
        #en m/s: paso 0.1, dos decimales
        self.campo(marco, 10, 6, nombre_sp, 0, -5, 5, 0.1, "%.2f", lectura=False,
                   envia="V" + lado)
        self.indicador(marco, 170, 6, f"VELOCIDAD_{lado}", 8)
        self.indicador(marco, 300, 6, f"OUT{lado}", 8)
        tk.Button(marco, text="Limpiar", font=F_PEQ,
                  command=lambda: (self.graf_izq if lado == "IZQ" else self.graf_der).limpia()
                  ).place(x=480, y=30)
        g = Grafica(marco, 550, 330, 0, 2)
        g.place(x=6, y=80)
        #resultado de la identificacion de esta llanta (IDENTIZQ= / IDENTDER=)
        iv = tk.StringVar(value="Identificacion: -")
        tk.Label(marco, textvariable=iv, font=F_ETIQ, fg=AZUL, bg=PANEL, anchor="w"
                 ).place(x=8, y=414, width=550)
        self.ind[f"IDENT_{lado}"] = iv
        return g

    # ======================================================================
    #                    ACCIONES (por definir con el usuario)
    # ======================================================================
    def accion(self, nombre, *args):
        extra = " " + " ".join(str(a) for a in args) if args else ""
        self.log(f"[pendiente] {nombre}{extra}")

    def envia_campo(self, nombre, prefijo, formato, solo_si_cambia=False):
        """manda PREFIJO=valor/ con el valor del cuadro.
        solo_si_cambia=True (al salir del cuadro): no manda si es igual al ultimo enviado"""
        texto = self.ctrl[nombre].get().strip().replace(",", ".")
        try:
            valor = float(texto)
        except ValueError:
            self.log(f"{nombre}: '{texto}' no es un numero")
            if nombre in self.enviado:              # vuelve al ultimo valor bueno
                self.ctrl[nombre].set(self.enviado[nombre])
            return
        texto = formato % valor
        self.ctrl[nombre].set(texto)               # ej. "1" -> "1.00": se ve que ya se envio
        if solo_si_cambia and self.enviado.get(nombre) == texto:
            return
        if self.enviar(f"{prefijo}={texto}/"):
            self.enviado[nombre] = texto

    def _click_foco(self, evento):
        """un clic pasa el foco al widget tocado (botones y fondo incluidos), asi el
        cuadro que se estaba escribiendo recibe <FocusOut> y manda su valor ANTES
        de que corra la accion del boton"""
        w = evento.widget
        if hasattr(w, "focus_set"):
            try:
                w.focus_set()
            except tk.TclError:
                pass

    def aprender_pista(self):
        """Inicia una vuelta de reconocimiento V5 a velocidad de V3."""
        if self.corriendo:
            self.log("MAPA: detener primero el robot")
            return
        if self.enviar("MAPBEGIN/"):
            self._pinta_run(True)
            self.t_run_local = time.monotonic()
            self.log("MAPA: aprendizaje solicitado; marque FIN VUELTA en la salida")

    def fin_vuelta_pista(self):
        """Marca el cierre del circuito y detiene DOBBY; el mapa queda solo en RAM."""
        if self.enviar("MAPEND/"):
            self._pinta_run(False)
            self.t_run_local = time.monotonic()

    def run(self):
        """boton grande: muestra el ESTADO del robot y al hacer clic lo cambia
        (detenido -> manda RUN,  andando -> manda STOP)"""
        if not self.corriendo:
            self.enviar("RUN")
            self._pinta_run(True)
            self.t_run_local = time.monotonic()
        else:
            self.stop()

    def stop(self):
        """envia STOP (urgente) y pone el boton grande en STOP rojo.
        Lo usan: boton grande, boton STOP, tecla Esc"""
        self.enviar("STOP", urgente=True)
        self._pinta_run(False)
        self.t_run_local = time.monotonic()

    def _pinta_run(self, corriendo):
        """el boton muestra el ESTADO:  detenido = STOP rojo,  andando = RUN verde"""
        self.corriendo = corriendo
        if corriendo:
            self.bt_run.config(text="▶    RUN", bg="#30b030", fg="white",
                               activebackground="#208020", activeforeground="white")
        else:
            self.bt_run.config(text="■    STOP", bg="#e03030", fg="white",
                               activebackground="#b02020", activeforeground="white")

    # ------------------------------------------------ modo TEST
    def cambia_test(self, encendido):
        """TESTON / TESTOFF.  Al prender manda tambien la prueba elegida y el PWM de los cuadros"""
        if encendido:
            self.enviar("TESTON")
            self.enviar(f"TESTSEL={self.cb_test.current()}/")
            self.envia_campo("PWM_IZQ", "PWMIZQ", "%.0f")
            self.envia_campo("PWM_DER", "PWMDER", "%.0f")
        else:
            self.enviar("TESTOFF")

    # ------------------------------------------------ calibracion pid de velocidad
    def modo_vel(self, comando, encender):
        """VPID=1/0  o  VESC=1/0.  Solo uno de los dos puede estar encendido"""
        if encender:
            otro = self.bt_esc if comando == "VPID" else self.bt_pid
            otro.set(False)
            self.graf_izq.limpia()
            self.graf_der.limpia()
        self.enviar(f"{comando}={1 if encender else 0}/")

    def identificar(self):
        """VIDENT=pwm/  rampa (zona muerta) + escalon de PWM_id, ruedas al aire"""
        self.bt_pid.set(False)
        self.bt_esc.set(False)
        self.sug.clear()
        for lado in ("IZQ", "DER"):
            self.ind[f"IDENT_{lado}"].set("Identificacion: midiendo...")
        self.ind["KP_SUG"].set("0")
        self.ind["KI_SUG"].set("0")
        self.graf_izq.limpia()
        self.graf_der.limpia()
        self.enviar(f"VIDENT={self.valor('PWM_ID'):.0f}/")

    def stop_motores(self):
        self.bt_pid.set(False)
        self.bt_esc.set(False)
        self.enviar("VSTOP/", urgente=True)

    def aplicar_sugeridos(self):
        """copia KP_SUG / KI_SUG a los cuadros KP_V / KI_V y los manda al robot"""
        if not self.sug:
            self.log("primero IDENTIFICAR (no hay valores sugeridos)")
            return
        kp = sum(v[0] for v in self.sug.values()) / len(self.sug)
        ki = sum(v[1] for v in self.sug.values()) / len(self.sug)
        self.ctrl["KP_V"].set("%.2f" % kp)
        self.ctrl["KI_V"].set("%.0f" % ki)
        self.envia_campo("KP_V", "VKP", "%.2f")
        self.envia_campo("KI_V", "VKI", "%.0f")

    def cambia_luces(self):
        """cada clic pasa al color siguiente; despues del ultimo, apaga. Manda LUZ=n/
        Sin conexion no cambia: el boton siempre muestra lo ultimo que se envio al robot"""
        siguiente = (self.luz + 1) % len(LUCES)
        if self.enviar(f"LUZ={siguiente}/"):
            self.luz = siguiente
            self._pinta_luces()

    def _pinta_luces(self):
        texto, fondo, letra = LUCES[self.luz]
        self.bt_luces.config(text=texto, bg=fondo, fg=letra, activebackground=fondo,
                             activeforeground=letra)

    def cambia_linea(self, blanco):
        """LINCO=0/ linea blanca,  LINCO=1/ linea negra  (IR_ANA_SEN.h: blanco_ 0, negro_ 1)"""
        self.enviar("LINCO=0/" if blanco else "LINCO=1/")

    def trama_dat(self):
        """arma la trama de parametros del firmware:
        DAT/kp/ki/kd/Vb/Vg/tau/kaw/Kg/errrect/velrect/largorect/color/girorect/vfreno/tfreno/8/acelr/
        el 8 (PARAM_VERSION de parametros.h) indica que velrect y vfreno van en PWM"""
        v = self.valor
        color = 0 if self.bt_linea.estado else 1      # 0 = blanca, 1 = negra
        return (f"DAT/{v('KP')}/{v('KI')}/{v('KD')}/{v('V_BAS')}/{v('V_GIR')}/{v('TAU')}/"
                f"{v('KAW')}/{v('KG')}/{v('ERROR_LIM')}/{v('V_LINEA')}/{v('TRAMO')}/{color}/"
                f"{v('GIRO_LIM')}/{v('V_FRENO')}/{v('T_FRENO')}/8/{v('ACEL_R')}/")

    def valor(self, nombre):
        try:
            return float(self.ctrl[nombre].get().replace(",", "."))
        except (KeyError, ValueError):
            return 0.0

    # ------------------------------------------------ tramas del robot
    # cada trama es una linea:  PREFIJO=v1/v2/v3/.../

    # para agregar una trama nueva: escribir su funcion y ponerla en TRAMAS
    def _pinta_tramas(self):
        self.lb_tramas.config(text=f"Tramas dañadas: {self.tramas_mal} de {self.tramas_ok + self.tramas_mal}",
                              fg=ROJO if self.tramas_mal else NEGRO)

    def _cuenta_trama(self, buena):
        if buena:
            self.tramas_ok += 1
        else:
            self.tramas_mal += 1
        self._pinta_tramas()

    def _trama_sensores(self, linea, inicio):
        """si se perdieron bytes, pueden quedar restos pegados antes de Nor= o Cru=: se usa
        desde el ultimo inicio, y solo si trae los NUM_SENSORES valores completos"""
        prefijo, datos = linea[inicio:].split("=", 1)
        try:
            valores = self.numeros(datos)
        except ValueError:
            valores = []
        completa = len(valores) == NUM_SENSORES
        if completa:
            self.TRAMAS[prefijo](self, valores)
        self._cuenta_trama(completa and inicio == 0)

    def procesa_linea(self, linea):
        linea = linea.strip()
        inicios = list(INICIO_SENSORES.finditer(linea))
        if inicios:
            self._trama_sensores(linea, inicios[-1].start())
            return
        if "*" in linea:                 # resto de un *dato* cortado: el texto del robot no lleva *
            self._cuenta_trama(False)
            return
        if "=" in linea:
            prefijo, datos = linea.split("=", 1)
            funcion = self.TRAMAS_TEXTO.get(prefijo.strip())
            if funcion:                  # tramas que se leen como texto (ej. hexadecimal)
                funcion(self, datos.strip().strip("/").strip())
                return
            funcion = self.TRAMAS.get(prefijo.strip())
            if funcion:
                try:
                    funcion(self, self.numeros(datos))
                except (ValueError, IndexError) as e:
                    self.log(f"trama {prefijo} mal formada: {linea}  ({e})")
                return
        if linea:
            self.log("< " + linea)       # lo que no es una trama conocida se muestra tal cual

    @staticmethod
    def numeros(datos):
        """'0.12/ 0.50/ 1.00/' -> [0.12, 0.5, 1.0]"""
        return [float(x.replace(",", ".")) for x in datos.split("/") if x.strip()]

    # Nor=n1/n2/.../n16/   sensores normalizados 0..1 (Manda_Normalizados)
    def trama_nor(self, v):
        for led, valor in zip(self.leds, v):
            led.set_valor(valor, UMBRAL_LED)

    # IDENTIZQ=zona_muerta/K/tau/Kp/Ki/   resultado de la identificacion (llanta.c)
    def _trama_ident(self, lado, v):
        zm, k, tau, kp, ki = v[:5]
        texto = (f"ZM={zm:.0f}   K={k:.5f} m/s/pwm   tau={tau * 1000:.0f} ms"
                 f"   ->  Kp={kp:.1f}   Ki={ki:.1f}")
        self.ind[f"IDENT_{lado}"].set(texto)
        self.log(f"IDENT {lado}: {texto}")
        self.sug[lado] = (kp, ki)
        # sugerido = promedio de las llantas identificadas (los 2 PID usan las mismas constantes)
        self.ind["KP_SUG"].set("%.1f" % (sum(s[0] for s in self.sug.values()) / len(self.sug)))
        self.ind["KI_SUG"].set("%.1f" % (sum(s[1] for s in self.sug.values()) / len(self.sug)))

    def trama_ident_izq(self, v):
        self._trama_ident("IZQ", v)

    def trama_ident_der(self, v):
        self._trama_ident("DER", v)

    # Cru=v1/.../v16/   lectura cruda del ADC (0..4095) de cada sensor (manda_crudos, en REPOSO)
    def trama_cru(self, v):
        for lb, valor in zip(self.lb_crudo, v):
            lb.config(text=f"{valor:.0f}", fg=AZUL)

    TRAMAS = {
        "Nor": trama_nor,
        "Cru": trama_cru,
        "IDENTIZQ": trama_ident_izq,
        "IDENTDER": trama_ident_der,
    }

    # codigo=B2BE0381/  o  codigo=0xB2BE0381/   codigo del control IR (lee_ir)
    def trama_codigo(self, texto):
        hexa = texto.upper()
        if hexa.startswith("0X"):
            hexa = hexa[2:]
        self.ind["IR_CODIGO"].set("0x" + hexa)

    TRAMAS_TEXTO = {
        "codigo": trama_codigo,
    }

    # ------------------------------------------------ datos *etiqueta valor*
    # (manda_pid_parametros)  etiqueta del robot -> indicador del panel
    DATOS = {
        # PID de linea (PRINCIPAL)
        "j": "KP_R", "k": "KI_R", "l": "KD_R",
        # partes del PID de linea (manda_operando)
        "p": "UP", "i": "UI", "d": "UD", "t": "UT",
        "VB": "V_BASE",                         # velocidad base en uso (carrera.c)
        "n": "TAU_R", "m": "KAW_R", "f": "KG_R",
        "vbas": "V_BAS_R", "g": "V_GIR_R",
        # rectas
        "RECTVEL": "V_LINEA_R",       # velocidad base en recta (PWM 0..1000)
        "FRENOV": "V_FRENO_R",        # velocidad base mientras frena (PWM 0..1000)
        "FRENOMS": "T_FRENO_R",       # tiempo maximo frenando (ms)
        "ACELR": "ACEL_R_R",          # rampa de V_Bas a V_LINEA (PWM/s)
        "RECTERR": "ERROR_LIM_R",     # rango +- de la posicion para considerar recta
        "RECTGIR": "GIRO_LIM_R",      # rango +- de giro_z (grados/s) para considerar recta
        "RECTLARG": "TRAMO_R",        # cm seguidos en recta para pasar a V_LINEA
        "RD": "RECTA_CM",             # cm que lleva en recta ahora (manda_operando)
        # PID de velocidad (VELO)
        "KPVEL": "KP_V_R", "KIVEL": "KI_V_R", "KDVEL": "KD_V_R",
        "TAUVEL": "TAU_V_R", "KAWVEL": "KAW_V_R",
        "SETIZ": "SET_IZQ", "SETDE": "SET_DER",
        "PIDIZQ": "OUTIZQ", "PIDDER": "OUTDER",
        "w": "VELOCIDAD_IZQ", "z": "VELOCIDAD_DER",     # velocidad medida de cada motor
        "UPVELDER": "UPVEL_DER", "UIVELDER": "UIVEL_DER", "UDVELDER": "UDVEL_DER",
        "UPVELIZQ": "UPVEL_IZQ", "UIVELIZQ": "UIVEL_IZQ", "UDVELIZQ": "UDVEL_IZQ",
    }
    # datos que tambien se muestran en una segunda casilla
    #  - el total de cada pid de velocidad (UTVEL) es la salida PIDDER / PIDIZQ
    #  - la velocidad de cada motor tambien va a VELO_IZQ / VELO_DER de PRINCIPAL
    COPIA = {"PIDDER": "UTVEL_DER", "PIDIZQ": "UTVEL_IZQ",
             "w": "VELO_IZQ", "z": "VELO_DER"}

    # datos que van a un reloj o grafico (ademas de su casilla, si tiene)
    def dato_vel_angular(self, valor):          # *q...*  MPU6050.giro_z_f (grados/s)
        self.vel_angular.set(valor)

    def dato_posicion(self, valor):             # *e...*  posicion de la linea (Calcula_Pesos)
        self.posicion.set(valor)

    def dato_hay_linea(self, valor):            # *L n*  1 = ve la linea, 0 = no
        self.posicion.set_linea(int(valor) == 1)

    def dato_bateria(self, valor):              # *b...*  voltaje de la bateria
        self.bateria.set(valor)
        self.ind["BATERIA_V"].set(f"{valor:.2f}")

    # *w...*  un punto nuevo: set point + velocidad.
    # el set point es el que reporta el robot (SETIZ, cambia solo en el escalon);
    # si el robot no lo manda, se usa el del cuadro
    def dato_vel_izq(self, valor):
        sp = self.sp_rx["IZQ"]
        self.graf_izq.agrega(self.valor("SET_POINT_I") if sp is None else sp, valor)

    def dato_vel_der(self, valor):              # *z...*
        sp = self.sp_rx["DER"]
        self.graf_der.agrega(self.valor("SET_POINT_D") if sp is None else sp, valor)

    def dato_set_izq(self, valor):              # *SETIZ...*  set point que esta usando el robot
        self.sp_rx["IZQ"] = valor

    def dato_set_der(self, valor):              # *SETDE...*
        self.sp_rx["DER"] = valor

    def dato_recta_cm(self, valor):             # *RD...*  cm seguidos en recta
        # el robot pone RD en 0 apenas el error o el giro salen de su limite (carrera.c),
        # asi que RD >= TRAMO significa que se cumplen los tres.
        # TRAMO: el que tiene el robot; si aun no lo ha reportado (0), el del cuadro
        try:
            tramo = float(self.ind["TRAMO_R"].get())
        except ValueError:
            tramo = 0.0
        tramo = tramo or self.valor("TRAMO")
        self.led_recta.set(valor > 0 and valor >= tramo)

    def dato_test(self, valor):                 # *TST n*  bandera de TEST del robot
        on = int(valor) == 1
        self.led_test.set(on)
        self.bt_test.set(on)
        self.led_test_v.set(on)                     # pestaña VELO
        prueba = self.cb_test.get() if on else ""
        self.lb_test_v.config(text=f"TEST ON\n{prueba}" if on else "TEST OFF",
                              fg=ROJO if on else NEGRO)

    def dato_test_estado(self, valor):          # *TSTE n*  prueba que tiene el robot
        i = int(valor)
        if 0 <= i < len(PRUEBAS_ROBOT) and self.cb_test.current() != i:
            self.cb_test.current(i)

    def dato_estado(self, valor):               # *EST n*  estado de la carrera (carrera.h)
        e = int(valor)
        self.ind["ESTADO"].set(ESTADOS_CARRERA[e] if 0 <= e < len(ESTADOS_CARRERA) else f"? {e}")
        # el boton sigue al robot (ej. START/STOP del control IR, linea perdida), pero
        # 1 s despues de un RUN/STOP propio se ignora: puede llegar un *EST* viejo
        # enviado antes de que el robot procesara la orden
        if e != 0:                              # en carrera no llegan crudos: quedan en gris
            for lb in self.lb_crudo:
                lb.config(fg="#808080")
        else:                                   # en REPOSO ya no llega *RD*: apaga el LED
            self.led_recta.set(False)
        reciente = time.monotonic() - self.t_run_local < 1.0
        if not reciente and (e != 0) != self.corriendo:
            self._pinta_run(e != 0)

    MODOS_VEL = {0: "OFF", 1: "PWM", 2: "PID", 3: "ESCALON", 4: "IDENT"}

    def dato_modo(self, valor):                 # *VMODO n*  modo del control de las llantas
        m = int(valor)
        self.ind["MODO_V"].set(self.MODOS_VEL.get(m, f"? {m}"))
        # los botones siguen al robot (ej. la identificacion termina sola, STOP del control IR)
        self.bt_pid.set(m == 2)
        self.bt_esc.set(m == 3)

    def dato_linea(self, valor):                # *LINCO=n*  color que sigue el robot
        texto = COLOR_LINEA_ROBOT.get(int(valor), f"? {valor:g}")
        if texto == "BLANCO":
            self.lb_linea.config(text=texto, bg="#f4f4f4", fg=AZUL)
        elif texto == "NEGRO":
            self.lb_linea.config(text=texto, bg="#202020", fg="white")
        else:
            self.lb_linea.config(text=texto, bg=CAJA, fg=ROJO)

    DATOS_FUNCION = {
        "LINCO": dato_linea,
        "q": dato_vel_angular,
        "e": dato_posicion,
        "L": dato_hay_linea,
        "RD": dato_recta_cm,
        "b": dato_bateria,
        "w": dato_vel_izq,
        "z": dato_vel_der,
        "SETIZ": dato_set_izq,
        "SETDE": dato_set_der,
        "VMODO": dato_modo,
        "TST": dato_test,
        "EST": dato_estado,
        "TSTE": dato_test_estado,
    }

    def carga_cuadro(self, nombre_r, valor):
        """parametro reportado por el robot (casilla gris KP_R) -> tambien a su cuadro blanco
        (KP), para seguir cambiando desde ese valor. No toca el cuadro que se esta escribiendo.
        Actualiza enviado: lo que trae el robot no se vuelve a mandar al salir del cuadro"""
        nombre = nombre_r[:-2] if nombre_r.endswith("_R") else ""
        if nombre not in self.editable:
            return
        sp, formato = self.editable[nombre]
        try:
            if self.focus_get() is sp:
                return
        except KeyError:                            # foco en la lista desplegable de un combobox
            pass
        try:
            v = float(valor)
        except ValueError:
            return
        if math.isfinite(v):
            texto = formato % v
            self.ctrl[nombre].set(texto)
            self.enviado[nombre] = texto

    def procesa_dato(self, etiqueta, valor):
        funcion = self.DATOS_FUNCION.get(etiqueta)
        nombre = self.DATOS.get(etiqueta)
        if funcion is None and nombre is None:
            self.log(f"< *{etiqueta}{valor}*")     # etiqueta que todavia no tiene casilla
            return
        if nombre:
            self.ind[nombre].set(valor)
            self.carga_cuadro(nombre, valor)
            if etiqueta in self.COPIA:
                self.ind[self.COPIA[etiqueta]].set(valor)
        if funcion:
            try:
                v = float(valor)
            except ValueError:
                return
            if math.isfinite(v):
                funcion(self, v)

    # ======================================================================
    #                              SERIAL
    # ======================================================================
    def actualiza_puertos(self):
        puertos = [p.device for p in serial.tools.list_ports.comports()]
        self.cb_puerto["values"] = puertos
        if puertos and self.cb_puerto.get() not in puertos:
            # COM9 por defecto si existe; si no, el primero de la lista
            self.cb_puerto.set(PUERTO_DEFECTO if PUERTO_DEFECTO in puertos else puertos[0])

    def conectar(self, encender):
        if encender:
            puerto = self.cb_puerto.get()
            try:
                self.ser = serial.Serial(puerto, BAUDIOS, timeout=0.05)
            except Exception as e:
                self.log(f"no se pudo abrir {puerto}: {e}")
                self.bt_conectar.set(False)
                return
            self.tramas_ok, self.tramas_mal = 0, 0
            self._pinta_tramas()
            self.leyendo = True
            threading.Thread(target=self._lee_serial, daemon=True).start()
            self.log(f"conectado a {puerto} ({BAUDIOS})")
        else:
            self.leyendo = False
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None
            self.led_recta.set(False)
            self.log("desconectado")

    def enviar(self, texto, urgente=False):
        """pone el comando en la cola de envio. devuelve False si no hay conexion.
        urgente=True (STOP): borra lo pendiente y lo manda YA, sin esperar turno"""
        if not self.ser:
            self.log("sin conexion: " + texto)
            return False
        if urgente:
            self.tx_cola.clear()
            try:
                self.ser.write(texto.encode())
                self.log("> " + texto + "   (URGENTE)")
                return True
            except Exception as e:
                self.log(f"error al enviar: {e}")
                return False
        self.tx_cola.append(texto)
        return True

    def _tx_ciclo(self):
        """manda UN comando cada TX_ESPACIO_MS: el robot recibe por IDLE y dos
        comandos seguidos llegarian pegados en una sola trama"""
        if self.tx_cola and self.ser:
            texto = self.tx_cola.popleft()
            try:
                self.ser.write(texto.encode())
                self.log("> " + texto)
            except Exception as e:
                self.log(f"error al enviar: {e}")
        elif not self.ser:
            self.tx_cola.clear()
        self.after(TX_ESPACIO_MS, self._tx_ciclo)

    def _lee_serial(self):
        buf = ""
        while self.leyendo and self.ser:
            try:
                datos = self.ser.read(256)
            except Exception as e:
                self.cola.put(("error", str(e)))
                break
            if datos:
                eventos, buf = separa(buf + datos.decode(errors="replace"))
                for ev in eventos:
                    self.cola.put(ev)

    def _ciclo(self):
        for _ in range(200):
            try:
                tipo, dato = self.cola.get_nowait()
            except queue.Empty:
                break
            if tipo == "linea":
                self.procesa_linea(dato)
            elif tipo == "dato":
                self.procesa_dato(*dato)
            else:
                self.log("error serial: " + dato)
                self.bt_conectar.set(False)
                self.conectar(False)
        self.graf_izq.redibuja()
        self.graf_der.redibuja()
        self.after(30, self._ciclo)

    def log(self, texto):
        self.trama.insert("end", texto + "\n")
        lineas = int(self.trama.index("end-1c").split(".")[0])
        if lineas > 300:
            self.trama.delete("1.0", f"{lineas - 300}.0")
        self.trama.see("end")

    def salir(self):
        self.conectar(False)
        self.destroy()

    # ------------------------------------------------ demo
    def _demo(self):
        t = self._t_demo = self._t_demo + 0.05
        pos = 6 * math.sin(t * 0.7)
        self.posicion.set(pos)
        centro = (pos + POS_MAX) / (2 * POS_MAX) * (NUM_SENSORES - 1)
        for i, led in enumerate(self.leds):
            led.set(abs(i - centro) < 1.2)
        self.bateria.set(7.4 + 0.3 * math.sin(t * 0.2))
        self.vel_angular.set(250 * math.sin(t * 0.5))
        sp_i, sp_d = self.valor("SET_POINT_I"), self.valor("SET_POINT_D")
        vi = sp_i + 40 * math.sin(t * 3) + random.uniform(-15, 15)
        vd = sp_d + 40 * math.cos(t * 3) + random.uniform(-15, 15)
        self.graf_izq.agrega(sp_i, vi)
        self.graf_der.agrega(sp_d, vd)
        self.ind["VELOCIDAD_IZQ"].set(f"{vi:.0f}")
        self.ind["VELOCIDAD_DER"].set(f"{vd:.0f}")
        self.ind["UT"].set(f"{pos * 20:.1f}")
        self.after(50, self._demo)


if __name__ == "__main__":
    try:    # nitidez en pantallas con escala de Windows
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    Panel(demo="--demo" in sys.argv).mainloop()
