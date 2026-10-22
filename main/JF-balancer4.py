import serial
import time
import numpy as np
import tkinter as tk
from tkinter import ttk
import threading
import queue

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,
                                               NavigationToolbar2Tk)
from matplotlib.figure import Figure

N_MESURES = 5  
COULEURS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#9467bd", "#8c564b"]

class AcquisitionReelle:
    def __init__(self, port="COM3", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baudrate
        self.ser.timeout = 1.0
        self.ser.setDTR(False)
        self.ser.setRTS(False)
        self.ser.open()
        time.sleep(1.5)

    def lancer_cycle_async(self, data_queue):
        try:
            self.ser.reset_input_buffer()
            self.ser.write(b"m")
            current_accel_id = None
            current_lockin_id = None
            t0_mesure = None
            
            while True:
                ligne = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not ligne:
                    continue
                if ligne.startswith("###sample="):
                    continue
                if ligne.startswith("###accel-"):
                    try:
                        current_accel_id = int(ligne.replace("###accel-", "")) - 1
                        current_lockin_id = None
                        t0_mesure = None
                        continue
                    except ValueError:
                        pass
                elif ligne.startswith("###lockin"):
                    try:
                        current_lockin_id = int(ligne.replace("###lockin", "")) - 1
                        current_accel_id = None
                        continue
                    except ValueError:
                        pass
                elif ligne == "###":
                    if current_lockin_id == (N_MESURES - 1):
                        data_queue.put({"type": "FIN"})
                        break
                    current_accel_id = None
                    continue
                
                if current_accel_id is not None and 0 <= current_accel_id < N_MESURES and ";" in ligne:
                    if "t;Y" in ligne:
                        continue
                    try:
                        elements = ligne.split(";")
                        t_us = float(elements[0])
                        y_val = float(elements[1])
                        t_s = t_us / 1e6
                        if t0_mesure is None:
                            t0_mesure = t_s
                        data_queue.put({"type": "POINT", "index": current_accel_id, "t": t_s - t0_mesure, "y": y_val})
                    except (ValueError, IndexError):
                        continue
                elif current_lockin_id is not None and 0 <= current_lockin_id < N_MESURES and ";" in ligne:
                    try:
                        elements = ligne.split(";")
                        if len(elements) >= 5:
                            f_ref = float(elements[0]) if elements[0].lower() != "nan" else 0.0
                            amp = float(elements[1]) if elements[1].lower() != "nan" else 0.0
                            c_amp = float(elements[2]) if elements[1].lower() != "nan" else 0.0
                            phase = float(elements[3]) if elements[1].lower() != "nan" else 0.0
                            angle = float(elements[-1]) if elements[-1].lower() != "nan" else 0.0
                            data_queue.put({"type": "LOCKIN", "index": current_lockin_id, "rpm": f_ref * 60.0, "amp": amp, "c_amp": c_amp, "phase" : phase, "angle": angle})
                        current_lockin_id = None
                    except (IndexError, ValueError):
                        pass
        except Exception as e:
            data_queue.put({"type": "ERREUR", "msg": str(e)})

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Équilibreuse Dynamique - 5 Mesures Temps Réel")
        self.geometry("1300x850")
        self.source = AcquisitionReelle(port="COM3", baudrate=115200)
        self.data_queue = queue.Queue()
        self.lignes_plot = []
        self.donnees_accumulees = {i: {"t": [], "y": [], "rpm": 0.0, "amp": 0.0, "c_amp": 0.0, "phase": 0.0, "angle" : 0.0} for i in range(N_MESURES)}
        self.vector_sum_list = []

        self.main_container = ttk.Frame(self)
        self.main_container.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(self.main_container, highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(self.main_container, orient="vertical", command=self.canvas.yview)
        self.scrollable_frame = ttk.Frame(self.canvas)
        self.scrollable_frame.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")
        self.bind_all("<MouseWheel>", lambda e: self.canvas.yview_scroll(int(-1 * (e.delta / 120)), "units"))

        self._barre()
        self._figure()
        self._tableau()

    def _barre(self):
        barre = ttk.Frame(self.scrollable_frame, padding=8)
        barre.pack(fill="x")
        self.btn = ttk.Button(barre, text=f"Lancer le cycle", command=self.lancer)
        self.btn.pack(side="left")
        self.var_etat = tk.StringVar(value="Prêt. Connecté à la carte.")
        ttk.Label(barre, textvariable=self.var_etat).pack(side="left", padx=12)

    def _figure(self):
        self.fig = Figure(figsize=(12, 7), tight_layout=True)
        gs = self.fig.add_gridspec(N_MESURES, 2, width_ratios=[2, 1])
        self.axes = []
        for i in range(N_MESURES):
            ax = self.fig.add_subplot(gs[i, 0], sharex=self.axes[0] if self.axes else None)
            ax.set_ylabel(f"#{i + 1}", color=COULEURS[i])
            ligne, = ax.plot([], [], lw=0.8, color=COULEURS[i])
            self.lignes_plot.append(ligne)
            self.axes.append(ax)
        self.axes[-1].set_xlabel("Temps (s)")
        self.ax_pol = self.fig.add_subplot(gs[:, 1], projection="polar")
        self.ax_pol.set_theta_zero_location("N")
        self.ax_pol.set_theta_direction(-1)
        self.graph_canvas = FigureCanvasTkAgg(self.fig, master=self.scrollable_frame)
        NavigationToolbar2Tk(self.graph_canvas, self.scrollable_frame)
        self.graph_canvas.get_tk_widget().pack(fill="both", expand=True)

    def _tableau(self):
        cadre_tab = ttk.Frame(self.scrollable_frame, padding=8)
        cadre_tab.pack(fill="x", padx=8, pady=8)
        cols = ("n", "rpm", "amp", "c_amp", "phase", "angle")
        titres = ("Mesure", "Vitesse (tr/min)", "Amplitude", "C_Amplitude", "Phase", "Angle (°)")
        self.tab = ttk.Treeview(cadre_tab, columns=cols, show="headings", height=6)
        for c, ti in zip(cols, titres):
            self.tab.heading(c, text=ti)
            self.tab.column(c, anchor="center", width=150)
        tab_scroll = ttk.Scrollbar(cadre_tab, orient="vertical", command=self.tab.yview)
        self.tab.configure(yscrollcommand=tab_scroll.set)
        self.tab.pack(side="left", fill="x", expand=True)
        tab_scroll.pack(side="right", fill="y")

    def lancer(self):
        self.btn.config(state="disabled")
        self.tab.delete(*self.tab.get_children())
        self.ax_pol.clear()
        self.ax_pol.set_theta_zero_location("N")
        self.ax_pol.set_theta_direction(-1)
        self.donnees_accumulees = {i: {"t": [], "y": [], "rpm": 0.0, "amp": 0.0, "c_amp": 0.0, "phase": 0.0} for i in range(N_MESURES)}
        self.vector_sum_list = []
        for i in range(N_MESURES):
            self.lignes_plot[i].set_data([], [])
            self.axes[i].relim()
            self.axes[i].autoscale_view()
        self.graph_canvas.draw_idle()
        self.var_etat.set("Acquisition en cours...")
        threading.Thread(target=self.source.lancer_cycle_async, args=(self.data_queue,), daemon=True).start()
        self.after(20, self.traiter_queue)

    def traiter_queue(self):
        points_recus = False
        while not self.data_queue.empty():
            try: msg = self.data_queue.get_nowait()
            except queue.Empty: break
            if msg["type"] == "POINT":
                idx = msg["index"]
                if idx is not None and 0 <= idx < N_MESURES:
                    self.donnees_accumulees[idx]["t"].append(msg["t"])
                    self.donnees_accumulees[idx]["y"].append(msg["y"])
                    points_recus = True
            elif msg["type"] == "LOCKIN":
                idx = msg["index"]
                if idx is not None and 0 <= idx < N_MESURES:
                    self.donnees_accumulees[idx]["rpm"] = msg["rpm"]
                    self.donnees_accumulees[idx]["amp"] = msg["amp"]
                    self.donnees_accumulees[idx]["c_amp"] = msg["c_amp"]
                    self.donnees_accumulees[idx]["phase"] = msg["phase"]
                    self.donnees_accumulees[idx]["angle"] = msg["angle"]
                    self.tab.insert("", "end", values=(idx + 1, f"{msg['rpm']:.1f}", f"{msg['amp']:.4f}", f"{msg['c_amp']:.4f}", f"{msg['phase']:.1f}", f"{msg['angle']:.1f}"))
                    if msg["c_amp"] > 0:
                        self.ax_pol.plot(np.radians(msg["angle"]), msg["c_amp"], "o", ms=9, color=COULEURS[idx])
                        self.vector_sum_list.append(msg["c_amp"] * np.exp(1j * np.radians(msg["angle"])))
                    self.graph_canvas.draw_idle()
            elif msg["type"] == "FIN":
                if self.vector_sum_list:
                    z_moy = np.mean(self.vector_sum_list)
                    self.ax_pol.annotate("", xy=(np.angle(z_moy), abs(z_moy)), xytext=(0, 0), arrowprops=dict(arrowstyle="->", lw=2.5, color="black"))
                    max_amp = max([self.donnees_accumulees[i]["c_amp"] for i in range(N_MESURES)])
                    #self.ax_pol.set_rlim(0,(max_amp if max_amp > 0 else 1.0) * 1.3)
                    self.var_etat.set(f"Moyenne : {abs(z_moy):.4f} à {np.degrees(np.angle(z_moy)) % 360:.1f}°")
                else: self.var_etat.set("Cycle terminé.")
                self.graph_canvas.draw_idle()
                self.btn.config(state="normal")
                return
            elif msg["type"] == "ERREUR":
                self.var_etat.set(f"Erreur : {msg['msg']}")
                self.btn.config(state="normal")
                return
        if points_recus:
            for i in range(N_MESURES):
                t_data = self.donnees_accumulees[i]["t"]
                y_data = self.donnees_accumulees[i]["y"]
                if t_data:
                    self.lignes_plot[i].set_data(t_data, y_data)
                    self.axes[i].relim()
                    self.axes[i].autoscale_view(scalex=True, scaley=True)
            self.graph_canvas.draw_idle()
        self.after(20, self.traiter_queue)

if __name__ == "__main__":
    App().mainloop()