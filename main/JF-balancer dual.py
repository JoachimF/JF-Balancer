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
            current_accel1_id = None
            current_accel2_id = None
            current_lockin1_id = None
            current_lockin2_id = None
            t0_mesure = None
            t0_mesure2 = None
            
            
            while True:
                ligne = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not ligne:
                    continue
                if ligne.startswith("###sample="):
                    continue
                if ligne.startswith("###ACCEL1-"):
                    try:
                        current_accel1_id = int(ligne.replace("###ACCEL1-", "")) - 1
                        current_lockin1_id = None
                        t0_mesure = None
                        continue
                    except ValueError:
                        pass
                if ligne.startswith("###ACCEL2-"):
                    try:
                        current_accel2_id = int(ligne.replace("###ACCEL2-", "")) - 1
                        current_lockin2_id = None
                        t0_mesure2 = None
                        continue
                    except ValueError:
                        pass
                elif ligne.startswith("###LOCKIN1-"):
                    try:
                        current_lockin1_id = int(ligne.replace("###LOCKIN1-", "")) - 1
                        current_accel1_id = None
                        continue
                    except ValueError:
                        pass
                elif ligne.startswith("###LOCKIN2-"):
                    try:
                        current_lockin2_id = int(ligne.replace("###LOCKIN2-", "")) - 1
                        current_accel2_id = None
                        continue
                    except ValueError:
                        pass
                elif ligne == "###":
                    if current_lockin2_id == (N_MESURES - 1):
                        data_queue.put({"type": "FIN"})
                        break
                    current_accel1_id = None
                    current_accel2_id = None
                    current_lockin1_id = None
                    current_lockin2_id = None
                    continue
                
                if current_accel1_id is not None and 0 <= current_accel1_id < N_MESURES and ";" in ligne:
                    if "t;Y" in ligne:
                        continue
                    try:
                        elements = ligne.split(";")
                        t_us = float(elements[0])
                        y_val = float(elements[1])
                        t_s = t_us / 1e6
                        if t0_mesure is None:
                            t0_mesure = t_s
                        data_queue.put({"type": "POINT1", "index1": current_accel1_id, "t1": t_s - t0_mesure, "y1": y_val})
                    except (ValueError, IndexError):
                        continue
                if current_accel2_id is not None and 0 <= current_accel2_id < N_MESURES and ";" in ligne:
                    if "t;Y" in ligne:
                        continue
                    try:
                        elements = ligne.split(";")
                        t_us = float(elements[0])
                        y_val = float(elements[1])
                        t_s = t_us / 1e6
                        if t0_mesure2 is None:
                            t0_mesure2 = t_s
                        data_queue.put({"type": "POINT2", "index2": current_accel2_id, "t2": t_s - t0_mesure2, "y2": y_val})
                    except (ValueError, IndexError):
                        continue
                elif current_lockin1_id is not None and 0 <= current_lockin1_id < N_MESURES and ";" in ligne:
                    try:
                        elements = ligne.split(";")
                        if len(elements) >= 5:
                            f_ref = float(elements[0]) if elements[0].lower() != "nan" else 0.0
                            amp = float(elements[1]) if elements[1].lower() != "nan" else 0.0
                            c_amp = float(elements[2]) if elements[1].lower() != "nan" else 0.0
                            phase = float(elements[3]) if elements[1].lower() != "nan" else 0.0
                            angle = float(elements[-1]) if elements[-1].lower() != "nan" else 0.0
                            data_queue.put({"type": "LOCKIN1", "index1": current_lockin1_id, "rpm1": f_ref * 60.0, "amp1": amp, "c_amp1": c_amp, "phase1" : phase, "angle1": angle})
                        current_lockin1_id = None
                    except (IndexError, ValueError):
                        pass
                elif current_lockin2_id is not None and 0 <= current_lockin2_id < N_MESURES and ";" in ligne:
                    try:
                        elements = ligne.split(";")
                        if len(elements) >= 5:
                            f_ref = float(elements[0]) if elements[0].lower() != "nan" else 0.0
                            amp = float(elements[1]) if elements[1].lower() != "nan" else 0.0
                            c_amp = float(elements[2]) if elements[1].lower() != "nan" else 0.0
                            phase = float(elements[3]) if elements[1].lower() != "nan" else 0.0
                            angle = float(elements[-1]) if elements[-1].lower() != "nan" else 0.0
                            data_queue.put({"type": "LOCKIN2", "index2": current_lockin2_id, "rpm2": f_ref * 60.0, "amp2": amp, "c_amp2": c_amp, "phase2" : phase, "angle2": angle})
                        current_lockin2_id = None
                    except (IndexError, ValueError):
                        pass
        except Exception as e:
            data_queue.put({"type": "ERREUR", "msg": str(e)})

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Équilibreuse Dynamique - 5 Mesures Temps Réel")
        self.geometry("1800x1050")
        self.source = AcquisitionReelle(port="COM3", baudrate=115200)
        self.data_queue = queue.Queue()
        self.lignes_plot = []
        self.lignes_plot2 = []
        self.donnees_accumulees = {i: {"t1": [], "y1": [], "rpm1": 0.0, "amp1": 0.0, "c_amp1": 0.0, "phase1": 0.0, "angle1" : 0.0} for i in range(N_MESURES)}
        self.donnees_accumulees2 = {j: {"t2": [], "y2": [], "rpm2": 0.0, "amp2": 0.0, "c_amp2": 0.0, "phase2": 0.0, "angle2" : 0.0} for j in range(N_MESURES)}
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
        self.btn = ttk.Button(barre, text=f"Lancer les mesures", command=self.lancer)
        self.btn.pack(side="left")
        self.var_etat = tk.StringVar(value="Prêt. Connecté à la carte.")
        ttk.Label(barre, textvariable=self.var_etat).pack(side="left", padx=12)

    def _figure(self):
        self.fig = Figure(figsize=(1750, 1000,'px'), tight_layout=True)
        gs = self.fig.add_gridspec(N_MESURES+1, 3, width_ratios=[1,1, 1])

        self.axes = []
        self.axes2 = []

        for i in range(N_MESURES):
            ax = self.fig.add_subplot(gs[i, 0], sharex=self.axes[0] if self.axes else None)
            ax.set_ylabel(f"#{i + 1}", color=COULEURS[i])
            ligne, = ax.plot([], [], lw=0.8, color=COULEURS[i])
            self.lignes_plot.append(ligne)
            self.axes.append(ax)
        
        for i in range(N_MESURES):
            ax2 = self.fig.add_subplot(gs[i, 1], sharex=self.axes2[0] if self.axes2 else None)
            ax2.set_ylabel(f"#{i + 1}", color=COULEURS[i])
            ligne2, = ax2.plot([], [], lw=0.8, color=COULEURS[i])
            self.lignes_plot2.append(ligne2)
            self.axes2.append(ax2)
        
        self.axes[0].set_title("Avant")
        self.axes2[0].set_title("Arrière")
        self.axes[-1].set_xlabel("Temps (s)")

        self.ax_pol = self.fig.add_subplot(gs[:3, 2], projection="polar")
        self.ax_pol.set_theta_zero_location("N")
        self.ax_pol.set_theta_direction(-1)
        self.ax_pol.set_ylim(0, 2)
        self.ax_pol.set_title("Avant")

        self.ax_pol2 = self.fig.add_subplot(gs[3:, 2], projection="polar")
        self.ax_pol2.set_theta_zero_location("N")
        self.ax_pol2.set_theta_direction(-1)
        self.ax_pol2.set_ylim(0, 2)
        self.ax_pol2.set_title("Arrière")

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
        self.ax_pol.set_ylim(0, 2)

        self.ax_pol2.clear()
        self.ax_pol2.set_theta_zero_location("N")
        self.ax_pol2.set_theta_direction(-1)
        self.ax_pol2.set_ylim(0, 2)

        self.donnees_accumulees = {i: {"t1": [], "y1": [], "rpm1": 0.0, "amp1": 0.0, "c_amp1": 0.0, "phase1": 0.0} for i in range(N_MESURES)}
        self.donnees_accumulees2 = {j: {"t2": [], "y2": [], "rpm2": 0.0, "amp2": 0.0, "c_amp2": 0.0, "phase2": 0.0} for j in range(N_MESURES)}
        self.vector_sum_list = []
        self.vector_sum_list2 = []

        for i in range(N_MESURES):
            self.lignes_plot[i].set_data([], [])
            self.lignes_plot2[i].set_data([], [])
            self.axes[i].relim()
            #self.axes[i].autoscale_view()
            self.axes1[j].set_ylim(-1, 1)
            self.axes2[i].relim()
            #self.axes2[i].autoscale_view()
            self.axes2[j].set_ylim(-1, 1)

        self.graph_canvas.draw_idle()
        self.var_etat.set("Attente de la mesure 1")
        threading.Thread(target=self.source.lancer_cycle_async, args=(self.data_queue,), daemon=True).start()
        self.after(20, self.traiter_queue)

    def traiter_queue(self):
        points_recus = False
        points_recus2 = False
        while not self.data_queue.empty():
            try: msg = self.data_queue.get_nowait()
            except queue.Empty: break
            if msg["type"] == "POINT1":
                idx = msg["index1"]
                if idx is not None and 0 <= idx < N_MESURES:
                    self.donnees_accumulees[idx]["t1"].append(msg["t1"])
                    self.donnees_accumulees[idx]["y1"].append(msg["y1"])
                    points_recus = True

            if msg["type"] == "POINT2":
                idx = msg["index2"]
                if idx is not None and 0 <= idx < N_MESURES:
                    self.donnees_accumulees2[idx]["t2"].append(msg["t2"])
                    self.donnees_accumulees2[idx]["y2"].append(msg["y2"])
                    points_recus2 = True

            elif msg["type"] == "LOCKIN1":
                idx = msg["index1"]
                if idx is not None and 0 <= idx < N_MESURES:
                    self.donnees_accumulees[idx]["rpm1"] = msg["rpm1"]
                    self.donnees_accumulees[idx]["amp1"] = msg["amp1"]
                    self.donnees_accumulees[idx]["c_amp1"] = msg["c_amp1"]
                    self.donnees_accumulees[idx]["phase1"] = msg["phase1"]
                    self.donnees_accumulees[idx]["angle1"] = msg["angle1"]
                    self.tab.insert("", "end", values=(idx + 1, f"{msg['rpm1']:.1f}", f"{msg['amp1']:.4f}", f"{msg['c_amp1']:.4f}", f"{msg['phase1']:.1f}", f"{msg['angle1']:.1f}"))
                    if msg["c_amp1"] > 0:
                        self.ax_pol.plot(np.radians(msg["angle1"]), msg["c_amp1"], "o", ms=9, color=COULEURS[idx])
                        self.vector_sum_list.append(msg["c_amp1"] * np.exp(1j * np.radians(msg["angle1"])))
                    self.var_etat.set(f"Attente de la mesure {idx + 1}")
                    self.graph_canvas.draw_idle()

            elif msg["type"] == "LOCKIN2":
                idx = msg["index2"]
                if idx is not None and 0 <= idx < N_MESURES:
                    self.donnees_accumulees2[idx]["rpm2"] = msg["rpm2"]
                    self.donnees_accumulees2[idx]["amp2"] = msg["amp2"]
                    self.donnees_accumulees2[idx]["c_amp2"] = msg["c_amp2"]
                    self.donnees_accumulees2[idx]["phase2"] = msg["phase2"]
                    self.donnees_accumulees2[idx]["angle2"] = msg["angle2"]
                    self.tab.insert("", "end", values=(idx + 1, f"{msg['rpm2']:.1f}", f"{msg['amp2']:.4f}", f"{msg['c_amp2']:.4f}", f"{msg['phase2']:.1f}", f"{msg['angle2']:.1f}"))
                    if msg["c_amp2"] > 0:
                        self.ax_pol2.plot(np.radians(msg["angle2"]), msg["c_amp2"], "o", ms=9, color=COULEURS[idx])
                        self.vector_sum_list2.append(msg["c_amp2"] * np.exp(1j * np.radians(msg["angle2"])))
                    self.graph_canvas.draw_idle()
            
            elif msg["type"] == "FIN":
                if self.vector_sum_list:
                    z_moy = np.mean(self.vector_sum_list)
                    self.ax_pol.annotate("", xy=(np.angle(z_moy), abs(z_moy)), xytext=(0, 0), arrowprops=dict(arrowstyle="->", lw=2.5, color="black"))
                    max_amp = max([self.donnees_accumulees[i]["c_amp1"] for i in range(N_MESURES)])
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
                t_data = self.donnees_accumulees[i]["t1"]
                y_data = self.donnees_accumulees[i]["y1"]
                if t_data:
                    self.lignes_plot[i].set_data(t_data, y_data)
                    #self.axes[i].relim()
                    #self.axes[i].autoscale_view(scalex=True, scaley=True)
            self.graph_canvas.draw_idle()
        if points_recus2:
            for j in range(N_MESURES):
                t_data = self.donnees_accumulees2[j]["t2"]
                y_data = self.donnees_accumulees2[j]["y2"]
                if t_data:
                    self.lignes_plot2[j].set_data(t_data, y_data)
                    #self.axes2[j].relim()
                    #self.axes2[j].autoscale_view(scalex=True, scaley=True)
            self.graph_canvas.draw_idle()

        self.after(20, self.traiter_queue)

if __name__ == "__main__":
    App().mainloop()