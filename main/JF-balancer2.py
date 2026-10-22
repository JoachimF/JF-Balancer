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

# CONFIGURATION À 5 MESURES
N_MESURES = 5  
COULEURS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#9467bd", "#8c564b"]

# --------------------------------------------------------------------------
# Gestion du Port Série Windows (Exécuté dans un Thread séparé)
# --------------------------------------------------------------------------
class AcquisitionReelle:
    def __init__(self, port="COM3", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        print(f"Connexion au port {port}...")
        
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baudrate
        self.ser.timeout = 1.0  # Réactivité accrue pour le temps réel
        
        # Empêche l'ESP32 de rebooter à l'ouverture sous Windows
        self.ser.setDTR(False)
        self.ser.setRTS(False)
        self.ser.open()
        
        time.sleep(1.5)
        print("Ligne série Windows prête.")

    def lancer_cycle_async(self, data_queue):
        """Envoie 'm' et lit le flux en tâche de fond pour alimenter la queue."""
        try:
            self.ser.reset_input_buffer()
            print("Envoi de la commande 'm' pour lancer le cycle...")
            self.ser.write(b'm')
            
            current_id = None
            t0_mesure = None  # Synchronisation du temps relatif pour chaque bloc
            
            while True:
                ligne = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not ligne:
                    continue
                    
                # Détection des entêtes de blocs de données de courbe (ex: ###0 à ###4)
                if ligne.startswith("###") and not ligne.endswith("###") and "=" not in ligne:
                    try:
                        current_id = int(ligne.replace("###", ""))
                        t0_mesure = None  # Réinitialisation du chrono pour la nouvelle courbe
                        continue
                    except ValueError:
                        pass
                
                # Fin de la capture globale (### après la dernière mesure ###4)
                if ligne == "###" and current_id == (N_MESURES - 1):
                    print("Fin de transmission détectée.")
                    data_queue.put({"type": "FIN"})
                    break
                    
                # Extraction des données calculées par le Lock-in de la carte
                if "Laser count" in ligne:
                    try:
                        parts = ligne.split('|')
                        f_ref = float(parts[1].split('=')[1].replace('Hz', '').strip())
                        amp = float(parts[2].split('=')[1].strip())
                        angle = float(parts[4].split(':')[1].strip())
                        
                        # Envoi des données calculées à l'IHM
                        data_queue.put({
                            "type": "LOCKIN",
                            "index": current_id,
                            "rpm": f_ref * 60.0,
                            "amp": amp,
                            "phase": angle
                        })
                    except (IndexError, ValueError) as e:
                        print("Erreur de parsing de la ligne Lock-in:", e)
                
                # Extraction des points de courbe brute (format: temps_us;valeur_Y)
                if current_id is not None and 0 <= current_id < N_MESURES and ';' in ligne:
                    if 't;Y;t' in ligne:
                        continue  # Ignore l'entête
                    try:
                        elements = ligne.split(';')
                        t_us = float(elements[0])
                        y_val = float(elements[1])
                        t_s = t_us / 1e6
                        
                        if t0_mesure is None:
                            t0_mesure = t_s
                            
                        # Envoi immédiat du point à l'IHM
                        data_queue.put({
                            "type": "POINT",
                            "index": current_id,
                            "t": t_s - t0_mesure,
                            "y": y_val
                        })
                    except (ValueError, IndexError):
                        continue
                        
        except Exception as e:
            data_queue.put({"type": "ERREUR", "msg": str(e)})


# --------------------------------------------------------------------------
# Interface Graphique
# --------------------------------------------------------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Équilibreuse Dynamique - 5 Mesures Temps Réel")
        self.source = AcquisitionReelle(port="COM3", baudrate=115200)
        self.data_queue = queue.Queue()
        
        # Structures pour stocker les lignes de tracé et les données accumulées
        self.lignes_plot = []
        self.donnees_accumulees = {i: {"t": [], "y": [], "rpm": 0.0, "amp": 0.0, "phase": 0.0} for i in range(N_MESURES)}
        self.vector_sum_list = []

        self._barre()
        self._figure()
        self._tableau()

    def _barre(self):
        barre = ttk.Frame(self, padding=8)
        barre.pack(fill="x")
        self.btn = ttk.Button(barre, text=f"Lancer le cycle de {N_MESURES} mesures", command=self.lancer)
        self.btn.pack(side="left")
        self.var_etat = tk.StringVar(value="Prêt. Connecté à la carte.")
        ttk.Label(barre, textvariable=self.var_etat).pack(side="left", padx=12)

    def _figure(self):
        self.fig = Figure(figsize=(12, 7), tight_layout=True)
        gs = self.fig.add_gridspec(N_MESURES, 2, width_ratios=[2, 1])
        self.axes = []
        self.lignes_plot = []
        
        for i in range(N_MESURES):
            ax = self.fig.add_subplot(gs[i, 0], sharex=self.axes[0] if self.axes else None)
            ax.set_ylabel(f"#{i + 1}", color=COULEURS[i])
            # Initialisation d'une ligne vide mise à jour dynamiquement
            ligne, = ax.plot([], [], lw=0.8, color=COULEURS[i])
            self.lignes_plot.append(ligne)
            self.axes.append(ax)
            
        self.axes[-1].set_xlabel("Temps (s)")
        self.ax_pol = self.fig.add_subplot(gs[:, 1], projection="polar")
        self._init_polaire()

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        NavigationToolbar2Tk(self.canvas, self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def _init_polaire(self):
        self.ax_pol.clear()
        self.ax_pol.set_theta_zero_location("N")
        self.ax_pol.set_theta_direction(-1)  # Sens horaire
        self.ax_pol.set_title("Vecteurs du Balourd (Amplitude, Angle)")

    def _tableau(self):
        cols = ("n", "rpm", "amp", "phase")
        titres = ("Mesure", "Vitesse (tr/min)", "Amplitude", "Angle (°)")
        self.tab = ttk.Treeview(self, columns=cols, show="headings", height=7)
        for c, ti in zip(cols, titres):
            self.tab.heading(c, text=ti)
            self.tab.column(c, anchor="center", width=150)
        self.tab.pack(fill="x", padx=8, pady=8)

    def lancer(self):
        self.btn.config(state="disabled")
        self.tab.delete(*self.tab.get_children())
        self._init_polaire()
        
        # Réinitialisation complète des structures de données
        self.donnees_accumulees = {i: {"t": [], "y": [], "rpm": 0.0, "amp": 0.0, "phase": 0.0} for i in range(N_MESURES)}
        self.vector_sum_list = []
        for i in range(N_MESURES):
            self.lignes_plot[i].set_data([], [])
            self.axes[i].relim()
            self.axes[i].autoscale_view()
            
        self.canvas.draw_idle()
        self.var_etat.set("Acquisition en temps réel en cours...")
        
        # Vidage des résidus de la queue
        while not self.data_queue.empty():
            try:
                self.data_queue.get_nowait()
            except queue.Empty:
                break
            
        # Lancement de l'acquisition en arrière-plan
        threading.Thread(target=self.source.lancer_cycle_async, args=(self.data_queue,), daemon=True).start()
        
        # Lancement du rafraîchissement synchrone avec l'IHM
        self.after(20, self.traiter_queue)

    def traiter_queue(self):
        """Vérifie la file d'attente et met à jour l'IHM de façon fluide."""
        points_recus = False
        
        while not self.data_queue.empty():
            try:
                msg = self.data_queue.get_nowait()
            except queue.Empty:
                break
                
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
                    self.donnees_accumulees[idx]["phase"] = msg["phase"]
                    
                    # Remplissage dynamique du tableau de synthèse
                    self.tab.insert("", "end", values=(
                        idx + 1, f"{msg['rpm']:.1f}", f"{msg['amp']:.4f}", f"{msg['phase']:.1f}"
                    ))
                    
                    # Mise à jour du graphique polaire
                    self.ax_pol.plot(np.radians(msg["phase"]), msg["amp"], "o", ms=9, color=COULEURS[idx])
                    self.vector_sum_list.append(msg["amp"] * np.exp(1j * np.radians(msg["phase"])))
                    self.canvas.draw_idle()
                    
            elif msg["type"] == "FIN":
                # Calcul de la moyenne vectorielle globale en fin de cycle
                if self.vector_sum_list:
                    z_moy = np.mean(self.vector_sum_list)
                    self.ax_pol.annotate("", xy=(np.angle(z_moy), abs(z_moy)), xytext=(0, 0),
                                         arrowprops=dict(arrowstyle="->", lw=2.5, color="black"),
                                         xycoords='data', textcoords='data')
                    
                    max_amp = max([self.donnees_accumulees[i]["amp"] for i in range(N_MESURES)])
                    self.ax_pol.set_rlim(0, (max_amp if max_amp > 0 else 1.0) * 1.3)
                    self.var_etat.set(f"Moyenne : {abs(z_moy):.4f} à {np.degrees(np.angle(z_moy)) % 360:.1f}°")
                else:
                    self.var_etat.set("Cycle terminé.")
                    
                self.canvas.draw_idle()
                self.btn.config(state="normal")
                return
                
            elif msg["type"] == "ERREUR":
                self.var_etat.set(f"Erreur : {msg['msg']}")
                self.btn.config(state="normal")
                return

        # Rafraîchissement global des graphiques temporels si de nouveaux points sont arrivés
        if points_recus:
            for i in range(N_MESURES):
                t_data = self.donnees_accumulees[i]["t"]
                y_data = self.donnees_accumulees[i]["y"]
                if t_data:
                    self.lignes_plot[i].set_data(t_data, y_data)
                    self.axes[i].relim()
                    self.axes[i].autoscale_view(scalex=True, scaley=True)
            self.canvas.draw_idle()

        # Rappel récursif pour maintenir la fluidité d'affichage
        self.after(20, self.traiter_queue)


if __name__ == "__main__":
    App().mainloop()
