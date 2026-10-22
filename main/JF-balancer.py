import serial
import time
import numpy as np
import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,
                                               NavigationToolbar2Tk)
from matplotlib.figure import Figure

# CONFIGURATION À 5 MESURES
N_MESURES = 5  
COULEURS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#9467bd", "#8c564b"]

# --------------------------------------------------------------------------
# Gestion du Port Série Windows
# --------------------------------------------------------------------------
class AcquisitionReelle:
    def __init__(self, port="COM3", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        print(f"Connexion au port {port}...")
        
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baudrate
        self.ser.timeout = 15.0  # Laisse le temps au moteur de changer de régime
        
        # Empêche l'ESP32 de rebooter à l'ouverture sous Windows
        self.ser.setDTR(False)
        self.ser.setRTS(False)
        self.ser.open()
        
        time.sleep(1.5)
        print("Ligne série Windows prête.")

    def acquerir_tout(self):
        """Envoie 'm', écoute le script complet et extrait les signaux bruts + variables Lock-in."""
        self.ser.reset_input_buffer()
        print("Envoi de la commande 'm' pour lancer le cycle...")
        self.ser.write(b'm')
        
        # Préparation des conteneurs pour les 5 mesures
        mesures_data = {i: {"t": [], "y": [], "rpm": 0.0, "amp": 0.0, "angle": 0.0} for i in range(1, N_MESURES + 1)}
        current_id = None
        
        while True:
            ligne = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not ligne:
                continue
                
            # Détection des entêtes de blocs de données de courbe (ex: ###1)
            if ligne.startswith("###accel-") and not ligne.endswith("###") and "=" not in ligne:
                try:
                    current_id = int(ligne.replace("###accel-", ""))
                    continue
                except ValueError:
                    pass
            
            # Fin de la capture globale
            if ligne == "###" and current_id == N_MESURES:
                print("Fin de transmission détectée.")
                break
                
            # Extraction des données calculées par le Lock-in de la carte
            if "###lockin" in ligne:
                try:
                    parts = ligne.split(';')
                    # Extraction propre des valeurs
                    f_ref = float(parts[1].strip()) #f_ref
                    amp = float(parts[2].strip()) #amplitude
                    angle = float(parts[5].strip()) #angle
                    
                    # On associe ces caractéristiques au bloc en cours
                    target_id = len([k for k, v in mesures_data.items() if v["amp"] > 0]) + 1
                    if target_id <= N_MESURES:
                        mesures_data[target_id]["rpm"] = f_ref * 60.0
                        mesures_data[target_id]["amp"] = amp
                        mesures_data[target_id]["phase"] = angle
                except (IndexError, ValueError) as e:
                    print("Erreur de parsing de la ligne Lock-in:", e)
            
            # Extraction des points de courbe brute (format: temps_us;valeur_Y)
            if current_id in mesures_data and ';' in ligne:
                if 't;Y;t' in ligne:
                    continue  # Ignore l'entête
                try:
                    elements = ligne.split(';')
                    t_us = float(elements[0])
                    y_val = float(elements[1])
                    
                    mesures_data[current_id]["t"].append(t_us / 1e6)  # µs -> secondes
                    mesures_data[current_id]["y"].append(y_val)
                except (ValueError, IndexError):
                    continue
                    
        # Conversion finale en Tableaux Numpy
        resultat_final = []
        for i in range(1, N_MESURES + 1):
            d = mesures_data[i]
            resultat_final.append({
                "t": np.array(d["t"]),
                "y": np.array(d["y"]),
                "rpm": d["rpm"],
                "amp": d["amp"],
                "phase": d["phase"]
            })
        return resultat_final

# --------------------------------------------------------------------------
# Interface Graphique
# --------------------------------------------------------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Équilibreuse Dynamique - 5 Mesures")
        self.source = AcquisitionReelle(port="COM3", baudrate=115200)
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
        for i in range(N_MESURES):
            ax = self.fig.add_subplot(gs[i, 0], sharex=self.axes[0] if self.axes else None)
            ax.set_ylabel(f"#{i + 1}", color=COULEURS[i])
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
        
        self.var_etat.set("Acquisition des 5 étapes en cours...")
        self.update()
        
        try:
            mesures = self.source.acquerir_tout()
        except Exception as e:
            self.var_etat.set(f"Erreur : {e}")
            self.btn.config(state="normal")
            return

        vector_sum_list = []

        for i in range(N_MESURES):
            m = mesures[i]
            t = m["t"]
            y = m["y"]
            
            # Graphique temporel
            ax = self.axes[i]
            ax.clear()
            if t.size > 0 and y.size > 0:
                ax.plot(t - t[0], y, lw=0.8, color=COULEURS[i])
            ax.set_ylabel(f"#{i + 1}", color=COULEURS[i])
            
            # Tableau de synthèse
            self.tab.insert("", "end", values=(
                i + 1, f"{m['rpm']:.1f}", f"{m['amp']:.4f}", f"{m['phase']:.1f}"
            ))
            
            # Plan polaire
            self.ax_pol.plot(np.radians(m["phase"]), m["amp"], "o", ms=9, color=COULEURS[i])
            vector_sum_list.append(m["amp"] * np.exp(1j * np.radians(m["phase"])))
            
        # Calcul de la moyenne vectorielle
        if vector_sum_list:
            z_moy = np.mean(vector_sum_list)
            self.ax_pol.annotate("", xy=(np.angle(z_moy), abs(z_moy)), xytext=(0, 0),
                                 arrowprops=dict(arrowstyle="->", lw=2.5, color="black"),
                                 xycoords='data', textcoords='data')
            
            max_amp = max([m["amp"] for m in mesures]) if max([m["amp"] for m in mesures]) > 0 else 1.0
            self.ax_pol.set_rlim(0, max_amp * 1.3)
            self.var_etat.set(f"Moyenne : {abs(z_moy):.4f} à {np.degrees(np.angle(z_moy)) % 360:.1f}°")
            
        self.axes[-1].set_xlabel("Temps (s)")
        self.canvas.draw_idle()
        self.btn.config(state="normal")

if __name__ == "__main__":
    App().mainloop()
