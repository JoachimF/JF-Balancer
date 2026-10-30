import serial
import serial.tools.list_ports
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
BAUDRATE = 115200
PORT_PAR_DEFAUT = "COM3"      # présélectionné s'il est présent
POLAR_RMAX = 0.3
ERR_SERIE = (serial.SerialException, OSError)


class AcquisitionReelle:
    """Gère le port série et le thread de lecture. Ne s'ouvre plus tout seul."""

    def __init__(self, baudrate=BAUDRATE):
        self.baudrate = baudrate
        self.ser = None
        self.thread = None
        self.stop_event = threading.Event()

    # ---------- port ----------
    @property
    def connecte(self):
        return self.ser is not None and self.ser.is_open

    def ouvrir(self, port):
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = self.baudrate
        ser.timeout = 0.2          # court : permet d'arrêter le thread rapidement
        ser.dtr = False            # évite de resetter l'ESP32 à l'ouverture
        ser.rts = False
        ser.open()
        self.ser = ser             # affecté seulement si l'ouverture a réussi

    def fermer(self):
        self.arreter()
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def port_present(self):
        """Teste que le port répond encore (à n'appeler que si aucun thread ne lit)."""
        try:
            self.ser.in_waiting
            return True
        except ERR_SERIE:
            return False
        except AttributeError:
            return False

    # ---------- thread de lecture ----------
    @property
    def en_cours(self):
        return self.thread is not None and self.thread.is_alive()

    def lancer(self, data_queue):
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._cycle, args=(data_queue,), daemon=True)
        self.thread.start()

    def arreter(self):
        self.stop_event.set()
        if self.thread is not None and self.thread is not threading.current_thread():
            self.thread.join(timeout=1.5)
        self.thread = None

    def _lire_ligne(self, buf):
        """Lit une ligne complète. Renvoie None si l'arrêt est demandé."""
        while b"\n" not in buf:
            if self.stop_event.is_set():
                return None
            morceau = self.ser.read(self.ser.in_waiting or 1)
            if morceau:
                buf.extend(morceau)
        i = buf.index(b"\n")
        ligne = bytes(buf[:i])
        del buf[:i + 1]
        return ligne.decode("utf-8", errors="ignore").strip()

    def _cycle(self, data_queue):
        try:
            self.ser.reset_input_buffer()
            self.ser.write(b"m")
            buf = bytearray()
            current_accel1_id = None
            current_accel2_id = None
            current_lockin1_id = None
            current_lockin2_id = None
            fin_prevue = False
            t0_mesure = None
            t0_mesure2 = None

            while True:
                ligne = self._lire_ligne(buf)
                if ligne is None:          # arrêt demandé (reset / déconnexion)
                    return
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
                    if fin_prevue:
                        data_queue.put({"type": "FIN"})
                        return
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
                            data_queue.put({"type": "LOCKIN1", "index1": current_lockin1_id, "rpm1": f_ref * 60.0, "amp1": amp, "c_amp1": c_amp, "phase1": phase, "angle1": angle})
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
                            data_queue.put({"type": "LOCKIN2", "index2": current_lockin2_id, "rpm2": f_ref * 60.0, "amp2": amp, "c_amp2": c_amp, "phase2": phase, "angle2": angle})
                            # Correctif : on mémorise ici que c'est la dernière mesure,
                            # car l'id est remis à None juste après.
                            if current_lockin2_id == (N_MESURES - 1):
                                fin_prevue = True
                        current_lockin2_id = None
                    except (IndexError, ValueError):
                        pass
        except ERR_SERIE as e:
            if not self.stop_event.is_set():
                data_queue.put({"type": "DECONNEXION", "msg": str(e)})
        except Exception as e:
            if not self.stop_event.is_set():
                data_queue.put({"type": "ERREUR", "msg": str(e)})


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Équilibreuse Dynamique - 5 Mesures Temps Réel")
        self.geometry("1800x1050")
        self.source = AcquisitionReelle(BAUDRATE)
        self.data_queue = queue.Queue()
        self.lignes_plot = []
        self.lignes_plot2 = []
        self.txt_rpm = []
        self.txt_rpm2 = []
        self.ports = {}
        self._ports_vus = None
        self.port_courant = None
        self.pret = False                 # vrai une fois la carte "stabilisée" après connexion
        self.acquisition_active = False
        self._after_pret = None
        self._reset_donnees()

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

        self._rafraichir_ports()
        self._maj_boutons()
        self.protocol("WM_DELETE_WINDOW", self.fermer_app)
        self.after(20, self.traiter_queue)       # une seule boucle, active en permanence
        self.after(1000, self._surveiller)

    # ------------------------------------------------------------------ données
    def _reset_donnees(self):
        self.donnees_accumulees = {i: {"t1": [], "y1": [], "rpm1": 0.0, "amp1": 0.0, "c_amp1": 0.0, "phase1": 0.0, "angle1": 0.0} for i in range(N_MESURES)}
        self.donnees_accumulees2 = {j: {"t2": [], "y2": [], "rpm2": 0.0, "amp2": 0.0, "c_amp2": 0.0, "phase2": 0.0, "angle2": 0.0} for j in range(N_MESURES)}
        self.vector_sum_list = []
        self.vector_sum_list2 = []

    # ------------------------------------------------------------------ interface
    def _barre(self):
        barre = ttk.Frame(self.scrollable_frame, padding=8)
        barre.pack(fill="x")

        ttk.Label(barre, text="Port COM :").pack(side="left")
        self.var_port = tk.StringVar()
        self.combo_port = ttk.Combobox(barre, textvariable=self.var_port, width=40, state="readonly")
        self.combo_port.pack(side="left", padx=(4, 2))
        self.combo_port.bind("<<ComboboxSelected>>", lambda e: self._maj_boutons())
        self.btn_actualiser = ttk.Button(barre, text="↻", width=3, command=lambda: self._rafraichir_ports(force=True))
        self.btn_actualiser.pack(side="left", padx=2)
        self.btn_connexion = ttk.Button(barre, text="Connexion", command=self.connecter)
        self.btn_connexion.pack(side="left", padx=(8, 2))
        self.btn_deconnexion = ttk.Button(barre, text="Déconnexion", command=self.deconnecter)
        self.btn_deconnexion.pack(side="left", padx=2)

        ttk.Separator(barre, orient="vertical").pack(side="left", fill="y", padx=12)

        self.btn = ttk.Button(barre, text="Lancer les mesures", command=self.lancer)
        self.btn.pack(side="left", padx=2)
        self.btn_reset = ttk.Button(barre, text="Reset des mesures", command=self.reset)
        self.btn_reset.pack(side="left", padx=2)

        self.var_etat = tk.StringVar(value="Non connecté. Choisissez un port COM.")
        self.lbl_etat = ttk.Label(barre, textvariable=self.var_etat)
        self.lbl_etat.pack(side="left", padx=12)
        self._couleur_normale = str(ttk.Style().lookup("TLabel", "foreground")) or "black"

    def _etat(self, msg, erreur=False):
        self.var_etat.set(msg)
        self.lbl_etat.configure(foreground="#c00000" if erreur else self._couleur_normale)

    def _init_polaire(self, ax, titre):
        ax.set_theta_zero_location("N")
        ax.set_theta_direction(-1)
        ax.set_ylim(0, POLAR_RMAX)
        ax.set_title(titre)

    def _figure(self):
        self.fig = Figure(figsize=(1750, 1000, 'px'), tight_layout=True)
        gs = self.fig.add_gridspec(N_MESURES + 1, 3, width_ratios=[1, 1, 1])

        self.axes = []
        self.axes2 = []

        def texte_rpm(ax, i):
            # vide au départ : matplotlib ne dessine pas un texte vide
            return ax.text(0.99, 0.93, "", transform=ax.transAxes, ha="right", va="top",
                           fontsize=9, color=COULEURS[i],
                           bbox=dict(boxstyle="round,pad=0.25", fc="white", ec=COULEURS[i], alpha=0.85))

        for i in range(N_MESURES):
            ax = self.fig.add_subplot(gs[i, 0], sharex=self.axes[0] if self.axes else None)
            ax.set_ylabel(f"#{i + 1}", color=COULEURS[i])
            ligne, = ax.plot([], [], lw=0.8, color=COULEURS[i])
            self.lignes_plot.append(ligne)
            self.axes.append(ax)
            self.txt_rpm.append(texte_rpm(ax, i))

        for i in range(N_MESURES):
            ax2 = self.fig.add_subplot(gs[i, 1], sharex=self.axes2[0] if self.axes2 else None)
            ax2.set_ylabel(f"#{i + 1}", color=COULEURS[i])
            ligne2, = ax2.plot([], [], lw=0.8, color=COULEURS[i])
            self.lignes_plot2.append(ligne2)
            self.axes2.append(ax2)
            self.txt_rpm2.append(texte_rpm(ax2, i))

        self.axes[0].set_title("Avant")
        self.axes2[0].set_title("Arrière")
        self.axes[-1].set_xlabel("Temps (s)")
        self.axes2[-1].set_xlabel("Temps (s)")

        self.ax_pol = self.fig.add_subplot(gs[:3, 2], projection="polar")
        self._init_polaire(self.ax_pol, "Avant")
        self.ax_pol2 = self.fig.add_subplot(gs[3:, 2], projection="polar")
        self._init_polaire(self.ax_pol2, "Arrière")

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

    # ------------------------------------------------------------------ état des boutons
    def _maj_boutons(self):
        conn = self.source.connecte
        actif = self.acquisition_active

        def etat(widget, actif_):
            widget.config(state="normal" if actif_ else "disabled")

        self.combo_port.config(state="disabled" if conn else "readonly")
        etat(self.btn_actualiser, not conn)
        etat(self.btn_connexion, (not conn) and bool(self._port_choisi()))
        etat(self.btn_deconnexion, conn)
        etat(self.btn, conn and self.pret and not actif)
        etat(self.btn_reset, not actif)

    # ------------------------------------------------------------------ ports COM
    def _port_choisi(self):
        return self.ports.get(self.var_port.get())

    def _rafraichir_ports(self, force=False):
        if self.source.connecte:
            return
        try:
            ports = sorted(serial.tools.list_ports.comports(), key=lambda p: (len(p.device), p.device))
        except Exception:
            ports = []
        mapping = {}
        for p in ports:
            desc = p.description if p.description and p.description != "n/a" else ""
            mapping[f"{p.device} - {desc}" if desc else p.device] = p.device
        if not force and list(mapping) == self._ports_vus:
            return
        self._ports_vus = list(mapping)
        ancien = self._port_choisi()
        self.ports = mapping
        self.combo_port["values"] = list(mapping)
        choix = (next((l for l, d in mapping.items() if d == ancien), None)
                 or next((l for l, d in mapping.items() if d == PORT_PAR_DEFAUT), None)
                 or next(iter(mapping), ""))
        self.var_port.set(choix)
        self._maj_boutons()

    def connecter(self):
        port = self._port_choisi()
        if not port:
            self._etat("Aucun port sélectionné.", True)
            return
        try:
            self.source.ouvrir(port)
        except ERR_SERIE as e:
            self._etat(f"Connexion impossible sur {port} : {e}", True)
            self._rafraichir_ports(force=True)
            return
        self.port_courant = port
        self.pret = False
        self._etat(f"Connexion à {port}…")
        self._maj_boutons()
        # petite pause (remplace le time.sleep(1.5) qui gelait l'interface)
        self._after_pret = self.after(1500, self._connexion_prete)

    def _connexion_prete(self):
        self._after_pret = None
        if self.source.connecte:
            self.pret = True
            self._etat(f"Connecté à {self.port_courant}. Prêt.")
            self._maj_boutons()

    def deconnecter(self):
        if self._after_pret is not None:
            self.after_cancel(self._after_pret)
            self._after_pret = None
        interrompu = self.acquisition_active
        self.source.fermer()
        self.acquisition_active = False
        self.pret = False
        self._vider_queue()
        self._etat(f"Déconnecté de {self.port_courant}." + (" Acquisition interrompue." if interrompu else ""))
        self._rafraichir_ports(force=True)
        self._maj_boutons()

    def _perte_connexion(self, detail=""):
        """Port perdu (câble débranché, carte coupée...)."""
        if self._after_pret is not None:
            self.after_cancel(self._after_pret)
            self._after_pret = None
        self.source.fermer()
        self.acquisition_active = False
        self.pret = False
        self._vider_queue()
        self._etat(f"Port {self.port_courant} déconnecté ({detail}). "
                   "Reconnectez la carte, choisissez le port puis cliquez sur Connexion.", True)
        self._rafraichir_ports(force=True)
        self._maj_boutons()

    def _surveiller(self):
        """Détecte une déconnexion quand aucune acquisition ne lit le port,
        et met à jour la liste des ports quand on est déconnecté."""
        try:
            if self.source.connecte:
                if not self.acquisition_active and not self.source.port_present():
                    self._perte_connexion("port introuvable")
            else:
                self._rafraichir_ports()
        finally:
            self.after(1000, self._surveiller)

    def fermer_app(self):
        self.source.fermer()
        self.destroy()

    # ------------------------------------------------------------------ mesures
    def _vider_queue(self):
        while True:
            try:
                self.data_queue.get_nowait()
            except queue.Empty:
                break

    def _reinitialiser_affichage(self):
        self.tab.delete(*self.tab.get_children())

        self.ax_pol.clear()
        self._init_polaire(self.ax_pol, "Avant")
        self.ax_pol2.clear()
        self._init_polaire(self.ax_pol2, "Arrière")

        self._reset_donnees()

        for i in range(N_MESURES):
            self.lignes_plot[i].set_data([], [])
            self.lignes_plot2[i].set_data([], [])
            self.txt_rpm[i].set_text("")
            self.txt_rpm2[i].set_text("")
            for ax in (self.axes[i], self.axes2[i]):
                ax.relim()
                ax.autoscale_view(scalex=True, scaley=True)
        self.graph_canvas.draw_idle()

    def reset(self):
        if self.acquisition_active:
            return
        self._vider_queue()
        if self.source.connecte:
            try:
                self.source.ser.reset_input_buffer()
            except ERR_SERIE as e:
                self._perte_connexion(str(e))
                return
        self._reinitialiser_affichage()
        if self.source.connecte and self.pret:
            self._etat(f"Mesures réinitialisées. Connecté à {self.port_courant}, prêt à relancer.")
        else:
            self._etat("Mesures réinitialisées.")

    def lancer(self):
        if not (self.source.connecte and self.pret) or self.acquisition_active:
            return
        self._vider_queue()
        self._reinitialiser_affichage()
        self.acquisition_active = True
        self._maj_boutons()
        self._etat("Attente de la mesure 1")
        self.source.lancer(self.data_queue)

    def traiter_queue(self):
        points_recus = False
        points_recus2 = False
        try:
            while True:
                try:
                    msg = self.data_queue.get_nowait()
                except queue.Empty:
                    break
                t = msg["type"]

                if t == "POINT1":
                    idx = msg["index1"]
                    if idx is not None and 0 <= idx < N_MESURES:
                        self.donnees_accumulees[idx]["t1"].append(msg["t1"])
                        self.donnees_accumulees[idx]["y1"].append(msg["y1"])
                        points_recus = True

                elif t == "POINT2":
                    idx = msg["index2"]
                    if idx is not None and 0 <= idx < N_MESURES:
                        self.donnees_accumulees2[idx]["t2"].append(msg["t2"])
                        self.donnees_accumulees2[idx]["y2"].append(msg["y2"])
                        points_recus2 = True

                elif t == "LOCKIN1":
                    idx = msg["index1"]
                    if idx is not None and 0 <= idx < N_MESURES:
                        d = self.donnees_accumulees[idx]
                        d["rpm1"] = msg["rpm1"]
                        d["amp1"] = msg["amp1"]
                        d["c_amp1"] = msg["c_amp1"]
                        d["phase1"] = msg["phase1"]
                        d["angle1"] = msg["angle1"]
                        self.txt_rpm[idx].set_text(f"{msg['rpm1']:.0f} tr/min")
                        self.tab.insert("", "end", values=(idx + 1, f"{msg['rpm1']:.1f}", f"{msg['amp1']:.4f}", f"{msg['c_amp1']:.4f}", f"{msg['phase1']:.1f}", f"{msg['angle1']:.1f}"))
                        if msg["c_amp1"] > 0:
                            self.ax_pol.plot(np.radians(msg["angle1"]), msg["c_amp1"], "o", ms=9, color=COULEURS[idx])
                            self.vector_sum_list.append(msg["c_amp1"] * np.exp(1j * np.radians(msg["angle1"])))
                        self._etat(f"Attente de la mesure {idx + 1}")
                        self.graph_canvas.draw_idle()

                elif t == "LOCKIN2":
                    idx = msg["index2"]
                    if idx is not None and 0 <= idx < N_MESURES:
                        d = self.donnees_accumulees2[idx]
                        d["rpm2"] = msg["rpm2"]
                        d["amp2"] = msg["amp2"]
                        d["c_amp2"] = msg["c_amp2"]
                        d["phase2"] = msg["phase2"]
                        d["angle2"] = msg["angle2"]
                        self.txt_rpm2[idx].set_text(f"{msg['rpm2']:.0f} tr/min")
                        self.tab.insert("", "end", values=(idx + 11, f"{msg['rpm2']:.1f}", f"{msg['amp2']:.4f}", f"{msg['c_amp2']:.4f}", f"{msg['phase2']:.1f}", f"{msg['angle2']:.1f}"))
                        if msg["c_amp2"] > 0:
                            self.ax_pol2.plot(np.radians(msg["angle2"]), msg["c_amp2"], "o", ms=9, color=COULEURS[idx])
                            self.vector_sum_list2.append(msg["c_amp2"] * np.exp(1j * np.radians(msg["angle2"])))
                        self.graph_canvas.draw_idle()

                elif t == "FIN":
                    if self.vector_sum_list:
                        z_moy = np.mean(self.vector_sum_list)
                        self.ax_pol.annotate("", xy=(np.angle(z_moy), abs(z_moy)), xytext=(0, 0),
                                             arrowprops=dict(arrowstyle="->", lw=2.5, color="black"))
                        self._etat(f"Moyenne : {abs(z_moy):.4f} à {np.degrees(np.angle(z_moy)) % 360:.1f}°")
                    else:
                        self._etat("Cycle terminé.")
                    self.graph_canvas.draw_idle()
                    self.acquisition_active = False
                    self._maj_boutons()

                elif t == "ERREUR":
                    self._etat(f"Erreur : {msg['msg']}", True)
                    self.acquisition_active = False
                    self._maj_boutons()

                elif t == "DECONNEXION":
                    self._perte_connexion(msg["msg"])

            if points_recus:
                for i in range(N_MESURES):
                    t_data = self.donnees_accumulees[i]["t1"]
                    y_data = self.donnees_accumulees[i]["y1"]
                    if t_data:
                        self.lignes_plot[i].set_data(t_data, y_data)
                        self.axes[i].relim()
                        self.axes[i].autoscale_view(scalex=True, scaley=True)
                self.graph_canvas.draw_idle()
            if points_recus2:
                for j in range(N_MESURES):
                    t_data = self.donnees_accumulees2[j]["t2"]
                    y_data = self.donnees_accumulees2[j]["y2"]
                    if t_data:
                        self.lignes_plot2[j].set_data(t_data, y_data)
                        self.axes2[j].relim()
                        self.axes2[j].autoscale_view(scalex=True, scaley=True)
                self.graph_canvas.draw_idle()
        finally:
            self.after(20, self.traiter_queue)


if __name__ == "__main__":
    App().mainloop()