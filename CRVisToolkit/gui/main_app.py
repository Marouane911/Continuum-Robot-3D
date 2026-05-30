import sys
import os
import numpy as np
import time


# Gestion des imports PyQt5 et Matplotlib
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QComboBox
from PyQt5.QtCore import QTimer, Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# --- GESTION DU REPERTOIRE DE PYTHON (Dossier frère) ---
current_dir = os.path.dirname(os.path.abspath(__file__)) # ~/CRVisToolkit/gui
root_toolkit = os.path.dirname(current_dir)              # ~/CRVisToolkit
python_dir = os.path.join(root_toolkit, "python")        # ~/CRVisToolkit/python

if python_dir not in sys.path:
    sys.path.append(python_dir)


class MainApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CRVisToolkit - CTR Control Dashboard (Version Blindée)")
        self.setGeometry(100, 100, 1400, 900)

        # --- RECONSTRUCTION DU CHEMIN ABSOLU VERS LE REPO DE QUENTIN ---
        project_parent = os.path.dirname(root_toolkit)
        
        self.data_path = os.path.join(
            project_parent, 
            "Modeling-and-Control-of-Concentric-Tube-Continuum-Robots", 
            "build", 
            "robot_backbone.txt"
        )

        self.steps_data = []
        self.current_step = 0
        
        # Rayons externes exacts issus de parameters.csv (rOut1, rOut2, rOut3)
        self.r_tube = np.array([0.000762, 0.0009, 0.001175])
        
        # Définition de la longueur de précourbure (50 mm d'après le csv)
        self.l_kappa = 0.05 

        self.load_trajectory_data()

        # --- Initialisation de l'interface GUI ---
        self.init_ui()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_animation)
        self.is_playing = False

    def load_trajectory_data(self):
        """Parser horizontal : Valide les blocs contenant exactement 19 lignes de variables."""
        if not os.path.exists(self.data_path):
            print(f"Erreur : Fichier introuvable à l'emplacement : {self.data_path}")
            self.steps_data = []
            return

        with open(self.data_path, "r") as f:
            content = f.read()
            
        blocks = content.strip().split("--- STEP_BREAK ---")
        self.steps_data = []
        
        for b in blocks:
            lines = [l.strip() for l in b.split('\n') if l.strip()]
            
            if len(lines) == 19:
                matrix_lines = [list(map(float, line.split())) for line in lines]
                self.steps_data.append(np.array(matrix_lines))
                
        print(f"[{len(self.steps_data)}] étapes d'animation chargées avec succès.")
    
    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        # --- PANNEAU DE CONTRÔLE (GAUCHE) ---
        control_panel = QWidget()
        control_panel.setFixedWidth(250)
        control_layout = QVBoxLayout(control_panel)
        
        self.status_label = QLabel(f"Étape : 0 / {len(self.steps_data)}")
        self.status_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        control_layout.addWidget(self.status_label)

        self.btn_play = QPushButton("Lancer l'Animation")
        self.btn_play.setStyleSheet("background-color: #4CAF50; color: white; font-size: 14px; padding: 10px;")
        self.btn_play.clicked.connect(self.toggle_animation)
        control_layout.addWidget(self.btn_play)

        self.btn_reset = QPushButton("Réinitialiser")
        self.btn_reset.clicked.connect(self.reset_animation)
        control_layout.addWidget(self.btn_reset)
        
        control_layout.addWidget(QLabel("Sélection du Graphique :"))
        self.graph_selector = QComboBox()
        self.graph_selector.addItems(["Orientations (X, Y, Z)", "Torsion relative (Vue MICRO)"])
        self.graph_selector.currentIndexChanged.connect(self.update_plots)
        control_layout.addWidget(self.graph_selector)

        control_layout.addStretch()
        main_layout.addWidget(control_panel)

        # --- ZONE GRAPHIQUE MATPLOTLIB (DROITE) ---
        self.fig = Figure(figsize=(12, 8))
        self.canvas = FigureCanvas(self.fig)
        main_layout.addWidget(self.canvas, stretch=5)

        self.ax_robot = self.fig.add_subplot(121, projection='3d') 
        self.ax_plots = self.fig.add_subplot(122)                  

        self.ax_robot.view_init(elev=20, azim=-45)
        self.ax_robot.disable_mouse_rotation()
        self.fig.subplots_adjust(left=0.05, right=0.95, wspace=0.35)

    def toggle_animation(self):
        if not self.steps_data:
            return
        if self.is_playing:
            self.timer.stop()
            self.btn_play.setText("Reprendre l'Animation")
        else:
            self.timer.start(50) 
            self.btn_play.setText("Pause")
        self.is_playing = not self.is_playing

    def reset_animation(self):
        self.timer.stop()
        self.is_playing = False
        self.current_step = 0
        self.btn_play.setText("Lancer l'Animation")
        self.status_label.setText(f"Étape : 0 / {len(self.steps_data)}")
        self.update_plots()

    def update_animation(self):
        if self.current_step >= len(self.steps_data):
            self.timer.stop()
            self.is_playing = False
            self.btn_play.setText("Animation terminée")
            return
        
        self.status_label.setText(f"Étape : {self.current_step + 1} / {len(self.steps_data)}")
        self.update_plots()
        self.current_step += 1

    def update_plots(self):
        t0 = time.perf_counter()
        if not self.steps_data or self.current_step >= len(self.steps_data):
            return

        # Récupération de la matrice (19, N) pré-filtrée
        matrix_data = self.steps_data[self.current_step]
        num_nodes = matrix_data.shape[1]

        nodes_per_seg = int(num_nodes / 3)
        n1 = nodes_per_seg      
        n2 = 2 * nodes_per_seg  
        tube_end = np.array([n1, n2, num_nodes])

        print("num_nodes =", num_nodes)
        print("n1 =", n1)
        print("n2 =", n2)
        print("tube_end =", tube_end)

        current_elev = self.ax_robot.elev
        current_azim = self.ax_robot.azim

        self.ax_robot.cla()
        self.ax_plots.cla()
        self.ax_robot.view_init(elev=current_elev, azim=current_azim)


        # --- A. RENDU DU ROBOT 3D OPTIMIZED---

        xyz = matrix_data[0:3, :]

        x = xyz[0]
        y = xyz[1]
        z = xyz[2]

        scale = 1

        # Tube externe
        self.ax_robot.plot(
            x[:n1],
            y[:n1],
            z[:n1],
            linewidth=1.524 * scale,
            color='red'
        )

        # Tube intermédiaire
        self.ax_robot.plot(
            x[:n2],
            y[:n2],
            z[:n2],
            linewidth=1.800 * scale,
            color='green'
        )

        # Tube interne
        self.ax_robot.plot(
            x,
            y,
            z,
            linewidth=2.350 * scale,
            color='blue'
        )

        # Pointe
        self.ax_robot.scatter(
            x[-1],
            y[-1],
            z[-1],
            s=40
        )

        # Axes du repère
        axis_len = 0.03

        self.ax_robot.quiver(
            0, 0, 0,
            axis_len, 0, 0,
            color='r'
        )

        self.ax_robot.quiver(
            0, 0, 0,
            0, axis_len, 0,
            color='g'
        )

        self.ax_robot.quiver(
            0, 0, 0,
            0, 0, axis_len,
            color='b'
        )

        self.ax_robot.set_title("CTR 3D")

        self.ax_robot.set_box_aspect((1, 1, 1))

        self.ax_robot.set_xlim(-0.09, 0.09)
        self.ax_robot.set_ylim(-0.09, 0.09)
        self.ax_robot.set_zlim(0.0, 0.18)

        # Calcul de la longueur curviligne réelle
        xyz = matrix_data[0:3, :].T

        dxyz = np.diff(xyz, axis=0)

        dl = np.linalg.norm(dxyz, axis=1)

        length_axis = np.concatenate(([0], np.cumsum(dl)))

        # --- B. GRAPHES ---
        selected_graph = self.graph_selector.currentIndex()

        if selected_graph == 0:
            # Angles directeurs
            orient_X = np.zeros(num_nodes)
            orient_Y = np.zeros(num_nodes)
            orient_Z = np.zeros(num_nodes)

            for i in range(num_nodes):
                R = matrix_data[3:12, i].reshape((3, 3), order='C')
                t_x, t_y, t_z = R[0, 2], R[1, 2], R[2, 2] 

                orient_X[i] = np.degrees(np.arccos(np.clip(t_x, -1.0, 1.0))) 
                orient_Y[i] = np.degrees(np.arccos(np.clip(t_y, -1.0, 1.0))) 
                orient_Z[i] = np.degrees(np.arccos(np.clip(t_z, -1.0, 1.0))) 

            self.ax_plots.plot(length_axis, orient_X, 'r-', linewidth=2, label="Orientation / Axe X")
            self.ax_plots.plot(length_axis, orient_Y, 'g-', linewidth=2, label="Orientation / Axe Y")
            self.ax_plots.plot(length_axis, orient_Z, 'b-', linewidth=2, label="Orientation / Axe Z")
            self.ax_plots.set_title("Orientation du CTR", fontsize=11, fontweight='bold')
            self.ax_plots.set_ylabel("Angle de référence (degrés)", labelpad=12)
            self.ax_plots.set_ylim([0, 120])
        else:
            # Torsion relative
            theta_2 = matrix_data[17, :]
            theta_3 = matrix_data[18, :]

            t2_display = np.copy(theta_2)
            t3_display = np.copy(theta_3)

            t3_display[n1:] = np.nan 
            t2_display[n2:] = np.nan

            self.ax_plots.plot(length_axis, t2_display, 'b-o', markersize=3, label=r"$\theta_2$ (intermédiaire)")
            self.ax_plots.plot(length_axis, t3_display, 'r-o', markersize=3, label=r"$\theta_3$ (externe)")
            self.ax_plots.axhline(y=0, color='black', linestyle='-', alpha=0.5, label=r"$\theta_1 = 0$ (Réf. interne)")

            l_transition1 = length_axis[n1 - 1] 
            l_transition2 = length_axis[n2 - 1] 
            
            self.ax_plots.axvline(x=l_transition1, color='red', linestyle='--', alpha=0.7, label="Fin Tube 3 (Ext)")
            self.ax_plots.axvline(x=l_transition2, color='blue', linestyle='--', alpha=0.7, label="Fin Tube 2 (Int)")

            # Zones de précourbure
            start_curve_t3 = l_transition1 - self.l_kappa
            if start_curve_t3 > 0:
                self.ax_plots.axvline(x=start_curve_t3, color='darkorange', linestyle=':', linewidth=2.5, 
                                      label=r"Début Précourbure Tube 3")
                self.ax_plots.axvspan(start_curve_t3, l_transition1, color='orange', alpha=0.07, zorder=1)

            start_curve_t2 = l_transition2 - self.l_kappa
            if start_curve_t2 > 0:
                self.ax_plots.axvline(x=start_curve_t2, color='purple', linestyle=':', linewidth=2.5, 
                                      label=r"Début Précourbure Tube 2")
                self.ax_plots.axvspan(start_curve_t2, l_transition2, color='purple', alpha=0.04, zorder=1)

            self.ax_plots.set_title("Profil de Torsion & Frontières de Précourbure", fontsize=11, fontweight='bold')
            self.ax_plots.set_ylabel("Torsion relative (rad)", labelpad=10)
            
            # Limites dynamiques
            min_y = np.nanmin(matrix_data[17:19, :])
            max_y = np.nanmax(matrix_data[17:19, :])
            self.ax_plots.set_ylim([min_y - 0.01, max_y + 0.01])

        # --- CORRECTION DES QUADRILLAGES ---
        self.ax_plots.set_xlabel("Longueur curviligne du robot (m)")
        self.ax_plots.set_xlim([0, length_axis[-1]])
        
        self.ax_plots.grid(True, linestyle=':', alpha=0.6, zorder=2)
        self.ax_plots.legend(loc="upper right", fontsize=9, framealpha=0.9)

        self.fig.subplots_adjust(left=0.05, right=0.95, wspace=0.35)
        t1 = time.perf_counter()
        print("total :", round((t1 - t0) * 1000, 1), "ms")
        print("----------------")
        self.canvas.draw_idle()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(app.exec_())