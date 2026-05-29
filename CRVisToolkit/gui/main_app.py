import sys
import os
import numpy as np

# Gestion des imports PyQt5 et Matplotlib
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QComboBox
from PyQt5.QtCore import QTimer, Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# Import de ta fonction de dessin 3D existante
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))
sys.path.append(os.path.join(project_root, "python"))

try:
    from draw_ctcr import draw_ctcr
except ImportError:
    sys.path.append(project_root)
    from python.draw_ctcr import draw_ctcr

class MainApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CRVisToolkit - CTR Control Dashboard")
        self.setGeometry(100, 100, 1400, 900)

        # --- Données et Trajectoire ---
        current_dir = os.path.dirname(os.path.abspath(__file__))
        timc_robotics_root = os.path.abspath(os.path.join(current_dir, "..", ".."))
        self.data_path = os.path.join(timc_robotics_root, "Modeling-and-Control-of-Concentric-Tube-Continuum-Robots", "build", "robot_backbone.txt")

        self.steps_data = []
        self.current_step = 0
        self.r_tube = np.array([1.52e-3, 1.80e-3, 2.35e-3]) / 2.0
        self.l_kappa = 0.040 

        self.load_trajectory_data()

        # --- Initialisation de l'interface GUI ---
        self.init_ui()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_animation)
        self.is_playing = False

    def load_trajectory_data(self):
        if not os.path.exists(self.data_path):
            print(f"Erreur : Fichier introuvable à l'emplacement : {self.data_path}")
            self.steps_data = []
            return

        with open(self.data_path, "r") as f:
            content = f.read()
        blocks = content.strip().split("--- STEP_BREAK ---")
        self.steps_data = [b.strip() for b in blocks if b.strip()]
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
        
        # Choix du graphique à afficher
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
        
        # Attribution d'un ratio d'affichage (Panneau: 1, Graphiques: 5) pour donner un maximum d'espace
        main_layout.addWidget(self.canvas, stretch=5)

        # Configuration : Moitié gauche = Robot 3D, Moitié droite = Le graphique unique choisi
        self.ax_robot = self.fig.add_subplot(121, projection='3d') 
        self.ax_plots = self.fig.add_subplot(122)                  

        # FIXATION STRICTE DE LA VUE (Désactive les mouvements à la souris)
        self.ax_robot.view_init(elev=20, azim=-45)
        self.ax_robot.disable_mouse_rotation()

        # Ajustement manuel des marges pour créer un espace net entre le robot et les graphiques
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
        if not self.steps_data or self.current_step >= len(self.steps_data):
            return

        # 1. Récupération de la frame actuelle
        step_str = self.steps_data[self.current_step]
        matrix_lines = [list(map(float, line.split())) for line in step_str.split('\n') if line.strip()]
        matrix_data = np.array(matrix_lines)
        num_nodes = matrix_data.shape[1]

        nodes_per_seg = int(num_nodes / 3)
        n1 = nodes_per_seg      
        n2 = 2 * nodes_per_seg  
        tube_end = np.array([n1, n2, num_nodes])

        # Sauvegarde de la vue caméra
        current_elev = self.ax_robot.elev
        current_azim = self.ax_robot.azim

        self.ax_robot.cla()
        self.ax_plots.cla()

        self.ax_robot.view_init(elev=current_elev, azim=current_azim)

        # 2. Reconstruction géométrique
        g = np.zeros((num_nodes, 16))
        for i in range(num_nodes):
            M = np.eye(4)
            M[0, 3] = matrix_data[0, i] # X
            M[1, 3] = matrix_data[1, i] # Y
            M[2, 3] = matrix_data[2, i] # Z
            R = matrix_data[3:12, i].reshape((3, 3), order='F')
            M[0:3, 0:3] = R
            g[i, :] = M.flatten(order='F')

        # --- A. RECONSTRUCTION DU ROBOT 3D ---
        draw_ctcr(g, tube_end, self.r_tube, tipframe=True, ax=self.ax_robot)

        # AJOUT DES AXES DE RÉFÉRENCE À LA BASE (0,0,0)
        longueur_axe = 0.03
        self.ax_robot.quiver(0, 0, 0, longueur_axe, 0, 0, color='r', linewidth=2, label='Axe X')
        self.ax_robot.quiver(0, 0, 0, 0, longueur_axe, 0, color='g', linewidth=2, label='Axe Y')
        self.ax_robot.quiver(0, 0, 0, 0, 0, longueur_axe, color='b', linewidth=2, label='Axe Z')

        self.ax_robot.set_aspect('equal')
        self.ax_robot.set_box_aspect((1, 1, 1))
        self.ax_robot.set_xlim3d([-0.05, 0.05])
        self.ax_robot.set_ylim3d([-0.05, 0.05])
        self.ax_robot.set_zlim3d([0.0, 0.18])
        self.ax_robot.set_title("Géométrie du Robot CTR")

        # Calcul de la longueur curviligne réelle pour l'axe X
        length_axis = np.zeros(num_nodes)
        for i in range(1, num_nodes):
            dl = np.sqrt((matrix_data[0, i]-matrix_data[0, i-1])**2 + 
                         (matrix_data[1, i]-matrix_data[1, i-1])**2 + 
                         (matrix_data[2, i]-matrix_data[2, i-1])**2)
            length_axis[i] = length_axis[i-1] + dl

        # --- B. AFFICHAGE DU GRAPHIQUE SÉLECTIONNÉ ---
        selected_graph = self.graph_selector.currentIndex()



        if selected_graph == 0:
            # --- CAS 1 : ANGLES DIRECTEURS DE LA STRUCTURE (Par rapport aux axes fixes X, Y, Z) ---
            orient_X = np.zeros(num_nodes)
            orient_Y = np.zeros(num_nodes)
            orient_Z = np.zeros(num_nodes)

            for i in range(num_nodes):
                R = matrix_data[3:12, i].reshape((3, 3), order='F')
                
                t_x = R[0, 2]
                t_y = R[1, 2]
                t_z = R[2, 2]

                orient_X[i] = np.degrees(np.arccos(np.clip(t_x, -1.0, 1.0))) 
                orient_Y[i] = np.degrees(np.arccos(np.clip(t_y, -1.0, 1.0))) 
                orient_Z[i] = np.degrees(np.arccos(np.clip(t_z, -1.0, 1.0))) 

            # Correction stricte des couleurs : r=Rouge=X, g=Vert=Y, b=Bleu=Z
            self.ax_plots.plot(length_axis, orient_X, 'r-', linewidth=2, label="Orientation / Axe X")
            self.ax_plots.plot(length_axis, orient_Y, 'g-', linewidth=2, label="Orientation / Axe Y")
            self.ax_plots.plot(length_axis, orient_Z, 'b-', linewidth=2, label="Orientation / Axe Z")
            
            self.ax_plots.set_title("Orientation du CTR")
            self.ax_plots.set_ylabel("Angle par rapport à l'axe de référence (en degrés)", labelpad=12) # labelpad évite le chevauchement
            self.ax_plots.set_ylim([0, 120]) # Plus zoomé et lisible que 180


        else:
            # --- CAS 2 : TORSION RELATIVE (VUE MICRO) ---
            theta_2 = matrix_data[17, :]
            theta_3 = matrix_data[18, :]

            t2_display = np.copy(theta_2)
            t3_display = np.copy(theta_3)
            t3_display[n1:] = np.nan 
            t2_display[n2:] = np.nan

            self.ax_plots.plot(length_axis, t2_display, 'b-o', markersize=3, label=r"$\theta_2$ (intermédiaire)")
            self.ax_plots.plot(length_axis, t3_display, 'r-o', markersize=3, label=r"$\theta_3$ (externe)")
            self.ax_plots.axhline(y=0, color='black', linestyle='-', alpha=0.3, label=r"$\theta_1 = 0$")

            l_transition1 = length_axis[n1 - 1]
            l_transition2 = length_axis[n2 - 1]
            self.ax_plots.axvline(x=l_transition1, color='gray', linestyle='--', alpha=0.8, label="Fin Tube 3")
            self.ax_plots.axvline(x=l_transition2, color='gray', linestyle='--', alpha=0.8, label="Fin Tube 2")

            if (l_transition2 - 0.03) > 0:
                self.ax_plots.axvline(x=(l_transition2 - 0.03), color='purple', linestyle=':', alpha=0.6, label=r"$L - l_\kappa$")

            self.ax_plots.set_title("Vue MICRO : Torsion relative (Réf: Tube 1)")
            self.ax_plots.set_ylabel("Torsion relative (rad)")

        # Paramètres communs aux graphiques
        self.ax_plots.set_xlabel("Longueur curviligne du robot (m)")
        self.ax_plots.set_xlim([0, length_axis[-1]])
        self.ax_plots.grid(True)
        self.ax_plots.legend(loc="best")

        # On applique la séparation forcée définie dans init_ui
        self.fig.subplots_adjust(left=0.05, right=0.95, wspace=0.35)
        self.canvas.draw()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(app.exec_())