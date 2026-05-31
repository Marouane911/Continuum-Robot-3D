import sys
import os
import numpy as np
import time
import subprocess

from PyQt5.QtWidgets import QDoubleSpinBox
from PyQt5.QtCore import QLocale
from PyQt5.QtWidgets import QCheckBox


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
        QLocale.setDefault(
            QLocale(QLocale.English, QLocale.UnitedStates)
        )
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

        # --- Mémoriser la dérnière configuration pour l'animation entre 2 positions entrées ---
        self.current_q = [
            -0.30,
            -0.20,
            -0.10,
            0.0,
            0.0,
            0.0
        ]

        self.saved_elev = 20
        self.saved_azim = -45

        self.compute_ctr_configuration(self.current_q)
        

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
                lengths = [len(line.split()) for line in lines]

                print("Nombre de colonnes par ligne :", lengths)

                matrix_lines = [list(map(float, line.split())) for line in lines]

                arr = np.array(matrix_lines)

                print("Shape chargée :", arr.shape)

                z = arr[2,:]

                print(
                    "z range :",
                    np.min(z),
                    "->",
                    np.max(z)
                )

                self.steps_data.append(arr)                
                
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
        self.btn_home = QPushButton("Position par défaut")
        self.btn_home.clicked.connect(self.go_home)
        control_layout.addWidget(self.btn_home)
        self.btn_reset.clicked.connect(self.reset_animation)
        control_layout.addWidget(self.btn_reset)

        # ----------------------------------
        # Actionneurs CTR
        # ----------------------------------

        control_layout.addWidget(QLabel("Actionneurs CTR"))

        self.q_inputs = []

        default_values = [
            -0.30,
            -0.20,
            -0.10,
            0.0,
            0.0,
            0.0
        ]

        for i in range(6):

            control_layout.addWidget(QLabel(f"q{i}"))

            spinbox = QDoubleSpinBox()
            spinbox.setLocale(
                QLocale(QLocale.English, QLocale.UnitedStates)
            )

            # Translations
            if i < 3:
                spinbox.setRange(-0.40, 0.05)
                spinbox.setDecimals(4)
                spinbox.setSingleStep(0.005)

            # Rotations
            else:
                spinbox.setRange(-6.28318, 6.28318)
                spinbox.setDecimals(3)
                spinbox.setSingleStep(0.1)

            spinbox.setValue(default_values[i])

            self.q_inputs.append(spinbox)

            spinbox.valueChanged.connect(
                self.on_spinbox_changed
            )

            control_layout.addWidget(spinbox)


        # - Actionneurs POSITION PAR DÉFAUT -
        self.home_q = [
            -0.30,
            -0.20,
            -0.10,
            0.0,
            0.0,
            0.0
        ]

        
        control_layout.addWidget(QLabel("Sélection du Graphique :"))
        self.graph_selector = QComboBox()
        self.graph_selector.addItems(["Orientations (X, Y, Z)", "Torsion relative (Vue MICRO)"])
        self.graph_selector.currentIndexChanged.connect(self.update_plots)
        control_layout.addWidget(self.graph_selector)

        self.auto_apply_checkbox = QCheckBox("Appliquer automatiquement")

        control_layout.addWidget(
            self.auto_apply_checkbox
        )

        # - Bouton -
        self.btn_apply = QPushButton("Appliquer configuration")
        self.btn_apply.clicked.connect(
            self.apply_configuration
        )

        control_layout.addWidget(self.btn_apply)

        control_layout.addStretch()
        main_layout.addWidget(control_panel)

        # --- ZONE GRAPHIQUE MATPLOTLIB (DROITE) ---
        self.fig = Figure(figsize=(12, 8))
        self.canvas = FigureCanvas(self.fig)
        main_layout.addWidget(self.canvas, stretch=5)

        self.ax_robot = self.fig.add_subplot(121, projection='3d') 
        self.ax_plots = self.fig.add_subplot(122)                  

        self.ax_robot.view_init(elev=20, azim=-45)
        # self.ax_robot.disable_mouse_rotation()
        self.canvas.mpl_connect(
            "button_release_event",
            self.on_view_changed
        )
        self.fig.subplots_adjust(left=0.05, right=0.95, wspace=0.35)
    
    def go_home(self):
        for i in range(6):
            self.q_inputs[i].setValue(
                self.home_q[i]
            )

        self.apply_configuration()
    
    def on_spinbox_changed(self):

        if self.auto_apply_checkbox.isChecked():

            q_values = [
                self.q_inputs[i].value()
                for i in range(6)
            ]

            self.compute_ctr_configuration(q_values)

            self.current_q = list(q_values)
            
    def apply_configuration(self):

        q_values = [
            self.q_inputs[i].value()
            for i in range(6)
        ]

        self.animate_to_configuration(
            q_values
        )


    def compute_ctr_configuration(self, q_values):


        try:

            project_parent = os.path.dirname(root_toolkit)

            executable = os.path.join(
                project_parent,
                "Modeling-and-Control-of-Concentric-Tube-Continuum-Robots",
                "build",
                "demo",
                "004_interactive_control"
            )

            cmd = [executable] + [ # q_value contient float et subprocess.run() attend str
                str(q)
                for q in q_values
            ]
            print(executable) # VÉRIF

            build_dir = os.path.join(
                project_parent,
                "Modeling-and-Control-of-Concentric-Tube-Continuum-Robots",
                "build"
            )

            result = subprocess.run(
                cmd,
                cwd=build_dir,
                capture_output=True,
                text=True
            )

            print(result.stdout)

            self.load_trajectory_data()

            self.current_step = 0

            self.update_plots()


        except Exception as e:

            print("Erreur :", e)
    
    def animate_to_configuration(self, target_q):

        n_steps = 10

        q_start = np.array(self.current_q)
        q_target = np.array(target_q)

        for alpha in np.linspace(
            0.0,
            1.0,
            n_steps + 1
        )[1:]:

            q_interp = (
                (1 - alpha) * q_start + alpha * q_target
            )

            self.compute_ctr_configuration(
                q_interp.tolist()
            )

            QApplication.processEvents()

        self.current_q = list(target_q)
        
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


    def on_view_changed(self, event):

        self.saved_elev = self.ax_robot.elev
        self.saved_azim = self.ax_robot.azim


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

        # print("num_nodes =", num_nodes)
        # print("n1 =", n1)
        # print("n2 =", n2)
        # print("tube_end =", tube_end)

        current_elev = getattr(self, "saved_elev", 20)
        current_azim = getattr(self, "saved_azim", -45)

        self.ax_robot.cla()
        self.ax_plots.cla()
        self.ax_robot.view_init(elev=current_elev, azim=current_azim)


        # --- A. RENDU DU ROBOT 3D OPTIMIZED---

        xyz = matrix_data[0:3, :]

        x = xyz[0]
        y = xyz[1]
        z = xyz[2]

        # --- DÉBUGGING ---
        print("\n========== DEBUG CTR ==========")

        print("q =", [f"{v:.6f}" for v in self.current_q])

        print("shape =", matrix_data.shape)

        print("z min =", np.min(z))
        print("z max =", np.max(z))

        print("num_nodes =", num_nodes)

        print("base =", x[0], y[0], z[0])
        print("tip  =", x[-1], y[-1], z[-1])

        print("tube 1 end =", x[n1-1], y[n1-1], z[n1-1])
        print("tube 2 end =", x[n2-1], y[n2-1], z[n2-1])

        print("n1 =", n1)
        print("n2 =", n2)

        print(
            "point n1 =",
            x[n1-1],
            y[n1-1],
            z[n1-1]
        )

        print(
            "point n2 =",
            x[n2-1],
            y[n2-1],
            z[n2-1]
        )

        print(
            "finite =",
            np.isfinite(matrix_data).all()
        )

        print("===============================\n")
        # print("------ POINTS CLÉS ------")
        # print("0      :", x[0], y[0], z[0])
        # print("n1-1   :", x[n1-1], y[n1-1], z[n1-1])
        # print("n2-1   :", x[n2-1], y[n2-1], z[n2-1])
        # print("last   :", x[-1], y[-1], z[-1])
        # print("-------------------------")

        scale = 2

        # Tube externe
        self.ax_robot.plot(
            x[:n1],
            y[:n1],
            z[:n1],
            linewidth=1.524 * scale,
            color='black'
        )

        # Tube intermédiaire
        self.ax_robot.plot(
            x[:n2],
            y[:n2],
            z[:n2],
            linewidth=1.800 * scale,
            color='dimgray'
        )

        # Tube interne
        self.ax_robot.plot(
            x,
            y,
            z,
            linewidth=2.350 * scale,
            color='lightgray'
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

        self.ax_robot.set_title("Modélisation CTR en 3D")

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
        # print("total :", round((t1 - t0) * 1000, 1), "ms")
        # print("----------------")
        self.canvas.draw_idle()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(app.exec_())