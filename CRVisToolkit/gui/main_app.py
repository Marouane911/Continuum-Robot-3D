import sys
import os
import numpy as np
import time
import subprocess

from PyQt5.QtWidgets import QCheckBox, QDoubleSpinBox, QSpinBox, QFileDialog, QMessageBox
from PyQt5.QtCore import QLocale


# Gestion des imports PyQt5 et Matplotlib
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QComboBox
from PyQt5.QtCore import QTimer, Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtWidgets import (
    QGroupBox,
    QGridLayout,
    QScrollArea
)
from matplotlib.ticker import MultipleLocator


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
        self.setWindowTitle("CTR VISUALIZATION TOOL")
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

        # Historique de la trajectoire de la pointe
        self.tip_path_x = []
        self.tip_path_y = []
        self.tip_path_z = []

        # --- Initialisation de l'interface GUI ---
        self.init_ui()

        # Historique orientation pointe
        self.tip_orientation_history = {
            "x": [],
            "y": [],
            "z": []
        }

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
            lines = [
                l.strip()
                for l in b.split('\n') if l.strip()
            ]

            iEnd = None
            S = None

            data_lines = []

            for line in lines:

                if line.startswith("# iEnd"):

                    vals = line.split()[2:]

                    iEnd = np.array(
                        list(map(int, vals))
                    )

                elif line.startswith("# S"):

                    vals = line.split()[2:]

                    S = np.array(
                        list(map(float, vals))
                    )

                else:

                    data_lines.append(line)

                if len(data_lines) == 19:

                    lengths = [
                        len(line.split())
                        for line in data_lines
                    ]

                    # print("Nombre de colonnes par ligne :", lengths)

                    matrix_lines = [
                        list(map(float, line.split()))
                        for line in data_lines
                    ]

                    arr = np.array(matrix_lines)

                    # print("Shape chargée :", arr.shape)

                    z = arr[2,:]

                    # print(
                    #     "z range :",
                    #     np.min(z),
                    #     "->",
                    #     np.max(z)
                    # )


                    self.steps_data.append(
                        {
                            "matrix": arr,
                            "iEnd": iEnd,
                            "S": S
                        }
                    )

        print(f"[{len(self.steps_data)}] étapes d'animation chargées avec succès.")

    def init_ui(self):

        self.setStyleSheet("""
            QMainWindow {
                background-color: #eef2f7;
            }

            QGroupBox {
                background-color: white;
                border: 1px solid #d9dee7;
                border-radius: 8px;
                margin-top: 8px;
                font-weight: bold;
                padding-top: 12px;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0px 5px;
            }

            QPushButton {
                min-height: 36px;
                border-radius: 6px;
            }

            QDoubleSpinBox,
            QSpinBox,
            QComboBox {
                min-height: 30px;
            }

            QLabel {
                font-size: 12px;
            }
        """)

        main_widget = QWidget()
        self.setCentralWidget(main_widget)

        main_layout = QHBoxLayout(main_widget)

        # ==========================================================
        # PANEL GAUCHE SCROLLABLE
        # ==========================================================

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFixedWidth(380)

        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)

        scroll.setWidget(control_panel)

        main_layout.addWidget(scroll)

        # ==========================================================
        # HEADER
        # ==========================================================

        self.status_label = QLabel(
            f"Étape : 0 / {len(self.steps_data)}"
        )

        self.status_label.setStyleSheet("""
            font-size: 15px;
            font-weight: bold;
        """)

        control_layout.addWidget(self.status_label)

        # ==========================================================
        # ANIMATION
        # ==========================================================

        animation_group = QGroupBox("Animation")

        animation_layout = QVBoxLayout()

        # Bouton mémoriser Home
        self.btn_set_home = QPushButton(
            "Mémoriser position actuelle"
        )


        self.btn_set_home.setStyleSheet("""
            background-color:#b6b8b6;
            color:white;
            font-size:14px;
        """)

        self.btn_set_home.clicked.connect(
            self.set_current_as_home
        )

        animation_layout.addWidget(
            self.btn_set_home
        )

        # Bouton retour Home
        self.btn_home = QPushButton(
            "Position Home"
        )

        self.btn_home.setStyleSheet("""
            background-color:#B6B8B6;
            color:white;
            font-size:14px;
        """)

        self.btn_home.clicked.connect(
            self.go_home
        )

        animation_layout.addWidget(
            self.btn_home
        )

        animation_group.setLayout(
            animation_layout
        )

        control_layout.addWidget(
            animation_group
        )
        # ==========================================================
        # DIAGNOSTIC
        # ==========================================================

        diagnostic_group = QGroupBox(
            "Diagnostic"
        )

        diagnostic_layout = QVBoxLayout()

        self.robot_state_label = QLabel(
            "🟢 OK"
        )

        self.robot_state_label.setStyleSheet("""
            color:green;
            font-size:14px;
            font-weight:bold;
        """)

        diagnostic_layout.addWidget(
            self.robot_state_label
        )

        self.tip_info_label = QLabel(
            "Tip:\n"
            "x = --- mm\n"
            "y = --- mm\n"
            "z = --- mm\n\n"
            "Orientation:\n"
            "X = ---°\n"
            "Y = ---°\n"
            "Z = ---°"
        )

        self.tip_info_label.setStyleSheet("""
            border:1px solid #d9dee7;
            background:white;
            padding:8px;
            font-size:11px;
        """)

        diagnostic_layout.addWidget(
            self.tip_info_label
        )

        self.error_label = QLabel("")

        self.error_label.setWordWrap(True)

        diagnostic_layout.addWidget(
            self.error_label
        )

        diagnostic_group.setLayout(
            diagnostic_layout
        )

        control_layout.addWidget(
            diagnostic_group
        )

        # ==========================================================
        # ACTIONNEURS
        # ==========================================================

        actuators_group = QGroupBox(
            "Actionneurs CTR"
        )

        actuators_layout = QGridLayout()

        self.q_inputs = []

        default_values = [
            -0.30,
            -0.20,
            -0.10,
            0.0,
            0.0,
            0.0
        ]

        q_labels = [
            "Translation Tube 1",
            "Translation Tube 2",
            "Translation Tube 3",
            "Rotation Tube 1",
            "Rotation Tube 2",
            "Rotation Tube 3"
        ]

        for i in range(6):

            actuators_layout.addWidget(
                QLabel(q_labels[i]),
                i,
                0
            )

            spinbox = QDoubleSpinBox()

            spinbox.setLocale(
                QLocale(
                    QLocale.English,
                    QLocale.UnitedStates
                )
            )

            if i < 3:

                spinbox.setRange(
                    -0.40,
                    0.05
                )

                spinbox.setDecimals(4)

                spinbox.setSingleStep(
                    0.005
                )

            else:

                spinbox.setRange(
                    -6.28318,
                    6.28318
                )

                spinbox.setDecimals(
                    3
                )

                spinbox.setSingleStep(
                    0.1
                )

            spinbox.setValue(
                default_values[i]
            )

            spinbox.valueChanged.connect(
                self.on_spinbox_changed
            )

            self.q_inputs.append(
                spinbox
            )

            actuators_layout.addWidget(
                spinbox,
                i,
                1
            )

        actuators_group.setLayout(
            actuators_layout
        )

        control_layout.addWidget(
            actuators_group
        )

        self.home_q = [
            -0.30,
            -0.20,
            -0.10,
            0.0,
            0.0,
            0.0
        ]

        # ==========================================================
        # CONFIGURATION
        # ==========================================================

        config_group = QGroupBox(
            "Configuration"
        )

        config_layout = QVBoxLayout()

        self.auto_apply_checkbox = QCheckBox(
            "Appliquer en temps réel"
        )

        config_layout.addWidget(
            self.auto_apply_checkbox
        )

        self.tip_path_checkbox = QCheckBox(
            "Afficher trajectoire pointe"
        )

        self.tip_path_checkbox.setChecked(True)

        self.tip_path_checkbox.toggled.connect(
            self.update_plots
        )

        control_layout.addWidget(
            self.tip_path_checkbox
        )

        config_layout.addWidget(
            self.tip_path_checkbox
        )


        config_layout.addWidget(
            QLabel("Nombre d'étapes")
        )

        self.steps_spinbox = QSpinBox()

        self.steps_spinbox.setRange(
            1,
            500
        )

        self.steps_spinbox.setValue(
            25
        )

        config_layout.addWidget(
            self.steps_spinbox
        )

        self.btn_apply = QPushButton(
            "Appliquer configuration"
        )

        self.btn_apply.setStyleSheet("""
            background-color:#1976D2;
            color:white;
            font-weight:bold;
        """)

        self.btn_apply.clicked.connect(
            self.apply_configuration
        )

        config_layout.addWidget(
            self.btn_apply
        )

        config_group.setLayout(
            config_layout
        )

        control_layout.addWidget(
            config_group
        )

        # ==========================================================
        # GRAPHIQUES
        # ==========================================================

        graph_group = QGroupBox(
            "Graphiques"
        )

        graph_layout = QVBoxLayout()

        self.graph_selector = QComboBox()

        self.graph_selector.addItems([
            "Orientation of the local tangent along the CTR",
            "Tip orientation history",
            "Accumulated relative twist from base (MICRO view)"
        ])

        self.graph_selector.currentIndexChanged.connect(
            self.update_plots
        )

        graph_layout.addWidget(
            self.graph_selector
        )

        self.btn_save_img = QPushButton(
            "Sauvegarder l'image"
        )

        self.btn_save_img.setStyleSheet("""
            background-color:#2196F3;
            color:white;
        """)

        self.btn_save_img.clicked.connect(
            self.save_plot
        )

        graph_layout.addWidget(
            self.btn_save_img
        )

        graph_group.setLayout(
            graph_layout
        )

        control_layout.addWidget(
            graph_group
        )

        control_layout.addStretch()

        # ==========================================================
        # ZONE MATPLOTLIB
        # ==========================================================

        self.fig = Figure(
            figsize=(12, 8)
        )

        self.canvas = FigureCanvas(
            self.fig
        )

        main_layout.addWidget(
            self.canvas,
            stretch=5
        )

        self.ax_robot = self.fig.add_subplot(
            121,
            projection='3d'
        )

        self.ax_plots = self.fig.add_subplot(
            122
        )

        self.ax_robot.view_init(
            elev=20,
            azim=-45
        )

        self.canvas.mpl_connect(
            "button_release_event",
            self.on_view_changed
        )

        self.fig.subplots_adjust(
            left=0.05,
            right=0.95,
            wspace=0.35
        )

    def set_current_as_home(self):

        self.home_q = [
            spinbox.value()
            for spinbox in self.q_inputs
        ]

        print("Nouvelle position Home :", self.home_q)

    def go_home(self):

        self.tip_path_x.clear()
        self.tip_path_y.clear()
        self.tip_path_z.clear()

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

        q_values = [ # Permet la génération de plusieurs positions intermédiaire
            self.q_inputs[i].value()
            for i in range(6)
        ]

        self.animate_to_configuration(
            q_values
        )


    def set_status_ok(self):

        self.robot_state_label.setText(
            "🟢 OK"
        )

        self.robot_state_label.setWordWrap(True) # sinon le message sera tronqué

        self.robot_state_label.setStyleSheet(
            """
            color: green;
            font-weight: bold;
            font-size: 14px;
            """
        )

        self.error_label.setText("")


    def set_status_collision(self, message):

        self.robot_state_label.setText(
            f"🔴 Collision : {message}"
        )

        self.robot_state_label.setStyleSheet(
            """
            color: red;
            font-weight: bold;
            font-size: 14px;
            """
        )

        self.error_label.setText("")

    def set_status_divergence(self, message):

        self.robot_state_label.setText(
            f"🟠 Solver divergence : {message}"
        )

        self.robot_state_label.setStyleSheet(
            """
            color: orange;
            font-weight: bold;
            font-size: 14px;
            """
        )

        self.error_label.setText("")
    
    def save_plot(self):

        options = QFileDialog.Options()

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Sauvegarder la fenêtre",
            "ctr_visualization.png",
            "PNG Image (*.png);;JPEG Image (*.jpg *.jpeg)"
        )

        if not file_path:
            return

        try:

            # Capture de toute la fenêtre
            pixmap = self.grab()
            pixmap = pixmap.scaled(
                pixmap.width() * 2,
                pixmap.height() * 2,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            pixmap.save(file_path)

            QMessageBox.information(
                self,
                "Sauvegarde réussie",
                f"La fenêtre complète a été enregistrée.\n\n{file_path}"
            )

        except Exception as e:

            QMessageBox.critical(
                self,
                "Erreur",
                str(e)
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



            collision = False
            divergence = False

            for line in result.stdout.splitlines():

                lower_line = line.lower()

                if "clashing" in lower_line:

                    collision = True
                    self.set_status_collision(line)
                    break

                if "failed to converge" in lower_line:

                    divergence = True
                    self.set_status_divergence(line)
                    break


            if collision or divergence:
                return

            self.set_status_ok()

            self.load_trajectory_data()

            self.current_step = 0

            self.update_plots()

        except Exception as e:

            print("Erreur :", e)

    def animate_to_configuration(self, target_q): # Creér plusieurs étapes entres 2 positions (can be deleted or edit n_steps = 1)

        self.tip_orientation_history = {
            "x": [],
            "y": [],
            "z": []
        }

        n_steps = self.steps_spinbox.value()

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


    def on_view_changed(self, event):

        self.saved_elev = self.ax_robot.elev
        self.saved_azim = self.ax_robot.azim



    def update_plots(self):
        t0 = time.perf_counter()
        if not self.steps_data or self.current_step >= len(self.steps_data):
            return

        # Récupération de la matrice (19, N) pré-filtrée
        step = self.steps_data[self.current_step]

        matrix_data = step["matrix"]
        iEnd = step["iEnd"]
        S = step["S"]

        num_nodes = matrix_data.shape[1]

        xyz = matrix_data[0:3, :]

        x = xyz[0]
        y = xyz[1]
        z = xyz[2]

        # Calcul de la longueur curviligne réelle
        xyz = matrix_data[0:3, :].T

        dxyz = np.diff(xyz, axis=0)

        dl = np.linalg.norm(dxyz, axis=1)

        length_axis = np.concatenate(([0], np.cumsum(dl)))


        # Longueurs physiques des extrémités

        s_ext = S[iEnd[2]]
        s_mid = S[iEnd[1]]

        # Indices réels correspondants dans le backbone

        end_ext = np.argmin(
            np.abs(length_axis - s_ext)
        ) + 1

        end_mid = np.argmin(
            np.abs(length_axis - s_mid)
        ) + 1

        end_int = num_nodes


        # Tentative debugging affichage

        current_elev = getattr(self, "saved_elev", 20)
        current_azim = getattr(self, "saved_azim", -45)

        self.ax_robot.cla()
        self.ax_plots.cla()
        self.ax_robot.view_init(elev=current_elev, azim=current_azim)


        # --- A. RENDU DU ROBOT 3D OPTIMIZED---

        scale = 1000

        # Tube externe
        self.ax_robot.plot(
            x[:end_ext],
            y[:end_ext],
            z[:end_ext],
            linewidth= (2*self.r_tube[0]) * scale,
            color='black'
        )

        # Tube intermédiaire
        self.ax_robot.plot(
            x[end_ext - 1 : end_mid],
            y[end_ext - 1 : end_mid],
            z[end_ext - 1 : end_mid],
            linewidth=(2*self.r_tube[1]) * scale,
            color='dimgray'
        )

        # Tube interne
        self.ax_robot.plot(
            x[end_mid - 1 : end_int],
            y[end_mid - 1 : end_int],
            z[end_mid - 1 : end_int],
            linewidth= (2*self.r_tube[2]) * scale,
            color='lightgray'
        )

        # Pointe
        self.ax_robot.scatter(
            x[-1],
            y[-1],
            z[-1],
            s=40
        )

        # Trajectoire historique de la pointe
        if (
            self.tip_path_checkbox.isChecked()
            and len(self.tip_path_x) > 1
        ):
            self.ax_robot.plot(
                self.tip_path_x,
                self.tip_path_y,
                self.tip_path_z,
                '--',
                linewidth=2,
                color='dodgerblue',
                alpha=0.8,
                label="Tip path"
            )

        # Calcul de l'orientation pointe
        R_tip = matrix_data[3:12, -1].reshape((3, 3), order='C')

        t_x = R_tip[0, 2]
        t_y = R_tip[1, 2]
        t_z = R_tip[2, 2]

        tip_x = matrix_data[0, -1]
        tip_y = matrix_data[1, -1]
        tip_z = matrix_data[2, -1]

        # Historique de la pointe, on évite de stocker 50 fois le même point
        if len(self.tip_path_x) == 0:
            self.tip_path_x.append(tip_x)
            self.tip_path_y.append(tip_y)
            self.tip_path_z.append(tip_z)

        elif (
            abs(tip_x - self.tip_path_x[-1]) > 1e-6
            or abs(tip_y - self.tip_path_y[-1]) > 1e-6
            or abs(tip_z - self.tip_path_z[-1]) > 1e-6
        ):
            self.tip_path_x.append(tip_x)
            self.tip_path_y.append(tip_y)
            self.tip_path_z.append(tip_z)

        tip_angle_x = np.degrees(
            np.arccos(np.clip(t_x, -1.0, 1.0))
        )

        tip_angle_y = np.degrees(
            np.arccos(np.clip(t_y, -1.0, 1.0))
        )

        tip_angle_z = np.degrees(
            np.arccos(np.clip(t_z, -1.0, 1.0))
        )

        # Stock historiques XYZ de la pointe pour graphique
        self.tip_orientation_history["x"].append(tip_angle_x)
        self.tip_orientation_history["y"].append(tip_angle_y)
        self.tip_orientation_history["z"].append(tip_angle_z)

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

        # AFFICHAGE 3d CTR
        self.ax_robot.set_title("Modélisation CTR en 3D")

        # 1. Calcul des limites dynamiques basées sur la géométrie réelle du robot
        # On prend le max absolu en X et Y pour garder un repère centré et carré
        max_xy = max(np.max(np.abs(x)), np.max(np.abs(y)), 0.05) * 1.2
        # On ajuste le Z max avec une marge de 10% au-dessus de la pointe
        max_z = max(np.max(z), 0.10) * 1.1

        # 2. Application des nouvelles limites adaptatives
        self.ax_robot.set_xlim(-max_xy, max_xy)
        self.ax_robot.set_ylim(-max_xy, max_xy)
        self.ax_robot.set_zlim(0.0, max_z)

        # 3. Forcer le ratio 1:1:1 pour éviter les déformations visuelles du robot
        self.ax_robot.set_box_aspect((1, 1, 1))

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

            self.ax_plots.plot(length_axis, orient_X, 'r-', linewidth=1, label="Axe X")
            self.ax_plots.plot(length_axis, orient_Y, 'g-', linewidth=1, label="Axe Y")
            self.ax_plots.plot(length_axis, orient_Z, 'b-', linewidth=1, label="Axe Z")
            self.ax_plots.set_title("Orientation of the local tangent along the CTR", fontsize=11, fontweight='bold')
            self.ax_plots.set_ylabel("Angle de référence (degrés)", labelpad=12)
            self.ax_plots.set_ylim([0, 120])

        elif selected_graph == 1:

            steps = np.arange(
                len(self.tip_orientation_history["x"])
            )

            self.ax_plots.plot(
                steps,
                self.tip_orientation_history["x"],
                'r-',
                label="Tip / X"
            )

            self.ax_plots.plot(
                steps,
                self.tip_orientation_history["y"],
                'g-',
                label="Tip / Y"
            )

            self.ax_plots.plot(
                steps,
                self.tip_orientation_history["z"],
                'b-',
                label="Tip / Z"
            )

            self.ax_plots.set_title(
                "Tip orientation during motion"
            )

            self.ax_plots.set_xlabel(
                "Motion step"
            )

            self.ax_plots.set_ylabel(
                "Angle (deg)"
            )



        elif selected_graph == 2:

            # Torsion relative
            theta_2 = matrix_data[17, :]
            theta_3 = matrix_data[18, :]

            # fin des theta 2 et 3
            # theta_2end = theta_2[-1]
            # theta_3end = theta_3[-1]


            t2_display = np.copy(theta_2)
            t3_display = np.copy(theta_3)

            t3_display[end_ext:] = np.nan
            t2_display[end_mid:] = np.nan

            self.ax_plots.plot(length_axis, t2_display, 'b-', markersize=2, label=r"$\theta_2(s)$")
            self.ax_plots.plot(length_axis, t3_display, 'r-', markersize=2, label=r"$\theta_3(s)$")

            l_transition1 = length_axis[end_ext - 1]
            l_transition2 = length_axis[end_mid - 1]

            self.ax_plots.axvline(x=l_transition1, color='orange', linestyle='--', alpha=0.7, label="End T3 (Ext)")
            self.ax_plots.axvline(x=l_transition2, color='purple', linestyle='--', alpha=0.7, label="End T2 (Mid)")

            # Zones de précourbure
            start_curve_t3 = l_transition1 - self.l_kappa
            if start_curve_t3 > 0:
                self.ax_plots.axvline(x=start_curve_t3, color='darkorange', linestyle=':', linewidth=2.5,
                                      label=r"Start curve T3")
                self.ax_plots.axvspan(start_curve_t3, l_transition1, color='orange', alpha=0.07, zorder=1)

            start_curve_t2 = l_transition2 - self.l_kappa
            if start_curve_t2 > 0:
                self.ax_plots.axvline(x=start_curve_t2, color='purple', linestyle=':', linewidth=2.5,
                                      label=r"Start curve T2")
                self.ax_plots.axvspan(start_curve_t2, l_transition2, color='purple', alpha=0.04, zorder=1)

            l_end_t1 = length_axis[-1]
            start_curve_t1 = l_end_t1 - self.l_kappa
            if start_curve_t1 > 0:
                self.ax_plots.axvline(x=start_curve_t1, color='green', linestyle=':', linewidth=2.5,
                                      label=r"Start curve T1 (Int)")
                self.ax_plots.axvspan(start_curve_t1, l_end_t1, color='green', alpha=0.03, zorder=1)
                # Ligne de fin désormais visible grâce à la marge de 5%
                self.ax_plots.axvline(x=l_end_t1, color='green', linestyle='--', alpha=0.7, label="End T1 (Tip)")


            self.ax_plots.set_title("Angles de torsion des tubes le long du CTR", fontsize=11, fontweight='bold')
            self.ax_plots.set_ylabel("Twist angle (rad)", labelpad=10)

            # Limites dynamiques
            min_y = np.nanmin([t2_display, t3_display])
            max_y = np.nanmax([t2_display, t3_display])

            self.ax_plots.set_ylim(
                min_y - 0.05,
                max_y + 0.05
            )

            # Préviens le cas où les torsions sont petites
            margin = 0.1 * (max_y - min_y)

            if margin < 1e-3:
                margin = 1e-3

            self.ax_plots.set_ylim(
                min_y - margin,
                max_y + margin
            )

            # #DEBUGGING VERIF
            # print("----------------")
            # print(f"np.min(theta_2) = {np.min(theta_2)} \n np.max(theta_2) = {np.max(theta_2)} \n")
            # print(f"np.min(theta_3) = {np.min(theta_3)} \n np.max(theta_3) = {np.max(theta_3)} \n")
            # print(f"theta_2[:10] = {theta_2[:10]} \n theta_2[-10:] = {theta_2[-10:]} \n")
            # print(f"theta_3[:10] = {theta_3[:10]} \n theta_3[-10:] = {theta_3[-10:]} \n")
            # print("theta_2(end) =", theta_2[-1])
            # print("theta_3(end) =", theta_3[-1])
            # # print(f"u_z = {matrix_data[16,:]} \n")
            # print("----------------")

            self.ax_plots.scatter(
                length_axis[end_mid-1],
                theta_2[end_mid-1],
                color='blue',
                s=40
            )

            self.ax_plots.scatter(
                length_axis[end_ext-1],
                theta_3[end_ext-1],
                color='red',
                s=40
            )

            self.ax_plots.annotate(
                f"{theta_2[end_mid-1]:.4f} rad",
                (
                    length_axis[end_mid-1],
                    theta_2[end_mid-1]
                ),
                xytext=(10,-15),
                textcoords="offset points",
                color="blue"
            )

            self.ax_plots.annotate(
                f"{theta_3[end_ext-1]:.4f} rad",
                (
                    length_axis[end_ext-1],
                    theta_3[end_ext-1]
                ),
                xytext=(10,-15),
                textcoords="offset points",
                color="red"
            )



        # --- CORRECTION DES QUADRILLAGES ---
        self.ax_plots.grid(True, linestyle=':', alpha=0.6, zorder=2)

        if selected_graph != 1:

            self.ax_plots.set_xlabel(
                "Longueur du robot (m)"
            )

            self.ax_plots.set_xlim(
                [0, length_axis[-1] * 1.05]
            )

            self.ax_plots.grid(
                True,
                linestyle=':',
                alpha=0.6,
                zorder=2
            )

            self.ax_plots.legend(
                loc="upper right",
                fontsize=9,
                framealpha=0.9
            )

        self.fig.subplots_adjust(left=0.05, right=0.95, wspace=0.35)
        t1 = time.perf_counter()
        print("total :", round((t1 - t0) * 1000, 1), "ms")
        print("----------------")

        # Mise à jour du panneau
        self.tip_info_label.setText(
            f"Tip:\n"
            f"x = {tip_x*1000:.1f} mm\n"
            f"y = {tip_y*1000:.1f} mm\n"
            f"z = {tip_z*1000:.1f} mm\n\n"
            f"Orientation:\n"
            f"X = {tip_angle_x:.1f}°\n"
            f"Y = {tip_angle_y:.1f}°\n"
            f"Z = {tip_angle_z:.1f}°"
        )
        self.canvas.draw_idle()
        # print(matrix_data[3:12,0])
        # print(matrix_data[3:12,-1])


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(app.exec_())