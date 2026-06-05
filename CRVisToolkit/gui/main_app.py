import sys
import os
import numpy as np
import subprocess
 
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QComboBox,
    QCheckBox, QDoubleSpinBox, QSpinBox,
    QFileDialog, QMessageBox,
    QGroupBox, QScrollArea
)
from PyQt5.QtCore import Qt, QLocale
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from scipy.spatial.transform import Rotation as Rot
 
from ctr_data import CTRData

from ctr_visualizer import CTRVisualizer

from ctr_graph import CTRGraphs



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

        self.ghost_robot = None

        # Historique de la trajectoire de la pointe
        self.tip_path_x = []
        self.tip_path_y = []
        self.tip_path_z = []

        # --- Initialisation de l'interface GUI ---
        self.init_ui()

        self.visualizer = CTRVisualizer(
            self.ax_robot
        )

        # Historique orientation pointe
        self.tip_orientation_history = {
            "x": [],
            "y": [],
            "z": []
        }

        self.home_q = [
            -0.30,
            -0.20,
            -0.10,
            0.0,
            0.0,
            0.0
        ]
        

        self.record_orientation_history = False

        # --- Mémoriser la dérnière configuration pour l'animation entre 2 positions entrées ---
        self.current_q = [
            -0.30,
            -0.20,
            -0.10,
            0.0,
            0.0,
            0.0
        ]

        self.last_valid_q = self.current_q.copy()

        self.saved_elev = 20
        self.saved_azim = -45

        self.compute_ctr_configuration(self.current_q)

        self.set_current_as_home()



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

                    matrix_lines = [
                        list(map(float, line.split()))
                        for line in data_lines
                    ]

                    arr = np.array(matrix_lines)

                    self.steps_data.append(
                        {
                            "matrix": arr,
                            "iEnd": iEnd,
                            "S": S
                        }
                    )







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

        self.btn_last_valid = QPushButton(
            "Dernière position valide"
        )

        self.btn_last_valid.setStyleSheet("""
            background-color:#FF9800;
            color:white;
            font-size:14px;
        """)

        self.btn_last_valid.clicked.connect(
            self.go_last_valid
        )

        animation_layout.addWidget(
            self.btn_last_valid
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
            "Organe terminal:\n"
            "x = --- mm\n"
            "y = --- mm\n"
            "z = --- mm\n\n"
            "Orientation terminale:\n"
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
                lambda _, idx=i:
                self.on_spinbox_changed(idx)
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
            QLabel("Nombre d'étapes")
        )

        self.steps_spinbox = QSpinBox()

        self.steps_spinbox.setRange(
            1,
            500
        )

        self.steps_spinbox.setValue(
            2
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


        # ghost robot
        self.ghost_checkbox = QCheckBox(
            "Afficher dernière position mémorisé"
        )

        self.ghost_checkbox.setChecked(False)

        self.ghost_checkbox.toggled.connect(
            self.update_plots
        )

        config_layout.addWidget(
            self.ghost_checkbox
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

        self.graphs = CTRGraphs(
            self.ax_plots,
            self.l_kappa
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


    # Gestion de la correction telescopique

    def enforce_telescopic_constraints(self,q,active):

        # Longueurs tubes
        l1 = 0.463
        l2 = 0.3305
        l3 = 0.199

        ### Convention générale pour toutes les positions
        
        # Tube 1 = interne
        # Tube 2 = intermédiaire
        # Tube 3 = externe
        
        # beta1 < beta2 < beta3
        
        # tip1 > tip2 > tip3
        
        ###

        eps = 0.005

        q = list(q)

        # -------------------
        # Tube 1 déplacé
        # -------------------

        if active == 0:

            if q[1] < q[0] + eps:
                q[1] = q[0] + eps

            if q[2] < q[1] + eps:
                q[2] = q[1] + eps

        # -------------------
        # Tube 2 déplacé
        # -------------------

        elif active == 1:

            if q[1] < q[0] + eps:
                q[1] = q[0] + eps

            if q[2] < q[1] + eps:
                q[2] = q[1] + eps

        # -------------------
        # Tube 3 déplacé
        # -------------------

        elif active == 2:

            if q[2] < q[1] + eps:
                q[2] = q[1] + eps
        

        # Calcul des positions des pointes
        tip1 = l1 + q[0]
        tip2 = l2 + q[1]
        tip3 = l3 + q[2]

        # -------------------
        # Collision distale tube 1 / tube 2
        # -------------------

        if tip1 <= tip2 + eps:

            q[1] = q[0] + (l1 - l2) - eps

            tip2 = l2 + q[1]

        # -------------------
        # Collision distale tube 2 / tube 3
        # -------------------

        if tip2 <= tip3 + eps:

            q[2] = q[1] + (l2 - l3) - eps

            tip3 = l3 + q[2]

        return q




    def go_last_valid(self):

        for i in range(6):
            
            self.q_inputs[i].blockSignals(True) # signaux

            self.q_inputs[i].setValue(
                self.last_valid_q[i]
            )
            
            self.q_inputs[i].blockSignals(False) # signaux

        self.compute_ctr_configuration(
            self.last_valid_q
        )

        self.current_q = self.last_valid_q.copy()

        print("Retour à la dernière position valide")

        
    def set_current_as_home(self):

        self.home_q = [
            spinbox.value()
            for spinbox in self.q_inputs
        ]

        if self.steps_data:

            step = self.steps_data[self.current_step]

            matrix_data = step["matrix"]
            iEnd = step["iEnd"]
            S = step["S"]

            xyz = matrix_data[0:3, :].T

            dxyz = np.diff(xyz, axis=0)

            dl = np.linalg.norm(dxyz, axis=1)

            length_axis = np.concatenate(([0], np.cumsum(dl)))

            s_ext = S[iEnd[2]]
            s_mid = S[iEnd[1]]

            end_ext = np.argmin(
                np.abs(length_axis - s_ext)
            ) + 1

            end_mid = np.argmin(
                np.abs(length_axis - s_mid)
            ) + 1

            end_int = matrix_data.shape[1]

            x = matrix_data[0,:].copy()
            y = matrix_data[1,:].copy()
            z = matrix_data[2,:].copy()

            self.ghost_robot = {
                "x": x,
                "y": y,
                "z": z,
                "end_ext": end_ext,
                "end_mid": end_mid,
                "end_int": end_int
            }

    def go_home(self):

        self.tip_path_x.clear()
        self.tip_path_y.clear()
        self.tip_path_z.clear()

        for i in range(6):
            self.q_inputs[i].setValue(
                self.home_q[i]
            )

        self.apply_configuration()

        
    def on_spinbox_changed(self, active_index):

        self.active_actuator = active_index

        if self.auto_apply_checkbox.isChecked():

            q_values = [
                self.q_inputs[i].value()
                for i in range(6)
            ]

            q_values = self.enforce_telescopic_constraints(
                q_values,
                active_index
            )

            for i in range(6):
                self.q_inputs[i].blockSignals(True)
                self.q_inputs[i].setValue(q_values[i])
                self.q_inputs[i].blockSignals(False)

            self.compute_ctr_configuration(q_values)

            self.current_q = q_values.copy()

    def apply_configuration(self):

        q_values = [ # Permet la génération de plusieurs positions intermédiaire
            self.q_inputs[i].value()
            for i in range(6)
        ]

        active = getattr(
                self,
                "active_actuator",
                0
        )
        
        q_values = self.enforce_telescopic_constraints(
            q_values,
            active
        )

        for i in range(6):
            self.q_inputs[i].blockSignals(True)
            self.q_inputs[i].setValue(q_values[i])
            self.q_inputs[i].blockSignals(False)

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
            f"🔴 Collision détectée :\n"
            f"{message}\n"
            f"Dernière configuration valide mémorisée.\n"
            f'Cliquer sur "Dernière position valide".'
        )

        self.robot_state_label.setWordWrap(True)

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
            f"🟠 Divergence du solveur :\n"
            f"{message}\n"
            f"Dernière configuration valide mémorisée.\n"
            f'Cliquer sur "Dernière position valide".'
        )

        self.robot_state_label.setWordWrap(True)

        self.robot_state_label.setStyleSheet(
            """
            color: orange;
            font-weight: bold;
            font-size: 14px;
            """
        )

        self.error_label.setText("")
    
    def save_plot(self):

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

            # Appel du solveur
            result = subprocess.run(
                cmd,
                cwd=build_dir,
                capture_output=True,
                text=True
            )

            print(result.stdout)

            for line in result.stdout.splitlines():

                lower_line = line.lower()

                if "clashing" in lower_line:

                    self.set_status_collision(line)
                    return False

                if "failed to converge" in lower_line:

                    self.set_status_divergence(line)
                    return False


            self.set_status_ok()

            self.load_trajectory_data()

            self.current_step = 0

            self.update_plots()

            return True

        except Exception as e:

            print("Erreur :", e)

    def animate_to_configuration(self, target_q):

        self.record_orientation_history = True

        self.tip_orientation_history = {
            "x": [],
            "y": [],
            "z": []
        }

        n_steps = self.steps_spinbox.value()

        self.status_label.setText(
            f"Étape : 0 / {n_steps}"
        )

        q_start = np.array(self.current_q)
        q_target = np.array(target_q)

        # dernière configuration valide connue
        last_valid_q = q_start.copy()

        for step_id, alpha in enumerate(
            np.linspace(0.0, 1.0, n_steps + 1)[1:],
            start=1
        ):

            q_interp = (
                (1 - alpha) * q_start +
                alpha * q_target
            )

            self.status_label.setText(
                f"Étape : {step_id} / {n_steps}"
            )

            success = self.compute_ctr_configuration(
                q_interp.tolist()
            )

            QApplication.processEvents()

            if not success:

                self.status_label.setText(
                    f"❌ Crash à l'étape {step_id}/{n_steps}"
                )

                self.current_q = list(last_valid_q)

                return

            last_valid_q = q_interp.copy()
            self.last_valid_q = q_interp.copy()

        self.status_label.setText(
            f"✅ Terminé : {n_steps}/{n_steps}"
        )

        self.current_q = list(target_q)

        self.record_orientation_history = False

    def on_view_changed(self, event):

        self.saved_elev = self.ax_robot.elev
        self.saved_azim = self.ax_robot.azim



    def update_plots(self):

        if not self.steps_data or self.current_step >= len(self.steps_data):
            return
        
        # === RÉCUPÉRATION DES TRANSLATIONS COURANTES ===
        q_current = [self.q_inputs[i].value() for i in range(3)]

        # Récupération de la matrice (19, N) pré-filtrée
        step = self.steps_data[self.current_step]

        data = CTRData(
            step["matrix"],
            step["iEnd"],
            step["S"]
        )

        matrix_data = data.matrix_data

        x = data.x
        y = data.y
        z = data.z

        length_axis = data.length_axis

        end_ext = data.end_ext
        end_mid = data.end_mid
        end_int = data.end_int

        num_nodes = data.num_nodes

        R_tip = data.R_tip

        tip_x = data.tip_x
        tip_y = data.tip_y
        tip_z = data.tip_z

        tip_angle_x = data.tip_angle_x
        tip_angle_y = data.tip_angle_y
        tip_angle_z = data.tip_angle_z

        theta_2 = data.theta_2
        theta_3 = data.theta_3

        # Tentative debugging affichage

        current_elev = getattr(self, "saved_elev", 20)
        current_azim = getattr(self, "saved_azim", -45)

        self.ax_robot.cla()
        self.ax_plots.cla()
        self.ax_robot.view_init(elev=current_elev, azim=current_azim)


        # RENDU DU ROBOT 3D

        self.visualizer.draw_robot(
            x,
            y,
            z,
            end_ext,
            end_mid,
            end_int,
            self.r_tube
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
                alpha=0.6,
                label="Tip path"
            )

        # Calcul de l'orientation pointe
        R_tip = matrix_data[3:12, -1].reshape((3, 3), order='C')

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


        rotation = Rot.from_matrix(R_tip)

        tip_angle_x, tip_angle_y, tip_angle_z = rotation.as_euler(
            'xyz',
            degrees=True
        )



        if len(self.tip_orientation_history["x"]) == 0:

            self.tip_orientation_history["x"].append(tip_angle_x)
            self.tip_orientation_history["y"].append(tip_angle_y)
            self.tip_orientation_history["z"].append(tip_angle_z)

        else:

            if (
                abs(self.tip_orientation_history["x"][-1] - tip_angle_x) > 1e-6
                or abs(self.tip_orientation_history["y"][-1] - tip_angle_y) > 1e-6
                or abs(self.tip_orientation_history["z"][-1] - tip_angle_z) > 1e-6
            ):

                self.tip_orientation_history["x"].append(tip_angle_x)
                self.tip_orientation_history["y"].append(tip_angle_y)
                self.tip_orientation_history["z"].append(tip_angle_z)


        # Axes du repère

        self.visualizer.draw_world_frame()

        # Axes du repère de l'organe terminal

        self.visualizer.draw_tip_frame(
            tip_x,
            tip_y,
            tip_z,
            R_tip
        )


        # ========= FANTÔME =========

        if self.ghost_checkbox.isChecked():

            self.visualizer.draw_ghost(
                self.ghost_robot,
                self.r_tube
            )

        # AFFICHAGE 3D CTR
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

        # GRAPHES
        selected_graph = self.graph_selector.currentIndex()

        if selected_graph == 0:

            self.graphs.plot_local_tangent_orientation(
                matrix_data,
                length_axis,
                num_nodes
            )
        
        elif selected_graph == 1:

            self.graphs.plot_tip_orientation_history(
                self.tip_orientation_history
            )

        elif selected_graph == 2:

            # Torsion relative
            theta_2 = matrix_data[17, :]
            theta_3 = matrix_data[18, :]

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

        # Mise à jour du panneau
        self.tip_info_label.setText(
            f"Organe terminal:\n"
            f"x = {tip_x*1000:.1f} mm\n"
            f"y = {tip_y*1000:.1f} mm\n"
            f"z = {tip_z*1000:.1f} mm\n\n"
            f"Orientation:\n"
            f"X = {tip_angle_x:.1f}°\n"
            f"Y = {tip_angle_y:.1f}°\n"
            f"Z = {tip_angle_z:.1f}°"
        )
        self.canvas.draw_idle()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(app.exec_())