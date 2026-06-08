import sys
import os
import numpy as np
import subprocess
import shutil
import csv
 
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
from ctr_data import CTRData
from ctr_visualizer import CTRVisualizer
from ctr_graph import CTRGraphs
from ctr_constraints import CTRConstraints
from ctr_loader import CTRLoader
from ctr_status import CTRStatus
from ctr_solver import CTRSolver



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

        # gestion de la modification des parameters
        # RECONSTRUCTION DU CHEMIN ABSOLU VERS LE REPO DE QUENTIN 
        project_parent = os.path.dirname(root_toolkit)




# GESTION DES PARAMÈTRES (COPIE PARFAITE DANS parameters_temp.csv)
        self.params_path = os.path.join(
            project_parent,
            "Modeling-and-Control-of-Concentric-Tube-Continuum-Robots",
            "parameters",
            "parameters.csv"
        )
        
        # === CORRECTION : On définit d'abord le chemin de base ===
        self.temp_params_path = os.path.join(python_dir, "parameters_temp.csv")

        # On le place dans le dossier python pour que le solver y accède facilement
        self.temp_params_path_abs = os.path.abspath(self.temp_params_path)

        # Sécurité : On force la copie propre depuis l'original pour réparer les 0.0
        if os.path.exists(self.temp_params_path):
            os.remove(self.temp_params_path)
            
        # === CORRECTION : On recrée le fichier temporaire propre ===
        shutil.copy2(self.params_path, self.temp_params_path)




        # Mettre à jour data_path si votre projet génère son backbone avec ce paramètre
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

        self.steps_data = CTRLoader.load(
            self.data_path
        )

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
            -0.015,
            -0.010,
            -0.005,
            0.0,
            0.0,
            0.0
        ]
        
        self.record_orientation_history = False

        # --- Mémoriser la dérnière configuration pour l'animation entre 2 positions entrées ---
        self.current_q = [
            -0.015,
            -0.010,
            -0.005,
            0.0,
            0.0,
            0.0
        ]

        self.last_valid_q = self.current_q.copy()

        # === MULTI-MODIFICATION : SAUVEGARDE DYNAMIQUE DE LA POSITION DE LA CAMÉRA ===
        current_elev = getattr(self, "saved_elev", 20)
        current_azim = getattr(self, "saved_azim", -45)
        # AJOUT : On récupère la distance actuelle (le zoom) choisie par l'utilisateur à la souris. 
        # Si elle n'existe pas encore, on met 10 par défaut (la valeur standard de Matplotlib).
        current_dist = getattr(self, "saved_dist", 10)

        self.ax_robot.cla()
        self.ax_plots.cla()
        
        # Mémorisation immédiate des angles et du zoom pour le prochain rafraîchissement
        self.ax_robot.view_init(elev=current_elev, azim=current_azim)
        self.ax_robot.dist = current_dist

        self.compute_ctr_configuration(self.current_q)

        self.set_current_as_home()




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

        # Crée l'affichage des coordonnées de l'organe terminal
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

        self.status = CTRStatus(
            self.robot_state_label,
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
            -0.015,
            -0.010,
            -0.005,
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


        # Affichage des chariots du ghost robot
        self.ghost_checkbox = QCheckBox(
            "Afficher dernière position mémorisé"
        )

        self.ghost_checkbox.setChecked(False)

        self.ghost_checkbox.toggled.connect(
            self.update_plots
        )

        config_layout.addWidget(self.ghost_checkbox)

        # Affichage des chariots

        self.chariots_checkbox = QCheckBox(
            "Afficher les chariots (Base)"
        )
        self.chariots_checkbox.setChecked(False)
        self.chariots_checkbox.toggled.connect(
            self.update_plots
        )
        config_layout.addWidget(self.chariots_checkbox)

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
            "Orientation along the CTR",
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
            figsize=(6, 9)
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


        # ==========================================================
        # PARAMÈTRES PHYSIQUES (Friction & Longueur)
        # ==========================================================
        params_group = QGroupBox("Paramètres physiques")
        params_layout = QGridLayout()

        self.param_inputs = {}
        keys = ['Ux1', 'Ux2', 'Ux3', 'l1', 'l2', 'l3']

        display_names = {
            'Ux1': "Courbure Tube 1",
            'Ux2': "Courbure Tube 2",
            'Ux3': "Courbure Tube 3",
            'l1': "Longueur Totale Tube 1",
            'l2': "Longueur Totale Tube 2",
            'l3': "Longueur Totale Tube 3"
        }
        
        # --- LECTURE DES VALEURS ACTUELLES DU CSV ---
        current_csv_values = {}
        try:
            with open(self.temp_params_path, 'r') as f:
                reader = list(csv.DictReader(f))
                if reader:
                    current_csv_values = reader[0]
        except Exception as e:
            print("Erreur de lecture initiale du CSV :", e)

        for i, key in enumerate(keys):
            friendly_name = display_names.get(key, key)
            params_layout.addWidget(QLabel(friendly_name), i, 0)
            spin = QDoubleSpinBox()
            spin.setLocale(QLocale(QLocale.English, QLocale.UnitedStates))
            spin.setDecimals(4)
            spin.setSingleStep(1.0 if 'Ux' in key else 0.005)
            spin.setRange(0, 100000)
            
            # Appliquer la vraie valeur du CSV si elle existe, sinon mettre une valeur de secours
            val_init = float(current_csv_values.get(key, 0.0))
            spin.setValue(val_init)
            
            self.param_inputs[key] = spin
            params_layout.addWidget(spin, i, 1)

        btn_save_params = QPushButton("Enregistrer et Appliquer")
        btn_save_params.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        btn_save_params.clicked.connect(self.save_parameters_to_csv)
        params_layout.addWidget(btn_save_params, 6, 0, 1, 2)
        
        params_group.setLayout(params_layout)
        control_layout.addWidget(params_group)



    def save_parameters_to_csv(self):
            print("=== SAUVEGARDE ===")
            print("Fichier :", self.temp_params_path)
            
            # --- MODIFICATION ICI : On vire le 'for key, spin in...' qui englobait tout ---

            # 1. Lire le dictionnaire existant pour conserver toutes les autres colonnes intactes
            with open(self.temp_params_path, 'r') as f:
                reader = list(csv.DictReader(f))
            
            if not reader:
                return
                
            row = reader[0]
            # 2. Injecter les nouvelles valeurs modifiées par l'utilisateur
            for key, spin in self.param_inputs.items():
                row[key] = f"{spin.value():.6f}"
                
            # 3. Réécrire le fichier temporaire proprement
            with open(self.temp_params_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=reader[0].keys())
                writer.writeheader()
                writer.writerows(reader)
                                            
            # 4. RECALCUL IMMÉDIAT : Le solver va maintenant lire les vraies nouvelles valeurs
            q_values = [self.q_inputs[i].value() for i in range(6)]
            self.compute_ctr_configuration(q_values)
            


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
                
            try:
                data = CTRData(step["matrix"], step["iEnd"], step["S"])

                self.ghost_robot = {
                    "x": data.x.copy(),
                    "y": data.y.copy(),
                    "z": data.z.copy(),
                    "end_ext": data.end_ext,
                    "end_mid": data.end_mid,
                    "end_int": data.end_int
                }

            except Exception as e:
                self.ghost_robot = None


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

            q_values = CTRConstraints.enforce_telescopic_constraints(
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
        
        q_values = CTRConstraints.enforce_telescopic_constraints(
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

    def compute_ctr_configuration(
        self,
        q_values
    ):

        print("=== COMPUTE CTR ===")
        print("q =", q_values)

        try:

            result = CTRSolver.run(
                root_toolkit,
                q_values,
                self.temp_params_path_abs
            )

            if not result["success"]:

                if result["status"] == "collision":

                    self.status.set_collision(
                        result["message"]
                    )

                elif result["status"] == "divergence":

                    self.status.set_divergence(
                        result["message"]
                    )

                return False

            self.status.set_ok()

            self.steps_data = CTRLoader.load(
                self.data_path
            )

            self.current_step = 0

            self.update_plots()

            return True

        except Exception as e:

            print("Erreur :", e)

            return False
        

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

        # ========= RENDU DES CHARIOTS AVEC AXE DE LIAISON (AMÉLIORÉ) =========
        if self.chariots_checkbox.isChecked():
            colors = ['#08ff08', 'blue', 'red']
            labels = ["Tube 1 (Interne)", "Tube 2 (Milieu)", "Tube 3 (Externe)"]
            
            # 1. Dessiner l'axe central (Z) transparent ou pointillé qui sert de guide
            min_beta = min(q_current)
            self.ax_robot.plot(
                [0, 0], [0, 0], [min_beta * 1.1, 0],
                color='gray', linestyle='--', linewidth=1, alpha=0.7
            )
            
            # 2. Dessiner chaque chariot avec sa ligne de guidage et sa projection
            for idx, beta in enumerate(q_current):
                # Le point du chariot sur l'axe Z
                self.ax_robot.plot(
                    [0], [0], [beta], 
                    marker='o', markersize=12, color=colors[idx],
                    zorder=5, label=f"Chariot {labels[idx]}"
                )
                
                # Une ligne horizontale (croisillon) pour accentuer l'effet de plateau/chariot
                self.ax_robot.plot(
                    [-0.01, 0.01], [0, 0], [beta, beta], 
                    color=colors[idx], linewidth=2
                )
                self.ax_robot.plot(
                    [0, 0], [-0.01, 0.01], [beta, beta], 
                    color=colors[idx], linewidth=2
                )

                # Une ligne fine pointillée qui relie le chariot à la base visible du robot (Z=0)
                # Cela permet de voir la "tige" virtuelle ou la partie cachée du tube
                self.ax_robot.plot(
                    [0, 0], [0, 0], [beta, 0],
                    color=colors[idx], linestyle=':', linewidth=1.5, alpha=0.6
                )


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

        # === MULTI-MODIFICATION : RIGIDIFICATION DU CADRE 3D (FINI LA GRILLE DANSANTE) ===
        self.ax_robot.set_title("Modélisation CTR en 3D", pad=50, fontsize=12, fontweight='bold')

        # 1. Définition des dimensions physiques fixes du cadre (en mètres)
        XY_CADRE = 0.15     # Largeur de la boîte : +/- 8 cm à gauche et à droite
        
        # Le Z minimal descend à -15 cm si les chariots sont visibles pour les afficher proprement
        Z_MIN_CADRE = -0.15 if self.chariots_checkbox.isChecked() else -0.02
        Z_MAX_CADRE = 0.45   # Hauteur maximale du graphique fixe (35 cm)

        # 2. Application des limites strictes
        self.ax_robot.set_xlim(-XY_CADRE, XY_CADRE)
        self.ax_robot.set_ylim(-XY_CADRE, XY_CADRE)
        self.ax_robot.set_zlim(Z_MIN_CADRE, Z_MAX_CADRE)

        # 3. ÉQUIVALENT "EQUAL AXES" 3D : On impose des proportions réalistes (sans déformation)
        hauteur_totale = Z_MAX_CADRE - Z_MIN_CADRE
        largeur_totale = 2 * XY_CADRE

        # on applique directement les dimensions physiques réelles
        self.ax_robot.set_box_aspect((largeur_totale, largeur_totale, hauteur_totale))

        self.ax_robot.dist = 15 # Dézoome de la zone matplolib 3D CTR

        # === AJOUT : ENREGISTREMENT DU ZOOM ET DES ROTATIONS FAITS À LA SOURIS ===
        # À chaque rafraîchissement, on mémorise la position de la caméra laissée par l'utilisateur
        self.saved_elev = self.ax_robot.elev
        self.saved_azim = self.ax_robot.azim
        self.saved_dist = self.ax_robot.dist  # Sauvegarde du niveau de zoom !


        # # AFFICHAGE 3D CTR
        # self.ax_robot.set_title("Modélisation CTR en 3D")

        # # 1. Calcul des limites dynamiques basées sur la géométrie réelle du robot
        # # On prend le max absolu en X et Y pour garder un repère centré et carré
        # max_xy = max(np.max(np.abs(x)), np.max(np.abs(y)), 0.05) * 1.2
        # # On ajuste le Z max avec une marge de 10% au-dessus de la pointe
        # max_z = max(np.max(z), 0.10) * 1.1

        # # 2. Application des nouvelles limites adaptatives
        # self.ax_robot.set_xlim(-max_xy, max_xy)
        # self.ax_robot.set_ylim(-max_xy, max_xy)
        # self.ax_robot.set_zlim(0.0, max_z)

        # # # On ajuste le Z minimal pour voir les chariots s'ils sont cochés
        # # min_z = min(q_current) * 1.2 if self.chariots_checkbox.isChecked() else 0.0
        # # self.ax_robot.set_zlim(min_z, max_z)

        # # 3. Forcer le ratio 1:1:1 pour éviter les déformations visuelles du robot
        # self.ax_robot.set_box_aspect((1, 1, 1))

        # GRAPHES
        selected_graph = self.graph_selector.currentIndex()

        if selected_graph == 0:

            self.graphs.plot_orientation(
                matrix_data,
                length_axis,
                num_nodes
            )
        
        elif selected_graph == 1:

            self.graphs.plot_tip_orientation_history(
                self.tip_orientation_history
            )
        
        elif selected_graph == 2:

            self.graphs.plot_twist_distribution(
                data
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

        # ========= AFFICHAGE DES CHARIOTS (RAIL HORIZONTAL) =========
        # 1. Nettoyage : On supprime l'ancien rail s'il existe pour éviter de superposer les graphiques
        for ax in self.fig.axes:
            if ax.get_label() == 'ax_chariots':
                ax.remove()

        if self.chariots_checkbox.isChecked():
            # 2. Création d'un axe très fin en bas de l'écran [gauche, bas, largeur, hauteur]
            ax_chariots = self.fig.add_axes([0.15, 0.04, 0.30, 0.03], label='ax_chariots')
            
            
            colors = ['#08ff08', 'blue', 'red']
            
            # Ligne de guidage horizontale (le rail)
            ax_chariots.axhline(y=0, color='gray', linestyle='-', linewidth=2, alpha=0.5)
            
            # Positionnement des curseurs de chariots (sans texte)
            for idx, beta in enumerate(q_current):
                # On utilise un marqueur vertical '|' très épais pour simuler la butée
                ax_chariots.plot(
                    beta * 1000, 0, 
                    marker='|', markersize=25, markeredgewidth=4, 
                    color=colors[idx]
                )
            
            # Décoration minimale de l'axe
            ax_chariots.set_xlim(-400, 50) # Plage fixe pour ne pas que l'échelle bouge
            ax_chariots.set_ylim(-1, 1)
            
            # Masquer complètement l'axe Y et affiner l'axe X
            ax_chariots.get_yaxis().set_visible(False)
            ax_chariots.set_xlabel("Espace de translation des tubes (mm)", fontsize=10, fontweight='bold', color='#333333')
            ax_chariots.spines['top'].set_visible(False)
            ax_chariots.spines['right'].set_visible(False)
            ax_chariots.spines['left'].set_visible(False)
            ax_chariots.grid(True, axis='x', linestyle=':', alpha=0.6)
            
        self.canvas.draw_idle()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(app.exec_())