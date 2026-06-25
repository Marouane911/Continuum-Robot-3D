from ctr_solver import CTRSolver
from ctr_loader import CTRLoader
from ctr_vtk_export import CTRVTKExporter
from ctr_constraints import CTRConstraints
import os
import subprocess
import csv
from ctr_data import CTRData
import numpy as np


current_dir = os.path.dirname(os.path.abspath(__file__))
root_toolkit = os.path.dirname(current_dir)

python_dir = os.path.join(root_toolkit, "python")
project_parent = os.path.dirname(root_toolkit)

# temp_params_path_abs = os.path.join(
#     python_dir,
#     "parameters_temp.csv"
# )


temp_params_path_abs = "/home/ketebm/Projet/Continuum-Robot-3D/Modeling-and-Control-of-Concentric-Tube-Continuum-Robots/parameters/parameters.csv"


rOut1, rOut2, rOut3 = 0.000762, 0.000900, 0.001175  # Valeurs de secours
if os.path.exists(temp_params_path_abs):
    with open(temp_params_path_abs, "r", newline="") as f:
        rows = list(csv.DictReader(f))
        if len(rows) > 0:
            rOut1 = float(rows[0].get("rOut1", rOut1))
            rOut2 = float(rows[0].get("rOut2", rOut2))
            rOut3 = float(rows[0].get("rOut3", rOut3))


data_path = os.path.join(
    project_parent,
    "Modeling-and-Control-of-Concentric-Tube-Continuum-Robots",
    "build",
    "robot_backbone.txt"
)

# paramètres de test
q_brut = [-0.015, -0.010, -0.005]

q_securise = CTRConstraints.enforce_telescopic_constraints(
    q=q_brut,
    active=2, # On définit le tube externe comme actif pour le test
    entrainement=True,
    tubes_lengths=[0.463, 0.3305, 0.199] # Longueurs par défaut
)

q_final = list(q_securise) + [0.0, 0.0, 0.0]

# lancer le solveur
result = CTRSolver.run(
    root_toolkit,
    q_final,
    temp_params_path_abs
)

if result["success"]:

    steps = CTRLoader.load(data_path)

    if len(steps) == 0:
        print("Aucune donnée chargée")
        exit(1)

    step = steps[0]

    data = CTRData(
        step["matrix"],
        step["iEnd"],
        step["S"]
    )

    print("end_int =", data.end_int)
    print("end_mid =", data.end_mid)
    print("end_ext =", data.end_ext)

    CTRVTKExporter.export_centerline(
        data.matrix_data,
        data.end_int,
        data.end_mid,
        data.end_ext,
        rOut1,
        rOut2,
        rOut3,
        "/tmp/robot_centerline.vtk"
    )

    subprocess.run([
        "camitk-imp",
        "/tmp/robot_centerline.vtk"
    ])    

    print("VTK généré")

else:
    print(result)