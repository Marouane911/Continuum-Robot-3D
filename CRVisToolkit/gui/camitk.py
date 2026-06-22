from ctr_solver import CTRSolver
from ctr_loader import CTRLoader
from ctr_vtk_export import CTRVTKExporter
import os
import subprocess

from ctr_data import CTRData


current_dir = os.path.dirname(os.path.abspath(__file__))
root_toolkit = os.path.dirname(current_dir)

python_dir = os.path.join(root_toolkit, "python")
project_parent = os.path.dirname(root_toolkit)

temp_params_path_abs = os.path.join(
    python_dir,
    "parameters_temp.csv"
)

data_path = os.path.join(
    project_parent,
    "Modeling-and-Control-of-Concentric-Tube-Continuum-Robots",
    "build",
    "robot_backbone.txt"
)

# paramètres de test
q_values = [-0.015, -0.010, -0.005, -3.14, 0.00, 0.00]

# lancer le solveur
result = CTRSolver.run(
    root_toolkit,
    q_values,
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
        "/tmp/robot_centerline.vtk"
    )

    subprocess.run([
        "camitk-imp",
        "/tmp/robot_centerline.vtk"
    ])

    print("VTK généré")

else:
    print(result)