import camitk
import sys
import csv
import vtk

sys.path.append("/home/ketebm/Projet/Continuum-Robot-3D/CRVisToolkit/gui")

from ctr_solver import CTRSolver
from ctr_loader import CTRLoader
from ctr_vtk_export import CTRVTKExporter
from ctr_data import CTRData
from ctr_solver import CTRSolver
from vtk.util import numpy_support
from ctr_constraints import CTRConstraints


def process(self):

    camitk.warning("PROCESS CALLED")

    # 1. Lecture brute des translations actuelles
    q_brut = [
        self.getParameterValue("Q0"),
        self.getParameterValue("Q1"),
        self.getParameterValue("Q2")
    ]

    # 2. Sécurité : Création de la mémoire au premier lancement
    if not hasattr(self, 'prev_q'):
        self.prev_q = list(q_brut)

    # 3. Détection : Quel chariot vient d'être manipulé ?
    active_tube = 2 # Externe par défaut
    for i in range(3):
        if abs(q_brut[i] - self.prev_q[i]) > 1e-6:
            active_tube = i
            break

    # 4. Lecture brute des longueurs
    l_tubes = [
        self.getParameterValue("L1"),
        self.getParameterValue("L2"),
        self.getParameterValue("L3")
    ]

    # 5. Garde-fou physique (L'entraînement)
    q_corrige = CTRConstraints.enforce_telescopic_constraints(
        q=q_brut,
        active=active_tube,
        entrainement=True,
        tubes_lengths=l_tubes
    )

    # 6. Rétroaction visuelle : Si la physique a bougé un tube, on met à jour l'interface
    if q_brut != q_corrige:
        # camitk.warning("Entraînement activé : Ajustement automatique des chariots.")
        self.setParameterValue("Q0", float(q_corrige[0]))
        self.setParameterValue("Q1", float(q_corrige[1]))
        self.setParameterValue("Q2", float(q_corrige[2]))

    # 7. On mémorise la position valide pour le prochain clic
    self.prev_q = list(q_corrige)

    # 8. Affectation finale des variables pour la suite de ton script
    q0, q1, q2 = q_corrige[0], q_corrige[1], q_corrige[2]

    q3 = self.getParameterValue("Q3")
    q4 = self.getParameterValue("Q4")
    q5 = self.getParameterValue("Q5")

    l1, l2, l3 = l_tubes[0], l_tubes[1], l_tubes[2]

    # q0 = self.getParameterValue("Q0")
    # q1 = self.getParameterValue("Q1")
    # q2 = self.getParameterValue("Q2")
    # q3 = self.getParameterValue("Q3")
    # q4 = self.getParameterValue("Q4")
    # q5 = self.getParameterValue("Q5")

    # l1  = self.getParameterValue("L1")
    # l2  = self.getParameterValue("L2")
    # l3  = self.getParameterValue("L3")

    l_k1 = self.getParameterValue("L_k1")
    l_k2 = self.getParameterValue("L_k2")
    l_k3 = self.getParameterValue("L_k3")

    Ux1 = self.getParameterValue("Ux1")
    Ux2 = self.getParameterValue("Ux2")
    Ux3 = self.getParameterValue("Ux3")

    rOut1 = self.getParameterValue("ROut1")
    rOut2 = self.getParameterValue("ROut2")
    rOut3 = self.getParameterValue("ROut3")

    base_x = self.getParameterValue("BaseX")
    base_y = self.getParameterValue("BaseY")
    base_z = self.getParameterValue("BaseZ")

    rot_x = self.getParameterValue("AxeX")
    rot_y = self.getParameterValue("AxeY")
    rot_z = self.getParameterValue("AxeZ")

    ## Écriture des nouveaux paramètres dans parameters_tmp.csv
    csv_path = (
    "/home/ketebm/Projet/Continuum-Robot-3D/"
    "CRVisToolkit/python/parameters_temp.csv"
    )

    # Sauvegarde sur le disque
    with open(csv_path, "r", newline="") as f:
        rows = list(csv.DictReader(f))

    if len(rows) == 0:
        # camitk.warning("parameters_temp.csv vide")
        return False

    # On fait une copie exacte de la ligne qui fonctionnait avant modification
    backup_row = rows[0].copy()

    # Mise à jour du dictionnaire
    rows[0]["l1"] = str(l1)
    rows[0]["l2"] = str(l2)
    rows[0]["l3"] = str(l3)
    rows[0]["l_k1"] = str(l_k1)
    rows[0]["l_k2"] = str(l_k2)
    rows[0]["l_k3"] = str(l_k3)
    rows[0]["Ux1"] = str(Ux1)
    rows[0]["Ux2"] = str(Ux2)
    rows[0]["Ux3"] = str(Ux3)
    rows[0]["rOut1"] = str(rOut1)
    rows[0]["rOut2"] = str(rOut2)
    rows[0]["rOut3"] = str(rOut3)

    fieldnames = rows[0].keys()

    # On écrit la tentative de modification des paramètres dans le fichier CSV
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    # camitk.warning("parameters_temp.csv updated")

    ###### fin de l'écriture

    result = CTRSolver.run(
        root_toolkit="/home/ketebm/Projet/Continuum-Robot-3D/CRVisToolkit",
        q_values=[q0, q1, q2, q3, q4, q5],
        params_path="/home/ketebm/Projet/Continuum-Robot-3D/CRVisToolkit/python/parameters_temp.csv"
    )

    # camitk.warning(f"Succès du solveur : {result['success']}")

    if not result["success"]:
        camitk.warning(f"{str(result)}, restauration des anciens paramètres")
        # On réécrit immédiatement l'ancienne sauvegarde dans le CSV
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerow(backup_row) # On utilise le backup !
        return False

    # camitk.warning("Calcul réussi, mise à jour du mesh ...")

    steps = CTRLoader.load(
        "/home/ketebm/Projet/Continuum-Robot-3D/"
        "Modeling-and-Control-of-Concentric-Tube-Continuum-Robots/"
        "build/robot_backbone.txt"
    )

    if len(steps) == 0:
        # camitk.warning("No backbone data loaded")
        return False

    step = steps[0]

    data = CTRData(
        step["matrix"],
        step["iEnd"],
        step["S"]
    )

    # camitk.warning("BEFORE EXPORT")

    polydata = CTRVTKExporter.export_centerline(
        data.matrix_data,
        data.end_int,
        data.end_mid,
        data.end_ext,
        rOut1,
        rOut2,
        rOut3,
        "/tmp/robot_centerline.vtk"
    )

    # camitk.warning("EXPORT FINI")

    # camitk.warning("AFTER EXPORT")

    # camitk.warning(f"mesh points = {polydata.GetNumberOfPoints()}")

    # camitk.warning(f"mesh polys = {polydata.GetNumberOfPolys()}")

    # Transformation Globale (translation et rotation)
    transform = vtk.vtkTransform()
    transform.Translate(base_x, base_y, base_z)
    transform.RotateZ(rot_z)
    transform.RotateY(rot_y)
    transform.RotateX(rot_x)
    # Application de la transformation sur le maillage 3D
    transform_filter = vtk.vtkTransformPolyDataFilter()
    transform_filter.SetInputData(polydata)
    transform_filter.SetTransform(transform)
    transform_filter.Update()
    # Récupération du maillage après ses transformations de translation et rotation globale
    polydata_transforme = transform_filter.GetOutput()

    # Extraction des points composant le maillage 3D
    points = numpy_support.vtk_to_numpy(
        polydata_transforme.GetPoints().GetData()
    )

    polys = numpy_support.vtk_to_numpy(
        polydata_transforme.GetPolys().GetData()
    ).reshape(-1, 4)

    mesh = self.getTargets()[0]

    old_points = mesh.getPointSetAsNumpy()

    # camitk.warning(
    #     f"OLD={old_points.shape[0]} NEW={points.shape[0]}"
    # )

    # camitk.warning(
    #     f"NEW_POLYS={polys.shape[0]}"
    # )

    if old_points.shape[0] != points.shape[0]:
        # camitk.warning("Topology changed")
        return True

    mesh.replacePointSet(points)

    camitk.refresh()

    return True



