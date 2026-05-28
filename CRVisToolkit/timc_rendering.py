import sys
import os
import numpy as np

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, "python"))

from draw_ctcr import draw_ctcr

data_path = os.path.expanduser("~/TIMC_Robotics/Modeling-and-Control-of-Concentric-Tube-Continuum-Robots/build/robot_backbone.txt")

if not os.path.exists(data_path):
    print(f"Erreur : Le fichier {data_path} est introuvable !")
    exit()

matrix_data = np.loadtxt(data_path)
num_nodes = matrix_data.shape[1]
print(f"Données chargées ! Nombre de nœuds d'intégration : {num_nodes}")

x_coords = matrix_data[0, :]
y_coords = matrix_data[1, :]
z_coords = matrix_data[2, :]

g = np.zeros((num_nodes, 16))
for i in range(num_nodes):
    M = np.eye(4)
    M[0, 3] = x_coords[i]
    M[1, 3] = y_coords[i]
    M[2, 3] = z_coords[i]
    g[i, :] = M.flatten(order='F')

nodes_per_segment = 30

# Each tube has two successive segments in this 3-tube model (6 segments in total)
tube_end = np.array([
    2 * nodes_per_segment,       # end of tube 1 (node 0 to 60)
    4 * nodes_per_segment,       # end of tube 2 (node 61 to 120)
    num_nodes                    # end of tube 3 (node 121 to 180)
])

r_tube = np.array([3.0e-3, 2.0e-3, 1.0e-3])

print("Affichage du robot CTCR...")
draw_ctcr(g, tube_end, r_tube, tipframe=True)
