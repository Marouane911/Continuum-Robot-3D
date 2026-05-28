import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, "python"))

from draw_ctcr import draw_ctcr

# --- 1. Chargement et découpage du fichier de trajectoire ---
data_path = os.path.expanduser("~/TIMC_Robotics/Modeling-and-Control-of-Concentric-Tube-Continuum-Robots/build/robot_backbone.txt")

if not os.path.exists(data_path):
    print("Erreur : Fichier robot_backbone.txt introuvable.")
    exit()

with open(data_path, "r") as f:
    content = f.read()

# Séparation des blocs temporels
blocks = content.strip().split("--- STEP_BREAK ---")
steps_data = [b.strip() for b in blocks if b.strip()]

print(f"Nombre d'images d'animation chargées : {len(steps_data)}")

# Les rayons physiques de tes tubes
r_tube = np.array([3.0e-3, 2.0e-3, 1.0e-3])

# --- 2. Préparation de la FENÊTRE UNIQUE ---
plt.ion() # Active le mode interactif
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

print("Lancement de l'animation...")

for step_idx, step_str in enumerate(steps_data):
    # Nettoyage de l'image précédente (Ici on efface TOUT, forçant à tout recalculer)
    ax.cla()
    
    matrix_lines = [list(map(float, line.split())) for line in step_str.split('\n') if line.strip()]
    matrix_data = np.array(matrix_lines)
    num_nodes = matrix_data.shape[1]
    
    # ---- RECALCUL DYNAMIQUE DES TUBES POUR CETTE FRAME ----
    # Basé sur la modification C++ (NB_INTEGRATION_NODES = 30)
    nodes_per_segment = 30
    tube_end = np.array([
        2 * nodes_per_segment,       # Fin du tube 1 (nœud 60)
        4 * nodes_per_segment,       # Fin du tube 2 (nœud 120)
        num_nodes                    # Fin du tube 3 (nœud 180)
    ])
    
    # Extraction des coordonnées pour construire les matrices de transformation 4x4
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
    
    # ---- APPEL DE LA FONCTION DE BASE (L'origine de la lenteur) ----
    # C'est cette fonction qui recrée des milliers de polygones à chaque itération
    draw_ctcr(g, tube_end, r_tube, tipframe=True, ax=ax)
    
    # ---- CADRAGE ET CONFIGURATION DE LA BOÎTE ----
    ax.set_aspect('equal')
    ax.set_box_aspect((1, 1, 1)) # Cube parfait !
    
    ax.set_xlim3d([-0.09, 0.09])
    ax.set_ylim3d([-0.09, 0.09])
    ax.set_zlim3d([0.0, 0.18])
    
    ax.view_init(elev=25, azim=-60)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f"Animation CTCR - Étape {step_idx + 1}/{len(steps_data)}")
    
    # Rafraîchissement forcé de la fenêtre unique
    plt.draw()
    plt.pause(0.01)

# Laisse la fenêtre ouverte à la fin de l'animation
plt.ioff()
plt.show()
