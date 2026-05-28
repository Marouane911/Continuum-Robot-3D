import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt  # <-- AJOUTÉ : pour contrôler la fenêtre unique

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, "python"))

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

tube_end = np.array([4, 8, 12])
r_tube = np.array([3.0e-3, 2.0e-3, 1.0e-3])

# --- 2. Préparation de la FENÊTRE UNIQUE ---
plt.ion() # Active le mode interactif (permet de rafraîchir le graphique en direct)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

print("Lancement de l'animation...")

for step_idx, step_str in enumerate(steps_data):
    # Nettoyage de l'image précédente pour dessiner la nouvelle
    ax.cla()
    
    matrix_lines = [list(map(float, line.split())) for line in step_str.split('\n') if line.strip()]
    matrix_data = np.array(matrix_lines)
    num_nodes = matrix_data.shape[1]
    
    # Extraction des coordonnées X, Y, Z pour cette étape
    x_coords = matrix_data[0, :]
    y_coords = matrix_data[1, :]
    z_coords = matrix_data[2, :]
    
    # Dessin du squelette du robot (Ligne continue)
    ax.plot(x_coords, y_coords, z_coords, '-o', color='black', linewidth=3, label='Robot backbone')
    
    # Dessin des points d'extrémité pour marquer les sections des tubes
    ax.scatter(x_coords[tube_end-1], y_coords[tube_end-1], z_coords[tube_end-1], color='red', s=50, label='Tube ends')

    # Fixer les limites des axes pour éviter que le graphique ne bouge/saute pendant l'animation
    ax.set_xlim([-0.05, 0.05])
    ax.set_ylim([-0.05, 0.05])
    ax.set_zlim([0, 0.18])
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f"Animation CTCR - Étape {step_idx + 1}/{len(steps_data)}")
    ax.legend()
    
    # Rafraîchissement forcé de la fenêtre unique
    plt.draw()
    plt.pause(0.03) # 30 millisecondes de pause entre chaque frame (~30 FPS)

# Laisse la fenêtre ouverte à la fin de l'animation
plt.ioff()
plt.show()
