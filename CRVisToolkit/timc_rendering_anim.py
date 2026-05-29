import sys
import os
import numpy as np
import matplotlib.pyplot as plt

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, "python"))

from draw_ctcr import draw_ctcr

# --- 1. Chargement du fichier de trajectoire ---
data_path = os.path.expanduser("~/TIMC_Robotics/Modeling-and-Control-of-Concentric-Tube-Continuum-Robots/build/robot_backbone.txt")

if not os.path.exists(data_path):
    print("Erreur : Fichier robot_backbone.txt introuvable.")
    exit()

with open(data_path, "r") as f:
    content = f.read()

blocks = content.strip().split("--- STEP_BREAK ---")
steps_data = [b.strip() for b in blocks if b.strip()]

print(f"Nombre d'images d'animation chargées : {len(steps_data)}")
r_tube = np.array([3.0e-3, 2.0e-3, 1.0e-3])

# --- 2. PRÉPARATION DE L'ÉCRAN PARTAGÉ COMPACT (Grille 2x2) ---
plt.ion()
fig = plt.figure(figsize=(14, 10))

# [Haut Gauche] ROBOT 3D
ax_robot = fig.add_subplot(221, projection='3d')

# [Haut Droite] TORSION RÉELLE GÉOMÉTRIQUE EN DEGRÉS
ax_torsion = fig.add_subplot(222)

# [Bas Gauche] EDO VUE D'ENSEMBLE (Échelle fixe -3.5 à 1)
ax_edo_global = fig.add_subplot(223)

# [Bas Droite] EDO VUE DÉTAILLÉE (Zoom automatique)
ax_edo_detail = fig.add_subplot(224)

print("Lancement de l'animation pré-structurée...")

for step_idx, step_str in enumerate(steps_data):
    # Nettoyage de toutes les zones de dessin
    ax_robot.cla() 
    ax_torsion.cla()
    ax_edo_global.cla()
    ax_edo_detail.cla()
    
    matrix_lines = [list(map(float, line.split())) for line in step_str.split('\n') if line.strip()]
    matrix_data = np.array(matrix_lines)
    num_nodes = matrix_data.shape[1]
    
    nodes_per_segment = int(num_nodes / 6)
    tube_end = np.array([2 * nodes_per_segment, 4 * nodes_per_segment, num_nodes])
    
    # ---- EXTRACTION ET CALCULS ----
    g = np.zeros((num_nodes, 16))
    torsion_angle_deg = np.zeros(num_nodes)
    
    for i in range(num_nodes):
        M = np.eye(4)
        M[0, 3] = matrix_data[0, i] # X
        M[1, 3] = matrix_data[1, i] # Y
        M[2, 3] = matrix_data[2, i] # Z

        R_elements = matrix_data[3:12, i] 
        R = R_elements.reshape((3, 3), order='F')
        M[0:3, 0:3] = R

        g[i, :] = M.flatten(order='F')
        
        # Extraction de l'angle alpha et conversion immédiate en DEGRÉS
        alpha_rad = np.arctan2(R[1, 0], R[0, 0])
        torsion_angle_deg[i] = np.degrees(alpha_rad)
    
    # ---- 1. ANIMATION 3D [Haut Gauche] ----
    draw_ctcr(g, tube_end, r_tube, tipframe=True, ax=ax_robot)
    ax_robot.set_aspect('equal')
    ax_robot.set_box_aspect((1, 1, 1))
    ax_robot.set_xlim3d([-0.04, 0.04])
    ax_robot.set_ylim3d([-0.04, 0.04])
    ax_robot.set_zlim3d([0.0, 0.14])
    ax_robot.view_init(elev=20, azim=-45)
    ax_robot.set_title(f"Robot CTR - Étape {step_idx + 1}/{len(steps_data)}")

    nodes_axis = np.arange(num_nodes)
    theta_2_profile = matrix_data[17, :]  
    theta_3_profile = matrix_data[18, :]  

    # ---- 2. TORSION GÉOMÉTRIQUE ALPHA EN DEGRÉS [Haut Droite] ----
    ax_torsion.set_xlim(0, num_nodes - 1)
    ax_torsion.set_ylim(-180, 180) # Échelle naturelle en degrés (-180° à 180°)
    ax_torsion.set_xlabel("Nœuds d'intégration")
    ax_torsion.set_ylabel("Orientation réelle alpha (°)")
    ax_torsion.grid(True)
    ax_torsion.set_title("Orientation cinématique du corps (alpha)")
    ax_torsion.plot(nodes_axis, torsion_angle_deg, 'g-s', markersize=4, color='darkgreen', label=r"$\alpha$ (Orientation)")
    for end_node in tube_end[:-1]:
        ax_torsion.axvline(x=end_node, color='gray', linestyle='--', alpha=0.5)
    ax_torsion.legend(loc="lower left")

    # ---- 3. EDO VUE D'ENSEMBLE [Bas Gauche] ----
    ax_edo_global.set_xlim(0, num_nodes - 1)
    ax_edo_global.set_ylim(-3.5, 1.0) # Échelle fixe d'origine
    ax_edo_global.set_xlabel("Nœuds d'intégration")
    ax_edo_global.set_ylabel("Valeur d'état EDO")
    ax_edo_global.grid(True)
    ax_edo_global.set_title("EDO : Vue Macro (Convergence)")
    ax_edo_global.plot(nodes_axis, theta_2_profile, 'b-o', markersize=4, label=r"$\theta_2$")
    ax_edo_global.plot(nodes_axis, theta_3_profile, 'r-o', markersize=4, label=r"$\theta_3$")
    for end_node in tube_end[:-1]:
        ax_edo_global.axvline(x=end_node, color='gray', linestyle='--', alpha=0.5)
    ax_edo_global.legend(loc="lower left")

    # ---- 4. EDO VUE DÉTAILLÉE [Bas Droite] ----
    ax_edo_detail.set_xlim(0, num_nodes - 1)
    ax_edo_detail.relim()
    ax_edo_detail.autoscale_view(True, True, True) # Zoom automatique actif !
    ax_edo_detail.set_xlabel("Nœuds d'intégration")
    ax_edo_detail.set_ylabel("Variables EDO (Zoom micro)")
    ax_edo_detail.grid(True)
    ax_edo_detail.set_title("EDO : Vue Micro (Détail des paliers)")
    ax_edo_detail.plot(nodes_axis, theta_2_profile, 'b-o', markersize=4, label=r"$\theta_2$")
    ax_edo_detail.plot(nodes_axis, theta_3_profile, 'r-o', markersize=4, label=r"$\theta_3$")
    for end_node in tube_end[:-1]:
        ax_edo_detail.axvline(x=end_node, color='gray', linestyle='--', alpha=0.5)
    ax_edo_detail.legend(loc="best")
    
    # Rafraîchissement synchrone
    fig.tight_layout()
    plt.draw()
    plt.pause(0.001)

plt.ioff()
print("Animation terminée. Dashboard complet prêt pour l'analyse.")
plt.show()