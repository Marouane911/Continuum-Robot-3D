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

# --- 2. PRÉPARATION DE L'ÉCRAN PARTAGÉ (Architecture Dashboard) ---
plt.ion()
# Création d'une figure plus haute pour accueillir les deux graphiques
fig = plt.figure(figsize=(8, 10))

# Zone Supérieure (211 = 2 lignes, 1 colonne, position 1) -> ROBOT 3D
ax_robot = fig.add_subplot(211, projection='3d')

# Zone Inférieure (212 = 2 lignes, 1 colonne, position 2) -> GRAPHIQUE 2D (yTot)
ax_graph = fig.add_subplot(212)

# Préparation temporaire du graphique du bas pour que l'affichage soit propre
ax_graph.set_xlim(0, len(steps_data))
ax_graph.set_ylim(-np.pi, np.pi) # Exemple d'échelle pour un angle de torsion
ax_graph.set_xlabel("Étape Temporelle (Time step)")
ax_graph.set_ylabel("Paramètre de torsion (rad) - À venir")
ax_graph.grid(True)
ax_graph.set_title("Évolution temporelle des paramètres de yTot")

print("Lancement de l'animation pré-structurée...")

for step_idx, step_str in enumerate(steps_data):
    # On nettoie uniquement la zone du robot, pas tout l'écran !
    ax_robot.cla() 
    
    matrix_lines = [list(map(float, line.split())) for line in step_str.split('\n') if line.strip()]
    matrix_data = np.array(matrix_lines)
    num_nodes = matrix_data.shape[1]
    
    # Configuration des nœuds (NB_INTEGRATION_NODES = 30)
    nodes_per_segment = int(num_nodes / 6)
    tube_end = np.array([2 * nodes_per_segment, 4 * nodes_per_segment, num_nodes])
    
    # ---- EXTRACTION GÉOMÉTRIQUE ET PHYSIQUE COMPLÈTE ----
    g = np.zeros((num_nodes, 16))
    for i in range(num_nodes):
        M = np.eye(4)
        # Position exacte
        M[0, 3] = matrix_data[0, i]
        M[1, 3] = matrix_data[1, i]
        M[2, 3] = matrix_data[2, i]
        
        # Vraie rotation R issue de Quentin (lignes 3 à 11)
        R_elements = matrix_data[3:12, i] 
        M[0:3, 0:3] = R_elements.reshape((3, 3), order='F')
        
        g[i, :] = M.flatten(order='F')
    
    # ---- AFFICHAGE DU ROBOT DANS LA ZONE SUPÉRIEURE ----
    draw_ctcr(g, tube_end, r_tube, tipframe=True, ax=ax_robot)
    
    # Configuration de la boîte 3D (ax_robot uniquement)
    ax_robot.set_aspect('equal')
    ax_robot.set_box_aspect((1, 1, 1))
    ax_robot.set_xlim3d([-0.09, 0.09])
    ax_robot.set_ylim3d([-0.09, 0.09])
    ax_robot.set_zlim3d([0.0, 0.18])
    ax_robot.view_init(elev=25, azim=-60)
    
    ax_robot.set_xlabel('X (m)')
    ax_robot.set_ylabel('Y (m)')
    ax_robot.set_zlabel('Z (m)')
    ax_robot.set_title(f"Visualisation 3D - Étape {step_idx + 1}/{len(steps_data)}")
    
    # ---- ANIMATION TEMPORAIRE DE LA ZONE DU BAS ----
    # On ajoute juste un point qui avance pour valider que le temps défile de manière synchrone
    if step_idx == 0:
        # On crée le point au premier passage
        cursor, = ax_graph.plot(step_idx, 0, 'ro', markersize=8, label="Position actuelle")
        ax_graph.legend()
    else:
        # On met à jour sa position sans effacer le graphique (gain de vitesse énorme)
        cursor.set_data([step_idx], [0]) # Le 0 sera remplacé par la vraie valeur de yTot plus tard
    
    # Rafraîchissement de la figure globale
    plt.draw()
    plt.pause(0.01)

plt.ioff()
print("Animation terminée. En attente de fermeture de la fenêtre...")
plt.show()
