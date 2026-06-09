import numpy as np

class CTRVisualizer:

    def __init__(self, ax_robot):
        self.ax_robot = ax_robot

    def draw_robot(
        self,
        x,
        y,
        z,
        end_ext,
        end_mid,
        end_int,
        r_tube,
        l_kappa_tubes,
        l_tubes,
        length_axis,
        q # translations
    ):
        scale = 1000
        print(l_kappa_tubes)
        print(f"DEBUG: end_ext={end_ext}, end_mid={end_mid}, end_int={end_int}")
        print(f"Tailles segments : Ext={end_ext}, Mid={end_mid-end_ext}, Int={end_int-end_mid}")

        # tube 3 : externe (Zone de la base à end_ext)
        x3 = x[:end_ext]
        y3 = y[:end_ext]
        z3 = z[:end_ext]

        # 1. Trouver la position globale de la pointe du tube 3
        pos_pointe_3 = length_axis[end_ext - 1]
        # 2. Soustraire la longueur courbée pour trouver le début de la courbure
        pos_debut_courbure_3 = pos_pointe_3 - l_kappa_tubes[2]
        # 3. Trouver l'index global le plus proche de cette position
        idx_global_3 = np.argmin(np.abs(length_axis - pos_debut_courbure_3))
        # 4. Pour le tube 3, l'index local est égal à l'index global
        idx_local_3 = idx_global_3
        # 5. Sécurité pour ne pas déborder du segment
        if idx_local_3 < 0: idx_local_3 = 0
        if idx_local_3 > len(x3): idx_local_3 = len(x3)

        # Dessin Tube 3
        self.ax_robot.plot(
            x3[:idx_local_3],
            y3[:idx_local_3],
            z3[:idx_local_3],
            linewidth=(2 * r_tube[2]) * scale * 3, color="#ff0000")
        self.ax_robot.plot(
            x3[idx_local_3:],
            y3[idx_local_3:],
            z3[idx_local_3:],
            linewidth=(2 * r_tube[2]) * scale * 3, color="#8c0000")


        # tube 2 : intermédiaire (Zone de end_ext à end_mid)
        x2 = x[end_ext:end_mid]
        y2 = y[end_ext:end_mid]
        z2 = z[end_ext:end_mid]

        # 1. Trouver la position globale de la pointe du tube 2
        pos_pointe_2 = length_axis[end_mid - 1]
        # 2. Soustraire la longueur courbée
        pos_debut_courbure_2 = pos_pointe_2 - l_kappa_tubes[1]
        # 3. Trouver l'index global
        idx_global_2 = np.argmin(np.abs(length_axis - pos_debut_courbure_2))
        # 4. Convertir en index LOCAL (on soustrait le décalage du début du segment)
        idx_local_2 = idx_global_2 - end_ext
        # 5. Sécurité : si la courbure a commencé avant (dans la zone 3), idx_local sera négatif -> on le force à 0
        if idx_local_2 < 0: idx_local_2 = 0
        if idx_local_2 > len(x2): idx_local_2 = len(x2)

        # Dessin Tube 2
        self.ax_robot.plot(
            x2[:idx_local_2],
            y2[:idx_local_2],
            z2[:idx_local_2],
            linewidth=(2 * r_tube[1]) * scale * 2, color="#00f2ff")
        self.ax_robot.plot(
            x2[idx_local_2:],
            y2[idx_local_2:],
            z2[idx_local_2:],
            linewidth=(2 * r_tube[1]) * scale * 2, color="#00a2ab")


        # tube 1 : interne (Zone de end_mid à end_int)
        x1 = x[end_mid:end_int]
        y1 = y[end_mid:end_int]
        z1 = z[end_mid:end_int]

        # 1. Trouver la position globale de la pointe du tube 1
        pos_pointe_1 = length_axis[end_int - 1]
        # 2. Soustraire la longueur courbée
        pos_debut_courbure_1 = pos_pointe_1 - l_kappa_tubes[0]
        # 3. Trouver l'index global
        idx_global_1 = np.argmin(np.abs(length_axis - pos_debut_courbure_1))
        # 4. Convertir en index LOCAL (on soustrait le décalage du début du segment)
        idx_local_1 = idx_global_1 - end_mid
        # 5. Sécurité
        if idx_local_1 < 0: idx_local_1 = 0
        if idx_local_1 > len(x1): idx_local_1 = len(x1)

        # Dessin Tube 1
        self.ax_robot.plot(
            x1[:idx_local_1],
            y1[:idx_local_1],
            z1[:idx_local_1],
            linewidth=(2 * r_tube[0]) * scale, color='#33ff00')
        self.ax_robot.plot(
            x1[idx_local_1:],
            y1[idx_local_1:],
            z1[idx_local_1:],
            linewidth=(2 * r_tube[0]) * scale, color='#1e9600')


        # Pointe
        self.ax_robot.scatter(
            x[-1],
            y[-1],
            z[-1],
            s=40
        )

        # === MULTI-MODIFICATION : FIXATION DES AXES (GRILLE STATIQUE) ===
        # Les dimensions sont ici exprimées en MÈTRES (car axis_len = 0.03 soit 3 cm)
        
        XY_BOITE = 0.08   # Largeur du cadre (gère l'amplitude gauche/droite à +/- 8 cm)
        Z_MIN_BOITE = -0.15 # Profondeur max pour voir les chariots descendre (ici -15 cm)
        Z_MAX_BOITE = 0.35  # Hauteur max du graphique (ici 35 cm)

        # On applique les limites strictes à la caméra
        self.ax_robot.set_xlim([-XY_BOITE, XY_BOITE])
        self.ax_robot.set_ylim([-XY_BOITE, XY_BOITE])
        self.ax_robot.set_zlim([Z_MIN_BOITE, Z_MAX_BOITE])

        # On force un aspect visuel proportionnel (Évite que le robot soit aplati ou étiré)
        hauteur_totale = Z_MAX_BOITE - Z_MIN_BOITE
        largeur_totale = 2 * XY_BOITE
        self.ax_robot.set_box_aspect((1, 1, hauteur_totale / largeur_totale))
    

    def draw_chariots(self, q_values):
            # q_values[0]=q1 (Int), q_values[1]=q2 (Mid), q_values[2]=q3 (Ext)
            z_c1 = q_values[0]
            z_c2 = q_values[1]
            z_c3 = q_values[2]

            # 1. Dessin du rail de guidage central (solide et discret)
            z_min = min(z_c1, z_c2, z_c3) - 0.02
            self.ax_robot.plot([0, 0], [0, 0], [z_min, 0], color="dimgray", linestyle="-", linewidth=1.5, alpha=0.6)

            # 2. Dessin des Chariots sous forme de disques d'embases (plus réaliste)
            # s=80 réduit la taille des blocs pour qu'ils s'ajustent à la taille du robot
            self.ax_robot.scatter(0, 0, z_c1, color="lightgray", marker="o", s=90, edgecolors="black", zorder=4, label="Chariot T1")
            self.ax_robot.scatter(0, 0, z_c2, color="dimgray", marker="o", s=110, edgecolors="black", zorder=4, label="Chariot T2")
            self.ax_robot.scatter(0, 0, z_c3, color="black", marker="s", s=70, edgecolors="black", zorder=4, label="Chariot T3")

    def draw_tip_frame(
    self,
    tip_x,
    tip_y,
    tip_z,
    R_tip
    ):

        axis_len_tip = 0.02

        # Axe X
        self.ax_robot.quiver(
            tip_x,
            tip_y,
            tip_z,
            R_tip[0, 0] * axis_len_tip,
            R_tip[1, 0] * axis_len_tip,
            R_tip[2, 0] * axis_len_tip,
            color="r",
            linewidth=1
        )

        # Axe Y
        self.ax_robot.quiver(
            tip_x,
            tip_y,
            tip_z,
            R_tip[0, 1] * axis_len_tip,
            R_tip[1, 1] * axis_len_tip,
            R_tip[2, 1] * axis_len_tip,
            color="g",
            linewidth=1
        )

        # Axe Z
        self.ax_robot.quiver(
            tip_x,
            tip_y,
            tip_z,
            R_tip[0, 2] * axis_len_tip,
            R_tip[1, 2] * axis_len_tip,
            R_tip[2, 2] * axis_len_tip,
            color="b",
            linewidth=1
        )
    
    def draw_world_frame(self):

        axis_len = 0.03

        self.ax_robot.quiver(
            0, 0, 0,
            axis_len, 0, 0,
            color="r"
        )

        self.ax_robot.quiver(
            0, 0, 0,
            0, axis_len, 0,
            color="g"
        )

        self.ax_robot.quiver(
            0, 0, 0,
            0, 0, axis_len,
            color="b"
        )

    def draw_ghost(
        self,
        ghost_robot,
        r_tube
    ):

        if ghost_robot is None:
            return

        scale = 1000

        gx = ghost_robot["x"]
        gy = ghost_robot["y"]
        gz = ghost_robot["z"]

        g_ext = ghost_robot["end_ext"]
        g_mid = ghost_robot["end_mid"]
        g_int = ghost_robot["end_int"]


        self.ax_robot.plot(
            gx[:g_ext],
            gy[:g_ext],
            gz[:g_ext],
            color="cyan",
            alpha=0.25,
            linewidth=(2 * r_tube[2]) * scale * 3,
            solid_capstyle="butt" # Coupe nette des extrémités

        )

        self.ax_robot.plot(
            gx[g_ext - 1:g_mid],
            gy[g_ext - 1:g_mid],
            gz[g_ext - 1:g_mid],
            color="cyan",
            alpha=0.25,
            linewidth=(2 * r_tube[1]) * scale * 2,
            solid_capstyle="butt" # Coupe nette des extrémités
        )

        self.ax_robot.plot(
            gx[g_mid - 1:g_int],
            gy[g_mid - 1:g_int],
            gz[g_mid - 1:g_int],
            color="cyan",
            alpha=0.25,
            linewidth=(2 * r_tube[0]) * scale,
            solid_capstyle="butt" # Coupe nette des extrémités
        )