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
        r_tube
    ):

        scale = 1000

        # Tube externe (Zone à 3 tubes)
        self.ax_robot.plot(
            x[:end_ext],
            y[:end_ext],
            z[:end_ext],
            linewidth=(2 * r_tube[2]) * scale * 3,
            color="red",
            solid_capstyle="butt" # Coupe nette des extrémités
        )

        # Tube intermédiaire (Zone à 2 tubes)
        self.ax_robot.plot(
            x[end_ext - 1:end_mid],
            y[end_ext - 1:end_mid],
            z[end_ext - 1:end_mid],
            linewidth=(2 * r_tube[1]) * scale * 2,
            color="blue",
            solid_capstyle="butt" # Coupe nette des extrémités
        )

        # Tube interne (Zone à 1 tube / Pointe)
        self.ax_robot.plot(
            x[end_mid - 1:end_int],
            y[end_mid - 1:end_int],
            z[end_mid - 1:end_int],
            linewidth=(2 * r_tube[0]) * scale,
            color='#08ff08',
            solid_capstyle="butt" # Coupe nette des extrémités
        )
        
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