import numpy as np


class CTRGraphs:

    def __init__(self, ax_plots, l_kappa):
        self.ax_plots = ax_plots
        self.l_kappa = l_kappa
    
    def plot_tip_orientation_history(
        self,
        history
    ):
        steps = np.arange(
            1,
            len(history["x"]) + 1
        )

        self.ax_plots.plot(
            steps,
            history["x"],
            'ro-',
            label="Tip / X"
        )

        self.ax_plots.plot(
            steps,
            history["y"],
            'go-',
            label="Tip / Y"
        )

        self.ax_plots.plot(
            steps,
            history["z"],
            'bo-',
            label="Tip / Z"
        )

        self.ax_plots.set_title(
            "Tip orientation during motion"
        )

        self.ax_plots.set_xlabel(
            "Motion step"
        )

        self.ax_plots.set_ylabel(
            "Angle (deg)"
        )



    def plot_orientation(
        self,
        matrix_data,
        length_axis,
        num_nodes
    ):

        orient_X = np.zeros(num_nodes)
        orient_Y = np.zeros(num_nodes)
        orient_Z = np.zeros(num_nodes)

        for i in range(num_nodes):

            R = matrix_data[3:12, i].reshape(
                (3, 3),
                order="C"
            )

            t_x = R[0, 0]
            t_y = R[1, 1]
            t_z = R[2, 2]

            orient_X[i] = np.degrees(
                np.arccos(
                    np.clip(t_x, -1.0, 1.0)
                )
            )

            orient_Y[i] = np.degrees(
                np.arccos(
                    np.clip(t_y, -1.0, 1.0)
                )
            )

            orient_Z[i] = np.degrees(
                np.arccos(
                    np.clip(t_z, -1.0, 1.0)
                )
            )

        self.ax_plots.plot(
            length_axis,
            orient_X,
            "r-",
            linewidth=1,
            label="Axe X"
        )

        self.ax_plots.plot(
            length_axis,
            orient_Y,
            "g-",
            linewidth=1,
            label="Axe Y"
        )

        self.ax_plots.plot(
            length_axis,
            orient_Z,
            "b-",
            linewidth=1,
            label="Axe Z"
        )

        self.ax_plots.set_title(
            "Orientation along the CTR",
            fontsize=11,
            fontweight="bold"
        )

        self.ax_plots.set_ylabel(
            "Angle de référence (degrés)",
            labelpad=12
        )

        # self.ax_plots.set_ylim([0, 120])

        # 1. On regroupe toutes les valeurs pour trouver le min et le max réels
        min_y = np.min([orient_X, orient_Y, orient_Z])
        max_y = np.max([orient_X, orient_Y, orient_Z])
        
        # 2. On calcule une marge de 5% pour que les courbes ne collent pas aux bords
        margin = 0.05 * (max_y - min_y)
        if margin < 5.0: 
            margin = 5.0 # Marge minimale de 5 degrés si le robot bouge peu

        # 3. On applique les limites en sécurisant pour ne pas dépasser les limites physiques [0, 180]
        self.ax_plots.set_ylim(
            max(0, min_y - margin), 
            min(180, max_y + margin)
        )

    def plot_twist_distribution(
        self,
        data
    ):
    
        # Torsion relative
        theta_2 = data.theta_2
        theta_3 = data.theta_3

        t2_display = np.copy(theta_2)
        t3_display = np.copy(theta_3)

        t3_display[data.end_ext:] = np.nan
        t2_display[data.end_mid:] = np.nan

        self.ax_plots.plot(data.length_axis, t2_display, 'b-', markersize=2, label=r"$\theta_2(s)$")
        self.ax_plots.plot(data.length_axis, t3_display, 'r-', markersize=2, label=r"$\theta_3(s)$")

        # self._set_twist_ylim(
        #     t2_display,
        #     t3_display
        # )

        l_transition1 = data.length_axis[data.end_ext - 1]
        l_transition2 = data.length_axis[data.end_mid - 1]

        self.ax_plots.axvline(x=l_transition1, color='red', linestyle='--', alpha=0.7, label="End T3 (Ext)")
        self.ax_plots.axvline(x=l_transition2, color='blue', linestyle='--', alpha=0.7, label="End T2 (Mid)")


        # Zones de précourbure
        # Zone de précourbure Tube 3 (Externe)
        start_curve_t3 = l_transition1 - self.l_kappa
        if start_curve_t3 >= 0:
            # AJOUT : Partie DROITE du Tube 3 (Foncée)
            self.ax_plots.axvspan(0, start_curve_t3, color='orange', alpha=0.12, zorder=1)
            self.ax_plots.axvline(x=start_curve_t3,color='red', linestyle=':', linewidth=2.5, label=r"Start curve T3")
            self.ax_plots.axvspan(start_curve_t3, l_transition1, color='red', alpha=0.07, zorder=1)

        # Zone de précourbure Tube 2 (Intermédiaire)
        start_curve_t2 = l_transition2 - self.l_kappa
        if start_curve_t2 >= 0:
            # AJOUT : Partie DROITE du Tube 2 (Foncée)
            self.ax_plots.axvspan(0, start_curve_t2, color='purple', alpha=0.08, zorder=1)
            self.ax_plots.axvline(x=start_curve_t2, color='blue', linestyle=':', linewidth=2.5, label=r"Start curve T2")
            self.ax_plots.axvspan(start_curve_t2, l_transition2, color='blue', alpha=0.04, zorder=1)

        # Zone de précourbure Tube 1 (Interne)
        l_end_t1 = data.length_axis[-1]
        start_curve_t1 = l_end_t1 - self.l_kappa
        if start_curve_t1 >= 0:
            # AJOUT : Partie DROITE du Tube 1 (Foncée)
            self.ax_plots.axvspan(0, start_curve_t1, color='yellow', alpha=0.06, zorder=1)
            self.ax_plots.axvline(x=start_curve_t1, color='green', linestyle=':', linewidth=2.5, label=r"Start curve T1 (Int)")
            self.ax_plots.axvspan(start_curve_t1, l_end_t1, color='black', alpha=0.03, zorder=1)
            # Ligne de fin désormais visible grâce à la marge de 5%
            self.ax_plots.axvline(x=l_end_t1, color='green', linestyle='--', alpha=0.7, label="End T1 (Tip)")


        # # Zones de précourbure
        # # Zone de précourbure Tube 3 (Externe)
        # start_curve_t3 = l_transition1 - self.l_kappa
        # if start_curve_t3 >= 0:
        #     self.ax_plots.axvline(x=start_curve_t3,color='darkorange', linestyle=':', linewidth=2.5, label=r"Start curve T3")
        #     self.ax_plots.axvspan(start_curve_t3, l_transition1, color='orange', alpha=0.07, zorder=1)

        # # Zone de précourbure Tube 2 (Intermédiaire)
        # start_curve_t2 = l_transition2 - self.l_kappa
        # if start_curve_t2 >= 0:
        #     self.ax_plots.axvline(x=start_curve_t2, color='purple', linestyle=':', linewidth=2.5, label=r"Start curve T2")
        #     self.ax_plots.axvspan(start_curve_t2, l_transition2, color='purple', alpha=0.04, zorder=1)

        # # Zone de précourbure Tube 1 (Interne)
        # l_end_t1 = data.length_axis[-1]
        # start_curve_t1 = l_end_t1 - self.l_kappa
        # if start_curve_t1 >= 0:
        #     self.ax_plots.axvline(x=start_curve_t1, color='green', linestyle=':', linewidth=2.5, label=r"Start curve T1 (Int)")
        #     self.ax_plots.axvspan(start_curve_t1, l_end_t1, color='green', alpha=0.03, zorder=1)
        #     # Ligne de fin désormais visible grâce à la marge de 5%
        #     self.ax_plots.axvline(x=l_end_t1, color='green', linestyle='--', alpha=0.7, label="End T1 (Tip)")


        self.ax_plots.set_title("Angles de torsion des tubes le long du CTR", fontsize=11, fontweight='bold')
        self.ax_plots.set_ylabel("Twist angle (rad)", labelpad=10)

        # Limites dynamiques
        min_y = np.nanmin([t2_display, t3_display])
        max_y = np.nanmax([t2_display, t3_display])

        self.ax_plots.set_ylim(
            min_y - 0.05,
            max_y + 0.05
        )

        # Préviens le cas où les torsions sont petites
        margin = 0.1 * (max_y - min_y)

        if margin < 1e-3:
            margin = 1e-3

        self.ax_plots.set_ylim(
            min_y - margin,
            max_y + margin
        )

        self.ax_plots.scatter(
            data.length_axis[data.end_mid-1],
            theta_2[data.end_mid-1],
            color='blue',
            s=40
        )

        self.ax_plots.scatter(
            data.length_axis[data.end_ext-1],
            theta_3[data.end_ext-1],
            color='red',
            s=40
        )

        self.ax_plots.annotate(
            f"{theta_2[data.end_mid-1]:.4f} rad",
            (
                data.length_axis[data.end_mid-1],
                theta_2[data.end_mid-1]
            ),
            xytext=(10,-15),
            textcoords="offset points",
            color="blue"
        )

        self.ax_plots.annotate(
            f"{theta_3[data.end_ext-1]:.4f} rad",
            (
                data.length_axis[data.end_ext-1],
                theta_3[data.end_ext-1]
            ),
            xytext=(10,-15),
            textcoords="offset points",
            color="red"
        )

