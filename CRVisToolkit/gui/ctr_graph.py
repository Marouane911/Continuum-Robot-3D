import numpy as np
import warnings
warnings.filterwarnings("ignore", message="No artists with labels")

class CTRGraphs:

    def __init__(self, ax_plots, l_kappa):
        self.ax_plots = ax_plots
        self.l_kappa = l_kappa
        self._u_ref = None
    
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
            fig = self.ax_plots.figure

            # =====================================================================
            # 1. SÉCURITÉ & NETTOYAGE DYNAMIQUE (Pour la compatibilité Dropdown)
            # =====================================================================
            # On intercepte le .clear() d'origine pour nettoyer nos sous-graphes 
            # dès qu'on change d'item dans le menu déroulant
            if not hasattr(self, '_clear_patched'):
                self._orig_clear = self.ax_plots.clear
                def custom_clear():
                    self._orig_clear()
                    self.ax_plots.set_visible(True) # Réaffiche le graphe standard
                    if hasattr(self, 'ax_theta') and self.ax_theta in fig.axes:
                        self.ax_theta.remove()
                        self.ax_u.remove()
                        del self.ax_theta
                        del self.ax_u
                self.ax_plots.clear = custom_clear
                self._clear_patched = True

            # Si les axes temporaires existent déjà, on les nettoie avant de redessiner
            if hasattr(self, 'ax_theta') and self.ax_theta in fig.axes:
                self.ax_theta.remove()
                self.ax_u.remove()

            # On cache l'axe principal pour laisser la place à notre double-graphe
            self.ax_plots.set_visible(False)

            # Division de la zone droite (122) en deux sous-graphes superposés (222 et 224)
            # Le robot en 3D (121) reste intact sur la moitié gauche !
            self.ax_theta = fig.add_subplot(222)
            self.ax_u = fig.add_subplot(224)

            # u, theta
            theta_data = np.zeros(num_nodes)
            ux_data = np.zeros(num_nodes)
            uy_data = np.zeros(num_nodes)
            uz_data = np.zeros(num_nodes)

            # for i in range(num_nodes):

            #     R = matrix_data[3:12, i].reshape((3, 3), order="C")

            #     # Vecteur axe (non normalisé) — contient déjà le signe
            #     ax = R[2, 1] - R[1, 2]
            #     ay = R[0, 2] - R[2, 0]
            #     az = R[1, 0] - R[0, 1]
                
            #     cos_theta = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
            #     sin_theta = 0.5 * np.sqrt(ax**2 + ay**2 + az**2) 

            #     theta = np.arctan2(sin_theta, cos_theta)
            #     theta_data[i] = np.degrees(theta)

            #     if sin_theta > 1e-5:
            #         ux_data[i] = ax / (2 * sin_theta)
            #         uy_data[i] = ay / (2 * sin_theta)
            #         uz_data[i] = az / (2 * sin_theta)

            #     else:
            #         ux_data[i], uy_data[i], uz_data[i] = 1.0, 0.0, 0.0











# ---------------------------------------------------------------
            # 1. Extraction brute des angles et axes
            # ---------------------------------------------------------------
            for i in range(num_nodes):
                R = matrix_data[3:12, i].reshape((3, 3), order="C")

                ax = R[2, 1] - R[1, 2]
                ay = R[0, 2] - R[2, 0]
                az = R[1, 0] - R[0, 1]

                cos_theta = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
                sin_theta = 0.5 * np.sqrt(ax**2 + ay**2 + az**2)

                theta = np.arctan2(sin_theta, cos_theta)
                theta_data[i] = np.degrees(theta)

                if sin_theta > 0.01:
                    ux_data[i] = ax / (2.0 * sin_theta)
                    uy_data[i] = ay / (2.0 * sin_theta)
                    uz_data[i] = az / (2.0 * sin_theta)

                elif cos_theta < 0:  # Singularité θ ≈ 180°
                    S = (R + R.T) / 2.0
                    diag = np.clip(np.diag(S) + 1.0, 0.0, None)
                    u_raw = np.sqrt(diag / 2.0)
                    if S[0, 1] < 0: u_raw[1] *= -1
                    if S[0, 2] < 0: u_raw[2] *= -1
                    norm = np.linalg.norm(u_raw)
                    u_raw = u_raw / norm if norm > 1e-10 else u_raw
                    ux_data[i], uy_data[i], uz_data[i] = u_raw

                else:  # Portion droite θ ≈ 0°
                    # On marque comme indéfini (0,0,0) pour le moment
                    ux_data[i], uy_data[i], uz_data[i] = 0.0, 0.0, 0.0

            # ---------------------------------------------------------------
            # 2. Rétro-propagation : Assigner le vrai plan de courbure aux parties droites
            # ---------------------------------------------------------------
            # On cherche le premier noeud où l'axe est bien défini
            valid_indices = np.where(np.abs(ux_data) + np.abs(uy_data) + np.abs(uz_data) > 0.1)[0]
            
            if len(valid_indices) > 0:
                first_valid = valid_indices[0]
                # On propage l'axe vers la base du robot (rétro-propagation)
                for i in range(first_valid - 1, -1, -1):
                    ux_data[i], uy_data[i], uz_data[i] = ux_data[i+1], uy_data[i+1], uz_data[i+1]
                
                # On propage vers l'extrémité s'il y a des trous au milieu (ex: sections droites intermédiaires)
                for i in range(first_valid + 1, num_nodes):
                    if np.abs(ux_data[i]) + np.abs(uy_data[i]) + np.abs(uz_data[i]) < 0.1:
                        ux_data[i], uy_data[i], uz_data[i] = ux_data[i-1], uy_data[i-1], uz_data[i-1]
            else:
                # Le robot est 100% droit. L'axe de flexion par défaut doit être orthogonal à Z.
                ux_data.fill(1.0) # Axe X, pas Z !
                uy_data.fill(0.0)
                uz_data.fill(0.0)

            # ---------------------------------------------------------------
            # 3. Continuité intra-appel (Lissage mathématique)
            # ---------------------------------------------------------------
            for i in range(1, num_nodes):
                prev = np.array([ux_data[i-1], uy_data[i-1], uz_data[i-1]])
                curr = np.array([ux_data[i],   uy_data[i],   uz_data[i]])
                
                if np.dot(prev, curr) < 0:
                    ux_data[i] *= -1
                    uy_data[i] *= -1
                    uz_data[i] *= -1
                    # IMPORTANT : Si on inverse l'axe, il FAUT inverser l'angle pour que la rotation
                    # reste identique dans l'espace 3D, sinon la dérivée de theta sera cassée.
                    theta_data[i] *= -1

            # ---------------------------------------------------------------
            # 4. Alignement inter-appels (Cohérence temporelle dans l'IHM)
            # ---------------------------------------------------------------
            # On vérifie avec le noeud final (organe terminal)
            u_tip = np.array([ux_data[-1], uy_data[-1], uz_data[-1]])
            
            if self._u_ref is not None:
                if np.dot(self._u_ref, u_tip) < 0:
                    ux_data *= -1
                    uy_data *= -1
                    uz_data *= -1
                    theta_data *= -1

            # Mémoriser la référence proprement
            if np.abs(theta_data[-1]) > 5.0:
                self._u_ref = np.array([ux_data[-1], uy_data[-1], uz_data[-1]])
            else:
                self._u_ref = None































































            # SOUS-GRAPHES
            # --- GRAPHIQUE DU HAUT : AMPLITUDE THETA ---
            self.ax_theta.plot(length_axis, theta_data, "k-", linewidth=1.5, label=r"$\theta$")
            self.ax_theta.set_title("Orientation du CTR", fontsize=11, fontweight="bold")
            self.ax_theta.set_ylabel("Angle de déviation $\theta$ (°)")
            self.ax_theta.set_xlabel("Position le long du robot (m)")
            self.ax_theta.grid(True, linestyle="--", alpha=0.5)
            
            # Marge dynamique pour l'affichage de Theta
            min_t, max_t = np.min(theta_data), np.max(theta_data)
            margin_t = max(5.0, 0.05 * (max_t - min_t))
            self.ax_theta.set_ylim(max(0, min_t - margin_t), min(180, max_t + margin_t))

            # --- GRAPHIQUE DU BAS : DIRECTION DE L'AXE U ---
            self.ax_u.plot(length_axis, ux_data, "r-", linewidth=1.2, label="$u_x$")
            self.ax_u.plot(length_axis, uy_data, "g-", linewidth=1.2, label="$u_y$")
            self.ax_u.plot(length_axis, uz_data, "b-", linewidth=1.2, label="$u_z$")
            self.ax_u.set_xlabel("Position le long du robot (m)")
            self.ax_u.set_ylabel("Composantes de l'axe $\mathbf{u}$")
            self.ax_u.set_ylim(-1.05, 1.05) # Un vecteur unitaire oscille strictement entre -1 et 1
            self.ax_u.grid(True, linestyle="--", alpha=0.5)
            self.ax_u.legend(loc="lower left", fontsize=9)

            # Ajustement de l'espacement vertical pour éviter les chevauchements
            fig.subplots_adjust(hspace=0.3)
            
            # Rafraîchissement du canvas
            fig.canvas.draw()


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

