import numpy as np
import warnings
warnings.filterwarnings("ignore", message="No artists with labels")

from scipy.spatial.transform import Rotation
import numpy as np

    

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



    @staticmethod
    def _extract_axis_angle(R):
        """
        Extrait (theta, u) depuis une matrice de rotation 3x3.
 
        Trois régimes :
          - Régime normal    : formule antisymétrique standard
          - θ ≈ π (180°)    : vecteur propre de R associé à la valeur propre +1
                               (seule méthode mathématiquement stable à cette singularité)
          - θ ≈ 0°          : retourne (0.0, [0,0,0]) — axe physiquement indéfini,
                               sera comblé par la propagation spatiale
 
        Retourne : (theta_deg : float, u : ndarray shape (3,))
        """
        cos_theta = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
 
        # Partie antisymétrique : vecteur [ax, ay, az] = 2 sin(θ) · u
        ax = R[2, 1] - R[1, 2]
        ay = R[0, 2] - R[2, 0]
        az = R[1, 0] - R[0, 1]
        sin_theta = 0.5 * np.sqrt(ax**2 + ay**2 + az**2)
 
        theta = np.arctan2(sin_theta, cos_theta)  # θ ∈ [0, π]
 
        # --- Régime normal : sin(θ) suffisamment grand ---
        if sin_theta > 1e-4:
            u = np.array([ax, ay, az]) / (2.0 * sin_theta)
            return np.degrees(theta), u
 
        # --- Singularité θ ≈ π : vecteur propre de R pour valeur propre +1 ---
        # R·u = u  ⟺  (R - I)·u = 0
        # On résout via SVD : u est le vecteur singulier droit associé
        # à la plus petite valeur singulière de (R - I).
        if cos_theta < 0.0:
            _, _, Vt = np.linalg.svd(R - np.eye(3))
            u = Vt[-1]  # Dernière ligne de Vt = vecteur singulier pour σ_min
            u = u / np.linalg.norm(u)
            return np.degrees(theta), u
 
        # --- Singularité θ ≈ 0° : axe indéfini ---
        return 0.0, np.zeros(3)
 
    def plot_orientation(
            self,
            matrix_data,
            length_axis,
            num_nodes
        ):
            fig = self.ax_plots.figure
 
            # =====================================================================
            # SÉCURITÉ & NETTOYAGE DYNAMIQUE (compatibilité Dropdown)
            # =====================================================================
            if not hasattr(self, '_clear_patched'):
                self._orig_clear = self.ax_plots.clear
                def custom_clear():
                    self._orig_clear()
                    self.ax_plots.set_visible(True)
                    if hasattr(self, 'ax_theta') and self.ax_theta in fig.axes:
                        self.ax_theta.remove()
                        self.ax_u.remove()
                        del self.ax_theta
                        del self.ax_u
                self.ax_plots.clear = custom_clear
                self._clear_patched = True
 
            if hasattr(self, 'ax_theta') and self.ax_theta in fig.axes:
                self.ax_theta.remove()
                self.ax_u.remove()
 
            self.ax_plots.set_visible(False)
            self.ax_theta = fig.add_subplot(222)
            self.ax_u = fig.add_subplot(224)
 
            # ---------------------------------------------------------------
            # ÉTAPE 1 : Extraction géométrique pure — sans état, sans mémoire
            # ---------------------------------------------------------------
            theta_data = np.zeros(num_nodes)
            u_data = np.zeros((num_nodes, 3))  # Chaque ligne = [ux, uy, uz]
 
            for i in range(num_nodes):
                R = matrix_data[3:12, i].reshape((3, 3), order="C")
                theta_data[i], u_data[i] = self._extract_axis_angle(R)
 
            # ---------------------------------------------------------------
            # ÉTAPE 2 : Propagation spatiale des zones θ ≈ 0° (axe indéfini)
            # On ne remplace QUE les nœuds où u = [0,0,0].
            # θ reste à 0° dans ces zones — c'est physiquement correct.
            # ---------------------------------------------------------------
            norms = np.linalg.norm(u_data, axis=1)  # 0.0 pour les zones indéfinies
            valid = np.where(norms > 0.5)[0]
 
            if len(valid) > 0:
                # Backward : base du robot ← premier axe valide
                for i in range(valid[0] - 1, -1, -1):
                    u_data[i] = u_data[i + 1]
                # Forward : trous intermédiaires → nœud précédent
                for i in range(1, num_nodes):
                    if np.linalg.norm(u_data[i]) < 0.5:
                        u_data[i] = u_data[i - 1]
            else:
                # Robot 100% droit : convention explicite
                u_data[:] = [1.0, 0.0, 0.0]
 
            # ---------------------------------------------------------------
            # ÉTAPE 3 : Cohérence spatiale intra-frame
            # (u, θ) et (-u, -θ) décrivent la même rotation.
            # On choisit le signe de u qui assure la continuité le long du robot,
            # et on ajuste θ en conséquence pour que la rotation soit préservée.
            # On ignore les nœuds où θ < 1° (zone numérique instable).
            # ---------------------------------------------------------------
            for i in range(1, num_nodes):
                if theta_data[i] < 1.0 or theta_data[i - 1] < 1.0:
                    continue
                if np.dot(u_data[i - 1], u_data[i]) < 0.0:
                    u_data[i] *= -1.0
                    theta_data[i] *= -1.0
 
            # ---------------------------------------------------------------
            # ÉTAPE 4 : Cohérence temporelle inter-frames
            # Entre deux appels successifs, la formule peut globalement inverser
            # tous les u. On détecte ça en comparant avec la frame précédente.
            # Nœud de référence : celui avec |θ| maximal dans (1°, 179°),
            # là où la décomposition est la plus stable numériquement.
            # ---------------------------------------------------------------
            theta_abs = np.abs(theta_data)
            stable_mask = (theta_abs > 1.0) & (theta_abs < 179.0)
 
            if np.any(stable_mask):
                ref_idx = np.where(stable_mask)[0][np.argmax(theta_abs[stable_mask])]
            else:
                # Tout est à ~0° ou ~180° : prendre le nœud le plus courbé disponible
                ref_idx = np.argmax(theta_abs)
 
            u_ref_current = u_data[ref_idx].copy()
 
            if self._u_ref is not None and np.dot(self._u_ref, u_ref_current) < 0.0:
                u_data   *= -1.0
                theta_data *= -1.0
 
            # Mémoriser uniquement si le nœud de référence est fiable
            if theta_abs[ref_idx] > 1.0:
                self._u_ref = u_data[ref_idx].copy()
            else:
                self._u_ref = None
 
            # ---------------------------------------------------------------
            # TRACÉ
            # ---------------------------------------------------------------
            self.ax_theta.plot(length_axis, theta_data, "k-", linewidth=1.5)
            self.ax_theta.set_title("Orientation du CTR", fontsize=11, fontweight="bold")
            self.ax_theta.set_ylabel("Angle de déviation $\\theta$ (°)")
            self.ax_theta.set_xlabel("Position le long du robot (m)")
            self.ax_theta.grid(True, linestyle="--", alpha=0.5)
 
            min_t, max_t = np.min(theta_data), np.max(theta_data)
            margin_t = max(5.0, 0.05 * abs(max_t - min_t))
            self.ax_theta.set_ylim(min_t - margin_t, max_t + margin_t)
 
            self.ax_u.plot(length_axis, u_data[:, 0], "r-", linewidth=1.2, label="$u_x$")
            self.ax_u.plot(length_axis, u_data[:, 1], "g-", linewidth=1.2, label="$u_y$")
            self.ax_u.plot(length_axis, u_data[:, 2], "b-", linewidth=1.2, label="$u_z$")
            self.ax_u.set_xlabel("Position le long du robot (m)")
            self.ax_u.set_ylabel("Composantes de l'axe $\\mathbf{u}$")
            self.ax_u.set_ylim(-1.05, 1.05)
            self.ax_u.grid(True, linestyle="--", alpha=0.5)
            self.ax_u.legend(loc="lower left", fontsize=9)
 
            fig.subplots_adjust(hspace=0.3)
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


    # def plot_orientation(
    #         self,
    #         matrix_data,
    #         length_axis,
    #         num_nodes
    #     ):
    #         fig = self.ax_plots.figure

    #         # =====================================================================
    #         # 1. SÉCURITÉ & NETTOYAGE DYNAMIQUE (Pour la compatibilité Dropdown)
    #         # =====================================================================
    #         # On intercepte le .clear() d'origine pour nettoyer nos sous-graphes 
    #         # dès qu'on change d'item dans le menu déroulant
    #         if not hasattr(self, '_clear_patched'):
    #             self._orig_clear = self.ax_plots.clear
    #             def custom_clear():
    #                 self._orig_clear()
    #                 self.ax_plots.set_visible(True) # Réaffiche le graphe standard
    #                 if hasattr(self, 'ax_theta') and self.ax_theta in fig.axes:
    #                     self.ax_theta.remove()
    #                     self.ax_u.remove()
    #                     del self.ax_theta
    #                     del self.ax_u
    #             self.ax_plots.clear = custom_clear
    #             self._clear_patched = True

    #         # Si les axes temporaires existent déjà, on les nettoie avant de redessiner
    #         if hasattr(self, 'ax_theta') and self.ax_theta in fig.axes:
    #             self.ax_theta.remove()
    #             self.ax_u.remove()

    #         # On cache l'axe principal pour laisser la place à notre double-graphe
    #         self.ax_plots.set_visible(False)

    #         # Division de la zone droite (122) en deux sous-graphes superposés (222 et 224)
    #         # Le robot en 3D (121) reste intact sur la moitié gauche !
    #         self.ax_theta = fig.add_subplot(222)
    #         self.ax_u = fig.add_subplot(224)

    #         # u, theta
    #         theta_data = np.zeros(num_nodes)
    #         ux_data = np.zeros(num_nodes)
    #         uy_data = np.zeros(num_nodes)
    #         uz_data = np.zeros(num_nodes)

    #         # for i in range(num_nodes):

    #         #     R = matrix_data[3:12, i].reshape((3, 3), order="C")

    #         #     # Vecteur axe (non normalisé) — contient déjà le signe
    #         #     ax = R[2, 1] - R[1, 2]
    #         #     ay = R[0, 2] - R[2, 0]
    #         #     az = R[1, 0] - R[0, 1]
                
    #         #     cos_theta = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    #         #     sin_theta = 0.5 * np.sqrt(ax**2 + ay**2 + az**2) 

    #         #     theta = np.arctan2(sin_theta, cos_theta)
    #         #     theta_data[i] = np.degrees(theta)

    #         #     if sin_theta > 1e-5:
    #         #         ux_data[i] = ax / (2 * sin_theta)
    #         #         uy_data[i] = ay / (2 * sin_theta)
    #         #         uz_data[i] = az / (2 * sin_theta)

    #         #     else:
    #         #         ux_data[i], uy_data[i], uz_data[i] = 1.0, 0.0, 0.0

    #         # 1. Extraction brute

    #         for i in range(num_nodes):
    #             R = matrix_data[3:12, i].reshape((3, 3), order="C")

    #             ax = R[2, 1] - R[1, 2]
    #             ay = R[0, 2] - R[2, 0]
    #             az = R[1, 0] - R[0, 1]

    #             cos_theta = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    #             sin_theta = 0.5 * np.sqrt(ax**2 + ay**2 + az**2)

    #             theta = np.arctan2(sin_theta, cos_theta)
    #             theta_data[i] = np.degrees(theta)

    #             if sin_theta > 10**(-4):
    #                 ux_data[i] = ax / (2.0 * sin_theta)
    #                 uy_data[i] = ay / (2.0 * sin_theta)
    #                 uz_data[i] = az / (2.0 * sin_theta)

    #             # Cas de la Singularité en theta ~ 180°
    #             elif cos_theta < 0:
    #                 Sym_R = (R + R.T) / 2.0
                    
    #                 diag = np.clip(np.diag(Sym_R) + 1.0, 0.0, None)
                    
    #                 u_raw = np.sqrt(diag / 2.0) # Avoir |ux|, |uy|, |uz| mais sqrt perd le signe (-)

    #                 if Sym_R[0, 1] < 0: u_raw[1] *= -1
    #                 if Sym_R[0, 2] < 0: u_raw[2] *= -1
                    
    #                 norm = np.linalg.norm(u_raw)
                    
    #                 ux_data[i], uy_data[i], uz_data[i] = u_raw / norm if norm > 1e-10 else u_raw

    #             else:  # θ ≈ 0° : indéfini
    #                 ux_data[i], uy_data[i], uz_data[i] = 0.0, 0.0, 0.0


    #         # 2. Remplissage des zones θ ≈ 0°

    #         valid = np.where(ux_data**2 + uy_data**2 + uz_data**2 > 0.1)[0]

    #         if len(valid) > 0:
    #             # Backward : remplir la base depuis le premier axe valide
    #             for i in range(valid[0] - 1, -1, -1):
    #                 ux_data[i], uy_data[i], uz_data[i] = ux_data[i+1], uy_data[i+1], uz_data[i+1]
    #             # Forward : remplir les trous intermédiaires
    #             for i in range(1, num_nodes):
    #                 if ux_data[i]**2 + uy_data[i]**2 + uz_data[i]**2 < 0.1:
    #                     ux_data[i], uy_data[i], uz_data[i] = ux_data[i-1], uy_data[i-1], uz_data[i-1]
    #         else:
    #             # Robot 100% droit : convention arbitraire explicite
    #             ux_data.fill(1.0)
    #             uy_data.fill(0.0)
    #             uz_data.fill(0.0)


    #         # 3. Cohérence intra-frame : flip u, et signe de θ déduit après

    #         for i in range(1, num_nodes):
    #             if np.abs(theta_data[i]) < 2.0 or np.abs(theta_data[i-1]) < 2.0:
    #                 continue  # Zone droite : on ne touche pas

    #             prev = np.array([ux_data[i-1], uy_data[i-1], uz_data[i-1]])
    #             curr = np.array([ux_data[i],   uy_data[i],   uz_data[i]])

    #             if np.dot(prev, curr) < 0:
    #                 ux_data[i] *= -1
    #                 uy_data[i] *= -1
    #                 uz_data[i] *= -1
    #                 # θ reste POSITIF ici — on ne le flippe PAS dans l'étape 3
    #                 # Le signe de θ est géré globalement par l'étape 4 uniquement


    #         # 4. Cohérence inter-frames : flip global u ET θ ensemble

    #         # Nœud de référence : le premier nœud avec θ fiable (loin de 0° et 180°)
    #         stable_mask = (np.abs(theta_data) > 5.0) & (np.abs(theta_data) < 170.0)

    #         if np.any(stable_mask):
    #             ref_idx = np.where(stable_mask)[0][np.argmax(np.abs(theta_data[stable_mask]))]
    #             u_ref_node = np.array([ux_data[ref_idx], uy_data[ref_idx], uz_data[ref_idx]])

    #             if self._u_ref is not None and np.dot(self._u_ref, u_ref_node) < 0:
    #                 ux_data *= -1
    #                 uy_data *= -1
    #                 uz_data *= -1
    #                 theta_data *= -1

    #             self._u_ref = np.array([ux_data[ref_idx], uy_data[ref_idx], uz_data[ref_idx]])

    #         elif np.any(np.abs(theta_data) > 5.0):
    #             # Tous les nœuds sont proches de 180° : on prend quand même le meilleur
    #             ref_idx = np.argmax(np.abs(theta_data))
    #             u_ref_node = np.array([ux_data[ref_idx], uy_data[ref_idx], uz_data[ref_idx]])

    #             if self._u_ref is not None and np.dot(self._u_ref, u_ref_node) < 0:
    #                 ux_data *= -1
    #                 uy_data *= -1
    #                 uz_data *= -1
    #                 theta_data *= -1

    #             self._u_ref = np.array([ux_data[ref_idx], uy_data[ref_idx], uz_data[ref_idx]])

    #         else:
    #             self._u_ref = None



    #         # SOUS-GRAPHES
    #         # --- GRAPHIQUE DU HAUT : AMPLITUDE THETA ---
    #         self.ax_theta.plot(length_axis, theta_data, "k-", linewidth=1.5, label=r"$\theta$")
    #         self.ax_theta.set_title("Orientation du CTR", fontsize=11, fontweight="bold")
    #         self.ax_theta.set_ylabel("Angle de déviation $\theta$ (°)")
    #         self.ax_theta.set_xlabel("Position le long du robot (m)")
    #         self.ax_theta.grid(True, linestyle="--", alpha=0.5)
            
    #         # Marge dynamique pour l'affichage de Theta
    #         min_t, max_t = np.min(theta_data), np.max(theta_data)
    #         margin_t = max(5.0, 0.15 * (max_t - min_t))
    #         # self.ax_theta.set_ylim(max(0, min_t - margin_t - 2), min(180, max_t + margin_t))

    #         limite_basse = min_t - (margin_t * 0.5) # Ajustez le facteur (0.5) selon vos besoins

    #         self.ax_theta.set_ylim(limite_basse, min(180, max_t + margin_t))

    #         # --- GRAPHIQUE DU BAS : DIRECTION DE L'AXE U ---
    #         self.ax_u.plot(length_axis, ux_data, "r-", linewidth=1.2, label="$u_x$")
    #         self.ax_u.plot(length_axis, uy_data, "g-", linewidth=1.2, label="$u_y$")
    #         self.ax_u.plot(length_axis, uz_data, "b-", linewidth=1.2, label="$u_z$")
    #         self.ax_u.set_xlabel("Position le long du robot (m)")
    #         self.ax_u.set_ylabel("Composantes de l'axe $\mathbf{u}$")
    #         self.ax_u.set_ylim(-1.05, 1.05) # Un vecteur unitaire oscille strictement entre -1 et 1
    #         self.ax_u.grid(True, linestyle="--", alpha=0.5)
    #         self.ax_u.legend(loc="lower left", fontsize=9)

    #         # Ajustement de l'espacement vertical pour éviter les chevauchements
    #         fig.subplots_adjust(hspace=0.3)
            
    #         # Rafraîchissement du canvas
    #         fig.canvas.draw()


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

