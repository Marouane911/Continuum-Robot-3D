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



    def plot_local_tangent_orientation(
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

            t_x = R[0, 2]
            t_y = R[1, 2]
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
            "Orientation of the local tangent along the CTR",
            fontsize=11,
            fontweight="bold"
        )

        self.ax_plots.set_ylabel(
            "Angle de référence (degrés)",
            labelpad=12
        )

        self.ax_plots.set_ylim([0, 120])