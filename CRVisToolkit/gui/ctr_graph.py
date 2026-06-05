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


