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

        # Tube externe
        self.ax_robot.plot(
            x[:end_ext],
            y[:end_ext],
            z[:end_ext],
            linewidth=(2 * r_tube[0]) * scale,
            color="black"
        )

        # Tube intermédiaire
        self.ax_robot.plot(
            x[end_ext - 1:end_mid],
            y[end_ext - 1:end_mid],
            z[end_ext - 1:end_mid],
            linewidth=(2 * r_tube[1]) * scale,
            color="dimgray"
        )

        # Tube interne
        self.ax_robot.plot(
            x[end_mid - 1:end_int],
            y[end_mid - 1:end_int],
            z[end_mid - 1:end_int],
            linewidth=(2 * r_tube[2]) * scale,
            color="lightgray"
        )

        # Pointe
        self.ax_robot.scatter(
            x[-1],
            y[-1],
            z[-1],
            s=40
        )