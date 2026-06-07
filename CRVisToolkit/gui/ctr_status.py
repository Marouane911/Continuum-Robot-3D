class CTRStatus:

    def __init__(self, robot_state_label, error_label):
        self.robot_state_label = robot_state_label
        self.error_label = error_label

    def set_ok(self):
        self.robot_state_label.setText("🟢 OK")

        self.robot_state_label.setStyleSheet("""
            color: green;
            font-weight: bold;
            font-size: 14px;
        """)

        self.error_label.setText("")

    def set_collision(self, message):
        self.robot_state_label.setText(
            "🔴 Collision détectée :\n"
            f"{message}\n"
            "Dernière configuration valide mémorisée.\n"
            "Cliquer sur \"Dernière position valide\"."
        )

        self.robot_state_label.setStyleSheet("""
            color: red;
            font-weight: bold;
            font-size: 14px;
        """)

        self.error_label.setText("")

    def set_divergence(self, message):
        self.robot_state_label.setText(
            "🟠 Divergence du solveur :\n"
            f"{message}\n"
            "Dernière configuration valide mémorisée.\n"
            "Cliquer sur \"Dernière position valide\"."
        )

        self.robot_state_label.setStyleSheet("""
            color: orange;
            font-weight: bold;
            font-size: 14px;
        """)

        self.error_label.setText("")