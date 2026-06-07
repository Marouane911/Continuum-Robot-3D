import os
import numpy as np


class CTRLoader:

    @staticmethod
    def load(data_path):

        if not os.path.exists(data_path):

            print(
                f"Erreur : Fichier introuvable : {data_path}"
            )

            return []

        with open(data_path, "r") as f:

            content = f.read()

        blocks = content.strip().split(
            "--- STEP_BREAK ---"
        )

        steps_data = []

        for block in blocks:

            lines = [
                line.strip()
                for line in block.split("\n")
                if line.strip()
            ]

            iEnd = None
            S = None

            data_lines = []

            for line in lines:

                if line.startswith("# iEnd"):

                    values = line.split()[2:]

                    iEnd = np.array(
                        list(map(int, values))
                    )

                elif line.startswith("# S"):

                    values = line.split()[2:]

                    S = np.array(
                        list(map(float, values))
                    )

                else:

                    data_lines.append(line)

                if len(data_lines) == 19:

                    matrix_lines = [
                        list(map(float, line.split()))
                        for line in data_lines
                    ]

                    matrix = np.array(
                        matrix_lines
                    )

                    steps_data.append(
                        {
                            "matrix": matrix,
                            "iEnd": iEnd,
                            "S": S
                        }
                    )

        return steps_data