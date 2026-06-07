import os
import subprocess


class CTRSolver:

    @staticmethod
    def run(
        root_toolkit,
        q_values
    ):

        project_parent = os.path.dirname(
            root_toolkit
        )

        executable = os.path.join(
            project_parent,
            "Modeling-and-Control-of-Concentric-Tube-Continuum-Robots",
            "build",
            "demo",
            "004_interactive_control"
        )

        build_dir = os.path.join(
            project_parent,
            "Modeling-and-Control-of-Concentric-Tube-Continuum-Robots",
            "build"
        )

        cmd = [
            executable
        ] + [
            str(q)
            for q in q_values
        ]

        result = subprocess.run(
            cmd,
            cwd=build_dir,
            capture_output=True,
            text=True
        )

        # 1. Vérification d'un crash brutal de l'exécutable C++
        if result.returncode != 0:
            error_msg = result.stderr.strip() if result.stderr else "Erreur fatale ou Crash du binaire C++."
            return {
                "success": False,
                "status": "divergence",
                "message": f"Code {result.returncode}: {error_msg}"
            }
        
        for line in result.stdout.splitlines():

            lower_line = line.lower()

            if "clashing" in lower_line:

                return {
                    "success": False,
                    "status": "collision",
                    "message": line
                }

            if "failed to converge" in lower_line:

                return {
                    "success": False,
                    "status": "divergence",
                    "message": line
                }

        return {
            "success": True,
            "status": "ok",
            "message": result.stdout
        }