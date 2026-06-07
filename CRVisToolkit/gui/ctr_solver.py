import os
import subprocess


class CTRSolver:

    @staticmethod
    def run(root_toolkit, q_values):

        try:
            project_parent = os.path.dirname(root_toolkit)

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

            cmd = [executable] + [str(q) for q in q_values]

            result = subprocess.run(
                cmd,
                cwd=build_dir,
                capture_output=True,
                text=True
            )

            print("STDOUT:", result.stdout)
            print("STDERR:", result.stderr)
            print("RETURNCODE:", result.returncode)

            if result.returncode != 0:
                error_msg = result.stderr.strip() if result.stderr else "Erreur C++"
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


        except Exception as e:
            return {
                "success": False,
                "status": "python_error",
                "message": str(e)
            }
    