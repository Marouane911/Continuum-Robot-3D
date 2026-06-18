import os
# Module standard Python permettant de manipuler les variables d'environnement.

os.environ["LC_NUMERIC"] = "C"
# Force le format numérique C/POSIX afin que VTK écrive les nombres
# avec un point décimal (0.123) et non une virgule (0,123).

os.environ["LANG"] = "C"
# Force également la locale générale en mode C pour éviter
# les problèmes de formatage liés à la langue du système.


import numpy as np
# Bibliothèque de calcul numérique utilisée pour manipuler les matrices.

import vtk
# Bibliothèque VTK utilisée pour construire et exporter la géométrie 3D.


class CTRVTKExporter:
    # Classe utilitaire chargée d'exporter la géométrie du robot.

    @staticmethod
    def export_centerline(matrix, output_path):
        # Méthode statique qui reçoit :
        # - matrix : matrice 3xN contenant les coordonnées du centre du robot
        # - output_path : chemin du fichier .vtk à générer

        x = matrix[0, :]
        # Coordonnées X de tous les points de la centreline.

        y = matrix[1, :]
        # Coordonnées Y de tous les points de la centreline.

        z = matrix[2, :]
        # Coordonnées Z de tous les points de la centreline.

        points = vtk.vtkPoints()
        # Structure VTK destinée à stocker les points 3D.

        for xi, yi, zi in zip(x, y, z):
            # Parcourt simultanément les coordonnées X, Y et Z.

            points.InsertNextPoint(
                float(xi),
                float(yi),
                float(zi)
            )
            # Ajoute un point 3D dans la structure VTK.

        polyline = vtk.vtkPolyLine()
        # Création d'une polyligne reliant les points de la centreline.

        polyline.GetPointIds().SetNumberOfIds(len(x))
        # Réserve un identifiant pour chaque point.

        for i in range(len(x)):
            # Parcourt tous les points.

            polyline.GetPointIds().SetId(i, i)
            # Relie chaque identifiant au point correspondant.

        cells = vtk.vtkCellArray()
        # Structure VTK contenant les cellules géométriques.

        cells.InsertNextCell(polyline)
        # Ajoute la polyligne dans le tableau de cellules.

        polydata = vtk.vtkPolyData()
        # Objet VTK principal contenant la géométrie.

        polydata.SetPoints(points)
        # Associe les points à la géométrie.

        polydata.SetLines(cells)
        # Associe la polyligne à la géométrie.

        # ------------------------------------------------------------------
        # Transformation de la ligne centrale en tube 3D

        tube = vtk.vtkTubeFilter()
        # Filtre VTK transformant une ligne en cylindre.

        tube.SetInputData(polydata)
        # Fournit la centreline comme entrée du filtre.

        tube.SetRadius(0.002)
        # Rayon du tube : 2cm.

        tube.SetNumberOfSides(16)
        # Nombre de faces utilisées pour approximer le cercle.
        # Plus la valeur est grande, plus le tube paraît lisse.

        tube.CappingOn()
        # Ferme les extrémités du tube.

        tube.Update()
        # Exécute le filtre.

        polydata = tube.GetOutput()
        # Récupère le résultat généré par le TubeFilter.

        print("Points =", polydata.GetNumberOfPoints())
        # Nombre de sommets du tube.

        print("Lines =", polydata.GetNumberOfLines())
        # Nombre de lignes restantes.

        print("Polys =", polydata.GetNumberOfPolys())
        # Nombre de polygones.

        print("Strips =", polydata.GetNumberOfStrips())
        # Nombre de triangle strips.

        print("Verts =", polydata.GetNumberOfVerts())
        # Nombre de sommets isolés.

        # ------------------------------------------------------------------
        # Conversion des strips en triangles

        triangleFilter = vtk.vtkTriangleFilter()
        # Filtre transformant toutes les surfaces en triangles.

        triangleFilter.SetInputData(polydata)
        # Fournit le tube en entrée.

        triangleFilter.Update()
        # Exécute le filtre.

        polydata = triangleFilter.GetOutput()
        # Récupère le maillage triangulé.

        print("Points =", polydata.GetNumberOfPoints())
        # Nombre de sommets après triangulation.

        print("Lines =", polydata.GetNumberOfLines())
        # Nombre de lignes après triangulation.

        print("Polys =", polydata.GetNumberOfPolys())
        # Nombre total de triangles.

        print("Strips =", polydata.GetNumberOfStrips())
        # Vérifie que les strips ont disparu.

        # ------------------------------------------------------------------
        # Informations géométriques

        print("Bounds =", polydata.GetBounds())
        # Boîte englobante :
        # (xmin, xmax, ymin, ymax, zmin, zmax)

        print("Center =", polydata.GetCenter())
        # Centre géométrique du maillage.

        bounds = polydata.GetBounds()
        # Récupération de la boîte englobante.

        xmin, xmax, ymin, ymax, zmin, zmax = bounds
        # Décomposition des bornes.

        print("Longueur X =", xmax - xmin, "m")
        # Taille du robot selon X.

        print("Longueur Y =", ymax - ymin, "m")
        # Taille du robot selon Y.

        print("Longueur Z =", zmax - zmin, "m")
        # Taille du robot selon Z.

        print("Dimensions =", bounds)
        # Affiche la boîte englobante complète.

        print("polydata.GetBounds =", polydata.GetBounds())
        # Vérification supplémentaire.

        # ------------------------------------------------------------------
        # Export VTK

        writer = vtk.vtkPolyDataWriter()
        # Écrivain VTK legacy (.vtk).

        writer.SetFileName(output_path)
        # Définit le chemin du fichier de sortie.

        writer.SetInputData(polydata)
        # Définit le maillage à écrire.

        writer.SetFileTypeToASCII()
        # Écriture au format texte lisible.

        print("Writing:", output_path)
        # Affiche le fichier en cours d'écriture.

        writer.Write()
        # Sauvegarde le fichier.

        # ------------------------------------------------------------------
        # Vérification de la lecture du fichier généré

        reader = vtk.vtkPolyDataReader()
        # Lecteur VTK legacy.

        reader.SetFileName(output_path)
        # Ouvre le fichier précédemment écrit.

        reader.Update()
        # Lit le fichier.

        pd = reader.GetOutput()
        # Récupère le maillage relu.

        print("pd.GetNumberOfPoints =", pd.GetNumberOfPoints())
        # Vérifie le nombre de sommets relus.

        print("pd.GetNumberOfLines =", pd.GetNumberOfLines())
        # Vérifie le nombre de lignes relues.

        print("pd.GetNumberOfPolys =", pd.GetNumberOfPolys())
        # Vérifie le nombre de triangles relus.