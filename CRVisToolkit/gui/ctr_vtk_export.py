import os
# Module standard Python permettant de manipuler les variables d'environnement.

os.environ["LC_NUMERIC"] = "C"
# Force le format numérique C/POSIX afin que VTK écrive les nombres
# avec un point décimal (0.123) et non une virgule (0,123).

os.environ["LANG"] = "C"
# Force également la locale générale en mode C pour éviter
# les problèmes de formatage liés à la langue du système.

import vtk
# Bibliothèque VTK utilisée pour construire et exporter la géométrie 3D.
from vtk.util import numpy_support


class CTRVTKExporter:
    # Classe utilitaire chargée d'exporter la géométrie du robot.

    @staticmethod
    def create_polyline(x, y, z, n_points):

        points = vtk.vtkPoints()

        for i in range(n_points):
            points.InsertNextPoint(
                float(x[i]),
                float(y[i]),
                float(z[i])
            )

        polyline = vtk.vtkPolyLine()
        polyline.GetPointIds().SetNumberOfIds(n_points)

        for i in range(n_points):
            polyline.GetPointIds().SetId(i, i)

        cells = vtk.vtkCellArray()
        cells.InsertNextCell(polyline)

        polydata = vtk.vtkPolyData()
        polydata.SetPoints(points)
        polydata.SetLines(cells)

        return polydata

    @staticmethod
    def export_centerline(matrix, end_int, end_mid, end_ext, output_path):
        # Méthode statique qui reçoit :
        # - matrix : matrice 3xN contenant les coordonnées du centre du robot
        # - les longueurs des différents tubes
        # - output_path : chemin du fichier .vtk à générer

        x = matrix[0, :]
        # Coordonnées X de tous les points de la centreline.

        y = matrix[1, :]
        # Coordonnées Y de tous les points de la centreline.

        z = matrix[2, :]
        # Coordonnées Z de tous les points de la centreline.

        poly_int = CTRVTKExporter.create_polyline(
            x, y, z, end_int
        ) # Création de polyline du tube interne

        poly_mid = CTRVTKExporter.create_polyline(
            x, y, z, end_mid
        ) # Création de polyline du tube intermédiaire

        poly_ext = CTRVTKExporter.create_polyline(
            x, y, z, end_ext
        ) # Création de polyline du tube externe

        # ------------------------------------------------------------------
        # Transformation de la ligne centrale en tube 3D

        # Tube interne
        tube_int = vtk.vtkTubeFilter()
        tube_int.SetInputData(poly_int)
        tube_int.SetRadius(0.000762)
        tube_int.SetNumberOfSides(16)
        tube_int.CappingOn()
        tube_int.Update()

        # Tube milieu
        tube_mid = vtk.vtkTubeFilter()
        tube_mid.SetInputData(poly_mid)
        tube_mid.SetRadius(0.000900)
        tube_mid.SetNumberOfSides(16)
        tube_mid.CappingOn()
        tube_mid.Update()

        # Tube externe
        tube_ext = vtk.vtkTubeFilter()
        tube_ext.SetInputData(poly_ext)
        tube_ext.SetRadius(0.001175)
        tube_ext.SetNumberOfSides(16)
        tube_ext.CappingOn()
        tube_ext.Update()

        # Fusion des trois tubes
        append = vtk.vtkAppendPolyData()

        append.AddInputData(tube_int.GetOutput())
        append.AddInputData(tube_mid.GetOutput())
        append.AddInputData(tube_ext.GetOutput())

        append.Update()

        polydata = append.GetOutput()
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

        cells = polydata.GetPolys()
        print("Cell array :", cells)

        triangleFilter.SetInputData(polydata)
        # Fournit le tube en entrée.

        triangleFilter.Update()
        # Exécute le filtre.

        polydata = triangleFilter.GetOutput()
        # Récupère le maillage triangulé.

        # DEBUGGINNGG TRIANGLE NUMPY ?
        print("Points :", polydata.GetNumberOfPoints())
        print("Polys :", polydata.GetNumberOfPolys())
        cells = polydata.GetPolys()
        print("Cell array :", cells)
        points = numpy_support.vtk_to_numpy(polydata.GetPoints().GetData())
        polys = numpy_support.vtk_to_numpy(polydata.GetPolys().GetData()).reshape(-1,4)
        print(points.shape)
        print(polys.shape)
        print(polys[:5])

        transform = vtk.vtkTransform()
        transform.Scale(1000.0, 1000.0, 1000.0)
        transformFilter = vtk.vtkTransformPolyDataFilter()
        transformFilter.SetInputData(polydata)
        transformFilter.SetTransform(transform)
        transformFilter.Update()
        polydata = transformFilter.GetOutput()
        # Transformation rigide

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

        print("poly_int =", poly_int.GetNumberOfPoints())
        print("poly_mid =", poly_mid.GetNumberOfPoints())
        print("poly_ext =", poly_ext.GetNumberOfPoints())