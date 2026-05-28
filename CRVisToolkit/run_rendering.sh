#!/bin/bash

# Chemins absolus
BUILD_DIR="$HOME/TIMC_Robotics/Modeling-and-Control-of-Concentric-Tube-Continuum-Robots/build"
TOOLKIT_DIR="$HOME/TIMC_Robotics/CRVisToolkit"

echo " 1. Calcul du modèle physique (C++) "

cd "$BUILD_DIR" || { echo "Erreur dossier build C++"; exit 1; }
# Optionnel : on recompile si tu as fait des changements, puis on exécute
make -j$(nproc)
./demo/001_model_computation

echo " 2. Affichage du rendu statique 3D (Python) "

cd "$TOOLKIT_DIR" || { echo "Erreur dossier Python"; exit 1; }
python3 timc_rendering.py
