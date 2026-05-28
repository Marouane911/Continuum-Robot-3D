#!/bin/bash

# Chemins absolus
BUILD_DIR="$HOME/TIMC_Robotics/Modeling-and-Control-of-Concentric-Tube-Continuum-Robots/build"
TOOLKIT_DIR="$HOME/TIMC_Robotics/CRVisToolkit"

echo " 1. Génération de la trajectoire (C++) "

cd "$BUILD_DIR" || { echo "Erreur dossier build C++"; exit 1; }
make -j$(nproc)
./demo/001_model_computation_anim

echo " 2. Lancement de l'animation (Python) "

cd "$TOOLKIT_DIR" || { echo "Erreur dossier Python"; exit 1; }
python3 timc_rendering_anim.py
