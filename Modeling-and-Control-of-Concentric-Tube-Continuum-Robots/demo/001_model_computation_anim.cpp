#include <iostream>
#include <fstream>

#include "loadParameters.h"
#include "CtrModel.h"

using namespace Eigen;
using namespace CtrLib;

int main(int, char**)
{
    // Chargement paramètres robot
    std::vector<parameters> vParameters;

    if(loadParameters("../parameters/parameters.csv", vParameters) != 0)
    {
        std::cout << "Erreur chargement paramètres" << std::endl;
        return -1;
    }

    parameters& pNominal = vParameters[0];

    CtrModel ctr(pNominal);

    computationOptions opt =
    {
        .isExternalLoads = true,
        .isComputeJacobian = false,
        .isComputeCompliance = false,
        .nbThreads = 4
    };

    std::ofstream dataFile("robot_backbone.txt");

    if(!dataFile.is_open())
    {
        std::cout << "Impossible d'ouvrir robot_backbone.txt" << std::endl;
        return -1;
    }

    constexpr int num_steps = 100;

    Vector_q q;

    for(int step = 0; step < num_steps; ++step)
    {
        double t =
            static_cast<double>(step)
            /
            static_cast<double>(num_steps - 1);

        //--------------------------------------------------
        // ACTIONNEURS
        //--------------------------------------------------

        // Translations (m)

        q(0) = -0.30 + 0.05 * t;  // interne
        q(1) = -0.20 + 0.05 * t;  // intermédiaire
        q(2) = -0.10 + 0.05 * t;  // externe

        // Rotations (rad)

        q(3) = 2.0 * pi * t;
        q(4) = 1.5 * pi * t;
        q(5) = 1.0 * pi * t;

        //---- AUTRES CONFIGURATION POUR TEST ----
        // -- ROTATION UNIQUEMENT --
        // q(0) = -0.30;
        // q(1) = -0.20;
        // q(2) = -0.10;

        // q(3) = 2.0 * pi * t;
        // q(4) = 1.5 * pi * t;
        // q(5) = 1.0 * pi * t;
        // --------------------------

        // -- TRANSLATION UNIQUEMENT --
        // q(0) = -0.30 + 0.10 * t;
        // q(1) = -0.20 + 0.08 * t;
        // q(2) = -0.10 + 0.05 * t;

        // q(3) = 0.0;
        // q(4) = 0.0;
        // q(5) = 0.0;
        // --------------------------

        // -- TRANSLATION TUBE INTERNE UNIQUEMENT --
        // q(0) = -0.30 + 0.15 * t;
        // q(1) = -0.20;
        // q(2) = -0.10;

        // q(3) = 0.0;
        // q(4) = 0.0;
        // q(5) = 0.0;
        // --------------------------

        // -- MOUVEMENT SINUSOIDAL sinusoïdal --
        // q(0) = -0.30 + 0.02 * sin(2.0 * pi * t);
        // q(1) = -0.20 + 0.02 * sin(2.0 * pi * t);
        // q(2) = -0.10 + 0.02 * sin(2.0 * pi * t);

        // q(3) = pi * sin(2.0 * pi * t);
        // q(4) = 0.5 * pi * sin(2.0 * pi * t);
        // q(5) = 0.25 * pi * sin(2.0 * pi * t);
        // --------------------------
        // -------------------------------------------



        std::cout
            << "step = "
            << step
            << " q = "
            << q.transpose()
            << std::endl;

        //--------------------------------------------------
        // CALCUL MODELE
        //--------------------------------------------------

        Vector_q q_last_valid = q;

        if(ctr.Compute(q,opt) < 0){
            
        std::cout
            << "Configuration invalide au step "
            << step
            << ", réutilisation de la dernière forme valide."
            << std::endl;

        ctr.Compute(q_last_valid,opt);

        auto yTot = ctr.GetYTot(); // Forme géométrique initiale

        for(int row=0; row<yTot.rows(); ++row)
        {
            for(int col=0; col<yTot.cols(); ++col)
            {
                dataFile << yTot(row,col);

                if(col < yTot.cols()-1)
                    dataFile << " ";
            }

            dataFile << "\n";
        }

        dataFile << "--- STEP_BREAK ---\n";

        continue;
    }

    q_last_valid = q;

        //--------------------------------------------------
        // RECUPERATION DONNEES
        //--------------------------------------------------

        auto yTot = ctr.GetYTot();

        auto P = ctr.GetP();

        std::cout
            << "Tip = "
            << P.transpose()
            << std::endl;

        //--------------------------------------------------
        // ECRITURE FRAME
        //--------------------------------------------------

        for(int row = 0; row < yTot.rows(); ++row)
        {
            for(int col = 0; col < yTot.cols(); ++col)
            {
                dataFile << yTot(row, col);

                if(col < yTot.cols() - 1)
                    dataFile << " ";
            }

            dataFile << "\n";
        }

        dataFile << "--- STEP_BREAK ---\n";
    }

    dataFile.close();

    std::cout << std::endl;
    std::cout << "robot_backbone.txt généré avec "
              << num_steps
              << " étapes."
              << std::endl;

    return 0;
}