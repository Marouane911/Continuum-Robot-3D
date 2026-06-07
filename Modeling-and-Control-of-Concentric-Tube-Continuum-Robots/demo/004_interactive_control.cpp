#include <iostream>
#include <fstream>

#include "loadParameters.h"
#include "CtrModel.h"

#include <iomanip>

using namespace Eigen;
using namespace CtrLib;

#include <cmath>

int main(int argc, char** argv)
{

    std::cout << std::fixed;

    if(argc != 7)
    {
        std::cout << std::endl;
        std::cout << "Usage :" << std::endl;
        std::cout
            << "./004_interactive_control "
            << "q1 q2 q3 q4 q5 q6"
            << std::endl;

        std::cerr << "ERROR: invalid number of arguments\n";
        return -1;
    }

    //--------------------------------------------------
    // PARAMETRES ROBOT
    //--------------------------------------------------

    std::vector<parameters> vParameters;

    if(loadParameters("../parameters/parameters.csv",
                      vParameters) != 0)
    {
        std::cerr << "ERROR: invalid number of arguments\n";
        return -1;
    }

    parameters& pNominal = vParameters[0];

    CtrModel ctr(pNominal);

    //--------------------------------------------------
    // OPTIONS
    //--------------------------------------------------

    computationOptions opt =
    {
        .isExternalLoads = true,
        .isComputeJacobian = false,
        .isComputeCompliance = false,
        .nbThreads = 1
    };

    //--------------------------------------------------
    // ACTIONNEURS
    //--------------------------------------------------

    Vector_q q;

    q(0) = atof(argv[1]);
    q(1) = atof(argv[2]);
    q(2) = atof(argv[3]);
    q(3) = atof(argv[4]);
    q(4) = atof(argv[5]);
    q(5) = atof(argv[6]);

    std::cout << "q = " << q.transpose() << std::endl;

    // Afficher ce qui ce passe entre les chariots (debugging)
    std::cout << "beta1 = " << q(0) << std::endl;
    std::cout << "beta2 = " << q(1) << std::endl;
    std::cout << "beta3 = " << q(2) << std::endl;


    std::cout << std::setprecision(16);


    //--------------------------------------------------
    // CALCUL
    //--------------------------------------------------

    if(ctr.Compute(q,opt) < 0)
    {
        std::cout
            << "Configuration invalide"
            << std::endl;

        std::cerr << "ERROR: invalid number of arguments\n";
        return -1;
    }

    auto yTot = ctr.GetYTot();

    // VÉRIF DEBUGGING --------------
    // std::cout << "\n=== Z FIN ===\n";

    // for(int col = 295; col < 360; col += 5)
    // {
    //     std::cout
    //         << col
    //         << " -> "
    //         << yTot(2,col)
    //         << std::endl;
    // }

    // VÉRIF DEBUGGING --------------
    // std::cout
    // << "yTot.cols() = "
    // << yTot.cols()
    // << std::endl;

    int validCols = (ctr.segmented.iEnd(0) + 1) * NB_INTEGRATION_NODES;

    // VÉRIF DEBUGGING --------------
    // std::cout
    // << "validCols = "
    // << validCols
    // << std::endl;

    // std::cout
    // << "segmented.iEnd = "
    // << ctr.segmented.iEnd.transpose()
    // << std::endl;

    // std::cout
    // << "nSeg final = "
    // << nSeg
    // << std::endl;

    // for(int col = 350; col < yTot.cols(); ++col)
    // {
    //     std::cout
    //         << "z[" << col << "] = "
    //         << yTot(2,col)
    //         << std::endl;
    // }

    // if(yTot.allFinite())
    // {
    //     std::cout
    //         << "yTot OK : toutes les valeurs sont finies."
    //         << std::endl;
    // }
    // else
    // {
    //     std::cout
    //         << "yTot contient des NaN ou Inf !"
    //         << std::endl;
    // }

    //--------------------------------------------------
    // SAUVEGARDE
    //--------------------------------------------------

    std::ofstream file("robot_backbone.txt");

    // VÉRIF DEBUGGING --------------
    // std::cout << "\n===== SEGMENTATION =====\n";

    // std::cout << "iEnd(0) = "
    //         << ctr.segmented.iEnd(0)
    //         << std::endl;
    
    // std::cout << "s(iEnd(0)) = "
    //       << ctr.segmented.S(ctr.segmented.iEnd(0))
    //       << std::endl;

    // std::cout << "iEnd(1) = "
    //         << ctr.segmented.iEnd(1)
    //         << std::endl;
    
    // std::cout << "s(iEnd(1)) = "
    //         << ctr.segmented.S(ctr.segmented.iEnd(1))
    //         << std::endl;
            
    // std::cout << "iEnd(2) = "
    //         << ctr.segmented.iEnd(2)
    //         << std::endl;

    // std::cout << "s(iEnd(2)) = "
    //         << ctr.segmented.S(ctr.segmented.iEnd(2))
    //         << std::endl;

    // std::cout << "========================\n";

    // std::cout << "\n=== SEGMENTS ===\n";

    // std::cout << "\n=== S ===\n";

    // for(int i = 0; i < ctr.segmented.S.size(); ++i)
    // {
    //     std::cout
    //     << "S = "
    //     << ctr.segmented.S.transpose()
    //     << std::endl;
    // }

    file << "# iEnd "
        << ctr.segmented.iEnd(0) << " "
        << ctr.segmented.iEnd(1) << " "
        << ctr.segmented.iEnd(2) << "\n";

    file << "# S ";

    for(int i=0;i<NB_SEGMENT_MAX;i++)
    {
        file << ctr.segmented.S(i) << " ";
    }

    file << "\n";

    // std::cout << "================\n";
    // ---------------------


    // VÉRIF DEBUGGING --------------
    // std::cout
    // << "yTot size = "
    // << yTot.rows()
    // << " x "
    // << yTot.cols()
    // << std::endl;
    // for(int row = 0; row < yTot.rows(); ++row){

    //     for(int col = 0; col < yTot.cols(); ++col){
    //         if(!std::isfinite(yTot(row,col))){
    //             std::cout
    //                 << "NaN/Inf detecte : "
    //                 << "row=" << row
    //                 << " col=" << col
    //                 << " value=" << yTot(row,col)
    //                 << std::endl;
    //         }
    //     }
    // }
    // ------------------



    for(int row = 0; row < yTot.rows(); ++row)
    {
        for(int col = 0; col < validCols; ++col)
        {
            file << yTot(row,col);

            if(col < yTot.cols()-1)
                file << " ";
        }

        file << "\n";
    }
    

    file << "--- STEP_BREAK ---\n";

    file.close();

    // std::cout
    //     << "robot_backbone.txt mis à jour."
    //     << std::endl;

    return 0;
}