#include <iostream>
#include <fstream>
#include "loadParameters.h"
#include "CtrModel.h"

using namespace Eigen;
using namespace CtrLib;

int main(int, char**){
  // Example 1 : simply compute the model

  // Load parameters corresponding to CTR
  std::vector<parameters> vParameters;
	if(loadParameters("../parameters/parameters.csv",vParameters) != 0){
    return -1;
  }
  parameters &pNominal =  vParameters[0];

  CtrModel ctr(pNominal);
  // Declare actuation variables q
  Vector_q q(-0.3, -0.2, -0.1, 0, 0, 0); // arbitrary initial configuration (translation and rotation data)

  // Compute model
  // Use predefined options (defined in "ctrConstants.h") :
  //    - opt_LOAD      to compute the loaded model
  //    - opt_LOAD_J    to compute the loaded model and the robot Jacobian matrix
  //    - opt_LOAD_J_C  to compute the loaded model and the robot Jacobian and compliance matrices
  if(ctr.Compute(q, opt_LOAD) < 0){
    std::cout << "main()>> Error ! ctr.Compute() returned non-zero " << std::endl;
    return -1;
  }

  // Or create user-defined option (e.g. to adjust the number of threads for parallel computing)
  computationOptions opt = {.isExternalLoads = true, // Take into account the forces
                            .isComputeJacobian = false, // Disable Jacobian to save calculation time
                            .isComputeCompliance = false, // Same
                            .nbThreads = 4};
  if(ctr.Compute(q, opt) < 0){
    std::cout << "main()>> Error ! ctr.Compute() returned non-zero " << std::endl;
    return -1;
  }

// TRAJECTORY GENERATION FOR PYTHON ANIMATION
std::ofstream dataFile("robot_backbone.txt");

if (dataFile.is_open()) {
    int num_steps = 50;
    std::cout << "Trajectory generation (" << num_steps << " steps)..." << std::endl;

    for (int step = 0; step < num_steps; ++step) {
        // On fait varier doucement les variables d'actionnement à chaque étape
        // Example : modify the translation & rotation of the first tube
        q(0) = -0.3 + (0.1 * (double)step / num_steps); // Insertion
        q(1) = -0.2 + (0.05 * (double)step / num_steps);
        q(3) = 0.5 * (double)step / num_steps;          // Rotation

        // Compute the physical model for this new position
        if (ctr.Compute(q, opt) >= 0) {
            auto yTot = ctr.GetYTot();

            // Save the current state's matrix
            dataFile << yTot << "\n";


            dataFile << "--- STEP_BREAK ---\n";
        }
    }
    dataFile.close();
    std::cout << "--> Trajectory saved in 'robot_backbone.txt' !" << std::endl;
}
return 0;
}

