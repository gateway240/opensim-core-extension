/* -------------------------------------------------------------------------- *
 *                    OpenSim:  main.cpp                     *
 * -------------------------------------------------------------------------- *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/*
 *  Below is an example of an OpenSim application that provides its own
 *  main() routine.
 */

// Author: Alexander Beattie

//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>
#include <OpenSim/Analyses/IMUDataReporter.h>
#include <OpenSim/Extension/Analyses/DistanceKinematics.h>
#include <chrono> // for std::chrono functions
#include <iostream>
#include <string>

using namespace OpenSim;
using namespace SimTK;

//______________________________________________________________________________
/**
 * First exercise: create a model that does nothing.
 */
int main() {

  std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();

  std::string model_name = "bin/calibrated_gait2392_thelen2003muscle.osim";
  OpenSim::Model model = OpenSim::Model(model_name);
  model.initSystem();

  std::string coordinates_file_name = "bin/ik_l_comf_01-000_orientations.mot";

  double start_time = 0;
  double end_time = std::numeric_limits<double>::infinity();

  // Setup Analysis Tool
  OpenSim::AnalyzeTool analyzeIMU;
  analyzeIMU.setName("test_distance_analysis");
  analyzeIMU.setModelFilename(model_name);
  analyzeIMU.setCoordinatesFileName(coordinates_file_name);
  analyzeIMU.setLowpassCutoffFrequency(-1);
  analyzeIMU.setInitialTime(start_time);
  analyzeIMU.setFinalTime(end_time);
  analyzeIMU.setResultsDir("results");

  // Setup IMU Data Reporter => Sanity Check
  OpenSim::IMUDataReporter imuDataReporter;
  imuDataReporter.setName("IMUDataReporter_no_forces");
  imuDataReporter.set_compute_accelerations_without_forces(true);
  imuDataReporter.setInDegrees(true);
  analyzeIMU.updAnalysisSet().cloneAndAppend(imuDataReporter);

  // Setup Point Kinematics Reporting

  // List all Bodies & IMUs
  std::cout << "\nList all IMUs in the model." << std::endl;
  int i = 0;
  const auto imus = model.getComponentList<OpenSim::IMU>();
  for (auto &component : imus) {
    std::cout << "frame[" << ++i << "] is " << component.getName()
              << " of type " << typeid(component).name() << std::endl;
  }

  std::cout << "\nList all bodies in the model." << std::endl;
  i = 0;
  int j = 0;
  const auto bodies = model.getComponentList<OpenSim::Body>();

  // Get the begin and end iterators for the bodies
  auto bodyBegin = bodies.begin();
  auto bodyEnd = bodies.end();
  size_t totalBodies = std::distance(bodyBegin, bodyEnd); // Calculate the total number of bodies

  // Silly loop to exclude self matches (ex. pelvis-pelvis) and reverse matches (pevis-toe) & (toe-pelvis)
  for (size_t outerIndex = 0; outerIndex < totalBodies; ++outerIndex) {
      auto it1 = std::next(bodyBegin, outerIndex); // Get the iterator for the current outer index
      const auto &root = *it1; // Dereference to get the body
      const std::string &root_name = root.getName();
      std::cout << "frame-i[" << outerIndex << "] is " << root_name << " of type "
                << typeid(root).name() << std::endl;
      // Start the inner loop from the next index after the current outer loop index
      for (size_t innerIndex = outerIndex + 1; innerIndex < totalBodies; ++innerIndex) {
          auto it2 = std::next(bodyBegin, innerIndex); // Get the iterator for the current inner index
          const auto &sub_component = *it2; // Dereference to get the sub-component
          const std::string &sub_component_name = sub_component.getName();
          std::cout << "frame-j[" << innerIndex << "] is " << sub_component_name
                    << " of type " << typeid(sub_component).name() << std::endl;
          // Create point kinematics reporter
          OpenSim::DistanceKinematics distKin;
          distKin.setPointName(root_name + "-" + sub_component_name);
          distKin.setBody(&root);
          distKin.setRelativeToBody(&sub_component);
          analyzeIMU.updAnalysisSet().cloneAndAppend(distKin);
      }
  }

  // Running Analysis!
  std::string output_file_name = "test_distance_analysis_imu.xml";
  analyzeIMU.print(output_file_name);
  OpenSim::AnalyzeTool roundTrip(output_file_name);
  roundTrip.run();

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cout << "Runtime = "
            << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()
            << "[µs]" << std::endl;

  std::cout << "OpenSim completed successfully" << std::endl;
  exit(EXIT_SUCCESS);
}
