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
#include <OpenSim/Extension/Common/DistanceDataReaderSettings.h>
#include <OpenSim/Extension/Common/DistanceDataReader.h>
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
  std::string settingsFile{"bin/myIMUMappings.xml"};
  OpenSim::DistanceDataReaderSettings readerSettings(settingsFile);
  OpenSim::DistanceDataReader reader(readerSettings);
  std::string folder = readerSettings.get_data_folder() + "/";
  OpenSim::DataAdapter::OutputTables tables = reader.read(folder);


  // // Magnetometer
  // const OpenSim::TimeSeriesTableVec3 &magTableTyped =
  //     reader.getMagneticHeadingTable(tables);
  // OpenSim::STOFileAdapterVec3::write(magTableTyped, folder + readerSettings.get_trial_prefix() +
  //                                              "_magnetometers.sto");
  // // Gyro
  // const OpenSim::TimeSeriesTableVec3 &gyroTableTyped =
  //     reader.getAngularVelocityTable(tables);
  // OpenSim::STOFileAdapterVec3::write(gyroTableTyped,
  //                           folder + readerSettings.get_trial_prefix() + "_gyros.sto");
  // // Orientations
  // const OpenSim::TimeSeriesTableQuaternion &quatTableTyped =
  //     reader.getOrientationsTable(tables);

  // OpenSim::STOFileAdapter_<SimTK::Quaternion>::write(
  //     quatTableTyped, folder + readerSettings.get_trial_prefix() + "_orientations.sto");
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cout << "Runtime = "
            << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()
            << "[µs]" << std::endl;

  std::cout << "OpenSim completed successfully" << std::endl;
  exit(EXIT_SUCCESS);
}
