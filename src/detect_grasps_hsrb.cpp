#include <string>

#include <gpd/grasp_detector.h>

namespace gpd {
    namespace apps {
        namespace detect_grasps {

            bool checkFileExists(const std::string &file_name) {
              std::ifstream file;
              file.open(file_name.c_str());
              if (!file) {
                std::cout << "File " + file_name + " could not be found!\n";
                return false;
              }
              file.close();
              return true;
            }

            void writeToFile(const std::string &file_name,
                std::vector<std::unique_ptr<candidate::Hand>> &clustered_grasps,
                std::vector<std::unique_ptr<candidate::Hand>> &all_grasps) {
              // Output stream
              std::ofstream of(file_name.c_str());
              // Write all clustered grasps
              for (std::size_t i = 0; i < clustered_grasps.size(); ++i) {
                Eigen::Vector3d pos = clustered_grasps[i]->getPosition();
                Eigen::Matrix3d rot = clustered_grasps[i]->getOrientation();
                double score = clustered_grasps[i]->getScore();
                of << score << " " << rot(0, 0) << " " << rot(0, 1) << " " << rot(0, 2) << " " << pos[0] << " "
                   << rot(1, 0) << " " << rot(1, 1) << " " << rot(1, 2) << " " << pos[1] << " "
                   << rot(2, 0) << " " << rot(2, 1) << " " << rot(2, 2) << " " << pos[2] << " "
                   << "0.0 0.0 0.0 1.0\n";
              }
              // Write all grasps
/*              for (std::size_t i = 0; i < all_grasps.size(); ++i) {
                Eigen::Vector3d pos = all_grasps[i]->getPosition();
                Eigen::Matrix3d rot = all_grasps[i]->getOrientation();
                double score = all_grasps[i]->getScore();
                of << score << " " << rot(0, 0) << " " << rot(0, 1) << " " << rot(0, 2) << " " << pos[0] << " "
                   << rot(1, 0) << " " << rot(1, 1) << " " << rot(1, 2) << " " << pos[1] << " "
                   << rot(2, 0) << " " << rot(2, 1) << " " << rot(2, 2) << " " << pos[2] << " "
                   << "0.0 0.0 0.0 1.0\n";
              }
*/              // Close stream
              of.close();
            }

            int DoMain(int argc, char *argv[]) {
              // Read arguments from command line.
              if (argc < 5) {
                std::cout << "Error: Not enough input arguments!\n\n";
                std::cout << "Usage: detect_grasps CONFIG_FILE PCD_FILE NUM_GRASPS OUTPUT_FILE\n\n";
                std::cout << "Detect NUM_GRASPS grasp poses for a point cloud, PCD_FILE (*.pcd), "
                             "using parameters from CONFIG_FILE (*.cfg) "
                             "and recording the results to OUTPUT_FILE.\n\n";
                return (-1);
              }

              std::string config_filename = argv[1];
              std::string pcd_filename = argv[2];
              int num_grasps = std::atoi(argv[3]);
              std::string out_filename = argv[4];
              if (!checkFileExists(config_filename)) {
                printf("Error: config file not found!\n");
                return (-1);
              }
              if (!checkFileExists(pcd_filename)) {
                printf("Error: PCD file not found!\n");
                return (-1);
              }

              // Read parameters from configuration file.
              util::ConfigFile config_file(config_filename);
              config_file.ExtractKeys();

              // Set the camera position. Assumes a single camera view.
              std::vector<double> camera_position =
                config_file.getValueOfKeyAsStdVectorDouble("camera_position",
                                                           "0.0 0.0 0.0");
              Eigen::Matrix3Xd view_points(3, 1);
              view_points << camera_position[0], camera_position[1], camera_position[2];

              // Load point cloud from file.
              util::Cloud cloud(pcd_filename, view_points);
              if (cloud.getCloudOriginal()->size() == 0) {
                std::cout << "Error: Input point cloud is empty or does not exist!\n";
                return (-1);
              }

              GraspDetector detector(config_filename);

              // Preprocess the point cloud.
              detector.preprocessPointCloud(cloud);

              // If the object is centered at the origin, reverse all surface normals.
              bool centered_at_origin =
                config_file.getValueOfKey<bool>("centered_at_origin", false);
              if (centered_at_origin) {
                printf("Reversing normal directions ...\n");
                cloud.setNormals(cloud.getNormals() * (-1.0));
              }

              // Detect grasp poses.
              std::vector<std::unique_ptr<candidate::Hand>> all_grasps, clustered_grasps;
              clustered_grasps = detector.detectGrasps(cloud, num_grasps, all_grasps);
              std::cout << "Detected " << all_grasps.size() << " grasps" << std::endl;
              std::cout << "Converted to " << clustered_grasps.size() << " clustered grasps" << std::endl;

              /*std::cout << "--- Detected grasps ---" << std::endl;
              for (std::size_t i = 0; i < all_grasps.size(); ++i) {
                Eigen::Vector3d pos = all_grasps[i]->getPosition();
                Eigen::Matrix3d rot = all_grasps[i]->getOrientation();
                double score = all_grasps[i]->getScore();
                std::cout << i << " : " << score << "\n"
                          << pos[0] << " " << pos[1] << " " << pos[2] << "\n"
                          << rot << std::endl;
              }
              std::cout << "--- --- ---" << std::endl;

              std::cout << "--- Clustered grasps ---" << std::endl;
              for (std::size_t i = 0; i < clustered_grasps.size(); ++i) {
                Eigen::Vector3d pos = clustered_grasps[i]->getPosition();
                Eigen::Matrix3d rot = clustered_grasps[i]->getOrientation();
                double score = clustered_grasps[i]->getScore();
                std::cout << i << " : " << score << "\n"
                          << pos[0] << " " << pos[1] << " " << pos[2] << "\n"
                          << rot << std::endl;
              }
              std::cout << "--- --- ---" << std::endl;*/

              // Save to file
              writeToFile(out_filename, clustered_grasps, all_grasps);

              return 0;
            }

        }  // namespace detect_grasps
    }  // namespace apps
}  // namespace gpd

int main(int argc, char *argv[]) {
  return gpd::apps::detect_grasps::DoMain(argc, argv);
}

