#include <iostream>
#include "ekf.h"
#include <Eigen/Eigen>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include "kalman_filter.h"
#include "finite-state-machines/fsm_states.h"

void store_data(std::vector<Eigen::Matrix<float, 3, 1>> &accelerations,
                std::vector<Eigen::Matrix<float, 6, 1>> &orientations,
                std::vector<float> &altitudes,
                std::vector<float> &times, std::vector<std::string> &FSM);
FSMState stringToFSMState(const std::string &s);

void write_kalman_csv(const std::string &filename,
                      const std::vector<float> &times,
                      const std::vector<float> &altitudes,
                      const std::vector<float> &k_pos_x,
                      const std::vector<float> &k_pos_y,
                      const std::vector<float> &k_pos_z,
                      const std::vector<float> &k_vel_x,
                      const std::vector<float> &k_vel_y,
                      const std::vector<float> &k_vel_z,
                      const std::vector<float> &k_acc_x,
                      const std::vector<float> &k_acc_y,
                      const std::vector<float> &k_acc_z,
                      const std::vector<std::string> &FSM);
int main()
{
    EKF ekf;

    std::vector<Eigen::Matrix<float, 3, 1>> accelerations;
    std::vector<Eigen::Matrix<float, 6, 1>> orientations;
    std::vector<float> altitudes;
    std::vector<float> times;

    std::vector<std::string> FSM;
    store_data(accelerations, orientations, altitudes, times, FSM);
    std::vector<float> k_times;
    std::vector<float> k_alts;

    std::vector<float> k_pos_x;
    std::vector<float> k_pos_y;
    std::vector<float> k_pos_z;
    std::vector<float> k_vel_x;
    std::vector<float> k_vel_y;
    std::vector<float> k_vel_z;
    std::vector<float> k_acc_x;
    std::vector<float> k_acc_y;
    std::vector<float> k_acc_z;
    std::vector<std::string> k_fsm;
    for (int i = 0; i < altitudes.size(); i += 50)
    {
        if(times[i] <= 2523800){
            // ekf.initialize();
            // std::cout<<"ran"<<std::endl;
            continue;
        }
        FSMState actual = stringToFSMState(FSM[i]);
        ekf.tick(0.05f, 0.001f, altitudes[i], accelerations[i], orientations[i], actual);
        KalmanState val = ekf.getState();
        k_times.push_back(times[i]);
        k_alts.push_back(altitudes[i]);
        k_pos_x.push_back(val.state_est_pos_x);
        k_pos_y.push_back(val.state_est_pos_y);
        k_pos_z.push_back(val.state_est_pos_z);
        k_vel_x.push_back(val.state_est_vel_x);
        k_vel_y.push_back(val.state_est_vel_y);
        k_vel_z.push_back(val.state_est_vel_z);
        k_acc_x.push_back(val.state_est_accel_x);
        k_acc_y.push_back(val.state_est_accel_y);
        k_acc_z.push_back(val.state_est_accel_z);
        k_fsm.push_back(FSM[i]);
    }

   write_kalman_csv("./data_files/kalman_output.csv",
                 k_times, k_alts,
                 k_pos_x, k_pos_y, k_pos_z,
                 k_vel_x, k_vel_y, k_vel_z,
                 k_acc_x, k_acc_y, k_acc_z,
                 k_fsm );

    return 0;
}

void store_data(std::vector<Eigen::Matrix<float, 3, 1>> &accelerations,
                std::vector<Eigen::Matrix<float, 6, 1>> &orientations,
                std::vector<float> &altitudes,
                std::vector<float> &times,
                std::vector<std::string> &FSM)
{
    std::ifstream file("./data_files/MIDAS_booster.csv");
    if (!file.is_open())
    {
        std::cerr << "Error: Unable to open file." << std::endl;
        return;
    }
    std::cout << "opened" << std::endl;
    std::string line;
    std::getline(file,line);
    while (std::getline(file, line))
    {
        Eigen::Matrix<float, 3, 1> accel;
        Eigen::Matrix<float, 6, 1> orient;
        float altitude;
        float time;
        std::string fsm_state;

        std::stringstream ss(line);
        std::string token;
        int count = 1;
        while (std::getline(ss, token, ','))
        {
            try
            {
                if (count == 3)
                    time = std::stof(token);
                else if (count == 7)
                    accel(0) = std::stof(token);
                else if (count == 8)
                    accel(1) = std::stof(token);
                else if (count == 9)
                    accel(2) = std::stof(token);
                else if (count == 12)
                    altitude = std::stof(token);
                else if (count == 31)
                    orient(0) = std::stof(token);
                else if (count == 32)
                    orient(1) = std::stof(token);
                else if (count == 33)
                    orient(2) = std::stof(token);
                else if (count == 37)
                    orient(3) = std::stof(token);
                else if (count == 38)
                    orient(4) = std::stof(token);
                else if (count == 39)
                    orient(5) = std::stof(token);
                else if (count == 65)
                    fsm_state = token;
            }
            catch (const std::exception &e)
            {
                // ignore bad tokens
            }
            ++count;
        }
        accelerations.push_back(accel);
        orientations.push_back(orient);
        altitudes.push_back(altitude);
        times.push_back(time);
        FSM.push_back(fsm_state);
    }

    file.close();
}

FSMState stringToFSMState(const std::string &s)
{
    static const std::unordered_map<std::string, FSMState> lookup = {
        {"STATE_SAFE", STATE_SAFE},
        {"STATE_PYRO_TEST", STATE_PYRO_TEST},
        {"STATE_IDLE", STATE_IDLE},
        {"STATE_FIRST_BOOST", STATE_FIRST_BOOST},
        {"STATE_BURNOUT", STATE_BURNOUT},
        {"STATE_COAST", STATE_COAST},
        {"STATE_APOGEE", STATE_APOGEE},
        {"STATE_DROGUE_DEPLOY", STATE_DROGUE_DEPLOY},
        {"STATE_DROGUE", STATE_DROGUE},
        {"STATE_MAIN_DEPLOY", STATE_MAIN_DEPLOY},
        {"STATE_MAIN", STATE_MAIN},
        {"STATE_LANDED", STATE_LANDED},
        {"STATE_SUSTAINER_IGNITION", STATE_SUSTAINER_IGNITION},
        {"STATE_SECOND_BOOST", STATE_SECOND_BOOST},
        {"STATE_FIRST_SEPARATION", STATE_FIRST_SEPARATION}};

    auto it = lookup.find(s);
    if (it != lookup.end())
    {
        return it->second;
    }

    // Default case if string not found
    std::cerr << "Warning: Unknown FSM string \"" << s << "\"\n";
    return STATE_SAFE;
}

void write_kalman_csv(const std::string &filename,
                      const std::vector<float> &times,
                      const std::vector<float> &altitudes,
                      const std::vector<float> &k_pos_x,
                      const std::vector<float> &k_pos_y,
                      const std::vector<float> &k_pos_z,
                      const std::vector<float> &k_vel_x,
                      const std::vector<float> &k_vel_y,
                      const std::vector<float> &k_vel_z,
                      const std::vector<float> &k_acc_x,
                      const std::vector<float> &k_acc_y,
                      const std::vector<float> &k_acc_z,
                      const std::vector<std::string> &FSM)
{
    std::ofstream outfile(filename);
    if (!outfile.is_open())
    {
        std::cerr << "Error: Unable to open output file " << filename << " for writing.\n";
        return;
    }

    outfile << "time,altitude,"
            << "pos_x,pos_y,pos_z,"
            << "vel_x,vel_y,vel_z,"
            << "acc_x,acc_y,acc_z,"
            << "fsm_state\n";

    // Write each row
    for (size_t i = 0; i < k_pos_x.size(); ++i)
    {
        outfile << times[i] << ","
                << altitudes[i] << ","
                << k_pos_x[i] << "," << k_pos_y[i] << "," << k_pos_z[i] << ","
                << k_vel_x[i] << "," << k_vel_y[i] << "," << k_vel_z[i] << ","
                << k_acc_x[i] << "," << k_acc_y[i] << "," << k_acc_z[i] << ","
                << FSM[i] << "\n";
    }

    outfile.close();
    std::cout << "Kalman output written to " << filename << "\n";
}
