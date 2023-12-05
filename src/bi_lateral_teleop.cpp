
#include <inttypes.h>
#include <stdio.h>

#include <algorithm>
#include <vector>
#include <list>
#include <memory>  // For std::shared_ptr

#include "moteus.h"
#include "moteus_transport.h"

#include <ros/ros.h>
#include <string>
#include <unistd.h>


// Testing
// rosrun bam_experimental bi_lateral_teleop

// python3 -m moteus_gui.tview  --fdcanusb /dev/ttyACM1 --target 11,12,13
// python3 -m moteus_gui.tview  --fdcanusb /dev/ttyACM0 --target 21,22,23

// When Sending Position Commands, set velcoity to zero!
using namespace mjbots;

// A simple way to get the current time accurately as a double.
static double GetNow() {
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<double>(ts.tv_sec) +
      static_cast<double>(ts.tv_nsec) / 1e9;
}

struct ServoGroup {
    std::shared_ptr<mjbots::moteus::Transport> transport;
    std::vector<int> ids;
    std::map<int, std::shared_ptr<moteus::Controller>> controllers;
    std::map<int, moteus::Query::Result> servo_data;
    std::vector<float> ratios; //servo to joint gear ratio
    std::vector<float> limits_upper; //Joint limits in RAD
    std::vector<float> limits_lower; //Joint limits in RAD

    int bus;
    std::string path;
};

struct Robot {
    std::vector<ServoGroup> servo_groups;
};


int main(int argc, char** argv) {

    std::cout << "Starting teleop demo" << std::endl;

    // bilateral teleop

    // ROBOT 1

    Robot robot1;

    // Create Servo Groups
    ServoGroup sg1;
    sg1.ids = {11, 12, 13};
    sg1.ratios = {3.125, 3.125, 3.125};
    sg1.limits_upper = {0.785398, 0.785398, 0.785398};
    sg1.limits_lower = {-0.785398, -0.785398, -0.785398};
    sg1.path = "/dev/ttyACM1";

    robot1.servo_groups.push_back(sg1);

    // ------------------------------------------------

    // ROBOT 2
    Robot robot2;

    // Create Servo Groups
    ServoGroup sg2;
    sg2.ids = {21, 23, 22};
    sg2.ratios = {3.125, 3.125, 3.125};
    sg2.limits_upper = {0.785398, 0.785398, 0.785398};
    sg2.limits_lower = {-0.785398, -0.785398, -0.785398};
    sg2.path = "/dev/ttyACM0";

    robot2.servo_groups.push_back(sg2);
    // ------------------------------------------------

    std::vector<Robot> bi_lateral_robots = {robot1, robot2}; 


    // TODO assert that robot 1 and 2 have same number of servo groups and servo id's

    // Generate Servo Objects

    moteus::FdcanusbFactory TransportFactory;

    for (auto& robot : bi_lateral_robots) {

        for (ServoGroup& group : robot.servo_groups) {

            std::vector<std::string> args = { "--fdcanusb", group.path };
            auto group_transport = TransportFactory.make(args);
            group.transport = group_transport.first;

            for (const int& id: group.ids){

                moteus::Controller::Options options;
                options.id = id;
                options.transport = group_transport.first;
                group.controllers[id] = std::make_shared<moteus::Controller>(options);

            }
        }
    }



    // Stop everything to clear faults.
    for (const Robot& robot : bi_lateral_robots) {
        for (const auto& group : robot.servo_groups) {
            for (const auto& pair : group.controllers) {
                pair.second->SetOutputNearest({});
                pair.second->SetStop();
            }
        }
    }



    while (true) {
        const auto now = GetNow();

        char buf[4096] = {};
        std::string status_line;

        bool simple_teleop = true;

        for (int i = 0; i < bi_lateral_robots.size(); i++) {

            int slave_id = i;
            int master_id = (i+1)%2;

            auto& slave_servo_groups = bi_lateral_robots[slave_id].servo_groups;
            auto& master_servo_groups = bi_lateral_robots[master_id].servo_groups;

            // std::cout << "Slave: " << slave_id << " Master: " << master_id << std::endl;

            for (int j = 0; j < slave_servo_groups.size(); j++) {
                std::vector<moteus::CanFdFrame> command_frames;

                ServoGroup& slave_servo_group = slave_servo_groups[j];
                ServoGroup& master_servo_group = master_servo_groups[j];

                for (int k = 0; k < slave_servo_group.controllers.size(); k++) {
                    moteus::PositionMode::Command position_command;

                    int slave_controller_id = slave_servo_group.ids[k];
                    int master_controller_id = master_servo_group.ids[k];

                    position_command.position = master_servo_group.servo_data[master_controller_id].position;
                    // position_command.position = NaN;

                    position_command.velocity = master_servo_group.servo_data[master_controller_id].velocity;

                    position_command.kp_scale = 0.1;
                    // position_command.velocity = 0.1 * std::sin(now + k);
                    // if (i == 0){
                    //     position_command.position = NaN;
                    //     position_command.velocity = NaN;
                    // }
                    if (k == 0){
                        // std::cout << slave_servo_group.servo_data[11];
                        // std::cout << "Slave: " << slave_controller_id;
                        for (const auto&pair : slave_servo_group.servo_data){
                            std::cout << "ID: " << pair.first << " POS: " << pair.second.position << std::endl;
                        }
                        // std::cout << "Slave: " << slave_servo_group.servo_data[slave_controller_id].position << " Master: " << master_servo_group.servo_data[master_controller_id].position << std::endl;
                    }

                    if ((i==0) && simple_teleop){
                        command_frames.push_back(slave_servo_group.controllers[slave_controller_id]->MakeQuery());
                    }

                    else {
                    command_frames.push_back(slave_servo_group.controllers[slave_controller_id]->MakePosition(position_command));
                    }
                }

                // Now send them in a single call to Transport::Cycle.
                std::vector<moteus::CanFdFrame> replies;
                const auto start = GetNow();
                slave_servo_group.transport->BlockingCycle(&command_frames[0], command_frames.size(), &replies);
                const auto end = GetNow();
                const auto cycle_time = end - start;

                // We parse these into a map to both sort and de-duplicate them,
                // and persist data in the event that any are missing.
                for (const auto& frame : replies) {
                    slave_servo_group.servo_data[static_cast<int>(frame.source)] = moteus::Query::Parse(frame.data, frame.size);
                    // std::cout << "Servo: " << static_cast<int>(frame.source) << " Position: " << slave_servo_group.servo_data[static_cast<int>(frame.source)].position << std::endl;

                }
                // std::cout << "Slave: " << slave_servo_group.servo_data[slave_controller_id].position << " Master: " << master_servo_group.servo_data[master_controller_id].position << std::endl;

                // ::snprintf(buf, sizeof(buf) - 1, "%10.2f dt=%7.4f) ", now, cycle_time);

                // status_line += buf;

                // for (const auto& pair : slave_servo_group.servo_data) {
                // const auto r = pair.second;
                // ::snprintf(buf, sizeof(buf) - 1,
                //             "%2d %3d p/v/t=(%7.3f,%7.3f,%7.3f)  ",
                //             pair.first,
                //             static_cast<int>(r.mode),
                //             r.position,
                //             r.velocity,
                //             r.torque);
                // status_line += buf;
                // }

            } 
        }
        // ::printf("%s  \r", status_line.c_str());
        // ::fflush(::stdout);
        ::usleep(20000);
    }

   
    std::cout << "Ready for Calibrating Motors\n";

    std::cout << "Ready for Teleop\n";


    std::cout << "Hello World\n";
    return 0;
}

// Runs on a single robot computer (matches the left arm to the right arm)



// SETUP

// CONTROL LOOP

// FOR servo group
