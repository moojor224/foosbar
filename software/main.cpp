#include <Network.h>
#include <chrono>
#include <mutex>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>


#include <cstddef>
#include <functional>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <csignal>
#include <iostream>
#include <algorithm>
#include <deque>
#include <queue>

#include <clearpath/pubMotion.h>
#include <clearpath/pubSysCls.h>

#include <qualisys_cpp_sdk/RTProtocol.h>
#include <qualisys_cpp_sdk/RTPacket.h>

#include <uWebSockets/App.h>
#include <uWebSockets/PerMessageDeflate.h>
#include <nlohmann/json.hpp>

#include "physical_params.hpp"
#include "algo.hpp"

using namespace std;
using json = nlohmann::json;

/******************************************************************************
 * Defines/const
 ******************************************************************************/

#define ever ;; // lol this is funny. for(ever){} is an infinite loop
#define eps 1e-3

const int init_vel_lin_cm_s = 100;
const int init_accel_lin_cm_ss = 1000;

const int init_vel_rot_deg_s = 5000;
const int init_accel_rot_deg_ss = 50000;

const int homing_timeout_ms = 10000;

/******************************************************************************
 * Global Variables
 ******************************************************************************/

// Might not want these to be global later, whatever for now
sFnd::SysManager mgr;
vector<reference_wrapper<sFnd::INode>> nodes[num_axis_t];

/******************************************************************************
 * Definitions
 ******************************************************************************/
void onLowHChange(int, void*) {}
void onHighHChange(int, void*) {}
void onLowSChange(int, void*) {}
void onHighSChange(int, void*) {}
void onLowVChange(int, void*) {}
void onHighVChange(int, void*) {}

/******************************************************************************
 * Motor Wrappers
 ******************************************************************************/


/**
 * Moves the specified rod to the given position in centimeters.
 * 
 * @param rod The rod to move.
 * @param position_cm The target position in centimeters.
 */
void move_lin(int rod, double position_cm){
    /* if(rod != three_bar) return; */
    int target_cnts = clamp(
            (int)(-lin_cm_to_cnts[rod] * position_cm),
            min(lin_range_cnts[rod][0], lin_range_cnts[rod][1]),
            max(lin_range_cnts[rod][0], lin_range_cnts[rod][1])
    );
    nodes[lin][rod].get().Motion.MovePosnStart(target_cnts, true);
}

/**
 * Rotates the specified rod to the given position in degrees.
 * 
 * @param rod The rod to be rotated.
 * @param position_deg The target position in degrees.
 */
void move_rot(int rod, double position_deg){
    /* if(rod != three_bar) return; */
    int target_cnts = rot_deg_to_cnts[rod] * (position_deg - cal_rot);
    nodes[rot][rod].get().Motion.MovePosnStart(target_cnts, true);
}

function<void(int, double)> mtr_move[num_axis_t] = {move_lin, move_rot};

/**
 * Sets the speed and acceleration limits for linear motion.
 * 
 * @param rod The rod index.
 * @param vel_cm_per_s The desired velocity in centimeters per second.
 * @param acc_cm_per_s2 The desired acceleration in centimeters per second squared.
 */
void set_speed_lin(int rod, double vel_cm_per_s, double acc_cm_per_s2){
    nodes[lin][rod].get().Motion.VelLimit = abs(vel_cm_per_s * lin_cm_to_cnts[rod]);
    nodes[lin][rod].get().Motion.AccLimit = abs(acc_cm_per_s2 * lin_cm_to_cnts[rod]);
}

/**
 * Sets the speed and acceleration limits for a specific rod rotation.
 * 
 * @param rod The index of the rod.
 * @param vel_deg_per_s The desired velocity limit in degrees per second.
 * @param acc_deg_per_s2 The desired acceleration limit in degrees per second squared.
 */
void set_speed_rot(int rod, double vel_deg_per_s, double acc_deg_per_s2){
    nodes[rot][rod].get().Motion.VelLimit = vel_deg_per_s * rot_deg_to_cnts[rod];
    nodes[rot][rod].get().Motion.AccLimit = acc_deg_per_s2 * rot_deg_to_cnts[rod];
}

function<void(int, double, double)> mtr_set_speed[num_axis_t] = {set_speed_lin, set_speed_rot};

/**
 * Closes all ports and disables all nodes.
 */
void close_all(){
    sFnd::IPort&port = mgr.Ports(0);
    for(int i = 0; i < port.NodeCount(); ++i){
        port.Nodes(i).EnableReq(false);
    }
    mgr.PortsClose();
}

/**
 * @brief Initializes the motors.
 * 
 * This function initializes the motors by performing the following steps:
 * 1. Identifies the connected motor hubs.
 * 2. Finds available ports and opens them.
 * 3. Arranges the nodes (motors) based on their names.
 * 4. Enables the nodes.
 * 5. Waits for the nodes to enable.
 * 6. Starts homing for the applicable nodes.
 * 7. Waits for the homing process to complete.
 * 
 * @return 0 if the motors are successfully initialized, -1 otherwise.
 */
int motors_init(){
    vector<string> comHubPorts;

    // Identify hubs
    sFnd::SysManager::FindComHubPorts(comHubPorts); // find connected motor hubs
    printf("Found %zu SC Hubs\n", comHubPorts.size());

    // Find available ports
    size_t portCount = 0;
    for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {
        mgr.ComHubPort(portCount, comHubPorts[portCount].c_str(), MN_BAUD_48X);
    }

    if (portCount < 0) {
        printf("Unable to locate SC hub port\n");
        return -1;
    }

    // Open ports (hubs)
    mgr.PortsOpen(portCount);

    sFnd::IPort &port = mgr.Ports(0);
    printf(" Port[%d]: state=%d, nodes=%d\n",
        port.NetNumber(), port.OpenState(), port.NodeCount());

    // Arrange nodes
    for(int i = 0; i < num_rod_t; ++i){
        string lin_name = "lin-" + rod_names[i];
        string rot_name = "rot-" + rod_names[i];
        bool lin_found = false, rot_found = false;

        // Search to find correct names
        for(int j = 0; j < port.NodeCount(); ++j){
            string name = port.Nodes(j).Info.UserID.Value();
            if(!lin_found && lin_name == name){
                nodes[lin].push_back(port.Nodes(j));
                lin_found = true;
            }
            if(!rot_found && rot_name == name){
                nodes[rot].push_back(port.Nodes(j));
                rot_found = true;
            }
        }
        if(!lin_found || !rot_found){
            printf("Not all motors are present!\n");
            return -1;
        }
    }

    // Enable nodes
    for(int i = 0; i < nodes[lin].size(); ++i){
        nodes[lin][i].get().Status.AlertsClear();
        nodes[lin][i].get().Motion.NodeStopClear();
        nodes[lin][i].get().EnableReq(true);
    }

    for(int i = 0; i < nodes[rot].size(); ++i){
        nodes[rot][i].get().Status.AlertsClear();
        nodes[rot][i].get().Motion.NodeStopClear();
        nodes[rot][i].get().EnableReq(true);
    }

    // Wait for enable
    double timeout = mgr.TimeStampMsec() + 2000;
    for(ever){
        bool ready = true;
        for(int i = 0; i < nodes[lin].size(); ++i){
            if(!nodes[lin][i].get().Motion.IsReady()) ready = false;
        }
        for(int i = 0; i < nodes[rot].size(); ++i){
            if(!nodes[rot][i].get().Motion.IsReady()) ready = false;
        }
        if(ready) break;
        if (mgr.TimeStampMsec() > timeout) {
            printf("Timed out waiting for Nodes to enable\n");
            return -1;
        }
    }


    // Start homing
    for(int i = 0; i < nodes[lin].size(); ++i){
        if(!nodes[lin][i].get().Motion.Homing.HomingValid()) continue;
        if(!nodes[lin][i].get().Motion.Homing.WasHomed()) 
            nodes[lin][i].get().Motion.Homing.Initiate();
    }

    // Wait for homing
    timeout = mgr.TimeStampMsec() + homing_timeout_ms;
    for(ever){
        bool homed = true;
        for(int i = 0; i < nodes[lin].size(); ++i){
            if(!nodes[lin][i].get().Motion.Homing.HomingValid()) continue;
            if(!nodes[lin][i].get().Motion.Homing.WasHomed()) homed = false;
        }
        if(homed) break;
        
        if(mgr.TimeStampMsec() > timeout){
            cout << "Homing timed out" << endl;
            close_all();
            return -1;
        }
    }

    // Set motion parameters
    for(int i = 0; i < nodes[lin].size(); ++i){
        nodes[lin][i].get().AccUnit(sFnd::INode::COUNTS_PER_SEC2);
        nodes[lin][i].get().VelUnit(sFnd::INode::COUNTS_PER_SEC);
        nodes[lin][i].get().Info.Ex.Parameter(98,1);
        nodes[lin][i].get().Limits.TrqGlobal = 100;
        set_speed_lin((rod_t)i, init_vel_lin_cm_s, init_accel_lin_cm_ss);
        /* set_speed_lin((rod_t)i, 100, 500); */
        /* cout << "Set linear speed for " << rod_names[i] << endl; */
        nodes[lin][i].get().Motion.PosnMeasured.AutoRefresh(true);
        /* move_lin(i, lin_range_cm[i]/2); */
    }
    for(int i = 0; i < nodes[rot].size(); ++i){
        nodes[rot][i].get().AccUnit(sFnd::INode::COUNTS_PER_SEC2);
        nodes[rot][i].get().VelUnit(sFnd::INode::COUNTS_PER_SEC);
        nodes[rot][i].get().Limits.TrqGlobal = 100;
        if(i != goalie)
            nodes[rot][i].get().Info.Ex.Parameter(98,1);
        nodes[rot][i].get().Motion.PosnMeasured.AutoRefresh(true);
        /* set_speed_rot((rod_t)i, 10000, 100000); */
        set_speed_rot((rod_t)i, init_vel_rot_deg_s, init_accel_rot_deg_ss);
        /* move_rot(i, 0); */
    }

    return 0;
}

/******************************************************************************
 * Misc
 ******************************************************************************/

/**
 * Checks if the program should terminate.
 * 
 * This function uses the select system call to check if there is any input available on the standard input stream (stdin).
 * If there is input available and the input is the character 'q', the function returns true indicating that the program should terminate.
 * Otherwise, it returns false indicating that the program should continue running.
 * 
 * @return true if the program should terminate, false otherwise.
 */
bool should_terminate(){
    fd_set set;
    struct timeval timeout;

    // Initialize the file descriptor set
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);

    // Initialize the timeout data structure
    timeout.tv_sec = 0;  // Wait for up to 1 second
    timeout.tv_usec = 500;

    // Check if input is available
    if (select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout) > 0) {
        string command;
        cin >> command;
        if (command == "q") {
            return true;
        }
    }
    return false;
}

/**
 * @brief Signal handler function that handles interrupt signals.
 * 
 * This function is called when an interrupt signal is received. It prints a message indicating that an interrupt signal has been received and performs some cleanup operations.
 * 
 * @param signal The signal number that triggered the handler.
 */
void signal_handler(int signal) {
    cout << "Got interrupt signal, closing..." << endl;
    if (signal == SIGINT) {
        /* close_all(); */
        /* cout << "Motors disabled" << endl; */
        /* exit(signal); */
        /* quit = true; */
    }
}

/**
 * Prints the status message to the console.
 * 
 * @param status The status message to be printed.
 * @param log The log message to be printed (optional).
 * @param erase Determines whether to erase the previous status message (default: true).
 */
void print_status(string status, string log = "", bool erase = true){
    // Assumes status string is always same length
    if(erase)
        for(char c : status)
            if(c == '\n')
                cout << "\033[A\33[2K\r";
    cout << status;
}

/******************************************************************************
 * Main
 ******************************************************************************/
/**
 * @brief The main entry point of the program.
 *
 * @param argc The number of command-line arguments.
 * @param argv An array of C-style strings containing the command-line arguments.
 * @return An integer representing the exit status of the program.
 */
int main(int argc, char** argv){

    /**************************************************************************
     * Setup
     **************************************************************************/
    bool controller = false, no_motors = false;

    for(int i = 1; i < argc; ++i){
        /**
         * @brief Represents the command stored in the `argv[i]` argument.
         */
        string cmd(argv[i]);
        if(cmd == "--controller"){
            controller = true;
        } else if(cmd == "--no-motors"){
            no_motors = true;
        } else {
            cout << "Unrecognized argument: " << cmd << endl;
            return -1;
        }
    }

    // For some reason first call gives garbage value
    (void) mgr.TimeStampMsec();

    /**************************************************************************
     * WebSocket Init
     **************************************************************************/

    struct socket_data {
        /* User data */
    };
    /**
     * @brief A vector of WebSocket clients.
     *
     * This vector stores pointers to WebSocket clients of type `uWS::WebSocket<false, true, socket_data>`.
     * It is used to keep track of the connected clients in the application.
     */
    vector<uWS::WebSocket<false, true, socket_data>*> clients;
    /**
     * @brief A synchronization primitive that can be used to protect shared data from being simultaneously accessed by multiple threads.
     */
    mutex ws_mutex;

    // Webapp state
    /**
     * @brief Initialize the target positions of the rods.
     * 
     * This array stores the target positions of the rods. The size of the array is `num_rod_t`.
     * Each element of the array represents the target position of a rod.
     * 
     * @note The initial target position for all rods is set to 0.5 (centered).
     */
    double tgt_pos[num_rod_t] = {0.5, 0.5, 0.5, 0.5};
    /**
     * @brief Array to store target rotation values for rods.
     *
     * This array, tgt_rot, is used to store the target rotation values for the rods.
     * The array has a size of num_rod_t and is initialized with zeros.
     */
    double tgt_rot[num_rod_t] = {0, 0, 0, 0};
    bool in_shot = false;
    /**
     * @brief An array to track whether the target position has been updated for each rod.
     *
     * This boolean array is used to keep track of whether the target position has been updated for each rod.
     * The array is initialized with all elements set to false (0).
     *
     * @note The size of the array is determined by the value of `num_rod_t`.
     */
    bool tgt_pos_updated[num_rod_t] = {0};
    /**
     * @brief An array to track whether the target rotation has been updated for each rod.
     *
     * This boolean array is used to keep track of whether the target rotation has been updated for each rod.
     * The array is initialized with all elements set to false (0).
     *
     * @note The size of the array is determined by the value of 'num_rod_t'.
     */
    bool tgt_rot_updated[num_rod_t] = {0};
    int ws_selection = -1;

    struct uWS::Loop *loop;
    // Thread for web socket handling
    /**
     * @brief This thread runs the uWS server and handles incoming WebSocket connections and messages.
     */
    thread uws_thread([&]() {
        loop = uWS::Loop::get();
        // C++20 acting funky and makes me specificy every field
        uWS::App app;
        app.ws<socket_data>("/position", {
            .compression = uWS::DISABLED,
            .maxPayloadLength = 16 * 1024 * 1024,
            .idleTimeout = 16,
            .maxBackpressure = 1 * 1024 * 1024,
            .closeOnBackpressureLimit = false,
            .resetIdleTimeoutOnSend = false,
            .sendPingsAutomatically = true,
            .maxLifetime = 0,

            .upgrade = nullptr,
            .open = [&clients, &ws_mutex](auto *ws) {
                /* cout << "Connection! " << 9001 << endl; */
                lock_guard<mutex> lock(ws_mutex);
                clients.push_back(ws);
                /* nlohmann::json params = {}; */
                /* for(int i = 0; i < num_rod_t; ++i){ */
                /*     params["spacing-" + rod_names[i]] = plr_gap[i]; */
                /* } */

            },
            // Handles incoming packets
            .message = [&ws_mutex, &tgt_pos, &tgt_pos_updated, &tgt_rot, &tgt_rot_updated, &ws_selection, &in_shot]
                    (auto *ws, string_view message, uWS::OpCode opCode) {
                lock_guard<mutex> lock(ws_mutex);
                json packet = json::parse(message);

                if(packet["type"].get<string>() == "selection"){
                    ws_selection = packet["selection"].get<int>();

                } else if(packet["type"].get<string>() == "move"){
                    if(ws_selection >= 0 && ws_selection <=num_rod_t){
                        tgt_pos[ws_selection] = clamp(tgt_pos[ws_selection]+packet["pos"].get<double>(), 0.0, 1.0);
                        tgt_rot[ws_selection] = tgt_rot[ws_selection]+packet["rot"].get<double>();
                        if(abs(packet["rot"].get<double>()) > 1) in_shot = true;
                        tgt_pos_updated[ws_selection] = abs(packet["pos"].get<double>()) > 0.001;
                        tgt_rot_updated[ws_selection] = abs(packet["rot"].get<double>()) > 0.001;
                    }
                }
            },
            .drain = [](auto * /*ws*/) {},
            .ping = [](auto * /*ws*/, string_view) {},
            .pong = [](auto * /*ws*/, string_view) {},
            .close = [&clients, &ws_mutex](auto* ws, int /*code*/, string_view /*message*/) {
                /* cout << "Client disconnected" << endl; */
                lock_guard<mutex> lock(ws_mutex);
                clients.erase(remove(clients.begin(), clients.end(), ws), clients.end());
            }
        }).listen(9001, [](auto *listen_socket) {
            if (listen_socket) {
                cout << "Listening on port " << 9001 << endl;
            }
        });

        app.run(); // Run the event loop
    });

    /**************************************************************************
     * QTM Init
     **************************************************************************/

    mutex qtm_mutex;
    /**
     * @brief The position of the ball in a fast coordinate system.
     *
     * This vector stores the x, y, and z coordinates of the ball in a fast coordinate system.
     * The coordinates are represented as double values.
     */
    vector<double> ball_pos_fast = {0, 0, 0};
    /**
     * @brief The position of the ball in a slow motion scenario.
     *
     * This vector stores the x, y, and z coordinates of the ball's position in a slow motion scenario.
     */
    vector<double> ball_pos_slow = {0, 0, 0};
    /**
     * @brief The velocity of the ball in three-dimensional space.
     *
     * This vector represents the velocity of the ball along the x, y, and z axes.
     * The values in the vector correspond to the velocity components in the order of x, y, and z.
     */
    vector<double> ball_vel = {0, 0, 0};
    bool ball_in_motion = false; // Crude measure of whether ball is in motion

    double rod_pos[num_axis_t][num_rod_t] = {{0,0,0,0}, {0,0,0,0}};
    double rod_in_vision[num_rod_t] = {false, false, false, false};

    double qtm_time = 0;

    /**
     * @brief The thread responsible for handling the communication with the QTM system.
     *
     * This thread connects to the QTM server, streams frames, and processes the received data.
     * It continuously receives packets from the QTM system and updates the ball and rod positions
     * based on the marker data. It also applies exponential weighted moving average (EWMA) filtering
     * to smooth out the positions and velocities.
     */
    thread qtm_thread([&qtm_mutex, &ball_pos_fast, &ball_pos_slow, &ball_vel, &rod_pos, &qtm_time, &rod_in_vision, &ball_in_motion]() {
        CRTProtocol rtProtocol;

        const char           serverAddr[] = "192.168.155.1";
        const unsigned short basePort = 22222;
        const int            majorVersion = 1;
        const int            minorVersion = 19;
        const bool           bigEndian = false;

        unsigned short udpPort = 6734;
        
        while (!rtProtocol.Connected())
        {
            if (!rtProtocol.Connect(serverAddr, basePort, &udpPort, majorVersion, minorVersion, bigEndian))
            {
                printf("rtProtocol.Connect: %s\n\n", rtProtocol.GetErrorString());
                sleep(1);
            }
        }

        if(!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort, nullptr, CRTProtocol::cComponent3dNoLabels)){
            printf("Failed streaming!\n");
            return -1;
        }

        
        deque<pair<double, vector<double>>> pos_buffer;
        const int buf_cap = vision_fps;
        // Higher = more noise, less latency
        const double gamma_vel = 0.2;
        /* const double gamma_vel = 0.05; */
        /* const double gamma_pos = 0.05; */
        const double gamma_pos = 0.1;
        for(ever){
            /**
             * Enum representing the type of packet in the CRTPacket class.
             */
            CRTPacket::EPacketType packetType;
            if(rtProtocol.Receive(packetType, true, 0) == CNetwork::ResponseType::success){
                if(packetType != CRTPacket::PacketData) continue;
                lock_guard<mutex> lock(qtm_mutex);
                double t_start = mgr.TimeStampMsec();

                CRTPacket *rtPacket = rtProtocol.GetRTPacket();

                bool ball_seen = false;
                for(int r = 0; r < num_rod_t; ++r)
                    rod_in_vision[r] = false;
                for(int m = 0; m < rtPacket->Get3DNoLabelsMarkerCount(); ++m){
                    
                    // Not read directly since we want doubles not floats
                    vector<float> marker_pos = {0, 0, 0};
                    unsigned int n;
                    rtPacket->Get3DNoLabelsMarker(m, marker_pos[0], marker_pos[1], marker_pos[2], n);
                    for(int i = 0; i < 3; ++i){
                        marker_pos[i] = marker_pos[i] / 10; // convert to mm from cm
                        marker_pos[i] -= cal_offset[i];
                    }
                    // Offset convention, 0 at edge of table
                    marker_pos[0] += play_height / 2;

                    if(marker_pos[2] < 2 && !ball_seen){
                        // ball
                        for(int i = 0; i < 3; ++i) ball_pos_fast[i] = marker_pos[i];
                        pos_buffer.push_front({mgr.TimeStampMsec(), ball_pos_fast});
                        ball_seen = true;
                    } else if(marker_pos[2] > plr_height){
                        // hat
                        auto [side, rod] = closest_rod(marker_pos[1]);
                        if(side != human) continue;
                        double dy = marker_pos[1] - (-rod_coord[rod]);
                        /* cout << marker_pos[1] << ", " << */ 
                        rod_pos[rot][rod] = asin(clamp(dy/hat_height, -1.0, 1.0)) / deg_to_rad;
                        rod_pos[lin][rod] = marker_pos[0] - bumper_width - plr_width / 2;
                        rod_in_vision[rod] = true;
                    }


                }
                if(!ball_seen){
                    if(pos_buffer.size() <= 0) continue;
                    pos_buffer.push_front({mgr.TimeStampMsec(), pos_buffer[0].second});
                }
                if(pos_buffer.size() < buf_cap) continue;
                pos_buffer.pop_back();
                // EWMA
                double vel[3] = {0,0,0}, pos[3] = {0,0,0}, denom_vel = 0, denom_pos = 0;
                for(int i = 0; i < pos_buffer.size()-1; ++i){
                    double scale_vel = exp(-gamma_vel * i);
                    double scale_pos = exp(-gamma_pos * i);
                    denom_vel += scale_vel;
                    denom_pos += scale_pos;
                    for(int j = 0; j < 3; ++j){
                        // Important: the first one seems like it should be better, but it actually isn't
                        // Network delay is unpredictable, so sometimes timing we get it doesn't represent
                        // timing the video was captured. This can give extraneous high spikes in velocity.
                        // It does however rely on having a hardcoded fps which isn't ideal
                        /* num[j] += (pos_buffer[i].second[j] - pos_buffer[i+1].second[j]) * scale */
                        /*     / ((pos_buffer[i].first - pos_buffer[i+1].first)/1000); */
                        vel[j] += (pos_buffer[i].second[j] - pos_buffer[i+1].second[j]) * scale_vel * vision_fps;
                        pos[j] += pos_buffer[i].second[j] * scale_pos;
                    }
                }

                double max[2] = {0,-play_width/2}, min[2] = {play_height, play_width/2};
                for(int i = 0; i < vision_fps/3; ++i){
                    for(int j = 0; j < 2; ++j){
                        if(pos_buffer[i].second[j] > max[j]) max[j] = pos_buffer[i].second[j];
                        if(pos_buffer[i].second[j] < min[j]) min[j] = pos_buffer[i].second[j];
                    }
                }
                ball_in_motion = max[0] - min[0] > 1 || max[1] - min[1] > 1;
                /* cout << max[0] - min[0] << ", "  << max[1] - min[1] << endl; */

                for(int j = 0; j < 3; ++j){
                    ball_vel[j] = vel[j] / denom_vel;
                    ball_pos_slow[j] = pos[j] / denom_pos;
                }
                qtm_time = mgr.TimeStampMsec() - t_start;
            }
        }
    });

    /**************************************************************************
     * Clearpath thread
     **************************************************************************/

    // Do this on the main thread just to make sure that everything is initialized
    mutex mtr_mutex;

    const struct motor_cmd null_cmd = {NAN, NAN, NAN};

    // Could be fancier with some kind of a priority queue, but I think this is fine
    vector<motor_cmd> mtr_cmds[num_axis_t];
    queue<function<void(void)>> mtr_fns;

    /**
     * @brief The last update time for each axis in the mtr_t_last_update vector.
     * 
     * This vector stores the last update time for each axis in the system. The index of the vector represents the axis number.
     * The update time is stored as a double value.
     */
    vector<double> mtr_t_last_update[num_axis_t];
    /**
     * @brief The last command sent to each axis motor.
     *
     * This vector stores the last command sent to each axis motor. It is a vector of doubles,
     * with each element representing the last command for a specific axis. The size of the vector
     * is determined by the value of `num_axis_t`.
     */
    vector<double> mtr_t_last_cmd[num_axis_t];

    /**
     * @brief Array of vectors to store the last motor command for each axis.
     * 
     * This variable is used to store the last motor command for each axis. It is an array of vectors, where each element of the array represents an axis and the corresponding vector stores the last motor command for that axis.
     * 
     * @note The size of the array is determined by the `num_axis_t` constant.
     */
    vector<motor_cmd> mtr_last_cmd[num_axis_t];

    bool disable_motor_updates = false;

    /**
     * @brief The current positions of the axes.
     *
     * This variable is a vector of doubles that stores the current positions of the axes.
     * The size of the vector is determined by the `num_axis_t` constant.
     */
    vector<double> cur_pos[num_axis_t];

    for(int a = 0; a < num_axis_t; ++a){
        for(int r = 0; r < num_rod_t; ++r){
            mtr_cmds[a].push_back(null_cmd);
            mtr_t_last_update[a].push_back(mgr.TimeStampMsec());
            mtr_t_last_cmd[a].push_back(mgr.TimeStampMsec());
            if(a == rot){
                mtr_last_cmd[a].push_back({0, init_vel_lin_cm_s, init_accel_lin_cm_ss});
                cur_pos[a].push_back(lin_range_cm[r]/2);
            }
            else{
                mtr_last_cmd[a].push_back({0, init_vel_rot_deg_s, init_accel_rot_deg_ss});
                cur_pos[a].push_back(0);
                /* move_lin(r, 0); */
            }
        }
    }

    if(!no_motors){
        int init_err = motors_init();
        if(init_err < 0) return init_err;

    }

    // This is the only thread that should ever query motors directly
    /**
     * @brief This thread is responsible for executing motor commands and updating motor positions.
     *
     * It continuously loops through the motor functions and executes them. It also updates the motor positions
     * based on the time elapsed since the last update. This thread ensures thread safety by using a mutex to
     * protect shared data.
     */
    thread mtr_thread([no_motors, &mtr_mutex, &mtr_cmds, &mtr_t_last_update, &mtr_t_last_cmd, &mtr_last_cmd, &cur_pos, &disable_motor_updates, &mtr_fns]() {
        if(no_motors) return;

        /**
         * @brief The refresh time in milliseconds for the motor.
         */
        const double mtr_refresh_t_ms = 100;

        /**
         * Executes the commands stored in the `mtr_fns` queue and updates the motor settings.
         * This lambda function is responsible for iterating over the `mtr_fns` queue and executing each command.
         * After executing each command, it checks if there are any changes in the motor settings (velocity, acceleration, position),
         * and if so, it updates the motor settings accordingly.
         * This function ensures thread safety by using a mutex to protect the shared data.
         */
        auto exec_cmds = [&](){
            while(mtr_fns.size() > 0){
                mtr_fns.front()();
                mtr_fns.pop();
            }
            for(int a = 0; a < num_axis_t; ++a){
                for(int r = 0; r < num_rod_t; ++r){
                    // Awkward to make sure thread safe
                    /**
                     * @brief A vector of function objects representing moves.
                     * 
                     * This vector stores function objects that represent different moves. Each function object takes no arguments and returns no value.
                     * The vector can be used to store and execute different moves in a game or simulation.
                     */
                    vector<function<void(void)>> moves;
                    {
                        /**
                         * @brief A RAII-style wrapper for a mutex lock.
                         *
                         * The lock_guard class is a convenient way to automatically lock and unlock a mutex using the RAII (Resource Acquisition Is Initialization) idiom.
                         * When a lock_guard object is created, it locks the associated mutex. When the lock_guard object goes out of scope, it automatically releases the lock.
                         * This ensures that the mutex is always properly locked and unlocked, even in the presence of exceptions.
                         *
                         * @param m The mutex to be locked.
                         */
                        lock_guard<mutex> lock(mtr_mutex);
                        /**
                         * @brief The motor command variable.
                         * 
                         * This variable stores the motor command for a specific action and rotation.
                         * It is used to control the motor behavior in the program.
                         */
                        motor_cmd cmd = mtr_cmds[a][r];
                        /**
                         * @brief Represents the last motor command.
                         * 
                         * This variable stores the last motor command that was executed for a specific motor and rotation.
                         * It is of type `motor_cmd`.
                         */
                        motor_cmd last_cmd = mtr_last_cmd[a][r];

                        if((!isnan(cmd.vel) && abs(cmd.vel - last_cmd.vel) > eps)
                                || (!isnan(cmd.accel) && abs(cmd.accel - last_cmd.accel) > eps)){
                            moves.push_back([a, r, cmd](){
                                mtr_set_speed[a](r, cmd.vel, cmd.accel);
                            });
                            if(!isnan(cmd.vel))
                                mtr_last_cmd[a][r].vel = cmd.vel;
                            if(!isnan(cmd.accel))
                                mtr_last_cmd[a][r].accel = cmd.accel;
                            mtr_t_last_cmd[a][r] = mgr.TimeStampMsec();
                        }

                        if(!isnan(cmd.pos) && abs(cmd.pos - last_cmd.pos) > eps){
                            moves.push_back([a, r, cmd](){
                                mtr_move[a](r, cmd.pos);
                            });
                            mtr_last_cmd[a][r].pos = cmd.pos;
                            mtr_t_last_cmd[a][r] = mgr.TimeStampMsec();
                        }
                    }
                    // This is outside the lock's scope to avoid holding mutex too long
                    for(auto fn : moves){

                        try{
                            fn();
                        } catch (sFnd::mnErr& theErr)
                        {
                            printf("Caught mnErr\n");
                            printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
                            cout << endl << endl << endl << endl << endl << endl;
                        }
                    }
                }
            }
        };
        for(ever){
            for(int a = 0; a < num_axis_t; ++a){
                for(int r = 0; r < num_rod_t; ++r){
                    exec_cmds();
                    if(mgr.TimeStampMsec() - mtr_t_last_update[a][r] > mtr_refresh_t_ms && !disable_motor_updates){
                        lock_guard<mutex> lock(mtr_mutex);
                        if(a == lin){
                            cur_pos[a][r] = abs(nodes[lin][r].get().Motion.PosnMeasured.Value()
                                    / lin_cm_to_cnts[r]);
                        } else {
                            cur_pos[a][r] = nodes[rot][r].get().Motion.PosnMeasured.Value()
                                    / rot_rad_to_cnts[r] / deg_to_rad + cal_rot;
                        }
                        mtr_t_last_update[a][r] = mgr.TimeStampMsec();
                    } else {
                        this_thread::sleep_for(chrono::microseconds(100));
                    }
                }
            }
        }
    });

    /**************************************************************************
     * Main Event Loop
     **************************************************************************/

    cout << endl << endl << endl << endl << endl << endl;
    cout << fixed << setprecision(2);
    
    /* state_t state = state_unknown; */
    /* state_t state = state_controlled_move; */
    /* state_t state = state_uncontrolled; */
    /**
     * @brief The current state of the system.
     * 
     * This variable represents the state of the system and is used to control the behavior of the program.
     */
    state_t state = state_defense;
    /* state_t state = state_snake; */
    /* state_t state = state_controlled_five_bar; */

    /**
     * @brief Represents the c5b task.
     * 
     * This variable is used to store the c5b task initialization value.
     */
    c5b_t c5b_task = c5b_init;
    /**
     * @brief The cmove_task variable represents the task for cmove.
     * 
     * It is initialized with the value of cmove_init.
     */
    cmove_t cmove_task = cmove_init;
    csnake_t csnake_task = csnake_init;
    double control_task_timer = mgr.TimeStampMsec();

    // Shot variables
    // 1 = right, -1 = left, 0 = not shooting
    int control_task_shoot_dir = 0;

    // Pass variables
    

    // Move variables
    /* vector<double> cmove_target_cm = {play_height/2,6}; // y is with respect to rod */
    /* vector<double> cmove_target_tol = {2,4}; */
    /* state_t cmove_next_state = state_snake; */
    /**
     * @brief The target position for the cmove operation in centimeters.
     * 
     * The `cmove_target_cm` vector represents the target position for the cmove operation.
     * The first element of the vector represents the x-coordinate, and the second element represents the y-coordinate.
     * The y-coordinate is measured with respect to the rod.
     */
    vector<double> cmove_target_cm = {14,0}; // y is with respect to rod
    /**
     * @brief The tolerance values for the target position in the cmove function.
     * 
     * This vector stores the tolerance values for the target position in the cmove function.
     * The first element represents the tolerance value for the x-coordinate, and the second element represents the tolerance value for the y-coordinate.
     * 
     * @note The tolerance values determine how close the actual position needs to be to the target position for the cmove function to consider the movement as complete.
     */
    vector<double> cmove_target_tol = {2,2};
    /**
     * Represents the next state for the controlled five bar mechanism.
     */
    state_t cmove_next_state = state_controlled_five_bar;
    int cmove_end_side = 1;

    /* for(int i = 0; i < 2; ++i){ */
    /*     double start_t = mgr.TimeStampMsec(); */
    /*     move_rot(i, 90); */
    /*     cout << mgr.TimeStampMsec() - start_t << endl; */
    /* } */

    // So that motor cur_pos is updated by the first loop
    this_thread::sleep_for(chrono::microseconds(200000));
    /**
     * @brief The time in milliseconds.
     */
    double time_ms = mgr.TimeStampMsec();

    for(ever){

        if(should_terminate()) break;

        double start_t = mgr.TimeStampMsec();

        /**
         * @brief The status message stream.
         * 
         * This stringstream object is used to store the status message that will be printed to the console.
         */
        stringstream status;
        /**
         * @brief The log message stream.
         * 
         * This stringstream object is used to store the log message that will be printed to the console.
         */
        stringstream log;

        lock_guard<mutex> qtm_lock(qtm_mutex);
        lock_guard<mutex> mtr_lock(mtr_mutex);

        

        /**
         * @brief Represents the position data in JSON format.
         * 
         * This JSON object contains information about the positions of various elements in the game.
         * It includes the positions of the blue and red players, their rotations, and the position of the ball.
         * The positions are represented as ratios relative to the range of motion of each element.
         */
        json positionData = {
            {"type", "pos"},
            {"bluepos", {
                0.5, 0.5, 0.5, 0.5
            }},
            {"redpos", {
                cur_pos[lin][three_bar] / lin_range_cm[three_bar],
                cur_pos[lin][five_bar] / lin_range_cm[five_bar],
                cur_pos[lin][two_bar] / lin_range_cm[two_bar],
                cur_pos[lin][goalie] / lin_range_cm[goalie],
            }},
            {"bluerot", {
                0,0,0,0
            }},
            {"redrot", {
                cur_pos[rot][three_bar],
                cur_pos[rot][five_bar],
                cur_pos[rot][two_bar],
                cur_pos[rot][goalie],
            }},
            {"ballpos", {
                ball_pos_fast[0]/(play_height),
                ball_pos_fast[1]/(play_width),
                0,
            }}
        };
        status << fixed << setprecision(3) << setw(10) << showpos;
        status << "Ball position fast: " << ball_pos_fast[0] << ", " << ball_pos_fast[1] << ", " << ball_pos_fast[2] << "; " << endl;;
        status << "Ball position slow: " << ball_pos_slow[0] << ", " << ball_pos_slow[1] << ", " << ball_pos_slow[2] << "; ";
        status << "Ball velocity: " << ball_vel[0] << ", " << ball_vel[1] << ", " << ball_vel[2] << endl;
        status << "Marker positions: " << rod_pos[lin][three_bar] << ", " << rod_pos[lin][five_bar] << ", " << rod_pos[lin][two_bar] << ", " << rod_pos[lin][goalie] << "; ";
        status << "Marker rotations: " << rod_pos[rot][three_bar] << ", " << rod_pos[rot][five_bar] << ", " << rod_pos[rot][two_bar] << ", " << rod_pos[rot][goalie] << endl;
        status << "State: " << state << endl;
        status << "Cmove task: " << cmove_task << endl;
        status << "Three bar pos: " << cur_pos[lin][three_bar] << ", rot: " << cur_pos[rot][three_bar] << endl;
        status << "Blocked: " << is_blocked(five_bar, 12, rod_pos, 0, three_bar) << endl;

        /* static int frame = 0; */
        /* status << "Frame: " << ++frame << endl; */
        /* status << "QTM time: " << qtm_time << endl; */
        /**
         * @brief The message variable stores the JSON representation of the position data.
         */
        string message = positionData.dump();

        {
            lock_guard<mutex> lock(ws_mutex);
            for (auto* client : clients) {
                loop->defer([client, message](){
                    client->send(message, uWS::OpCode::TEXT);
                });
            }
        }

        /**
         * Represents the time difference from the previous timestamp to the current time.
         */
        double dt_ms = mgr.TimeStampMsec() - time_ms;
        time_ms = mgr.TimeStampMsec();

        // if the machine is being controlled with a gamepad
        if(controller){
            for(int r = 0; r < num_rod_t; ++r){

                lock_guard<mutex> lock(ws_mutex);
                if(tgt_pos_updated[r]){
                    mtr_cmds[lin][r] = {
                        .pos = lin_range_cm[r] * tgt_pos[r],
                        .vel = 100,
                        .accel = 100,
                    };
                }
                if(tgt_rot_updated[r]){
                    mtr_cmds[rot][r] = {
                        .pos = tgt_rot[r] / deg_to_rad,
                        /*.vel = in_shot ? 10000.0 : 500,*/
                        .vel = 10000.0,
                        /*.accel = in_shot ? 100000.0 : 1000,*/
                        .accel = 100000.0,
                    };
                }
            }
            if(in_shot) in_shot = false;
        // Yes, else switch is just as much as a thing as else if
        } else switch(state){
        /**
         * Normal defense
         */
        case state_defense:
        {
            /**
             * @brief the rod that's in front of the ball
             */
            int front;
            /**
             * @brief Represents the closest rod to the ball position.
             *
             * The `closest` variable is a pair that consists of a `side_t` value representing the side of the rod and a `rod_t` value representing the rod number.
             * It stores the information about the closest rod to the ball position.
             */
            pair<side_t, rod_t> closest = closest_rod(ball_pos_fast[1]);
            if(closest.first == bot){
                /* break; */
                if(ball_vel[1] < -20){
                    state = state_shot_defense;
                } else{
                    state = state_uncontrolled;
                }
                break;
            }
            if(closest.second == three_bar) front = two_bar;
            else if(closest.second == five_bar) front = five_bar;
            else front = three_bar;

            /* ball_vel = {20,-200,0}; */
            double cooldown_time = 25;

            /**
             * @brief The offset from the player to the goalie.
             *
             * This variable represents the distance between the player and the goalie.
             * It is used to determine the positioning of the player relative to the goalie.
             */
            double offset_goalie = plr_offset(1, goalie);
            double offset_two_bar = plr_offset(0, two_bar);

            /**
             * Moves the motor to a specified position with a given velocity and acceleration.
             *
             * @param pos The desired position of the motor.
             * @param vel The desired velocity of the motor.
             * @param accel The desired acceleration of the motor.
             * @param plr The player value.
             * @param rod The rod value.
             * @param period The time period in milliseconds.
             * @param dx The difference threshold for position comparison.
             */
            auto move_motor = [&](double pos, double vel, double accel, int plr, int rod, double period, double dx){
                double offset = plr_offset(plr, rod);
                if(time_ms - mtr_t_last_cmd[lin][rod] > period && abs(pos - offset - mtr_last_cmd[lin][rod].pos) > dx){
                    mtr_cmds[lin][rod] = {pos - offset, vel, accel};
                }
            };

            /**
             * @brief The rod angle to catch the ball at
             */
            const double catch_angle = 30;

            if(front == two_bar){
                static int dir = 1; // Side that goalie is from two_bar
                double target_cm = ball_pos_fast[0];

                if(abs(ball_pos_fast[0] - play_height/2) > goal_width/2){
                    /**
                     * Which side of the field that the ball is on.
                     */
                    int side = ball_pos_fast[0] > play_height/2 ? 1 : -1;
                    double target_cm = play_height/2 + side*5;
                    mtr_cmds[lin][two_bar] = {target_cm - offset_two_bar, 100, 1000};
                    mtr_cmds[lin][goalie] = {side == 1 ? lin_range_cm[goalie] : 0, 100, 1000};
                    mtr_cmds[rot][two_bar] = {25, 4000, 40000};
                    mtr_cmds[rot][goalie] = {-25, 4000, 40000};
                } else if(ball_pos_fast[1] - (-rod_coord[three_bar]) < -3 && abs(ball_vel[0]) < 50){
                    // Snake
                    mtr_cmds[rot][two_bar] = {catch_angle, 4000, 40000};
                    mtr_cmds[rot][goalie] = {catch_angle, 4000, 40000};

                    move_motor(ball_pos_fast[0] - dir*2, 100, 1000, 0, two_bar, 20, 0.5);
                    move_motor(ball_pos_fast[0] + dir*3, 100, 1000, 1, goalie, 20, 0.5);

                } else if(abs(ball_pos_fast[0] - play_height/2) > 6 && abs(ball_pos_fast[0] - play_height/2) < 10){
                    // Pull/push shot
                    mtr_cmds[rot][two_bar] = {catch_angle, 4000, 40000};
                    mtr_cmds[rot][goalie] = {catch_angle, 4000, 40000};

                    static bool far = false;
                    static double t_pos_change = time_ms;

                    const int exp_t_move = 1000;
                    if(rand()%exp_t_move <= dt_ms){
                        far = !far;
                    }

                    int side = ball_pos_fast[0] > play_height/2 ? 1 : -1;
                    move_motor(ball_pos_fast[0] - side*1, 100, 500, 0, two_bar, 20, 0.5);
                    move_motor(ball_pos_fast[0] - side*(far ? 11 : 8), 100, 500, 1, goalie, 20, 0.5);
                } else {
                    mtr_cmds[rot][two_bar] = {catch_angle, 4000, 40000};
                    mtr_cmds[rot][goalie] = {catch_angle, 4000, 40000};

                    move_motor(ball_pos_fast[0] - dir*2, 100, 1000, 0, two_bar, 20, 0.5);
                    move_motor(ball_pos_fast[0] + dir*3, 100, 1000, 1, goalie, 20, 0.5);
                } 
            }
            if(front == five_bar || front == three_bar){
                mtr_cmds[rot][two_bar] = {catch_angle, 4000, 40000};
                mtr_cmds[rot][goalie] = {catch_angle, 4000, 40000};
                if(ball_pos_fast[0] < play_height/2){
                    mtr_cmds[lin][two_bar] = {27.5, 100, 300};
                    mtr_cmds[lin][goalie] = {2.5, 100, 300};
                } else {
                    mtr_cmds[lin][two_bar] = {32, 100, 300};
                    mtr_cmds[lin][goalie] = {15, 100, 300};
                }
            }
            if(front == five_bar){
                if(ball_pos_fast[0] >= play_height/2-goal_width/2 && ball_pos_fast[0] <= play_height/2+goal_width/2){
                    move_motor(ball_pos_fast[0], 100, 500, closest_plr(five_bar, ball_pos_fast[0], cur_pos[lin][five_bar]), five_bar, 20, 0.5);
                } else {
                    static bool lane = true;
                    const int exp_t_lane = 500;
                    if(rand()%exp_t_lane <= dt_ms){
                        lane = !lane;
                    }

                    if(lane){
                        move_motor(ball_pos_fast[0], 100, 1000, closest_plr(five_bar, ball_pos_fast[0], cur_pos[lin][five_bar]), five_bar, 20, 0.5);
                        mtr_cmds[rot][five_bar] = {-25, 4000, 40000};
                    } else{
                        mtr_cmds[lin][five_bar] = {ball_pos_fast[0] < play_height/2 ? 0 : lin_range_cm[five_bar], 100, 1000};
                        mtr_cmds[rot][five_bar] = {25, 4000, 40000};
                    }
                }
            }
            if(front == three_bar){
                mtr_cmds[rot][five_bar] = {-25, 4000, 40000};
                mtr_cmds[rot][three_bar] = {-25, 4000, 40000};
                if(ball_pos_fast[0] < play_height/2){
                    mtr_cmds[lin][five_bar] = {0, 100, 300};
                } else {
                    mtr_cmds[lin][five_bar] = {lin_range_cm[five_bar], 100, 300};
                }
                move_motor(ball_pos_fast[0], 100, 1000, closest_plr(three_bar, ball_pos_fast[0], cur_pos[lin][five_bar]), three_bar, 20, 0.5);
            }
            break;
        }
        /**
         * The human player has the ball, and the robot is stopping them from taking a shot on the goal
         */
        case state_shot_defense:
        {
            pair<side_t, rod_t> closest = closest_rod(ball_pos_fast[1]);
            if(abs(ball_vel[1]) < 10){
                if(closest.first == human){
                    state = state_defense;
                } else {
                    state = state_uncontrolled;
                }
                break;
            }
            for(int r = 0; r < num_rod_t; ++r){
                // If ball is already past this rod, do nothing
                if(ball_pos_fast[1] < rod_coord[r]-rod_gap/2) continue;

                // Predict trajectory
                double target_cm = ball_pos_fast[0];
                // ball_vel[1] is negative so this is positive
                double dt = (rod_coord[r] - ball_pos_fast[1]) / ball_vel[1];
                /* dt -= 30; */
                /* if(r != five_bar) */
                    target_cm += ball_vel[0] * dt;
                /* log << "dt: " << dt << ", ball_vel[0]: " << ball_vel[0] << ", ball_vel[1]: " << ball_vel[1] << ", target_cm: " << target_cm << endl; */

                int plr = closest_plr(r, target_cm, cur_pos[lin][r]);
                if(r == goalie) plr = 1;

                double plr_offset_cm = plr_offset(plr, r);

                mtr_cmds[lin][r] = {
                    .pos = target_cm - plr_offset_cm,
                    .vel = 150,
                    .accel = 1000,
                };
            }
            break;
        }
        /**
         * 
         */
        case state_uncontrolled:
        {
            /**
             * @brief Represents the closest rod to the ball position.
             *
             * The `closest` variable is a pair that consists of a `side_t` value representing the side of the rod and a `rod_t` value representing the rod number.
             * It stores the information about the closest rod to the ball position.
             */
            pair<side_t, rod_t> closest = closest_rod(ball_pos_fast[1]);
            if(closest.first != bot){
                state = state_defense;
                break;
            } else if(!ball_in_motion){
                state = state_controlled_move;
            }
            /**
             * @brief The distance to the closest object.
             *
             * This variable represents the distance to the closest object and is stored in the variable `rod`.
             */
            int rod = closest.second;
            /**
             * @brief The index of the closest player to the ball.
             */
            int plr = closest_plr(rod, ball_pos_fast[0], cur_pos[lin][rod]);
            mtr_cmds[rot][rod] = {35, 5'000, 50'000};

            if(time_ms - mtr_t_last_cmd[lin][rod] > 40){
                mtr_cmds[lin][rod] = {ball_pos_fast[0] - plr_offset(plr, rod), 100, 1000};
            }
            break;

            /**
             * @brief The difference between the y-coordinate of the ball and the rod.
             */
            double dy = ball_pos_fast[1]-rod_coord[rod];
            int dir = dy > 0 ? -1 : 1;
            /* if(abs(dy) > 6){ */
            /*     dir *= -1; */
            /* } */
            /* if(abs(ball_vel[1]) > 5){ */
            /*     dir = ball_vel[1] > 0 ? -1 : 1; */
            /* } */

            /**
             * @brief The angle at which the catch is positioned.
             * 
             * This variable represents the angle (in degrees) at which the catch is positioned.
             * It is used to determine the orientation of the catch in the system.
             */
            double catch_angle = 45;


            if(abs(cur_pos[rot][rod] - dir*catch_angle) < 2){
                status << "1" << endl;
                if(abs(cur_pos[lin][rod] + plr_offset(plr, rod) - ball_pos_fast[0]) > 0.5 && time_ms - mtr_t_last_cmd[lin][rot] > 20){
                    mtr_cmds[lin][rod] = {ball_pos_fast[0] - plr_offset(plr, rod), 100, 1000};
                }
            } else if(abs(ball_pos_fast[0] - cur_pos[lin][rod] - plr_offset(plr, rod)) > ball_rad+foot_width/2+2){
                status << "2" << endl;
                mtr_cmds[rot][rod] = {dir*catch_angle, 10'000, 100'000};
            } else if(abs(ball_pos_fast[0] - mtr_last_cmd[lin][rod].pos - plr_offset(plr, rod)) < ball_rad+foot_width/2+2) {
                status << "3" << endl;
                double target_cm = ball_pos_fast[0] + ball_rad + foot_width/2 + 3;
                if(!can_plr_reach(plr, rod, target_cm, 0)){
                    target_cm = ball_pos_fast[0] - ball_rad - foot_width/2 - 3;
                }
                mtr_cmds[lin][rod] = {target_cm - plr_offset(plr, rod), 100, 1000};
            }

            break;
        }
        // Convenience macros for waiting for rot/lin motions to finish respectively
#define wait_rot if(abs(cur_pos[rot][rod] - mtr_last_cmd[rot][rod].pos) < 1.0 && time_ms - control_task_timer > 50)
#define wait_lin if(abs(cur_pos[lin][rod] - mtr_last_cmd[lin][rod].pos) < 0.1 && time_ms - control_task_timer > 50)
#define wait_time(T) if(time_ms - control_task_timer >= (T))
        /******************************************************************************
         * Five bar passing
         ******************************************************************************/
        case state_controlled_five_bar:
        {
            static double t_wall_open = time_ms;
            static double t_lane_open = time_ms;
            static double t_start = time_ms;
            static double t_human = time_ms;

            pair<side_t, rod_t> closest = closest_rod(ball_pos_fast[1]);
            side_t side = closest.first;
            rod_t rod = closest.second;
            if(side == human && c5b_task != c5b_fast_5 && c5b_task != c5b_threaten_5){
                /* if(time_ms - t_human > 90){ */
                log << "Lost from pass" << endl;
                state = state_defense;
                c5b_task = c5b_init;
                /* } */
            } else t_human = time_ms;

            int plr = closest_plr(rod, ball_pos_slow[0], cur_pos[lin][rod]);
            double plr_offset_cm = plr_offset(plr, rod);

            /**
             * @brief The direction of the ball.
             *
             * The `ball_dir` variable represents the direction of the ball.
             * It is set to 1 if the ball's velocity in the x-axis (ball_vel[0]) is greater than 0,
             * indicating that the ball is moving to the right. Otherwise, it is set to -1, indicating that the ball is moving to the left.
             */
            int ball_dir = ball_vel[0] > 0 ? 1 : -1;

            const double hit_thresh = 4;

            const double left_cm = 2;
            const double right_cm = 12;


            /**
             * @brief Calculates the position of the hit point in centimeters.
             * 
             * The hit point is calculated based on the ball direction and the positions of the left and right sides.
             * If the ball direction is 1, the hit point is calculated as the right side position minus half of the hit threshold.
             * If the ball direction is not 1, the hit point is calculated as the left side position plus half of the hit threshold.
             * 
             * @return The position of the hit point in centimeters.
             */
            double hit_cm = ball_dir == 1 ? right_cm - hit_thresh/2 : left_cm + hit_thresh/2;
            double dt = abs(ball_pos_slow[0] - hit_cm) / ball_vel[0];
            /* double ball_cm = ball_pos_slow[1] + ball_vel[1]*dt; */
            double ball_cm = ball_pos_slow[1];

            /* double ball_deg = (rod_coord[rod] - ball_pos_fast[1]) / plr_height / deg_to_rad - 3; */
            /**
             * @brief The angle in degrees at which the ball is positioned relative to the rod.
             *
             * The `ball_deg` variable represents the angle in degrees at which the ball is positioned
             * relative to the rod. It is calculated using the `atan` function to determine the angle
             * between the vertical distance (`rod_coord[rod] - ball_cm`) and the horizontal distance
             * (`plr_height + plr_levitate - ball_rad/2`). The result is then converted from radians to
             * degrees by dividing by `deg_to_rad` and subtracting 3.
             */
            double ball_deg = atan((rod_coord[rod] - ball_cm) / (plr_height+plr_levitate-ball_rad/2)) / deg_to_rad-3;

            static double pass_cm;
            static int threaten_dir = ball_pos_fast[0] <= play_height/2;
            static int threaten_shake_dir = threaten_dir;

            switch(c5b_task){
            case c5b_init:
            {
                t_start = time_ms;

                if(rod == two_bar){
                    c5b_task = c5b_threaten_1;
                } else if(rod == five_bar){
                    c5b_task = c5b_fast_1;
                } else {
                    log << "Attempting to pass from wrong rod" << endl;
                    c5b_task = c5b_init;
                    state = state_controlled_move;
                }
                break;
            }
            case c5b_fast_1:
                mtr_cmds[lin][rod] = {ball_pos_fast[0]+ball_rad+foot_width/2-plr_offset(1, rod)+0.3, 50, 500};
                mtr_cmds[lin][three_bar] = {0.05, 150, 1500};
                mtr_cmds[rot][three_bar] = {-40, 5000, 50000};
                mtr_fns.push([](){
                    nodes[rot][three_bar].get().Limits.TrqGlobal = 100;
                });
                c5b_task = c5b_fast_2;
                control_task_timer = time_ms;
                break;
            case c5b_fast_2:
                wait_lin{
                    if(is_blocked(rod, ball_rad + 0.1, rod_pos, 1, three_bar)){
                        t_wall_open = time_ms;
                    }
                    if(is_blocked(rod, 12, rod_pos, 1, three_bar)){
                        t_lane_open = time_ms;
                    }
                    double t_thresh = 3*(1-(time_ms - t_start)/10'000)*1000 + (rand()%1000-200);
                    if((time_ms - t_wall_open) > t_thresh){
                    /* if(true){ */
                        mtr_cmds[lin][rod] = {0, 200, 2000};
                        c5b_task = c5b_fast_wall_3;
                        control_task_timer = time_ms;
                    } else if((time_ms - t_lane_open) > t_thresh){
                    /* } else if(true){ */
                        mtr_cmds[lin][rod] = {0.5, 50, 500};
                        mtr_cmds[lin][three_bar] = {10-plr_offset(0, rod), 120, 1500};
                        mtr_cmds[rot][three_bar] = {-47, 5000, 50000};
                        c5b_task = c5b_fast_lane_3;
                        control_task_timer = time_ms;
                    }
                }
                break;
            case c5b_fast_wall_3:
                wait_time(70){
                    mtr_cmds[rot][rod] = {60, 10000, 100000};
                    c5b_task = c5b_fast_wall_4;
                    control_task_timer = time_ms;
                }
                break;
            case c5b_fast_wall_4:
                wait_time(50){
                    mtr_cmds[rot][rod] = {-90, 4000, 40000};
                    mtr_cmds[rot][three_bar] = {-50, 50, 1000};
                    c5b_task = c5b_fast_5;
                    control_task_timer = time_ms;
                }
                break;
            case c5b_fast_lane_3:
                wait_time(90){
                    mtr_cmds[rot][rod] = {60, 10000, 100000};
                    mtr_cmds[lin][rod] = {13.5-plr_offset(0, rod), 150, 1500};
                    c5b_task = c5b_fast_lane_4;
                    control_task_timer = time_ms;
                }
                break;
            case c5b_fast_lane_4:
                wait_time(70){
                    mtr_cmds[rot][rod] = {-90, 4000, 40000};
                    c5b_task = c5b_fast_5;
                    control_task_timer = time_ms;
                }
                break;
            case c5b_fast_5:
                // Only adjust if lane pass
                if(time_ms - mtr_t_last_cmd[lin][three_bar] > 5 && mtr_last_cmd[lin][rod].pos > 0.5){
                    mtr_cmds[lin][three_bar] = {
                        .pos = clamp(ball_pos_fast[0] - plr_offset(0, three_bar), 0.0, lin_range_cm[rod]),
                        .vel = 300,
                        .accel = 3000,
                    };
                }
                wait_time(300){
                    mtr_cmds[rot][three_bar] = {-60, 1000, 10000};
                    c5b_task = c5b_fast_6;
                    control_task_timer = time_ms;
                    mtr_fns.push([](){
                        nodes[rot][three_bar].get().Limits.TrqGlobal = 100;
                    });
                }
                break;
            case c5b_fast_6:
                c5b_task = c5b_init;
                state = state_controlled_move;
                break;
            case c5b_threaten_1:
            {
                double start_cm = 19;
                mtr_cmds[lin][rod] = {
                    .pos = threaten_dir == 1 ? start_cm : lin_range_cm[rod] - start_cm,
                    .vel = 30,
                    .accel = 300,
                };
                mtr_cmds[rot][rod] = {
                    .pos = ball_deg*1.2,
                    .vel = 4000,
                    .accel = 40'000,
                };
                c5b_task = c5b_threaten_2;
                control_task_timer = time_ms;
                break;
            }
            case c5b_threaten_2:
                wait_time(200){
                    mtr_cmds[rot][rod] = {
                        .pos = 60,
                        .vel = 4000,
                        .accel = 40'000,
                    };
                    mtr_cmds[rot][rod-1] = {
                        .pos = -49,
                        .vel = 4000,
                        .accel = 40'000,
                    };
                    c5b_task = c5b_threaten_3;
                    control_task_timer = time_ms;
                }
                break;
            case c5b_threaten_3:
            {
                /* wait_lin{ */
                /*     threaten_shake_dir = cur_pos[lin][rod] > (left_cm + right_cm)/2 ? -1 : 1; */
                /*     log << threaten_dir << endl; */
                /*     mtr_cmds[lin][rod] = { */
                /*         .pos = threaten_shake_dir == 1 ? lin_range_cm[rod]-4 : 4, */
                /*         .vel = 10, */
                /*         .accel = 300, */
                /*     }; */
                /* } */
                double ball_cm = ball_pos_fast[0] + 1*threaten_dir;
                int plr_passer = threaten_dir == 1 ? 1 : 0;
                int plr_rcv = closest_plr(rod-1, ball_cm, cur_pos[lin][rod-1]);
                double dt = 0.075;
                /* double over_offset = ball_vel[0]*dt; */
                double over_offset = max(0.0, dt * 30 * (1-(time_ms-control_task_timer)/2000));
                if(time_ms - mtr_t_last_cmd[lin][rod] > 20){
                    mtr_cmds[lin][rod] = {ball_pos_fast[0]+over_offset*threaten_dir - plr_offset(plr_passer, rod), 100, 1000};
                }
                if(time_ms - mtr_t_last_cmd[lin][rod-1] > 40){
                    mtr_cmds[lin][rod-1] = {ball_pos_fast[0]+(over_offset+0.5)*threaten_dir - plr_offset(plr_rcv, rod-1), 100, 1000};
                }
                wait_time(300){

                    /* int plr_five_bar = closest_plr(five_bar, ball_cm, cur_pos[lin][rod-1]); */
                    if(!is_blocked(rod, ball_pos_fast[0], rod_pos, 0.2) && abs(cur_pos[lin][rod] + plr_offset(plr_passer, rod) - ball_pos_fast[0]) < 0.5){
                        break;
                        mtr_cmds[rot][rod] = {
                            .pos = -120,
                            .vel = 20'000,
                            .accel = 200'000,
                        };
                        mtr_cmds[rot][rod-1] = {
                            .pos = -90,
                            .vel = 10'000,
                            .accel = 100'000,
                        };
                        c5b_task = c5b_idle;
                        log << "Pass shot" << endl;
                    }else if(!is_blocked(rod, ball_cm, rod_pos, 2, rod-1) && abs(cur_pos[lin][rod] + plr_offset(plr_passer, rod) - ball_cm) < 4){
                        /* mtr_cmds[lin][rod] = { */
                        /*     .pos = ball_cm - plr_offset(plr_passer, five_bar), */
                        /*     .vel = 300, */
                        /*     .accel = 3000, */
                        /* }; */
                        /* mtr_cmds[lin][rod-1] = { */
                        /*     .pos = ball_cm + 1*threaten_dir - plr_offset(plr_rcv, rod-1), */
                        /*     .vel = 200, */
                        /*     .accel = 2000, */
                        /* }; */
                        mtr_cmds[rot][rod] = {
                            .pos = -60,
                            .vel = 7000,
                            .accel = 70'000,
                        };
                        c5b_task = c5b_threaten_5;
                        control_task_timer = time_ms;
                        pass_cm = ball_cm;
                        log << "Pass" << endl;
                    }else if(time_ms - mtr_t_last_cmd[lin][rod] > 50){
                        /* mtr_cmds[lin][rod] = { */
                        /*     .pos = ball_cm - plr_offset_cm, */
                        /*     .vel = 50, */
                        /*     .accel = 500, */
                        /* }; */
                        /* mtr_cmds[lin][rod-1] = { */
                        /*     .pos = ball_cm - plr_offset(plr_rod-1, rod-1), */
                        /*     .vel = 50, */
                        /*     .accel = 500, */
                        /* }; */
                    }
                }
                break;
            }
            case c5b_threaten_4:
                if(threaten_dir == 1 ? (ball_pos_fast[0] >= pass_cm-1) : (ball_pos_fast[0] <= pass_cm + 1)){
                    if(is_blocked(five_bar, pass_cm, rod_pos, 1, rod-1)){
                        log << "Abort pass!" << endl;
                        c5b_task = c5b_threaten_3;
                        break;
                    }
                    if(!is_blocked(rod, pass_cm, rod_pos, 0.5)){
                        mtr_cmds[rot][rod] = {
                            .pos = -120,
                            .vel = 20'000,
                            .accel = 200'000,
                        };
                        mtr_cmds[rot][rod-1] = {
                            .pos = -90,
                            .vel = 10'000,
                            .accel = 100'000,
                        };
                        c5b_task = c5b_idle;
                        log << "Slow shot!" << endl;
                    } else {
                        mtr_cmds[rot][rod] = {
                            .pos = -60,
                            .vel = 7000,
                            .accel = 70'000,
                        };
                        c5b_task = c5b_threaten_5;
                        control_task_timer = time_ms;
                        log << "Pass!" << endl;
                    }
                }
                break;
            case c5b_threaten_5:
            {
                if(time_ms - mtr_t_last_cmd[lin][five_bar] > 5){
                    int plr_rcv = closest_plr(five_bar, ball_pos_fast[0], cur_pos[lin][rod-1]);
                    /* double target_cm = kin_ball_dist(ball_pos_fast, ball_vel, rod_coord[rod-1]); */
                    mtr_cmds[lin][five_bar] = {
                        .pos = ball_pos_fast[0] - plr_offset(plr_rcv, five_bar),
                        .vel = 300,
                        .accel = 3000,
                    };
                }
                wait_time(300){
                    log << "Releasing pass" << endl;
                    mtr_cmds[rot][five_bar] = {
                        .pos = -90,
                        .vel = 4000,
                        .accel = 40'000,
                    };
                    c5b_task = c5b_init;
                    control_task_timer = time_ms;
                    state = state_controlled_move;
                }
                break;
            }
            case c5b_idle:
                break;
            }
            break;
        }
        /******************************************************************************
         * General move command
         ******************************************************************************/
        case state_controlled_move:
        {
            /**
             * @brief Represents the closest rod to the ball position.
             *
             * The `closest` variable is a pair consisting of a `side_t` and a `rod_t` value.
             * It represents the closest rod to the current ball position.
             */
            pair<side_t, rod_t> closest = closest_rod(ball_pos_fast[1]);
            /**
             * @brief Which side the closest rod is on.
             */
            side_t side = closest.first;
            /**
             * @brief Which rod is closest.
             */
            rod_t rod = closest.second;
            /* status << ball_pos_fast[1]-rod_coord[rod] << endl; */
            /* break; */
            /**
             * @brief The position of the ball.
             * 
             * This vector stores the position of the ball. It is initially set to the values
             * of the `ball_pos_fast` vector.
             */
            vector<double> ball_pos = ball_pos_fast;
            ball_pos[1] -= rod_offsets[rod];

            /**
             * @brief Represents the index of the closest player to the ball.
             *
             * This variable stores the index of the player that is closest to the ball.
             * It is used to determine which player should be selected to interact with the ball.
             *
             * @see closest_plr()
             */
            int cls_plr = closest_plr(rod, ball_pos_slow[0], cur_pos[lin][rod]);
            double cls_plr_offset_cm = plr_offset(cls_plr, rod);

            double ball_deg = atan((rod_coord[rod] - ball_pos[1]) / (plr_height+plr_levitate-ball_rad/2)) / deg_to_rad-5;

            double pin_setup = cmove_target_cm[1] >= 5;

            // Side of ball to go to
            static int target_side = -1;
            // What direction to go on top of ball, -1=forward, 1=backward
            static int target_top = -1;
            static cmove_t next_task = cmove_adjust_1;
            static int plr = 0;
            static bool end_side = false; // whether we've ended on the desired side
            static int cur_rod = goalie;
            
            // These are so we don't actually leave the motors in lower torque
            static vector<function<void(void)>> leave_fns;

            // Useful for debugging angles
            /* mtr_cmds[rot][rod] = {ball_deg, 500, 5000}; */
            /* break; */

            if(side != bot){
                state = state_defense;
                cmove_task = cmove_init;
                for(auto fn : leave_fns) fn();
                log << "Went to human" << endl;
                break;
            } else if(rod != cur_rod){
                cmove_task = cmove_init;
            }

            switch(cmove_task){
            case cmove_init:
            {
                cmove_task = cmove_decide_1;
                control_task_timer = time_ms;
                end_side = false;
                cur_rod = rod;
                if(rod == three_bar){
                    cmove_target_cm = {play_height/2,5.5};
                    cmove_target_tol = {2,2};
                    cmove_next_state = state_snake;
                    cmove_end_side = 0;
                } else if(rod == five_bar){
                    cmove_target_cm = {lin_range_cm[five_bar] + plr_offset(0, five_bar) + foot_width/2 + ball_rad,0};
                    cmove_target_tol = {2,4};
                    cmove_next_state = state_controlled_five_bar;
                    cmove_end_side = -1;
                } else if(rod == two_bar){
                    cmove_target_cm = {15,0};
                    cmove_target_tol = {2,4};
                    cmove_next_state = state_controlled_five_bar;
                    cmove_end_side = 1;
                } else if(rod == goalie){
                    cmove_target_cm = {10,0};
                    cmove_target_tol = {2,4};
                    cmove_next_state = state_controlled_five_bar;
                    cmove_end_side = 0;
                }
                break;
            }
            case cmove_decide_1:
                wait_time(300){
                    /* if(ball_in_motion == false){ */
                    if(!ball_in_motion){
                        cmove_task = cmove_decide_2;
                    }
                }
                wait_time(2500){
                    log << "Exited decision because of timeout" << endl;
                    cmove_task = cmove_decide_2;
                }
                break;
            case cmove_decide_2:
            {
                // Decide next action based on current state
                if(rod != goalie && abs(ball_pos[0] - cmove_target_cm[0]) < cmove_target_tol[0] && abs(ball_pos[1] - rod_coord[rod] - cmove_target_cm[1]) < cmove_target_tol[1]){
                    // Pin shots need to be setup up with pin even if they happen to get in position, since the player has to be on top
                    if(pin_setup && !(cur_pos[rot][rod] <= -40 && cur_pos[rot][rod] >= -65)){
                        log << "Doing pin " << cur_pos[rot][rod] << endl;
                        cmove_task = cmove_pin_1;
                        break;
                    }
                    if(cmove_end_side != 0 && !end_side){
                        next_task = cmove_decide_1;
                        cmove_task = cmove_side_1;
                        target_side = cmove_end_side;
                        end_side = true;
                        break;
                    }
                    log << "Move reached target, going to " << cmove_next_state << endl;
                    state = cmove_next_state;
                    cmove_task = cmove_init;
                    for(auto fn : leave_fns) fn();
                    break;
                }

                end_side = false;
                target_side = ball_pos[0] <= cmove_target_cm[0] ? 1 : -1;
                /**
                 * @brief The distance between the edge of the bumper and the player, plus 1.
                 */
                double edge_dist = bumper_width + plr_width+1;
                if(ball_pos[0] <= edge_dist || ball_pos_fast[0] >= play_height-edge_dist){
                    next_task = cmove_bounce_1;
                    target_side = ball_pos[0] < play_height/2 ? -1 : 1;
                    cmove_task = cmove_side_1;
                } else if(abs(ball_pos[1] - rod_coord[rod]) > 4
                        && (abs(ball_pos[1]-cmove_target_cm[0])>cmove_target_tol[0] || abs(ball_pos_fast[1]-rod_coord[rod]-cmove_target_cm[1])>cmove_target_tol[1])){
                    /* cmove_task = cmove_unstuck_1; */
                    cmove_task = cmove_top_1;
                    target_top = ball_pos[1] > rod_coord[rod] ? -1 : 1;
                    next_task = cmove_unstuck_1;
                } else if(rod == goalie){
                    target_top = 1;
                    cmove_task = cmove_top_1;
                    next_task = cmove_goalie_1;
                } else if(pin_setup && abs(cmove_target_cm[0] - ball_pos[0]) < cmove_target_tol[0]){ // Set up pin shot
                    cmove_task = cmove_pin_1;
                } else if(abs(ball_pos[0]-cmove_target_cm[0]) > (rod == five_bar ? 8 : 15)
                        && (rod == five_bar || abs(ball_pos[1]-rod_coord[rod]-cmove_target_cm[1]) < 2)){
                    /* if(abs(ball_pos[1]-rod_coord[rod]) > 1){ */
                    if(false){
                        target_top = ball_pos[1]-rod_coord[rod] > cmove_target_cm[1] ? -1 : 1;
                        cmove_task = cmove_top_1;
                        next_task = cmove_angle_1;
                    } else {
                        next_task = cmove_tap_1;
                        cmove_task = cmove_side_1;
                    }
                } else {
                    next_task = cmove_adjust_1;
                    cmove_task = cmove_side_1;
                }

                if(cmove_task != cmove_side_1) end_side = false;
                break;
            }
            case cmove_side_1:
            {
                /**
                 * @brief The rotation direction of the player.
                 * 
                 * The `rot_dir` variable determines the rotation direction of the player. It is calculated based on the current position of the player, the offset, ball position, ball radius, player width, player height, and rod coordinates. If the condition `abs(cur_pos[lin][rod] + cls_plr_offset_cm - ball_pos[0]) < ball_rad + plr_width/2 && cur_pos[rot][rod] * deg_to_rad * plr_height >= rod_coord[rod] - ball_pos[1]` is true, `rot_dir` is set to 1, otherwise it is set to -1.
                 */
                int rot_dir = abs(cur_pos[lin][rod] + cls_plr_offset_cm - ball_pos[0]) < ball_rad + plr_width/2
                    && cur_pos[rot][rod] * deg_to_rad * plr_height >= rod_coord[rod] - ball_pos[1] ? 1 : -1;
                
                mtr_cmds[rot][rod] = {
                    .pos = rot_dir == 1 ? 60.0 : -60,
                    .vel = 500,
                    .accel = 5000,
                };
                cmove_task = cmove_side_2;
                break;
            }
            case cmove_side_2:
                wait_rot{
                    /**
                     * @brief The distance in centimeters from the ball to the target.
                     * 
                     * The `target_cm` variable represents the distance in centimeters from the ball to the target.
                     * It is calculated by subtracting the sum of the ball radius, half of the foot width, and 0.5 from the x-coordinate of the ball position,
                     * and then multiplying it by the target side.
                     */
                    double target_cm = ball_pos[0]-(ball_rad+foot_width/2+0.5)*target_side;
                    plr = closest_plr(rod, target_cm, lin_range_cm[rod]/2);
                    if(rod == three_bar && target_cm <= 15.5) plr = 0; // very hacky, but whatever
                    if(!can_plr_reach(plr, rod, target_cm, 1)){
                        log << "Giving up from side" << endl;
                        cmove_task = cmove_give_up_1;
                    } else {
                        mtr_cmds[lin][rod] = {
                            .pos = target_cm - plr_offset(plr, rod),
                            .vel = 50,
                            .accel = 500,
                        };
                        cmove_task = cmove_side_3;
                        control_task_timer = time_ms;
                    }
                }
                break;
            case cmove_side_3:
                wait_lin{
                    mtr_cmds[rot][rod] = {
                        .pos = ball_deg*1.3,
                        .vel = 100,
                        .accel = 1000,
                    };
                    cmove_task = cmove_side_4;
                    control_task_timer = time_ms;
                }
                break;
            case cmove_side_4:
                wait_rot{
                    cmove_task = next_task;
                    control_task_timer = time_ms;
                }
                break;
            case cmove_top_1:
            {
                plr = closest_plr(rod, ball_pos[0], lin_range_cm[rod]/2);
                /**
                 * @brief The position of the player on the rod.
                 *
                 * This variable represents the position of the player on the rod. It is calculated by adding the player offset to the current position of the rod.
                 *
                 * @see plr_offset
                 * @see cur_pos
                 *
                 * @param plr The player object.
                 * @param rod The rod object.
                 *
                 * @return The position of the player on the rod.
                 */
                double plr_pos = plr_offset(plr, rod) + cur_pos[lin][rod];
                /**
                 * @brief The distance between the center of the foot and the ball.
                 * 
                 * This variable represents the distance between the center of the foot and the ball. 
                 * It is calculated by adding half of the foot width, the radius of the ball, and 0.5.
                 */
                double dx = foot_width/2 + ball_rad + 0.5;
                /**
                 * @brief The direction of movement.
                 * 
                 * This variable determines the direction of movement based on the relative positions of the player and the ball.
                 * If the player's position is greater than the x-coordinate of the ball's position, the direction is set to 1 (right).
                 * Otherwise, the direction is set to -1 (left).
                 */
                int dir = plr_pos > ball_pos[0] ? 1 : -1;
                // Prevents timeouts
                if(!can_plr_reach(plr, rod, ball_pos[0] + dir*dx)){
                    dir *= -1;
                    mtr_cmds[rot][rod] = {cur_pos[rot][rod] > ball_deg ? 60.0 : -60, 10000, 100000};
                }

                mtr_cmds[lin][rod] = {ball_pos[0] - plr_offset(plr,rod) + dir*dx, 50, 500};

                cmove_task = cmove_top_2;
                control_task_timer = time_ms;
                break;
            }
            case cmove_top_2:
                wait_lin{
                    mtr_cmds[rot][rod] = {target_top == 1 ? 60.0 : -60, 5000, 50000};
                    cmove_task = cmove_top_3;
                    control_task_timer = time_ms;
                }
                break;
            case cmove_top_3:
                wait_rot{
                    mtr_cmds[lin][rod] = {clamp(ball_pos[0] - plr_offset(plr,rod), 0.0, lin_range_cm[rod]), 50, 500};
                    cmove_task = cmove_top_4;
                    control_task_timer = time_ms;
                }
                break;
            case cmove_top_4:
                wait_lin{
                    cmove_task = next_task;
                    control_task_timer = time_ms;
                }
                break;
            case cmove_angle_1: // Bring y position to target
            {
                mtr_cmds[rot][rod] = {target_top == 1 ? 25.0 : -25, 500, 5000};
                cmove_task = cmove_angle_2;
                control_task_timer = time_ms;
                break;
            }
            case cmove_angle_2:
                wait_rot{
                    cmove_task = cmove_decide_1;
                    control_task_timer = time_ms;
                }
                break;
            case cmove_tap_1: // tap
            {
                double pos = cur_pos[lin][rod] + 3*target_side;
                if(pos < 0 || pos >= lin_range_cm[rod]){
                    log << "Giving up from tap" << endl;
                    cmove_task = cmove_give_up_1;
                    break;
                }
                mtr_cmds[lin][rod] = {
                    .pos = pos,
                    .vel = 60,
                    .accel = 600,
                };
                cmove_task = cmove_tap_2;
                control_task_timer = time_ms;
                break;
            }
            case cmove_tap_2: // get into position
            {
                wait_time(100){
                    double target_cm = cmove_target_cm[0] - target_side*2;
                    int new_plr = closest_plr(rod, target_cm, lin_range_cm[rod]/2);
                    /* if(abs(plr - new_plr) > 1){ */
                    mtr_cmds[rot][rod] = {
                        .pos = 60,
                        .vel = 1000,
                        .accel = 10000,
                    };
                    /* } */
                    if(rod == two_bar) new_plr = target_side == 1 ? 1 : 0;
                    mtr_cmds[lin][rod] = {
                        .pos = target_cm - plr_offset(new_plr, rod),
                        .vel = 50,
                        .accel = 500,
                    };
                    cmove_task = cmove_tap_3;
                    control_task_timer = time_ms;
                }
                break;
            }
            case cmove_tap_3: // start catch
            {
                if(abs(ball_pos[0] - cmove_target_cm[0]) < 9){
                    double target_cm = cmove_target_cm[0] + target_side*(ball_rad+foot_width/2);
                    int plr = closest_plr(rod, target_cm, lin_range_cm[rod]/2);
                    if(rod == two_bar) plr = target_side ? 1 : 0;

                    mtr_cmds[rot][rod] = {
                        .pos = ball_deg*1.3,
                        .vel = 10000,
                        .accel = 100000,
                    };
                    mtr_cmds[lin][rod] = {
                        .pos = clamp(target_cm - plr_offset(plr, rod), 0.0, lin_range_cm[rod]),
                        .vel = abs(ball_vel[0]),
                        .accel = 300,
                    };
                    cmove_task = cmove_tap_4;
                }
                wait_time(2000){
                    cmove_task = cmove_decide_1;
                    control_task_timer = time_ms;
                }
                break;
            }
            case cmove_tap_4: //wait for catch
                if(time_ms - mtr_t_last_cmd[lin][rod] > 10){
                    mtr_cmds[rot][rod] = {
                        .pos = ball_deg * 1.3,
                        .vel = 3000,
                        .accel = 30000,
                    };
                }
                status << cur_pos[lin][rod] << ", " << mtr_last_cmd[lin][rod].pos << endl;
                wait_lin{
                    cmove_task = cmove_decide_1;
                    control_task_timer = time_ms;
                }
                break;
            case cmove_adjust_1:
            {
                int side = ball_pos[0] >= cmove_target_cm[0] ? -1 : 1;
                double pos = clamp(cmove_target_cm[0] - side*(ball_rad+foot_width/2) - plr_offset(plr, rod), 0.0, lin_range_cm[rod]);
                double tol = 1;
                if((side == 1 && lin_range_cm[rod]-cur_pos[lin][rod] < tol) || (side == -1 && cur_pos[lin][rod] < tol)){
                    log << "Giving up from adjust" << endl;
                    cmove_task = cmove_give_up_1;
                } else {
                    mtr_cmds[lin][rod] = {
                        .pos = pos, // plr is still populated from side
                        .vel = 2,
                        .accel = 10,
                    };
                    cmove_task = cmove_adjust_2;
                }
                break;
            }
            case cmove_adjust_2:
            {
                if(time_ms - mtr_t_last_cmd[lin][rod] > 20){
                    mtr_cmds[rot][rod] = {
                        .pos = ball_deg * 1.4,
                        .vel = 500,
                        .accel = 5000,
                    };
                }
                wait_lin{
                    cmove_task = cmove_decide_1;
                    control_task_timer = time_ms;
                }
                break;
            }
            case cmove_unstuck_1:
            {
                mtr_fns.push([rod](){
                    nodes[rot][rod].get().Limits.TrqGlobal = 10;
                });
                leave_fns.push_back([&mtr_fns, rod](){
                    mtr_fns.push([rod](){
                        nodes[rot][rod].get().Limits.TrqGlobal = 100;
                    });
                });
                /* double dy = 2*(abs(ball_pos[1]-rod_coord[rod])-5); */
                mtr_cmds[rot][rod] = {
                    .pos = (cur_pos[rot][rod] > 0 ? 50.0 : -48),
                    .vel = 75,
                    .accel = 7500,
                };
                cmove_task = cmove_unstuck_2;
                control_task_timer = time_ms;
                break;
            }
            case cmove_unstuck_2:
                wait_time(500){
                    double dx = (ball_pos[0] >= cmove_target_cm[0] ? -1 : 1)*6;
                    // Always goes direction with more space
                    /* double dx = (ball_pos[0] >= plr_offset(plr, rod)+lin_range_cm[rod]/2 ? -1 : 1)*10; */
                    double target_cm = 0;
                    if(can_plr_reach(plr, rod, ball_pos[0]+dx, 0.2)){
                        target_cm = ball_pos[0]+dx;
                    } else {
                        target_cm = ball_pos[0]-dx;
                    }
                    mtr_cmds[lin][rod] = {
                        .pos = clamp(target_cm - plr_offset(plr, rod), 0.0, lin_range_cm[rod]),
                        .vel = 100,
                        .accel = 1000,
                    };
                    cmove_task = cmove_unstuck_3;
                    control_task_timer = time_ms;
                }
                break;
            case cmove_unstuck_3:
                wait_time(1000){
                    log << "Resetting torque" << endl;
                    mtr_fns.push([rod](){
                        nodes[rot][rod].get().Limits.TrqGlobal = 100;
                    });
                    cmove_task = cmove_decide_1;
                    control_task_timer = time_ms;
                }
                break;
            case cmove_bounce_1:
            {
                // Distance from edge of ball to edge of table
                /* double dist = min(ball_pos[0], abs(ball_pos_fast[0] - play_height)) - ball_rad; */
                mtr_cmds[lin][rod] = {
                    .pos = ball_pos[0] + (target_side == -1 ? 1 : -1)*(ball_rad+foot_width/2+4) - plr_offset(plr, rod),
                    .vel = 50,
                    .accel = 1000,
                };
                cmove_task = cmove_bounce_2;
                control_task_timer = time_ms;
            }

            case cmove_bounce_2:
            {
                wait_lin{
                    mtr_cmds[lin][rod] = {
                        .pos = (target_side == -1 ? 0.2 : play_height - 0.2),
                        .vel = 75,
                        .accel = 1500,
                    };
                    cmove_task = cmove_bounce_3;
                    control_task_timer = time_ms;
                }
                break;
            }
            case cmove_bounce_3:
                wait_time(100){
                    mtr_cmds[lin][rod] = {
                        .pos = lin_range_cm[rod]/2,
                        .vel = 100,
                        .accel = 1500,
                    };
                    mtr_cmds[rot][rod] = {
                        .pos = 60,
                        .vel = 5000,
                        .accel = 50000,
                    };
                    cmove_task = cmove_bounce_4;
                    control_task_timer = time_ms;
                }
                break;
            case cmove_bounce_4:
                wait_time(1000){
                    cmove_task = cmove_decide_1;
                    control_task_timer = time_ms;
                }
                break;
            case cmove_give_up_1:
            {
                if(ball_pos[1] > rod_coord[rod]-3){
                    mtr_cmds[rot][rod] = {
                        .pos = 90,
                        .vel = 5000,
                        .accel = 50000,
                    };
                } else {
                    mtr_cmds[rot][rod] = {
                        .pos = -90,
                        .vel = 5000,
                        .accel = 50000,
                    };
                    if(rod != goalie){
                        mtr_cmds[lin][rod+1] = {
                            .pos = ball_pos[0] - plr_offset(closest_plr(rod+1, ball_pos_fast[0], cur_pos[lin][rod+1]), rod+1),
                            .vel = 200,
                            .accel = 2000,
                        };
                        mtr_cmds[rot][rod+1] = {
                            .pos = -30,
                            .vel = 1000,
                            .accel = 10000,
                        };
                    }
                }
                log << "Giving up move" << endl;
                cmove_task = cmove_give_up_2;
                break;
            }
            case cmove_give_up_2:
                wait_rot{
                    int plr = closest_plr(rod, ball_pos[0], play_height/2);
                    mtr_cmds[lin][rod] = {
                        .pos = clamp(ball_pos[0]-plr_offset(plr, rod)+2*(rand()%2-0.5), 0.0, lin_range_cm[rod]),
                        .vel = 150,
                        .accel = 1500,
                    };
                    cmove_task = cmove_give_up_3;
                }
                break;
            case cmove_give_up_3:
                wait_lin{
                    if(ball_pos[1] > rod_coord[rod]-3){
                        mtr_cmds[rot][rod] = {
                            .pos = -90,
                            .vel = 5000,
                            .accel = 50000,
                        };
                    } else {
                        mtr_cmds[rot][rod] = {
                            .pos = 60,
                            .vel = 500,
                            .accel = 5000,
                        };
                    }
                    cmove_task = cmove_idle;
                    /* state = state_uncontrolled; */
                }
                break;
            case cmove_pin_1:
                mtr_cmds[rot][rod] = {
                    .pos = 60,
                    .vel = 5000,
                    .accel = 50000,
                };
                plr = closest_plr(rod, ball_pos[0], lin_range_cm[rod]/2);
                cmove_task = cmove_pin_2;
                break;
            case cmove_pin_2:
                wait_rot{
                    mtr_cmds[lin][rod] = {
                        .pos = ball_pos[0] - plr_offset(plr, rod),
                        .vel = 50,
                        .accel = 500,
                    };
                    cmove_task = cmove_pin_3;
                }
                break;
            case cmove_pin_3:
                wait_lin{
                    mtr_cmds[rot][rod] = {
                        .pos = -24,
                        .vel = 20,
                        .accel = 500,
                    };
                    cmove_task = cmove_pin_4;
                }
                break;
            case cmove_pin_4:
                wait_rot{
                    // Kinda assumes ball is in middle of plr range
                    mtr_cmds[lin][rod] = {
                        .pos = ball_pos[0]+ball_rad+foot_width - plr_offset(plr, rod),
                        .vel = 50,
                        .accel = 500,
                    };
                    cmove_task = cmove_pin_5;
                }
                break;
            case cmove_pin_5:
                wait_lin{
                    mtr_cmds[rot][rod] = {
                        .pos = -60,
                        .vel = 5000,
                        .accel = 50000,
                    };
                    cmove_task = cmove_pin_6;
                }
                break;
            case cmove_pin_6:
                wait_rot{
                    mtr_cmds[lin][rod] = {
                        .pos = ball_pos[0] - plr_offset(plr, rod),
                        .vel = 50,
                        .accel = 500,
                    };
                    cmove_task = cmove_init;
                    cmove_task = cmove_decide_1;
                    control_task_timer = time_ms;
                }
                break;
            case cmove_goalie_1:
            {
                mtr_cmds[rot][two_bar] = {-30, 500, 5000};
                int two_bar_plr = closest_plr(two_bar, ball_pos[0], cur_pos[lin][two_bar]);
                mtr_cmds[lin][two_bar] = {ball_pos[0]-plr_offset(two_bar_plr, two_bar), 75, 750};
                cmove_task = cmove_goalie_2;
                control_task_timer = time_ms;
                break;
            }
            case cmove_goalie_2:
                wait_time(500){
                    mtr_cmds[rot][rod] = {-60, 300, 3000};
                    cmove_task = cmove_goalie_3;
                    control_task_timer = time_ms;
                }
                break;
            case cmove_goalie_3:
                wait_time(300){
                    cmove_task = cmove_init;
                }
                break;
            case cmove_idle:
            default:
                break;
            }
            break;
        }
        /******************************************************************************
         * Snake
         ******************************************************************************/
        /**
         * Move rods back and forth to prevent shots.
         */
        case state_snake: {
            static int dir = 1;
            static double t_start = time_ms;
            static double t_shot = time_ms;
            static double t_last_turn = time_ms;
            static double t_left_open = time_ms;
            static double t_right_open = time_ms;

            const int rod = three_bar;

            pair<side_t, rod_t> closest = closest_rod(ball_pos_fast[1]);
            if(closest.first == human && time_ms - t_shot > 300){
                state = state_shot_defense;
                log << "Lost ball from snake " << time_ms - t_shot << endl;
                mtr_fns.push([](){
                    nodes[rot][rod].get().Limits.TrqGlobal = 100;
                });
                csnake_task = csnake_init;
                break;
            }

            double plr_offset_cm = plr_offset(1, rod);
            /* if(ball_pos_fast[0] == 0) ball_pos_fast[0] = play_height/2; */

            switch(csnake_task){
            case csnake_init:
                mtr_cmds[lin][rod] = {
                    .pos = ball_pos_fast[0] - plr_offset_cm,
                    .vel = 25,
                    .accel = 250,
                };
                t_start = time_ms;
                t_last_turn = time_ms;
                t_left_open = time_ms;
                t_right_open = time_ms;
                mtr_fns.push([](){
                    nodes[rot][rod].get().Limits.TrqGlobal = 5;
                });
                csnake_task = csnake_plan;
                break;
            case csnake_plan:

                /* if(time_ms > t_shoot && cur_pos[lin][rod] - plr_offset_cm < 0.5){ */
                if(!is_blocked(rod, ball_pos_fast[0], rod_pos, 0.1)){
                /* if(false){ */
                    mtr_fns.push([](){
                        nodes[rot][rod].get().Limits.TrqGlobal = 100;
                    });
                    mtr_cmds[rot][rod] = {
                        .pos = -450,
                        .vel = 20000,
                        .accel = 200000,
                    };
                    t_shot = time_ms;
                    log << "Snake straight shot" << endl;
                    csnake_task = csnake_init;
                    state = state_unknown;
                    break;
                }

                if(abs(cur_pos[lin][rod]+plr_offset_cm - ball_pos_fast[0]) < 0.5 && time_ms - t_start > 400){
                /* if(false){ */
                    const double move_cm = 5.5;
                    bool left_open = !is_blocked(rod, ball_pos_fast[0]-move_cm, rod_pos, 0.1);
                    bool right_open = !is_blocked(rod, ball_pos_fast[0]+move_cm, rod_pos, 0.1);
                    if(!left_open) t_left_open = time_ms;
                    if(!right_open) t_right_open = time_ms;
                    double t_thresh = (1-(time_ms - t_start)/15000)*1500;
                    if((left_open && time_ms - t_left_open > t_thresh) || (right_open && time_ms - t_right_open > t_thresh)){
                        mtr_cmds[lin][rod] = {
                            .pos = ball_pos_fast[0] + move_cm*((left_open && time_ms - t_left_open > t_thresh)?-1:1) - plr_offset_cm,
                            .vel = 200,
                            .accel = 3000,
                        };
                        mtr_fns.push([](){
                            nodes[rot][rod].get().Limits.TrqGlobal = 100;
                        });
                        log << ball_pos_fast[0] + move_cm * (left_open ? -1 : 1) - plr_offset_cm << endl;
                        t_shot = time_ms;
                        csnake_task = csnake_shoot;
                        break;
                    }
                }

                mtr_cmds[rot][rod] = {
                    .pos = -52.1,
                    .vel = 100,
                    .accel = 1000,
                };

                /* if(abs(cur_pos[lin][rod] - mtr_last_cmd[lin][rod].pos) < 0.1 && time_ms - t_last_turn > 20){ */
                /*     dir *= -1; */
                /*     t_last_turn = time_ms; */
                /*     mtr_cmds[lin][rod] = { */
                /*         .pos = ball_pos_fast[0] - plr_offset_cm + dir * 3, */
                /*         .vel = 15, */
                /*         .accel = 150, */
                /*     }; */
                /* } */
                break;
            case csnake_shoot:
                /* if((abs(ball_pos_fast[0] - (mtr_last_cmd[lin][rod].pos + plr_offset_cm)) < 0.5 && time_ms - t_shot > 20) */
                /*         || time_ms - t_shot > 1000){ */
                if(time_ms - t_shot > 60){
                    mtr_cmds[rot][rod] = {
                        .pos = -450,
                        .vel = 20000,
                        .accel = 200000,
                    };
                    log << "Snake moving shot" << endl;
                }
                if(time_ms - t_shot > 200){
                    csnake_task = csnake_init;
                    state = state_defense;
                }
                break;
            }
            break;
        }
        case state_test:
            // For misc testing
            mtr_cmds[lin][three_bar] = {
                /* .pos = ball_pos[0] - plr_offset(1, three_bar), */
                .pos = play_height/2 - plr_offset(1, three_bar),
                .vel = 25,
                .accel = 250,
            };
            break;
        /**
         * print status message
         */
        case state_unknown:
            status << is_blocked(three_bar, ball_pos_fast[0]-6, rod_pos, 0.3) << ", " << is_blocked(three_bar, ball_pos_fast[0], rod_pos, 0.1) << ", " << is_blocked(three_bar, ball_pos_fast[0]+6, rod_pos, 0.3) << endl;

            break;
        default:
            break;
        }

        print_status(status.str(), log.str(), true);

        this_thread::sleep_for(chrono::microseconds(500));
    }


    cout << "Got terminate command, quitting..." << endl;
    close_all();
    terminate();

    return 0;
}
