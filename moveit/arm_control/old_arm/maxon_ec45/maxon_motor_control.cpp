#include "Definitions.h"
#include <iostream>
#include <unistd.h>
#include <list>
#include <csignal>
#include <thread>
#include <atomic>
#include <eigen3/Eigen/Dense>


using namespace std;
using Eigen::MatrixXd;
using Eigen::Vector3d;

typedef void* HANDLE;
typedef int BOOL;

enum EAppMode
{
	AM_UNKNOWN,
	AM_DEMO,
	AM_INTERFACE_LIST,
	AM_PROTOCOL_LIST,
	AM_VERSION_INFO
};

struct Motor {
    unsigned short nodeId;
    string deviceName;
    string protocolStackName;
    string interfaceName;
    string portName;
    int baudrate;
    void* handle;
};

struct SubMotor {
    Motor* parent_motor;
    unsigned short subNodeId;
    string subDeviceName;
    string subProtocolStackName;
    void* subHandle;
};
Motor m1;
SubMotor m2;
SubMotor m3;

std::atomic<bool> running = true;

void listenForStop() {
    char ch;
    while (running) {
        cin >> ch;
        if (ch == 'q') {
            cout << "Stopping motor..." << endl;
            VCS_SetDisableState(m1.handle, m1.nodeId, nullptr);
            VCS_SetDisableState(m2.subHandle, m2.subNodeId, nullptr);
            VCS_SetDisableState(m3.subHandle, m3.subNodeId, nullptr);
            VCS_CloseAllDevices(nullptr);
            running = false;
            break;
        }
    }
}

uint32_t g_baudrate = 1000000; // Default baudrate

float G1, G2, G3; // gear ratios

int main(int argc, char** argv)
{
    G1 = 1/3.0f;
    G2 = 0.6f;
    G3 = 0.75f;
    Vector3d endVelocities(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]));
    cout << "End effector velocities: " << endVelocities.transpose() << endl;
    MatrixXd manipMatrix(3,3);
    manipMatrix << -G1, 0, 0,
                    -G1*G2*G3, -G2*G3/2, -G2*G3/2,
                    0, G2/2, -G2/2;
    cout << "Manipulation matrix: " << endl << manipMatrix << endl;
    Vector3d jointVelocities = manipMatrix.inverse() * endVelocities;
    cout << "Joint velocities: " << jointVelocities.transpose() << endl;
    Vector3d check = manipMatrix * jointVelocities;
    cout << "Check: " << check.transpose() << endl;

    void* motorHandles[3] = {nullptr, nullptr, nullptr};
    //motor setup
    m1 = {1, "EPOS4", "MAXON SERIAL V2", "USB", "USB0", 1000000, 0};
    m2 = {&m1, 2, "EPOS4", "CANopen", 0};
    m3 = {&m1, 3, "EPOS4", "CANopen", 0};

    uint32_t errorCode;
    m1.handle = VCS_OpenDevice(
                (char*)m1.deviceName.c_str(), 
                (char*)m1.protocolStackName.c_str(), 
                (char*)m1.interfaceName.c_str(), 
                (char*)m1.portName.c_str(), 
                &errorCode);

    if (m1.handle == nullptr) {
        char errorInfo[256];
        VCS_GetErrorInfo(errorCode, errorInfo, 256);
        cout << "Failed to open device " << m1.deviceName << ", error code " << errorCode << ": " << errorInfo << endl;
        VCS_CloseAllDevices(nullptr);
        return -1;
    }
    motorHandles[0] = m1.handle;

    m2.subHandle = VCS_OpenSubDevice(
                m1.handle, 
                (char*)m2.subDeviceName.c_str(), 
                (char*)m2.subProtocolStackName.c_str(), 
                &errorCode);

        
    if (m2.subHandle == nullptr) {
        cout << "Failed to open device " << m2.subDeviceName << ", error code: " << errorCode << endl;
        VCS_CloseAllDevices(nullptr);
        return -1;
    }

    motorHandles[1] = m2.subHandle;

    m3.subHandle = VCS_OpenSubDevice(
                m1.handle, 
                (char*)m3.subDeviceName.c_str(), 
                (char*)m3.subProtocolStackName.c_str(), 
                &errorCode);

    if (m3.subHandle == nullptr) {
        cout << "Failed to open device " << m3.subDeviceName << ", error code: " << errorCode << endl;
        VCS_CloseAllDevices(nullptr);
        return -1;
    }

    motorHandles[2] = m3.subHandle;
    
    std::thread newThread(listenForStop);
    //successfully opened device
    cout << "Device opened successfully." << endl;
    unsigned int lBaudrate = 0;
    unsigned int lTimeout = 0;

    bool lResult = 0;
    VCS_GetProtocolStackSettings(m1.handle, &lBaudrate, &lTimeout, nullptr);
    VCS_SetProtocolStackSettings(m1.handle, g_baudrate, lTimeout, nullptr);
    VCS_GetProtocolStackSettings(m1.handle, &lBaudrate, &lTimeout, nullptr);
    
    for (int i = 0; i < 3; i++) {
        VCS_SetEnableState(motorHandles[i], i+1, nullptr);
        VCS_ActivateProfileVelocityMode(motorHandles[i], i+1, nullptr);
    }
    
    int velocities[3] = {static_cast<int>(jointVelocities[0]), 
                        static_cast<int>(jointVelocities[1]), 
                        static_cast<int>(jointVelocities[2])};
    while (running) {
        for (int i = 0; i < 3; i++) {
            int res = VCS_MoveWithVelocity(motorHandles[i], i+1, velocities[i], nullptr);
        }
    }

    if (running) {
        VCS_SetDisableState(m1.handle, m1.nodeId, nullptr);
        VCS_SetDisableState(m2.subHandle, m2.subNodeId, nullptr);
        VCS_SetDisableState(m3.subHandle, m3.subNodeId, nullptr);
        VCS_CloseAllDevices(nullptr);
        running = false;
    }
    newThread.join();
    return 0;
}