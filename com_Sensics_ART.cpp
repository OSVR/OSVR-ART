/** @file
    @brief Implementation

    @date 2016

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2016 Sensics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>
#include <osvr/PluginKit/ButtonInterfaceC.h>
#include <osvr/PluginKit/AnalogInterfaceC.h>
#include <osvr/Util/Log.h>
#include <json/value.h>
#include <json/reader.h>
#include <quat/quat.h>

// Library/third-party includes
#include "DTrackSDK.hpp"

// Generated JSON header file
#include "com_Sensics_ART_json.h"

// Standard includes
#include <iostream>

namespace {

static const auto DRIVER_NAME = "OSVR_ART";
static const auto PREFIX = "[OSVR-ART]: ";
/// offset the ID for the tracker sensor by 10
static const auto FLYSTICK_ID_OFFSET = 10;
inline const uint16_t getDataPortDefault() { return 5000; }
inline const std::string getServerHostDefault() { return "192.168.0.1"; }
typedef std::shared_ptr<DTrackSDK> DTrackPtr;

class ARTDevice {
  public:
    ARTDevice(OSVR_PluginRegContext ctx, DTrackPtr dTrack) : m_dTrack(dTrack) {
        /// Create the initialization options
        OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

        osvrDeviceTrackerConfigure(opts, &m_tracker);
        osvrDeviceButtonConfigure(opts, &m_button,
                                  DTRACKSDK_FLYSTICK_MAX_BUTTON);
        osvrDeviceAnalogConfigure(opts, &m_analog,
                                  DTRACKSDK_FLYSTICK_MAX_JOYSTICK);

        /// Create the sync device token with the options
        m_dev.initSync(ctx, "ART", opts);

        /// Send JSON descriptor
        m_dev.sendJsonDescriptor(com_Sensics_ART_json);

        /// Register update callback
        m_dev.registerUpdateCallback(this);
    }

    ~ARTDevice() {
        m_dTrack->stopMeasurement();
        while (m_dTrack->getMessage()) {
            std::cout << "ATC Message: " << m_dTrack->getMessageStatus()
                      << "\" \"" << m_dTrack->getMessageMsg() << "\""
                      << std::endl;
        }
    }

    OSVR_ReturnCode update() {

        if (!m_dTrack->receive()) {
            if (m_dTrack->getLastDataError() == DTrackSDK::ERR_TIMEOUT) {
                std::cout << PREFIX
                          << "--- timeout while waiting for tracking data"
                          << std::endl;
                return OSVR_RETURN_FAILURE;
            }

            if (m_dTrack->getLastDataError() == DTrackSDK::ERR_NET) {
                std::cout << PREFIX << "--- error while receiving tracking data"
                          << std::endl;
                return OSVR_RETURN_FAILURE;
            }

            if (m_dTrack->getLastDataError() == DTrackSDK::ERR_PARSE) {
                std::cout << PREFIX << "--- error while parsing tracking data"
                          << std::endl;
                return OSVR_RETURN_FAILURE;
            }
        }

        // bodies:
        DTrack_Body_Type_d body;
        for (int i = 0; i < m_dTrack->getNumBody(); i++) {
            body = *m_dTrack->getBody(i);

            if (body.quality > 0) {

                OSVR_TimeValue timestamp;
                osvrTimeValueGetNow(&timestamp);
                OSVR_PoseState state;
                convertPose(&state, body.loc, body.rot);

                osvrDeviceTrackerSendPoseTimestamped(m_dev, m_tracker, &state,
                                                     i, &timestamp);
            }
        }

        return OSVR_RETURN_SUCCESS;
    }

  private:
      
    /* @brief converts position and orientation to OSVR report types*/
    void convertPose(OSVR_PoseState *state, double *loc, double *rot) {

        /// convert to unit meter
        state->translation.data[0] = (-1.0) * (loc[0] / 1000.0);
        state->translation.data[1] = (-1.0) * (loc[1] / 1000.0);
        state->translation.data[2] = (-1.0) * (loc[2] / 1000.0);

        q_type quat;
        q_matrix_type destMatrix;

        /// rot is a column-wise matrix
        destMatrix[0][0] = rot[0]; // mat[1][1]
        destMatrix[0][1] = rot[1]; // mat[1][2]
        destMatrix[0][2] = rot[2]; // mat[1][3]
        destMatrix[0][3] = 0.0;    // mat[1][4]

        destMatrix[1][0] = rot[3];
        destMatrix[1][1] = rot[4];
        destMatrix[1][2] = rot[5];
        destMatrix[1][3] = 0.0;

        destMatrix[2][0] = rot[6];
        destMatrix[2][1] = rot[7];
        destMatrix[2][2] = rot[8];
        destMatrix[2][3] = 0.0;

        destMatrix[3][0] = 0.0;
        destMatrix[3][1] = 0.0;
        destMatrix[3][2] = 0.0;
        destMatrix[3][3] = 1.0;

        q_from_row_matrix(quat, destMatrix);

        osvrQuatSetW(&state->rotation, quat[Q_W]);
        osvrQuatSetX(&state->rotation, quat[Q_X]);
        osvrQuatSetY(&state->rotation, quat[Q_Y]);
        osvrQuatSetZ(&state->rotation, quat[Q_Z]);
    }

    osvr::pluginkit::DeviceToken m_dev;
    OSVR_TrackerDeviceInterface m_tracker;
    OSVR_ButtonDeviceInterface m_button;
    OSVR_AnalogDeviceInterface m_analog;
    std::shared_ptr<DTrackSDK> m_dTrack;
};

class HardwareDetection {
  public:
    HardwareDetection() : m_found(false) {}

    OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx, const char *params) {

        // if the device has been discovered we don't need to go thru hardware
        // detection again
        if (m_found) {
            return OSVR_RETURN_SUCCESS;
        }

        std::cout << PREFIX << "Got a hardware detection request" << std::endl;

        Json::Value root;
        {
            Json::Reader reader;
            if (!reader.parse(params, root)) {
                std::cerr << PREFIX
                          << "Could not parse JSON for OSVR-ART plugin"
                          << std::endl;
                return OSVR_RETURN_FAILURE;
            }
        }

        uint16_t dataPortParam;
        std::string serverHostParam;
        if (!root.isMember("dataPort")) {
            std::cerr << PREFIX
                      << "Could not find data port parameter. Verify that "
                         "port is specified in the config file. "
                         "Using default data port value 5000"
                      << std::endl;
            dataPortParam = getDataPortDefault();
        } else {
            dataPortParam = root["dataPort"].asInt();
            if (!root["dataPort"].isInt()) {
                std::cerr << PREFIX << "Could not get data port value. Using "
                                       "default port value 5000"
                          << std::endl;
                dataPortParam = getDataPortDefault();
            }
        }

        if (!root.isMember("serverHost")) {
            std::cerr << PREFIX
                      << "Could not find server host parameter. Verify that "
                         "serverHost is specified in the config file. Using "
                         "default server host value 192.168.0.1"
                      << std::endl;
            serverHostParam = getServerHostDefault();
        } else {
            serverHostParam = root["serverHost"].asString();
            if (root["serverHost"].empty()) {
                std::cerr << PREFIX << "Could not get serverHost value. Using "
                                       "default server host value 192.168.0.1"
                          << std::endl;
                serverHostParam = getServerHostDefault();
            }
        }

        std::cout << PREFIX << "Establishing connection with ART device with "
                               "provided parameters"
                  << std::endl
                  << "Server host: " << serverHostParam
                  << "; Data port: " << dataPortParam << std::endl;

        m_dTrack = std::shared_ptr<DTrackSDK>(
            new DTrackSDK(serverHostParam.c_str(), dataPortParam));

        if (!m_dTrack->isLocalDataPortValid() ||
            !m_dTrack->isCommandInterfaceValid()) {
            std::cout << PREFIX << "Could not initialize DTrackSDK"
                      << std::endl;
            return OSVR_RETURN_FAILURE;
        }

        std::cout << PREFIX << "Connected to ATC " << serverHostParam
                  << " server port "
                  << ", data port " << dataPortParam << std::endl;

        bool measRunning = m_dTrack->startMeasurement();
        if (!measRunning) {
            std::cout << PREFIX << "Start measurement failed! " << std::endl;
            while (m_dTrack->getMessage()) {
                std::cout << "ATC Message: " << m_dTrack->getMessageStatus()
                          << "\" \"" << m_dTrack->getMessageMsg() << "\""
                          << std::endl;
                // ignore this error and continue
                if (m_dTrack->getMessageMsg() ==
                    "measurement already running") {
                    measRunning = true;
                }
            }
        }

        if (!measRunning) {
            return OSVR_RETURN_FAILURE;
        }

        m_found = true;
        osvr::pluginkit::registerObjectForDeletion(
            ctx, new ARTDevice(ctx, m_dTrack));

        return OSVR_RETURN_SUCCESS;
    }

  private:
    bool m_found;
    DTrackPtr m_dTrack;
};

} // namespace

OSVR_PLUGIN(com_Sensics_ART) {

    osvr::pluginkit::PluginContext context(ctx);

    context.registerDriverInstantiationCallback(DRIVER_NAME,
                                                HardwareDetection());

    return OSVR_RETURN_SUCCESS;
}