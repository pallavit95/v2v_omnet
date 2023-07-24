#include <stdlib.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include "veins/base/utils/Coord.h"
#include "IntersectionApp.h"

Define_Module(IntersectionApp);

std::map<std::pair<std::string, std::string>, double> IntersectionApp::carDistances;
std::map<std::string, Coord> IntersectionApp::carPositions;
std::map<std::string, Coord> IntersectionApp::rsuPositions;
//std::map<std::pair<std::string, std::string>, std::pair<int, simtime_t>> IntersectionApp::messageExchangeCount;
std::map<std::pair<std::string, std::string>, std::pair<int, std::string>> IntersectionApp::messageExchangeCount;

void IntersectionApp::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        lastSent = simTime();
        //By default, you are not allowed to go through intersection
        canGo = false;
        //Figuring out if you are an RSU
        std::string path = getFullPath();
        if (path.find("rsu") != std::string::npos) {
            isRSU = true;
        } else {
            isRSU = false;
        }
        //Starts channels
        if (dataOnSch) {
            startService(Channels::SCH2, 42, "Traffic Information Service");
        }
    }
}

void IntersectionApp::finish() {
    exportMessageExchangeCounts();
}

IntersectionApp::~IntersectionApp() {
    for (auto it = RSUData.begin(); it != RSUData.end(); ++it) {
        delete (*it);
    }
    RSUData.clear();
}
void IntersectionApp::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);

    //Cars send messages every 1 second
//    if (simTime() - lastSent >= 1 && !isRSU) {
    if (!isRSU) {
        std::string carId = mobility->getExternalId();
        Coord carPos = mobility->getPositionAt(simTime());

        carPositions[carId] = carPos;  // Update the position of the current car

//        for (const auto& pair : rsuPositions) { // Assuming rsuPositions is a map of RSU ids to their positions
//            std::string rsuId = pair.first;
//            Coord rsuPos = pair.second;
//
//            double distance = calculateDistance(carPos, rsuPos);
//            if (distance <= 30) { // Assuming RSU_COMM_RANGE is the communication range of an RSU
//                CarMessage* cm = new CarMessage();
//                populateCMBrodcast(cm);
//
//                if (dataOnSch) {
//                    scheduleAt(computeAsynchronousSendingTime(1, type_SCH), cm);
//                } else {
//                    simtime_t delayTimeCars = par("delayTimeCars");
//                    sendDelayedDown(cm, delayTimeCars);
//                }
//
//                // Log this message exchange
//                // When a message is exchanged, do:
//                auto& record = messageExchangeCount[ { carId, rsuId }];
//                record.first += 1; // increment message count
//
//                // Append the current simTime to the string.
//                if (!record.second.empty()) {
//                    record.second += ",";
//                }
//                record.second += std::to_string((int) simTime().dbl());
//            }
//        }

//         Loop over the positions of all cars and calculate the distances
        for (const auto& pair : carPositions) {
            std::string otherCarId = pair.first;
            Coord otherCarPos = pair.second;

            if (otherCarId == carId) {
                continue; // Skip self
            }

            double distance = calculateDistance(carPos, otherCarPos);
            carDistances[ { carId, otherCarId }] = distance;

            if (distance <= 100) {
                CarMessage* cm = new CarMessage();
                populateCMBrodcast(cm);

                //If there is currently data on the channel, the car cannot send the message right then
                if (dataOnSch) {
                    //schedule message to self to send later
                    scheduleAt(computeAsynchronousSendingTime(1, type_SCH), cm);
                }
                //Send on CCH, because channel switching is disabled
                else {
                    //Sends with specified delay in message
                    simtime_t delayTimeCars = par("delayTimeCars");
                    sendDelayedDown(cm, delayTimeCars);
                }

                // Log this message exchange
                // When a message is exchanged, do:
                auto& record = messageExchangeCount[ { carId, otherCarId }];
                record.first += 1; // increment message count

                // Append the current simTime to the string.
                if (!record.second.empty()) {
                    record.second += ",";
                }
                record.second += std::to_string((int) simTime().dbl());

            }

        }

    } else {
        for (int i = 0; i < 13; i++) {
            std::string rsuPath = "rsu[" + std::to_string(i) + "].mobility";
            BaseMobility* mobilityModule = check_and_cast<BaseMobility*>(
                    getSimulation()->getModuleByPath(rsuPath.c_str()));
            if (mobilityModule) {
                Coord pos = mobilityModule->getCurrentPosition();
                rsuPositions[rsuPath] = pos;
            } else {
                // Handle the case where the module was not found.
                EV_ERROR << "Could not find module at path: " << rsuPath
                                << "\n";
            }
        }
    }

}

double IntersectionApp::calculateDistance(const Coord& pos1,
        const Coord& pos2) {
    return pos1.distance(pos2);
}

void IntersectionApp::handleSelfMsg(cMessage* msg) {
    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    if (CarMessage* cm = dynamic_cast<CarMessage*>(wsm)) {
        sendDown(cm->dup());
        delete (cm);
    } else {
        BaseWaveApplLayer::handleSelfMsg(msg);
    }
}

void IntersectionApp::handleLowerMsg(cMessage* msg) {
    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    if (CarMessage* cm = dynamic_cast<CarMessage*>(wsm)) {
        onCarMessage(cm);
    }
    delete (msg);
}

void IntersectionApp::populateCMBrodcast(CarMessage* cm) {
    cm->setHelloMsg("Hello World");
    cm->setSenderAddress(myId);
    cm->setRecipientAddress(-1);  // -1 indicates broadcast
    cm->setWsmData(cm->getHelloMsg());
    cm->setWsmLength(strlen(cm->getHelloMsg()));
}

void IntersectionApp::onCarMessage(CarMessage* cm) {
    EV << "Received CarMessage from car " << cm->getSenderAddress() << ": "
              << cm->getHelloMsg() << "\n";
}

void IntersectionApp::exportMessageExchangeCounts() {
    std::ofstream outputFile("message_exchange.csv");
    outputFile << "CarId,RSUId,MessageCount,Timestamp\n";

    for (const auto& pair : messageExchangeCount) {
        std::string carId = pair.first.first;
        std::string rsuId = pair.first.second;
        int count = pair.second.first;
        std::string time = pair.second.second;

        outputFile << carId << "," << rsuId << "," << count << "," << time
                << "\n";
    }

    outputFile.close();
}
