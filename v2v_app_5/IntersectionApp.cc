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
std::map<std::pair<std::string, std::string>, std::pair<int, std::string>> IntersectionApp::messageExchangeCount;
std::priority_queue<CarMessage*, std::vector<CarMessage*>,
        IntersectionApp::ComparePriority> IntersectionApp::msgQueue;
std::set<std::string> IntersectionApp::interestingIds = { "car1" };

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

        for (const auto& pair : rsuPositions) { // Assuming rsuPositions is a map of RSU ids to their positions
            std::string rsuId = pair.first;
            Coord rsuPos = pair.second;

            double distance = calculateDistance(carPos, rsuPos);
            if (distance <= 30) { // Assuming RSU_COMM_RANGE is the communication range of an RSU
                CarMessage* cm = new CarMessage();
                if (dataOnSch) {
                    scheduleAt(computeAsynchronousSendingTime(1, type_SCH), cm);
                } else {
                    simtime_t delayTimeCars = par("delayTimeCars");
                    sendDelayedDown(cm, delayTimeCars);
                }

                if (interestingIds.count(carId) > 0) {
                    cm->setPriority(1);  // Set the priority of the message
                    cm->setRsuId(rsuId.c_str());
                    cm->setCarId(carId.c_str());

                    populateCMBrodcast(cm);

                    // Log this message exchange
                    // When a message is exchanged, do:
                    std::string carId = std::string(cm->getCarId());
                    std::string rsuId = std::string(cm->getRsuId());
                    auto& record = messageExchangeCount[ { carId, rsuId }];
                    record.first += 1; // increment message count
                    // Append the current simTime to the string.
                    if (!record.second.empty()) {
                        record.second += ",";
                    }
                    record.second += std::to_string((int) simTime().dbl());
                } else {
                    cm->setPriority(5);  // Set the priority of the message
                    cm->setRsuId(rsuId.c_str());
                    cm->setCarId(carId.c_str());
                    populateCMBrodcast(cm);
                }

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
        msgQueue.push(cm);
        if (!msgQueue.empty()) {
            CarMessage* cm = msgQueue.top(); // Get the highest-priority message
            msgQueue.pop();  // Remove the message from the queue
//            // Log this message exchange
//            // When a message is exchanged, do:
//            std::string carId = std::string(cm->getCarId());
//            std::string rsuId = std::string(cm->getRsuId());
//            auto& record = messageExchangeCount[ { carId, rsuId }];
//            record.first += 1; // increment message count
//            // Append the current simTime to the string.
//            if (!record.second.empty()) {
//                record.second += ",";
//            }
//            record.second += std::to_string((int) simTime().dbl());
            onCarMessage(cm);
        }
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

    // Log the message queue
    outputFile << "\nMessage Queue:\n";
    while (!msgQueue.empty()) {
        CarMessage* cm = msgQueue.top();
        std::string carId = std::string(cm->getCarId());
        std::string rsuId = std::string(cm->getRsuId());
        int priority = cm->getPriority();
        outputFile << "CarId: " << carId << ", RSUId: " << rsuId
                << ", Priority: " << priority << "\n";
        msgQueue.pop();
    }
    outputFile.close();
}
