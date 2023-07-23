#include <stdlib.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include "veins/base/utils/Coord.h"
#include "IntersectionApp.h"

Define_Module(IntersectionApp);

/*
 * Initialization and completion functions
 */
//added new code
std::map<std::string, cModule*> IntersectionApp::carModules = { };
std::map<std::pair<std::string, std::string>, double> IntersectionApp::carDistances;
std::map<std::string, Coord> IntersectionApp::carPositions;

void IntersectionApp::initialize(int stage) {
    /*
     * This function initializes all variables. It sets the lastSent time to the
     * beginning of the simulation time. It also figures out if we are an RSU or a car
     * and sets isRSU accordingly. All cars are default not allowed to go through
     * the intersection so canGo is false. This function starts the service channel.
     * It also uses all the initialization from the base wave application layer.
     */
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

IntersectionApp::~IntersectionApp() {
    /*
     * This function goes through RSUData and deletes all
     * remaining IntersectMessage*. It then clears the vector.
     */
    for (auto it = RSUData.begin(); it != RSUData.end(); ++it) {
        delete (*it);
    }
    RSUData.clear();
}

/*
 * RSU functions
 */

void IntersectionApp::onISM(IntersectMessage* ism) {
    /*
     * This function receives a message from a car, called an IntersectMessage.
     * It will cause the RSU to display a green circle around it. If the car has not
     * yet passed the intersection, it will add the car's message/update it in RSUData.
     * If the car has passed, it will delete the car's message from RSUData. It will
     * then calculate the vehicles allowed to go through the intersection based on the
     * data it has. It uses regular intersection rules to determine who is allowed to go.
     * It will send an RSUMessage with a list of the vehicleId's that are allowed to go
     * to all cars.
     */

    /* At the beginning of the simulation, lots of empty vehicle ID's get sent. We want
     * to make sure it is a vehicle we defined so we check the ID is not empty.
     * We also only want RSU to evaluate ISM msgs.
     */
    if (isRSU && ((ism->getVehicleId())[0] != '\0')) {
        //When the RSU receives a message, it will display a green circle around it
        findHost()->getDisplayString().updateWith("r=16,green");

        //If car has not yet passed intersection update info for RSU, o/w delete info
        if (!(ism->getPassed())) {
            addData(ism);
        } else {
            removeData(ism);
        }

        /* Every second, RSU calculates which cars are allowed to go and sends message
         * We wait a few seconds into the simulation because SUMO and OMNeT are not perfectly
         * in sync at the very beginning and so even if in SUMO, we defined two cars to
         * start at the same simulation time in SUMO, they will not send a message at the same
         * time. We want the behavior of SUMO and OMNeT to always match.
         */
        //
        if ((simTime() - lastSentRSU >= 1) && simTime() > 6.0) {
            //Calculates vehicles allowed to go
            std::list<const char*> allowedList = calculateAllowedVehicles();
            //Creates and populates RSU message
            RSUMessage* rsm = new RSUMessage();
            rsm->setAllowedVehiclesArraySize(allowedList.size());
            int i = 0;
            for (auto&& allowed : allowedList) {
                rsm->setAllowedVehicles(i, allowed);
                ++i;
            }
            //Sends RSU message with specified delay
            sendDelayedDown(rsm, delayTimeRSU);
            lastSentRSU = simTime();
        }
    }
}

bool greater(IntersectMessage* ism1, IntersectMessage* ism2) {
    /*
     * This function returns true if ism2 was sent later than ism1
     * and false otherwise.
     */
    return (ism1->getTimeSent() < ism2->getTimeSent());
}
void IntersectionApp::addData(IntersectMessage* ism) {
    /*
     * The RSU will make a copy of the IntersectMessage as the original will get
     * deleted. If there is already a message in the list corresponding to the
     * vehicle ID, it will update all the fields in the message except for time
     * sent which remains as the original time sent. If there is no message, it will
     * create a new entry in the list for the new message.
     */

    //Copies intersect message
    IntersectMessage *ism2 = new IntersectMessage();
    ism2 = ism->dup();

    //Loops through data, checking if any of them match the message's vehicle ID
    std::string vehicleId = ism2->getVehicleId();
    auto it = RSUData.begin();
    while (it != RSUData.end()) {
        std::string storedVehicleId = (*it)->getVehicleId();
        //Delete original message, update time sent in ism2 to be original time sent
        if (storedVehicleId == vehicleId) {
            simtime_t timeSent = (*it)->getTimeSent();
            delete (*it);
            it = RSUData.erase(it);
            //Time sent remains the first time the msg was sent
            ism2->setTimeSent(timeSent);
        } else {
            ++it;
        }
    }
    //Push back ism
    RSUData.push_back(ism2);
    //Order RSUData by when the car sent the msg
    std::sort(RSUData.begin(), RSUData.end(), greater);
}

void IntersectionApp::removeData(IntersectMessage* ism) {
    /*
     * This function will remove data from the RSUData vector based on the
     * matching vehicleID in the ism. If it cannot find the ism with the
     * vehicle ID corresponding to the ism in the vector, it will do nothing.
     */
    std::string vehicleId = ism->getVehicleId();
    auto it = RSUData.begin();
    while (it != RSUData.end()) {
        if ((*it)->getVehicleId() == vehicleId) {
            delete (*it);
            it = RSUData.erase(it);
        } else {
            ++it;
        }
    }
}


std::list<const char*> IntersectionApp::getVehicleIds(
        std::vector<IntersectMessage*> vehicles) {
    /*
     * This function loops through a vector of IntersectMessage*'s and returns a list
     * of vehicle IDs corresponding to the vehicle ID's in the messages.
     * If the vector is empty it will return an empty list.
     */
    std::list<const char*> vehicleIds;
    for (auto&& msg : vehicles) {
        const char* vehicleId = msg->getVehicleId();
        vehicleIds.push_back(vehicleId);
    }
    return vehicleIds;
}

std::list<const char*> IntersectionApp::calculateAllowedVehicles() {
    /*
     * This function calculates which vehicles are allowed to go through the
     * intersection according to the data the RSU has at that moment. It returns
     * a list of vehicle ID's of the vehicles that are allowed to go. If no one
     * wants to go through the intersection, it will return an empty list. The way
     * it figures out who is allowed to go through the intersection is based on
     * regular intersection rules. It will first get all the vehicles who sent their messages
     * at the earliest time out of any of the messages. Then it will figure out
     * which vehicles are allowed to go at that time step based on straight before
     * turning, right turns over left turns, and right before left as its priorities.
     */
    std::list<const char*> allowedVehicles;
    //Someone has requested to go through intersection
    if (!RSUData.empty()) {
        //Get all vehicles who sent message at same time
        auto it = RSUData.begin();
        std::vector<IntersectMessage*> vehicleMsgs;
        simtime_t earliest = (RSUData.front())->getTimeSent();
        while (it != RSUData.end() && ((*it)->getTimeSent() - earliest) < 0.01) {
            vehicleMsgs.push_back((*it));
            it++;
        }
        //Calculate out of the vehicles who sent a message at the same time, who gets priority
        allowedVehicles = priorityCars(vehicleMsgs);
        return allowedVehicles;
    }
    //No one wants to go, return empty list
    else {
        return allowedVehicles;
    }
}

/*
 * Car functions
 */

void IntersectionApp::onRSM(RSUMessage *rsm) {
    /*
     * This function handles what to do when a car receives a message from
     * the RSU. The car will loop through all the vehicles that are allowed
     * to go through the intersection according to the RSU. If they are are
     * allowed to go, they will set their canGo boolean variable to True.
     * Otherwise, nothing will happen, as the variable is by default False.
     * The car will also display a blue circle around it when it receives a message
     * from the RSU.
     */
    if (!isRSU) {
        findHost()->getDisplayString().updateWith("r=16,blue");
        /* If the vehicle's ID matches one of the allowed to go vehicles,
         * it sets canGo to true
         */
        std::string vehicleId = mobility->getExternalId().c_str();
        for (int i = 0; i < rsm->getAllowedVehiclesArraySize(); ++i) {
            if (rsm->getAllowedVehicles(i) == vehicleId) {
                canGo = true;
            }
        }
    }
}

void IntersectionApp::handlePositionUpdate(cObject* obj) {
    /*
     * This function gets called whenever a car moves. If a car is allowed to
     * go through the intersection, it will speed up and go. Otherwise, if it
     * getting close to the intersection, it will slow down and eventually stop.
     * The cars also create, populate, and send messages to the RSU in this function.
     * These messages indicate where the car wants to go, when it sent the message,
     * whether or not it has passed the junction, and other necessary fields.
     */
    BaseWaveApplLayer::handlePositionUpdate(obj);

    //Cars send messages every 1 second
//    if (simTime() - lastSent >= 1 && !isRSU) {
    if (!isRSU) {
        std::string carId = mobility->getExternalId();
        Coord carPos = mobility->getPositionAt(simTime());

        carPositions[carId] = carPos;  // Update the position of the current car

        // Loop over the positions of all cars and calculate the distances
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
            }

        }
    }

}

double IntersectionApp::calculateDistance(const Coord& pos1,
        const Coord& pos2) {
    return pos1.distance(pos2);
}

/*
 * General functions
 */

void IntersectionApp::handleSelfMsg(cMessage* msg) {
    /*
     * This a function inherited from the baseWaveApplLayer that handles a self message.
     * We simply do the same thing that layer of the application does in the handleSelfMsg
     * function, just with our new kinds of messages. If it is an IntersectMessage or
     * an RSUMessage, we will send a copy of the message down to the network layer
     * and then delete the message. Otherwise, we let the BaseWaveApplLayer handle
     * the message.
     */
    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
//It is an IntersectMessage from car
    if (IntersectMessage* ism = dynamic_cast<IntersectMessage*>(wsm)) {
        sendDown(ism->dup());
        delete (ism);
    }
//It is an RSUMessage from RSU
    else if (RSUMessage* rsm = dynamic_cast<RSUMessage*>(wsm)) {
        sendDown(rsm->dup());
        delete (rsm);
    }
//It is an RSUMessage from RSU
    else if (CarMessage* cm = dynamic_cast<CarMessage*>(wsm)) {
        sendDown(cm->dup());
        delete (cm);
    }
//Otherwise
    else {
        BaseWaveApplLayer::handleSelfMsg(msg);
    }
}

void IntersectionApp::handleLowerMsg(cMessage* msg) {
    /*
     * This is a function inherited from the baseWaveApplLayer that handles a lower
     * message to the network. We simply do the same thing that layer of the application
     * does in the handleLowerMsg function, just with our new kinds of messages. If
     * the network received an IntersectMessage, we call our onISM function. If it received
     * an RSUMessage, we call our on RSM function. We delete the message at the end of the
     * function.
     */
    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    if (IntersectMessage* ism = dynamic_cast<IntersectMessage*>(wsm)) {
        onISM(ism);
    } else if (CarMessage* cm = dynamic_cast<CarMessage*>(wsm)) {
        onCarMessage(cm);
    } else if (RSUMessage* rsm = dynamic_cast<RSUMessage*>(wsm)) {
        onRSM(rsm);
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
