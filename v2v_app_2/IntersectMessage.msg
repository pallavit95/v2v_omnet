cplusplus {{ 
#include "veins/modules/messages/WaveShortMessage_m.h"
}}

class WaveShortMessage;

enum directionType {
	STRAIGHT = 0;
	RIGHT = 1;
	LEFT = 2;
};

packet IntersectMessage extends WaveShortMessage {
	string vehicleId;
	string vehicleType;
	simtime_t timeSent;
	string roadId; //what road the car is currently on
	int direction @enum(directionType); //direction car wants to go
	bool passed; //if the car has already passed the intersection or not
//	Coord senderPosition; // The position of the sender
//added new
	double senderPosX; // The X position of the sender.
    double senderPosY; // The Y position of the sender.
    double senderPosZ; // The Z position of the sender.
}
