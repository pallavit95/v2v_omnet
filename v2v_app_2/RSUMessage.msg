cplusplus {{ 
#include "veins/modules/messages/WaveShortMessage_m.h"
}}

class WaveShortMessage;

packet RSUMessage extends WaveShortMessage {
	 string allowedVehicles[];
//	 Coord senderPosition; // The position of the sender
//added new
	double senderPosX; // The X position of the sender.
    double senderPosY; // The Y position of the sender.
    double senderPosZ; // The Z position of the sender.
}
