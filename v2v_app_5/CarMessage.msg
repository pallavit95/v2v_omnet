cplusplus {{ 
#include "veins/modules/messages/WaveShortMessage_m.h"
}}

class WaveShortMessage;



packet CarMessage extends WaveShortMessage {
	string helloMsg;
	int priority;
	string rsuId;
	string carId;
}