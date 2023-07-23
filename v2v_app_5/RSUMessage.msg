cplusplus {{ 
#include "veins/modules/messages/WaveShortMessage_m.h"
}}

class WaveShortMessage;

packet RSUMessage extends WaveShortMessage {
    double rsuPosX;
    double rsuPosY;
    double rsuPosZ;
}
