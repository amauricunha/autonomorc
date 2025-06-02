#ifndef CAN_COMM_H
#define CAN_COMM_H

void setupCAN();
void sendCANMessage(int id, const uint8_t* data, uint8_t len);
bool receiveCANMessage(int* id, uint8_t* data, uint8_t* len);

#endif
