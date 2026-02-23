#ifndef COMMS_ESP_NOW_H
#define COMMS_ESP_NOW_H

typedef struct struct_message {
    bool toggleState;
    bool override;
    bool homing;
} struct_message;

extern bool overrideActive;

void initEspNow();

#endif //COMMS_ESP_NOW_H
