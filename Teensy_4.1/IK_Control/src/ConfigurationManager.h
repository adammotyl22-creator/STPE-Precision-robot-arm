#ifndef CONFIGURATION_MANAGER_H
#define CONFIGURATION_MANAGER_H

#include <EEPROM.h>
#include "Config.h"

struct RobotConfig {
    uint32_t magic;
    float gear1;
    float gear2;
    bool jntEn[3];
    float speedOvr;
    float toolOff[3];
    float workOff[3];
};

class ConfigurationManager {
public:
    static void load(RobotConfig& config) {
        uint32_t magic;
        EEPROM.get(EEPROM_MAGIC_ADDR, magic);
        if (magic != EEPROM_MAGIC_NUMBER) {
            // Load defaults
            config.magic = EEPROM_MAGIC_NUMBER;
            config.gear1 = 8.33f;
            config.gear2 = 20.0f;
            config.jntEn[0] = config.jntEn[1] = config.jntEn[2] = true;
            config.speedOvr = 1.0f;
            for(int i=0; i<3; i++) config.toolOff[i] = config.workOff[i] = 0;
            return;
        }
        EEPROM.get(EEPROM_MAGIC_ADDR, config);
    }

    static void save(const RobotConfig& config) {
        EEPROM.put(EEPROM_MAGIC_ADDR, config);
    }
};

#endif
