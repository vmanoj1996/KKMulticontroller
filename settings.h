#ifndef SETTINGS_H
#define SETTINGS_H

#include "config.h"

/*** BEGIN DEFINITIONS ***/
#define EEPROM_DATA_START_POS 0      // Settings save offset in eeprom
/*** END DEFINITIONS ***/

/*** BEGIN TYPES ***/
// eeProm data structure
struct config {
  uint8_t  setup;                    // Byte to identify if already setup
  uint8_t RollGyroDirection;
  uint8_t PitchGyroDirection;
  uint8_t YawGyroDirection;
};
/*** END TYPES ***/

/*** BEGIN VARIABLES ***/
extern struct config Config;         // Holds configuration (from eeProm)
/*** END VARIABLES ***/

/*** BEGIN PROTOTYPES ***/
void eeprom_write_byte_changed(uint8_t *addr, uint8_t value);
void eeprom_write_block_changes(const uint8_t *src, void *dest, size_t size);
void Save_Config_to_EEPROM(void);
void Set_EEPROM_Default_Config(void);
void Initial_EEPROM_Config_Load(void);
void settingsSetup(void);
void settingsClearAll(void);
/*** END PROTOTYPES ***/

#endif