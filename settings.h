#ifndef SETTINGS_H
#define SETTINGS_H

#define EEPROM_DATA_START_POS 0      // Settings save offset in eeprom

// eeProm data structure
static struct config {
  uint8_t  setup;          // byte to identify if already setup

  uint8_t RollGyroDirection;
  uint8_t PitchGyroDirection;
  uint8_t YawGyroDirection;
} Config;        // Holds configuration (from eeProm)


static void eeprom_write_byte_changed(uint8_t *addr, uint8_t value)
{
  if(eeprom_read_byte(addr) != value)
    eeprom_write_byte(addr, value);
}

static void eeprom_write_block_changes(const uint8_t *src, void *dest, size_t size)
{
  size_t len;

  for(len = 0;len < size;len++) {
    eeprom_write_byte_changed(dest, *src);
    src++;
    dest++;
  }
}

static void Save_Config_to_EEPROM()
{
  eeprom_write_block_changes(
    (const void *)&Config, (void *)EEPROM_DATA_START_POS,
    sizeof(struct config));
}

static void Set_EEPROM_Default_Config()
{
  Config.RollGyroDirection  = GYRO_REVERSED;
  Config.PitchGyroDirection  = GYRO_REVERSED;
  Config.YawGyroDirection    = GYRO_NORMAL;
}

static void Initial_EEPROM_Config_Load()
{
  // load up last settings from EEPROM
  if(eeprom_read_byte((uint8_t *)EEPROM_DATA_START_POS) != 42) {
    Config.setup = 42;
    Set_EEPROM_Default_Config();
    // write to eeProm
    Save_Config_to_EEPROM();
  } else {
    // read eeprom
    eeprom_read_block(&Config, (void *)EEPROM_DATA_START_POS, sizeof(struct config));
  }
}

inline static void settingsSetup()
{
  Initial_EEPROM_Config_Load();
}

inline static void settingsClearAll()
{
  for(uint8_t i = 0;i < 5;i++) {
    LED = 1;
    _delay_ms(25);
    LED = 0;
    _delay_ms(25);
  }

  Set_EEPROM_Default_Config();
  while(1)
          ;
}

#endif