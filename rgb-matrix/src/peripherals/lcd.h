#define LCD_TRANSMIT_SIZE 3
#define LCD_DATABUS dbBits
#define LCD_DB_FUNCTION_SET (0x03 << 4)
#define LCD_DB_DISPLAY_CLR (1 << 0)
#define LCD_DB_DISPLAY_SET(display, cursor, blink) ((1 << 3) | (display << 2) | (cursor << 1) | (blink << 0))
#define LCD_DB_ENTRY_SET(right) ((1 << 2) | (right << 1))
#define LCD_DB_CURSOR_MOVE(right) ((1 << 4) | (right << 2)) 
#define LCD_SET_RS_RW_BITS(rs, rw) cmd[0] |= ((rs << 1) | (rw << 0))
#define LCD_CREATE_DATABUS uint8_t dbBits = 0
#define LCD_CREATE_CMD uint8_t cmd[LCD_TRANSMIT_SIZE] = {0}

// Macro functions:
// Replace for loops with arr[1] |= (bits & 0xF0)
// arr[2] |= (bits << 4)
#define SET_DB_BITS(bits, arr) { \
    arr[1] &= 0x0F; \
    arr[2] &= 0x0F; \
    arr[1] |= (bits & 0xF0);\
    arr[2] |= (bits << 4);\
}

#define CREATE_SYNC_BITS {\
    for(uint8_t i = 0; i < 6; i++) {\
        cmd[0] |= (1 << (8 - i));\
    }\
}

#define LCD_TRANSMIT_CMD_ENTRY {\
    dbBits = 0;\
    dbBits |= LCD_DB_ENTRY_SET(1);\
    SET_DB_BITS(dbBits, cmd)\
    transmit_spi(cmd, LCD_TRANSMIT_SIZE);\
}  

#define LCD_TRANSMIT_CMD_CLEAR {\
    dbBits = 0;\
    dbBits |= LCD_DB_DISPLAY_CLR;\
    SET_DB_BITS(dbBits, cmd);\
    transmit_spi(cmd, LCD_TRANSMIT_SIZE);\
}

#define LCD_TRANSMIT_CMD_DISPLAY(display, cursor, blink) {\
    dbBits = 0;\
    dbBits |= LCD_DB_DISPLAY_SET(display, cursor, blink);\
    SET_DB_BITS(dbBits, cmd);\
    transmit_spi(cmd, LCD_TRANSMIT_SIZE);\
}

#define LCD_TRANSMIT_CMD_FUNCTION_SET {\
    cmd[0] &= ~(0x03 << 1);\
    dbBits = LCD_DB_FUNCTION_SET;\
    SET_DB_BITS(dbBits, cmd);\
    transmit_spi(cmd, LCD_TRANSMIT_SIZE);\
}
