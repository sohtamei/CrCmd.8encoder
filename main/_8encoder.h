
#define _8ENCODER_REG_COUNTER		0x00	// [R/W]4byte*8
#define _8ENCODER_REG_INCREMENT		0x20	// [R] 4byte*8
#define _8ENCODER_REG_RESET			0x40	// [W] 1byte*8
#define _8ENCODER_REG_BUTTON		0x50	// [R] 1byte*8
#define _8ENCODER_REG_SWITCH		0x60	// [R] 1byte
#define _8ENCODER_REG_RGB			0x70	// [R/W]3byte*9

int _8encoder_init(void);
int _8encoder_delete(void);

int _8encoder_read(int reg, int32_t* data, int num);
int _8encoder_write(int reg, uint32_t data);

