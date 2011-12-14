
void crbtty_cleanup(struct crb_tty* tty);

int crbtty_init(struct crb_tty** tty, dev_t devt, int buf_len, struct class* class, int crbif_id, int fun_id, int core_id, int serial_id);

const char* crbtty_get_register_name(int offset, int is_read);

unsigned char crbtty_target_read_byte(struct crb_tty* tty, int offset);
void crbtty_target_write_byte(struct crb_tty* tty, int offset, unsigned char value);

