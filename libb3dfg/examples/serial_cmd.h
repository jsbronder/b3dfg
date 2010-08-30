
#define START_TRIGGERS      "01"
#define STOP_TRIGGERS       "02"
#define N_TRIGGERS          "03"
#define SET_LOW_TIME        "04"
#define SET_LED_INTENSITY   "06"
#define HW_RESET            "0D"

int send_cmd(int fd, char *command);
int open_serial(const char *path);
