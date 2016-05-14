// Calibration values - These are stored in the BMP085/180
short int bmp_ac1;
short int bmp_ac2;
short int bmp_ac3;
unsigned short int bmp_ac4;
unsigned short int bmp_ac5;
unsigned short int bmp_ac6;
short int bmp_b1;
short int bmp_b2;
int bmp_b5;
short int bmp_mb;
short int bmp_mc;
short int bmp_md;

int bmp_read_int(int fd, uint8_t *devValues, uint8_t startReg,
                uint8_t bytesToRead);
int bmp_write_cmd(int fd, uint8_t devAction);
int bmp_calibration(int fd);
int bmp_wait_for_conversion(int fd);
int bmp_get_pressure(int fd, double *Pres);
int bmp_get_temperature(int fd, double *Temp);

double bmp_altitude(double p);
double bmp_qnh(double p, double StationAlt);
double ppl_density_alt(double PAlt, double Temp);
