#ifndef __GYRO_H__
#define __GYRO_H__

#ifdef GYRO_ICM20608
extern void icm20608_init(void);
extern void icm20608_read_data(unsigned char *pbuf_accel,unsigned char *pbuf_gyro);
#endif
extern void gyro_sleep(void);
extern void gyro_wakeup(void);
extern void gyro_init(void);
#endif
