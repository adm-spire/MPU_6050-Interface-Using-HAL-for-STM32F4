
#ifndef INC_MPU_H_
#define INC_MPU_H_

void mpu_init(void);
void MPU_ReadAccelGyro(int16_t* ax, int16_t* ay, int16_t* az,
                       int16_t* gx, int16_t* gy, int16_t* gz);

extern I2C_HandleTypeDef hi2c1;

extern UART_HandleTypeDef huart2;

#endif /* INC_MPU_H_ */