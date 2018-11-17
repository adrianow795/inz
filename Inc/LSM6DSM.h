#ifndef LSM6DSM_H_
#define LSM6DSM_H_


#define LSM6DSM_FUNC_CFG_ACCESS  							0x01

#define LSM6DSM_SENSOR_SYNC_TIME  						0x04
#define LSM6DSM_SENSOR_RES_RATIO  						0x05

#define LSM6DSM_FIFO_CTRL1  									0x06
#define LSM6DSM_FIFO_CTRL2  									0x07
#define LSM6DSM_FIFO_CTRL3  									0x08
#define LSM6DSM_FIFO_CTRL4  									0x09
#define LSM6DSM_FIFO_CTRL5  									0x0A

#define LSM6DSM_DRDY_PULSE_CFG_G  						0x0B
#define LSM6DSM_INT1_CTRL  										0x0D
#define LSM6DSM_INT2_CTRL  										0x0E
#define LSM6DSM_WHO_AM_I_REG  								0x0F
#define LSM6DSM_CTRL1_XL  										0x10
#define LSM6DSM_CTRL2_G  											0x11
#define LSM6DSM_CTRL3_C  											0x12
#define LSM6DSM_CTRL4_C  											0x13
#define LSM6DSM_CTRL5_C  											0x14
#define LSM6DSM_CTRL6_C  											0x15
#define LSM6DSM_CTRL7_G  											0x16
#define LSM6DSM_CTRL8_XL  										0x17
#define LSM6DSM_CTRL9_XL  										0x18
#define LSM6DSM_CTRL10_C  										0x19

#define LSM6DSM_MASTER_CONFIG  								0x1A
#define LSM6DSM_WAKE_UP_SRC  									0x1B
#define LSM6DSM_TAP_SRC  											0x1C
#define LSM6DSM_D6D_SRC  											0x1D
#define LSM6DSM_STATUS_REG  									0x1E
#define LSM6DSM_STATUS_SPIAux  								0x1E

#define LSM6DSM_OUT_TEMP_L  									0x20
#define LSM6DSM_OUT_TEMP_H  									0x21
#define LSM6DSM_OUTX_L_G  										0x22
#define LSM6DSM_OUTX_H_G  										0x23
#define LSM6DSM_OUTY_L_G  										0x24
#define LSM6DSM_OUTY_H_G  										0x25
#define LSM6DSM_OUTZ_L_G  										0x26
#define LSM6DSM_OUTZ_H_G  										0x27
#define LSM6DSM_OUTX_L_XL  										0x28
#define LSM6DSM_OUTX_H_XL  										0x29
#define LSM6DSM_OUTY_L_XL  										0x2A
#define LSM6DSM_OUTY_H_XL  										0x2B
#define LSM6DSM_OUTZ_L_XL  										0x2C
#define LSM6DSM_OUTZ_H_XL  										0x2D
#define LSM6DSM_SENSORHUB1_REG  							0x2E
#define LSM6DSM_SENSORHUB2_REG  							0x2F
#define LSM6DSM_SENSORHUB3_REG  							0x30
#define LSM6DSM_SENSORHUB4_REG  							0x31
#define LSM6DSM_SENSORHUB5_REG  							0x32
#define LSM6DSM_SENSORHUB6_REG  							0x33
#define LSM6DSM_SENSORHUB7_REG  							0x34
#define LSM6DSM_SENSORHUB8_REG  							0x35
#define LSM6DSM_SENSORHUB9_REG  							0x36
#define LSM6DSM_SENSORHUB10_REG  							0x37
#define LSM6DSM_SENSORHUB11_REG  							0x38
#define LSM6DSM_SENSORHUB12_REG  							0x39
#define LSM6DSM_FIFO_STATUS1  								0x3A
#define LSM6DSM_FIFO_STATUS2  								0x3B
#define LSM6DSM_FIFO_STATUS3  								0x3C
#define LSM6DSM_FIFO_STATUS4  								0x3D
#define LSM6DSM_FIFO_DATA_OUT_L  							0x3E
#define LSM6DSM_FIFO_DATA_OUT_H  							0x3F
#define LSM6DSM_TIMESTAMP0_REG  							0x40
#define LSM6DSM_TIMESTAMP1_REG  							0x41
#define LSM6DSM_TIMESTAMP2_REG  							0x42

#define LSM6DSM_TIMESTAMP_L  									0x49
#define LSM6DSM_TIMESTAMP_H  									0x4A

#define LSM6DSM_STEP_COUNTER_L  							0x4B
#define LSM6DSM_STEP_COUNTER_H  							0x4C

#define LSM6DSM_SENSORHUB13_REG  							0x4D
#define LSM6DSM_SENSORHUB14_REG  							0x4E
#define LSM6DSM_SENSORHUB15_REG  							0x4F
#define LSM6DSM_SENSORHUB16_REG  							0x50
#define LSM6DSM_SENSORHUB17_REG  							0x51
#define LSM6DSM_SENSORHUB18_REG  							0x52

#define LSM6DSM_FUNC_SRC  										0x53
#define LSM6DSM_TAP_CFG1  										0x58
#define LSM6DSM_TAP_THS_6D  									0x59
#define LSM6DSM_INT_DUR2  										0x5A
#define LSM6DSM_WAKE_UP_THS  									0x5B
#define LSM6DSM_WAKE_UP_DUR  									0x5C
#define LSM6DSM_FREE_FALL  										0x5D
#define LSM6DSM_MD1_CFG  											0x5E
#define LSM6DSM_MD2_CFG  											0x5F

#define LSM6DSM_OUT_MAG_RAW_X_L  							0x66 
#define LSM6DSM_OUT_MAG_RAW_X_H  							0x67
#define LSM6DSM_OUT_MAG_RAW_Y_L  							0x68
#define LSM6DSM_OUT_MAG_RAW_Y_H  							0x69
#define LSM6DSM_OUT_MAG_RAW_Z_L  							0x6A
#define LSM6DSM_OUT_MAG_RAW_Z_H  							0x6B

#define LSM6DSM_X_OFS_USR  										0x73
#define LSM6DSM_Y_OFS_USR  										0x74
#define LSM6DSM_Z_OFS_USR  										0x75

/************** OIS Register SPI2 *******************/

#define LSM6DSM_CTRL_OIS  										0x70 
#define LSM6DSM_CTRL2_OIS  										0x71

/************** Embedded functions register mapping  *******************/
#define LSM6DSM_SLV0_ADD                     	0x02
#define LSM6DSM_SLV0_SUBADD                  	0x03
#define LSM6DSM_SLAVE0_CONFIG                	0x04
#define LSM6DSM_SLV1_ADD                     	0x05
#define LSM6DSM_SLV1_SUBADD                  	0x06
#define LSM6DSM_SLAVE1_CONFIG                	0x07
#define LSM6DSM_SLV2_ADD                     	0x08
#define LSM6DSM_SLV2_SUBADD                  	0x09
#define LSM6DSM_SLAVE2_CONFIG                	0x0A
#define LSM6DSM_SLV3_ADD                     	0x0B
#define LSM6DSM_SLV3_SUBADD                  	0x0C
#define LSM6DSM_SLAVE3_CONFIG                	0x0D
#define LSM6DSM_DATAWRITE_SRC_MODE_SUB_SLV0  	0x0E
#define LSM6DSM_CONFIG_PEDO_THS_MIN          	0x0F

#define LSM6DSM_SM_STEP_THS                  	0x13
#define LSM6DSM_PEDO_DEB_REG                	0x14
#define LSM6DSM_STEP_COUNT_DELTA            	0x15

#define LSM6DSM_MAG_SI_XX                    	0x24
#define LSM6DSM_MAG_SI_XY                    	0x25
#define LSM6DSM_MAG_SI_XZ                    	0x26
#define LSM6DSM_MAG_SI_YX                    	0x27
#define LSM6DSM_MAG_SI_YY                    	0x28
#define LSM6DSM_MAG_SI_YZ                    	0x29
#define LSM6DSM_MAG_SI_ZX                    	0x2A
#define LSM6DSM_MAG_SI_ZY                    	0x2B
#define LSM6DSM_MAG_SI_ZZ                    	0x2C
#define LSM6DSM_MAG_OFFX_L                   	0x2D
#define LSM6DSM_MAG_OFFX_H                   	0x2E
#define LSM6DSM_MAG_OFFY_L                   	0x2F
#define LSM6DSM_MAG_OFFY_H                   	0x30
#define LSM6DSM_MAG_OFFZ_L                   	0x31
#define LSM6DSM_MAG_OFFZ_H 										0x32


#endif
