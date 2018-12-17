#ifndef STRING_DECODER_H_
#define STRING_DECODER_H_

#define KEYWORDS_NUMBER      7
#define MAX_ARGS_NUMBER      5




enum eCommand { CMD_WRITE, CMD_READ, CMD_SET, CMD_ACCEL_M_XYZ, CMD_GYRO_M_PYR,
								CMD_ACCEL_M_XYZ_G, CMD_GYRO_M_PYR_DPS};
enum eOperationStatus { ERROR_STATUS, OK_STATUS };

typedef struct sRequest 
{
    enum eCommand Command;
    int Arguments [MAX_ARGS_NUMBER];
    int Number_Of_Arguments;
}Request_t;

	extern Request_t Request;

enum eOperationStatus Find_Tokens(char * InStr, char * Tokens[]);
enum eOperationStatus Decode_Tokens(char * Tokens[]);

#endif
