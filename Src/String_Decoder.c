
#include <stdio.h>
#include "string.h"
#include <stdlib.h>
#include "String_Decoder.h"


typedef struct sKeywords
{
    enum eCommand Command;
    char * Str;
}Keywords_t;

static Keywords_t Keywords [] = {
    {CMD_WRITE,"write"},
    {CMD_READ,"read"},
    {CMD_SET,"set"},
		{CMD_ACCEL_M_XYZ, "accel_m_xyz"},
		{CMD_GYRO_M_PYR, "gyro_m_pyr"},
		{CMD_ACCEL_M_XYZ_G, "accel_m_xyz_g"},
		{CMD_GYRO_M_PYR_DPS, "gyro_m_pyr_dps"}
};

Request_t Request;

static void  To_Lower_Case(char* InStr)
{
	int i = 0;
	while( InStr[i] != NULL)
	{
		if(InStr[i] > 'A' && InStr[i] < 'Z')
		{
			InStr[i] = InStr[i] + ( 'a' - 'A' );
			
		}
		i++;
	}
}

enum eOperationStatus Find_Tokens(char * InStr, char * Tokens[])
{
    int i = 1;
		To_Lower_Case(InStr);
    Tokens[0] = strtok(InStr," ");
    do 
    {
        Tokens[i] = strtok(NULL, " ");
        i++;
    } while ( Tokens[i-1] != NULL );
    return OK_STATUS;
}

enum eOperationStatus Decode_Tokens(char * Tokens[])
{
    for ( int j = 0; j < KEYWORDS_NUMBER; j++)
    {
        if( !(strcmp(Tokens[0], Keywords[j].Str)))
        {
            Request.Command = Keywords[j].Command;
            break;
        }
				else if ( j == KEYWORDS_NUMBER )
				{
					return ERROR_STATUS;
				}
    }
		
    for( int i = 1; i < MAX_ARGS_NUMBER; i++)
    {
        if( Tokens[i] != NULL)
        {
						if(Tokens[i][0] == '0' && Tokens[i][1] == 'x')
						{
							Request.Arguments[i-1] = strtol(Tokens[i],NULL, 16);
							Request.Number_Of_Arguments = i;
						}
						else
						{
							return ERROR_STATUS;
						}
        }
        else
        {
            Request.Number_Of_Arguments = i - 1;
            break; 
        }
    }
		if(Request.Number_Of_Arguments == 0 )
		{
			return ERROR_STATUS;
		}
		else
		{
			return OK_STATUS;
		}
    
}

