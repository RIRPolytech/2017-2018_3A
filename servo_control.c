#include "servo_control.h"

typedef enum bool
{
	TRUE = 1,
	FALSE = 0
} bool;


#define TEST_COMMANDS

uint8_t servoErrorCode = 0;

ServoResponse response;

volatile uint8_t receiveBuffer[REC_BUFFER_LEN];
volatile uint8_t* volatile receiveBufferStart = receiveBuffer;
volatile uint8_t* volatile receiveBufferEnd = receiveBuffer;

typedef enum ServoCommand
{
    PING = 1,
    READ = 2,
    WRITE = 3
} ServoCommand;

#define RETURN_DELAY        0x05
#define BLINK_CONDITIONS    0x11
#define SHUTDOWN_CONDITIONS 0x12
#define TORQUE              0x22
#define MAX_SPEED           0x20
#define CURRENT_SPEED       0x26
#define GOAL_ANGLE          0x1e
#define CURRENT_ANGLE       0x24

void sendServoCommand (UART_HandleTypeDef UART,
					   const uint8_t servoId,
                       const ServoCommand commandByte,
                       const uint8_t numParams,
                       const uint8_t *params)
{
    sendServoByte (0xff);
    sendServoByte (0xff);  // command header
    
    sendServoByte (servoId);  // servo ID
    uint8_t checksum = servoId;
    
    sendServoByte (numParams + 2);  // number of following bytes
    sendServoByte ((uint8_t)commandByte);  // command
    
    checksum += numParams + 2 + commandByte;
    
    for (uint8_t i = 0; i < numParams; i++)
    {
        sendServoByte (params[i]);  // parameters
        checksum += params[i];
    }
    
    sendServoByte (~checksum);  // checksum
}

bool getServoResponse(UART_HandleTypeDef UART)
{
    uint8_t retries = 0;
    
    clearServoReceiveBuffer();
    
    while (getServoBytesAvailable() < 4)
    {
        retries++;
        if (retries > REC_WAIT_MAX_RETRIES)
        {
            #ifdef SERVO_DEBUG
            #endif
            return false;
        }
        
        mWaitus (REC_WAIT_START_US);
    }
    retries = 0;
    
    getServoByte();  // servo header (two 0xff bytes)
    getServoByte();
    
    response.id = getServoByte();
    response.length = getServoByte();
    
    if (response.length > SERVO_MAX_PARAMS)
    {
        #ifdef SERVO_DEBUG
        #endif
        return false;
    }
    
    while (getServoBytesAvailable() < response.length)
    {
        retries++;
        if (retries > REC_WAIT_MAX_RETRIES)
        {
            #ifdef SERVO_DEBUG
            #endif
            return false;
        }
        
        mWaitus (REC_WAIT_PARAMS_US);
    }
    
    response.error = getServoByte();
    servoErrorCode = response.error;
    
    for (uint8_t i = 0; i < response.length - 2; i++)
        response.params[i] = getServoByte();
    
    
    uint8_t calcChecksum = response.id + response.length + response.error;
    for (uint8_t i = 0; i < response.length - 2; i++)
        calcChecksum += response.params[i];
		calcChecksum = ~calcChecksum;
    
    const uint8_t recChecksum = getServoByte();
    if (calcChecksum != recChecksum)
    {
        #ifdef SERVO_DEBUG
        #endif
        return false;
    }

    return true;
}

inline bool getAndCheckResponse(UART_HandleTypeDef UART, 
								const uint8_t servoId)
{
    if (!getServoResponse())
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Servo %d did not respond correctly or at all\n", (int)servoId);
        #endif
        return false;
    }
    
    if (response.id != servoId)
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Response ID %d does not match command ID %d\n", (int)response.id);
        #endif
        return false;
    }
    
    if (response.error != 0)
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Response error code was nonzero (%d)\n", (int)response.error);
        #endif
        return false;
    }
    
    return true;
}

// ping a servo, returns true if we get back the expected values
bool pingServo(UART_HandleTypeDef UART,
				const uint8_t servoId)
{
    sendServoCommand (servoId, PING, 0, 0);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool setServoReturnDelayMicros(UART_HandleTypeDef UART
							   const uint8_t servoId,
                               const uint16_t micros)
{
    if (micros > 510)
        return false;
    
    const uint8_t params[2] = {RETURN_DELAY,
                               (uint8_t)((micros / 2) & 0xff)};
    
    sendServoCommand (servoId, WRITE, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

// set the events that will cause the servo to blink its LED
bool setServoBlinkConditions (UART_HandleTypeDef UART,
							  const uint8_t servoId,
                              const uint8_t flags)
{
    const uint8_t params[2] = {BLINK_CONDITIONS,
                               flags};
    
    sendServoCommand (servoId, WRITE, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

// set the events that will cause the servo to shut off torque
bool setServoShutdownConditions(UART_HandleTypeDef UART,
								const uint8_t servoId,
                                const uint8_t flags)
{
    const uint8_t params[2] = {SHUTDOWN_CONDITIONS,
                               flags};
    
    sendServoCommand (servoId, WRITE, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}


// valid torque values are from 0 (free running) to 1023 (max)
bool setServoTorque(UART_HandleTypeDef UART,
					const uint8_t servoId,
                    const uint16_t torqueValue)
{
    const uint8_t highByte = (uint8_t)((torqueValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(torqueValue & 0xff);
    
    if (torqueValue > 1023)
        return false;
    
    const uint8_t params[3] = {TORQUE,
                               lowByte,
                               highByte};
    
    sendServoCommand (servoId, WRITE, 3, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool getServoTorque(UART_HandleTypeDef UART,
					const uint8_t servoId,
                    uint16_t *torqueValue)
{
    const uint8_t params[2] = {TORQUE,
                               2};  // read two bytes, starting at address TORQUE
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    *torqueValue = response.params[1];
    *torqueValue <<= 8;
    *torqueValue |= response.params[0];
    
    return true;
}

// speed values go from 1 (incredibly slow) to 1023 (114 RPM)
// a value of zero will disable velocity control
bool setServoMaxSpeed(UART_HandleTypeDef UART,
					  const uint8_t servoId,
                      const uint16_t speedValue)
{
    const uint8_t highByte = (uint8_t)((speedValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(speedValue & 0xff);
    
    if (speedValue > 1023)
        return false;
    
    const uint8_t params[3] = {MAX_SPEED,
                               lowByte,
                               highByte};
    
    sendServoCommand (servoId, WRITE, 3, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool getServoMaxSpeed(UART_HandleTypeDef UART,
					  const uint8_t servoId,
                      uint16_t *speedValue)
{
    const uint8_t params[2] = {MAX_SPEED,
                               2};  // read two bytes, starting at address MAX_SPEED
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    *speedValue = response.params[1];
    *speedValue <<= 8;
    *speedValue |= response.params[0];
    
    return true;
}

bool getServoCurrentVelocity(UART_HandleTypeDef UART,
							 const uint8_t servoId,
                             int16_t *velocityValue)
{
    const uint8_t params[2] = {CURRENT_SPEED,
                               2};  // read two bytes, starting at address CURRENT_SPEED
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    *velocityValue = response.params[1];
    *velocityValue <<= 8;
    *velocityValue |= response.params[0];
    
    return true;
}

// make the servo move to an angle
// valid angles are between 0 and 300 degrees
bool setServoAngle(UART_HandleTypeDef UART,
					const uint8_t servoId,
                    const float angle)
{
    if (angle < 0 || angle > 300)
        return false;
    
    // angle values go from 0 to 0x3ff (1023)
    const uint16_t angleValue = (uint16_t)(angle * (1023.0 / 300.0));
    
    const uint8_t highByte = (uint8_t)((angleValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(angleValue & 0xff);
    
    const uint8_t params[3] = {GOAL_ANGLE,
                               lowByte,
                               highByte};
    
    sendServoCommand (servoId, WRITE, 3, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool getServoAngle(UART_HandleTypeDef UART,
					const uint8_t servoId,
                    float *angle)
{
    const uint8_t params[2] = {CURRENT_ANGLE,
                               2};  // read two bytes, starting at address CURRENT_ANGLE
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    uint16_t angleValue = response.params[1];
    angleValue <<= 8;
    angleValue |= response.params[0];
    
    *angle = (float)angleValue * 300.0 / 1023.0;
    
    return true;
}



void sendServoByte(UART_HandleTypeDef *UART,
					const uint8_t byte)
{
	  USART_SendData (&UART, (uint8_t)byte, (uint16_t)sizeof(byte));
	  
	  //Loop until the end of transmission
	  while (USART_GetFlagStatus (USART3, USART_FLAG_TC) == RESET);//===> TO DO
	  //Suppostion :
	  //while(UART_EndTransmission_IT(&UART)!= 0);
}

void clearServoReceiveBuffer(UART_HandleTypeDef UART)
{
    receiveBufferStart = receiveBufferEnd;
}

size_t getServoBytesAvailable(UART_HandleTypeDef UART)
{
    volatile uint8_t *start = receiveBufferStart;
    volatile uint8_t *end = receiveBufferEnd;
    
    if (end >= start)
        return (size_t)(end - start);
    else
        return (size_t)(REC_BUFFER_LEN - (start - end));
}

uint8_t getServoByte(UART_HandleTypeDef UART)
{
    receiveBufferStart++;
	if (receiveBufferStart >= receiveBuffer + REC_BUFFER_LEN)
	{
        receiveBufferStart = receiveBuffer;
	}
 
    return *receiveBufferStart;
}


#ifdef TEST_COMMANDS

//int main (void)
//{
//    mInit();
//    mUSBInit();
//    initServoUSART();
//    
//    mRedON;
//    mWaitms (1500);
//    mRedOFF;
//    
//    printf ("Init complete\n");
//    
//    const uint8_t id = 4;
//    
//    if (!pingServo (id))
//    {
//        printf ("Ping failed\n");
//        error();
//    }
//    
//    printf ("Ping OK\n");
//    
//    if (!setServoReturnDelayMicros (id, 0))
//    {
//        printf ("Set return delay failed\n");
//        error();
//    }
//    
//    printf ("Set return delay OK\n");
//    
//    if (!setServoBlinkConditions (id, SERVO_RANGE_ERROR | SERVO_ANGLE_LIMIT_ERROR))
//    {
//        fflush (stdout);
//        printf ("Set blink conditions failed\n");
//        error();
//    }
//    
//    printf ("Set blink conditions OK\n");
//    
//    if (!setServoShutdownConditions (id, SERVO_OVERLOAD_ERROR | SERVO_OVERHEAT_ERROR))
//    {
//        fflush (stdout);
//        printf ("Set shutdown conditions failed\n");
//        error();
//    }
//    
//    printf ("Set shutdown conditions OK\n");
//    
//    uint16_t torque = 0;
//    if (!getServoTorque (id, &torque))
//    {
//        fflush (stdout);
//        printf ("Get servo torque failed\n");
//        error();
//    }
//    
//    printf ("Get torque OK: servo torque = %u\n", torque);
//    
//    torque = 512;
//    if (!setServoTorque (id, torque))
//    {
//        fflush (stdout);
//        printf ("Set servo torque failed\n");
//        error();
//    }
//    
//    printf ("Set torque OK\n");
//    
//    torque = 0;
//    if (!getServoTorque (id, &torque))
//    {
//        printf ("Get servo torque failed\n");
//        error();
//    }
//    
//    printf ("Get torque OK: servo torque = %u\n", torque);
//    
//    uint16_t speed = 0;
//    if (!getServoMaxSpeed (id, &speed))
//    {
//        printf ("Get servo max speed failed\n");
//        error();
//    }
//    
//    printf ("Get max speed OK: max speed = %u\n", speed);
//    
//    speed = 1023;
//    if (!setServoMaxSpeed (id, speed))
//    {
//        fflush (stdout);
//        printf ("Set servo max speed failed\n");
//        error();
//    }
//    
//    printf ("Set max speed OK\n");
//    
//    speed = 0;
//    if (!getServoMaxSpeed (id, &speed))
//    {
//        printf ("Get servo max speed failed\n");
//        error();
//    }
//    
//    printf ("Get max speed OK: max speed = %u\n", speed);
//    
//    int16_t currentSpeed = 0;
//    if (!getServoMaxSpeed (id, &currentSpeed))
//    {
//        printf ("Get servo current speed failed\n");
//        error();
//    }
//    
//    printf ("Get current speed OK: current speed = %d\n", currentSpeed);
//    
//    
//    for (int8_t i = 0; i < 5; i++)
//    {
//        if (!setServoMaxSpeed (id, 128 * (i + 1)))
//        {
//            fflush (stdout);
//            printf ("Set servo max speed failed\n");
//            error();
//        }
//        
//        float angle = 0;
//        if (!getServoAngle (id, &angle))
//        {
//            printf ("Get servo angle failed\n");
//            error();
//        }
//        
//        printf ("Angle = %f\n", angle);
//        
//        angle = 300.0 - angle;
//        if (!setServoAngle (id, angle))
//        {
//            printf ("Set servo angle failed\n");
//            error();
//        }
//        
//        printf ("Set angle to %f\n", angle);
//        
//        for (uint8_t i = 0; i < 1 + (4 - i); i++)
//            mWaitms (500);
//    }
//    
//    printf ("Done\n");
//    
//    for (;;)
//    {
//        mGreenTOGGLE;
//        mRedTOGGLE;
//        
//        mWaitms (250);
//    }
//    
//    return 0;
//}

#endif

