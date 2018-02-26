#define REC_BUFFER_LEN 32
#define SERVO_MAX_PARAMS (REC_BUFFER_LEN - 5)

#define REC_WAIT_START_US    75
#define REC_WAIT_PARAMS_US   (SERVO_MAX_PARAMS * 5)
#define REC_WAIT_MAX_RETRIES 200

#define SERVO_INSTRUCTION_ERROR   (1 << 6)
#define SERVO_OVERLOAD_ERROR      (1 << 5)
#define SERVO_CHECKSUM_ERROR      (1 << 4)
#define SERVO_RANGE_ERROR         (1 << 3)
#define SERVO_OVERHEAT_ERROR      (1 << 2)
#define SERVO_ANGLE_LIMIT_ERROR   (1 << 1)
#define SERVO_INPUT_VOLTAGE_ERROR (1)


typedef enum bool
{
	TRUE = 1,
	FALSE = 0
} bool;

extern uint8_t servoErrorCode;

void initServoUSART(HUART_HandleTypeDef huart);

//------------------------------------------------------------------------------
// All functions return true on success and false on failure

// ping a servo, returns true if we get back the expected values
bool pingServo(HUART_HandleTypeDef huart, const uint8_t servoId);

// set the number of microseconds the servo waits before returning a response
// servo factory default value is 500, but we probably want it to be 0
// max value: 510
bool setServoReturnDelayMicros(HUART_HandleTypeDef huart,
								const uint8_t servoId,
                                const uint16_t micros);

// set the errors that will cause the servo to blink its LED
bool setServoBlinkConditions(HUART_HandleTypeDef huart,
							const uint8_t servoId,
                            const uint8_t errorFlags);

// set the errors that will cause the servo to shut off torque
bool setServoShutdownConditions(HUART_HandleTypeDef huart,
								const uint8_t servoId,
                                const uint8_t errorFlags);


// valid torque values are from 0 (free running) to 1023 (max)
bool setServoTorque(HUART_HandleTypeDef huart,
					const uint8_t servoId,
                    const uint16_t torqueValue);

bool getServoTorque(HUART_HandleTypeDef huart
					const uint8_t servoId,
                    uint16_t *torqueValue);

// speed values go from 1 (incredibly slow) to 1023 (114 RPM)
// a value of zero will disable velocity control
bool setServoMaxSpeed(HUART_HandleTypeDef huart
					  const uint8_t servoId,
                      const uint16_t speedValue);

bool getServoMaxSpeed(HUART_HandleTypeDef huart,
					  const uint8_t servoId,
                      uint16_t *speedValue);

bool getServoCurrentVelocity(HUART_HandleTypeDef huart,
							 const uint8_t servoId,
                             int16_t *velocityValue);

// make the servo move to an angle
// valid angles are between 0 and 300 degrees
bool setServoAngle (HUART_HandleTypeDef huart,
					const uint8_t servoId,
                    const float angle);

bool getServoAngle (HUART_HandleTypeDef huart,
					const uint8_t servoId,
                    float *angle);




//------------------------------------------------------------------------------
// these shouldn't need to be called externally:

typedef struct ServoResponse
{
    uint8_t id;
    uint8_t length;
    uint8_t error;
    uint8_t params[SERVO_MAX_PARAMS];
    uint8_t checksum;
} ServoResponse;

void sendServoByte (const uint8_t byte);

void clearServoReceiveBuffer(HUART_HandleTypeDef huart);

size_t  getServoBytesAvailable(HUART_HandleTypeDef huart);
uint8_t getServoByte(HUART_HandleTypeDef huart);

void sendServoCommand (const uint8_t servoId,
                       const uint8_t commandByte,
                       const uint8_t numParams,
                       const uint8_t *params);

bool getServoResponse(HUART_HandleTypeDef huart);


