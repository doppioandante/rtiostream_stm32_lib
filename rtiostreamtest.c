/*
 * File: rtiostreamtest.c
 *
 * Test suite for rtiostream.
 *
 * See also rtiostreamtest.m.
 *
 * Copyright 2012-2014 The MathWorks, Inc.
 */

#include <limits.h>
#include "rtiostream.h"
#include "rtiostreamtest.h"

/***************** DEFINES ****************************************************/

#ifndef BUFFER_SIZE
/* Size of rtiostreamtest's buffer for receiving / transmitting
 * data via rtiostream. */
#define BUFFER_SIZE (128)
#else
#if BUFFER_SIZE < 128
#error "BUFFER_SIZE must be at least 128."
#endif
#endif

/* The number of repeats of the CPU blocking loop that causes the device
 * to sleep. Slower devices need lower numbers, fast devices need big numbers.
 * If the SLEEP_COEFF is too out of range and MATLAB can't synchronize the clocks,
 * it will ask for this coefficient to be changed accordingly. */
#ifndef SLEEP_COEFF
#define SLEEP_COEFF (100)
#endif

/** Other constants **/

/* Maximum number of commands that could be stored in a macro */
#define MAX_MACRO_LENGTH (8)

#define HS_STOP (51) /* case label */
#define HS_REPEAT (1) /* case label */

/* Maximum number that the data_counter can take */
#define FIRST_INVALID_COUNTER_VALUE (4228250626u) /* 255^4 */

/* Time out for pausing after opening the server connection */
#define OPEN_SERVER_SLEEP_TIMEOUT (100UL)

/***************** TYPEDEFS **************************************************/

#ifndef MEM_UNIT_TYPE
#define MEM_UNIT_TYPE char
#endif
typedef unsigned MEM_UNIT_TYPE MemUnit_T;/* at least 8 bits in size */

#ifndef MEM_UNIT_BYTES
/* default calculation will work on most architectures, e.g.
 * 32-bit word addressable: 1 * 32 / 8 = 4
 * 16-bit word addressable: 1 * 16 / 8 = 2
 * 8-bit byte addressable: 1 * 8 / 8 = 1
 * 16-bit simulated MEM_UNIT_T on host: 2 * 8 / 8 = 2
 * 32-bit simulated MEM_UNIT_T on host: 4 * 8 / 8 = 4 */
#define MEM_UNIT_BYTES (sizeof(MemUnit_T) * (CHAR_BIT / 8))
#endif

#ifdef HOST_WORD_ADDRESSABLE_TESTING
/* rtIOStream will handle data in single byte chunks */
typedef unsigned char IOUnit_T;
#else
/* rtIOStream will handle data in MemUnit_T size chunks */
typedef MemUnit_T IOUnit_T;
#endif

/* The macro is consisted of commands in an array */
typedef struct Command_tag
{
    MemUnit_T opcode;           /* The op-code of the command                                                                */
    unsigned long arg0;         /* The argument of the command                                                              */
} Command;

/* Possible commands
 * Note: When adding a new command make sure you modify:
 * 1. receiveCommand
 * 2. executeCommand
 * 3. MATLAB definitions
 */
enum opcodes
{

    /* real time commands */
    op_shutdown = 0,            /* shuts down server                                                                        */
    op_macro_run,               /* starts a macro                                                                           */
    op_blockingReceive_test,    /* hardcoded test for blocking receive                                                      */
    op_getBufferSize,           /* hardcoded test for blocking send                                                         */

    /* macro commands */
    op_macro_repeat_start,      /* marks a start of a repeat                                                                */
    op_macro_repeat,            /* specifies repeat of macro commands from last repeat_start                                */
    op_macro_blockRcvSlow,      /* does a blocking receive with 40 bit counter blocks (slow)                                */
    op_macro_blockSendSlow,     /* does a blocking send with 40 bit counter blocks (slow)                                   */
    op_macro_blockRcvFast,      /* does a blocking receive with counter checksum                                            */
    op_macro_blockSendFast,     /* does a blocking send with counter checksum                                               */
    op_macro_pause,             /* calls sleep.                                                                             */
    op_macro_sendWakeup         /* sends a message to indicate the pause is over                                            */
};

/* Error codes to be sent as arguments with ACK */
enum status_codes
{
    stat_OK = 0,                /* Operation has finished successfully                                                      */
    stat_UnknownCommand,        /* Unknown op-code received                                                                  */
    stat_RTIOSTREAM_ERROR,      /* Driver did not return RTIOSTREAM_OK                                                      */
    stat_DataIntegrity_Fail,    /* Incoming data does not match the expected pattern                                        */
    stat_CommandChecksum_Fail,  /* Scrambled command received                                                               */
    stat_notEnoughSpace         /* BUFFER_SIZE is not big enough to perform an IO operation                                 */
};

/* provide externally accessible state for testing purposes */
int rtiostrInBlockRcvFast = 0;
int rtiostrInBlockSendFast = 0;

/**************** LOCAL DATA *************************************************/

/* Contains the ID of the server */
static int streamID;

/* Whether server is running */
static unsigned int running = 1;

/* Buffer that the library will use for sending/receiving */
static MemUnit_T buff[BUFFER_SIZE];

/* For storing and running macros */
static Command macro[MAX_MACRO_LENGTH];
static unsigned long macro_length = 0;             /* Accumulated length of the macro so far          */
static unsigned long macro_execution_counter = 0;  /* The position of the EC inside the macro          */
static unsigned long macro_repeat_count = 0;       /* How many repeats have been done so far           */
static unsigned long macro_repeat_start_id = 0;    /* Where does the repeat start                      */

/* Integrity check */
static unsigned long data_counter = 0;                /* Incremented every time a chunk is sent/received  */

/* For the slow data_counter check */
static signed int storage_counter = 0; /* Counter to keep track of position in "memory" array */
static unsigned long last_sync_pos = FIRST_INVALID_COUNTER_VALUE; /* Counter to keep track of Z values */
static MemUnit_T counter_storage_arr[5]; /* Array to store "memory" */
static int dataCheckState = 0; /* Current state of the state machine */

/* Last error/succors code to be sent after each command has been executed */
static MemUnit_T AckCode = stat_OK;                /* Set on error. It will be sent back to the client */
static unsigned long AckArg0 = 0;                  /* Number giving detailed info about the error      */

/* Macros for use with the IO methods */
static const int RECEIVE = 0;
static const int SEND = 1;

/* One-byte ACK during handshake notifying host of incorrect data received */
static const MemUnit_T HS_INCORRECT_DATA_RECVD = 19;
static const MemUnit_T HS_STOP_ACK = 52;
static const MemUnit_T HS_RTIOSTREAM_RECV_ERROR = 53;
static const MemUnit_T WAKEUP_MESSAGE = 99; /* used for op_macro_sendWakeup */

/* Integrity check numbers. Don't change those! */
static const MemUnit_T SYNC_BYTE_VALUE = 255;
static const int SIZE_OF_COUNTER = 5;

/************** LOCAL FUNCTION PROTOTYPES ************************************/

static void executeCommand(Command * command);

static int openServer(int rtArgc, void * rtArgv[]);

static void handshake(void);

static int closeServer(void);

static void blockingIO(int send, unsigned long readStep);

static void sendAck(void);

static void receiveCommand(void);

static void enqueueCommand(Command * command);

static void sleep(unsigned long amount);

static void checkIncomingDataSlow(MemUnit_T * start, int length);

static void oneSleepTic(void);

static void generateCounter(unsigned long Z, MemUnit_T array[5]);

/************** ENTRY POINT **************************************************/

/* Function: rtiostreamtest =====================================================
 * Abstract:
 *  Launches the server. The command line arguments
 *  are redirected to the rtiostream.
 */
int rtiostreamtest(int argc, char *argv[])
{
    /* Open connection */
    if(openServer(argc, (void **) argv) == RTIOSTREAM_ERROR) {
        return RTIOSTREAM_ERROR;
    }

    /* Pause before sending Endianess data */
    sleep(OPEN_SERVER_SLEEP_TIMEOUT);

    /* Negotiate Endianess and MemUnit_T size */
    handshake();

    /* Handle incoming op-codes */
    while ( running ) receiveCommand();

    /* Close connection */
    if(closeServer() == RTIOSTREAM_ERROR) {
        return RTIOSTREAM_ERROR;
    }

    return 0;
}

/*************** LOCAL FUNCTIONS **********************************************/

/* Function: executeCommand =====================================================
 * Abstract:
 *  Executes the command that has been passed to the function. It could be real-time
 *  or queued. If it is a real-time command, it comes directly from enqueueCommand.
 *  If it is a queue command, it would be recursively invoked by op_macro_run.
 */
static void executeCommand(Command * command)
{
    unsigned long i;
    const unsigned long arg = command->arg0;

    /* Modify this switch statement to implement a command */
    switch ( command->opcode )
    {

        /** Real time commands **/

        /* Break the main loop */
    case op_shutdown:
        running = 0;
        break;

        /** Macro commands **/

        /* Invoke sleep() with the incoming argument */
    case op_macro_pause:
        sleep(arg);
        break;

    case op_macro_sendWakeup:
        /* sends a message to indicate the pause is over */
        buff[0] = WAKEUP_MESSAGE;
        blockingIO(SEND, 1);
        break;

        /* Use the 5 byte counter detection mode. Note that incoming data should be in this format. */
    case op_macro_blockRcvSlow:
        blockingIO(RECEIVE, arg);

        if (AckCode == stat_OK)
            checkIncomingDataSlow(buff, arg);
        break;

        /* Do detection with the 1 byte counter. Note that the incoming data should be in this format. */
    case op_macro_blockRcvFast:
        rtiostrInBlockRcvFast = 1;
        blockingIO(RECEIVE, arg);
        rtiostrInBlockRcvFast = 0;

        if (AckCode == stat_OK)
        {

            for (i = 0; i < arg; i++)
            {
                if (buff[i] != data_counter % 256)
                {
                    AckCode = stat_DataIntegrity_Fail;
                    AckArg0 = data_counter;
                    return;
                }
                data_counter++;
            }
        }

        break;

        /* Generates bytes in the 5 byte counter format and sends them to client. */
    case op_macro_blockSendSlow:

        /* If first data, initialize */
        if (last_sync_pos == FIRST_INVALID_COUNTER_VALUE)
        {
            last_sync_pos = 0;
            storage_counter = 5;
        }

        /* Send whatever is needed */
        for (i = 0; i < arg; i++)
        {

            /* Generate array to be used if needed */
            if (storage_counter == 5)
            {
                generateCounter(last_sync_pos, counter_storage_arr);
                storage_counter = 0;
                last_sync_pos += 5;
            }

            buff[i] = counter_storage_arr[storage_counter];
            data_counter++;
            storage_counter++;
        }

        blockingIO(SEND, arg);
        break;

        /* Generates data in the 1 byte counter format and sends them to the client */
    case op_macro_blockSendFast:

        for (i = 0; i < arg; i++)
        {
            buff[i] = data_counter % 256;
            data_counter++;
        }
        rtiostrInBlockSendFast = 1;
        blockingIO(SEND, arg);
        rtiostrInBlockSendFast = 0;
        break;

        /* The client uses this method to determine whether device blocks on receive */
    case op_blockingReceive_test:
        /* try to receive a single MemUnit; host will timeout if rtIOStreamRecv
         * is blocking */
        {
           MemUnit_T tmp;
           size_t numRecvd = 0;
           const int errorOccurred = rtIOStreamRecv(streamID, &tmp, sizeof(MemUnit_T), &numRecvd);
           if (errorOccurred != RTIOSTREAM_NO_ERROR)
           {
              if (AckCode == stat_OK) {
                 AckCode = stat_RTIOSTREAM_ERROR;
                 AckArg0 = data_counter;
              }
              return;
           }
        }
        break;
        /* The target sends its BUFFER_SIZE back to the client */
    case op_getBufferSize:
        AckArg0 = BUFFER_SIZE;
        break;

        /** Macro engine, commands governing the lifetime of a macro **/

        /* Marks the point where a macro repeat should start */
    case op_macro_repeat_start:
        macro_repeat_start_id = macro_execution_counter; /* mark position of the start */
        macro_repeat_count = 0; /* reset repeat */
        break;

        /* Jumps to the position marked by op_macro_repeat_start and increments macro_repeat_count until
         * The macro part has been repeated arg number of times. Nested repeats are not supported! */
    case op_macro_repeat:
        if (++macro_repeat_count < arg) macro_execution_counter = macro_repeat_start_id; /* move execution counter */
        break;

        /* Increments the execution counter, invokes the command pointed by it and upon all commands have been executed resets the macro */
    case op_macro_run:

        /* Run */
        while (AckCode == stat_OK && macro_execution_counter != macro_length)
        {

            /* Get next command and increment counter */
            Command * next = &macro[++macro_execution_counter];

            /* Execute the command */
            executeCommand(next);
        }

        /* Reset macro */
        macro_length = 0;
        macro_execution_counter = 0;
        macro_repeat_count = 0;
        macro_repeat_start_id = 0;

        /* Reset data_counter checks */
        data_counter = 0;
        storage_counter = 0;
        last_sync_pos = FIRST_INVALID_COUNTER_VALUE;

        break;

        /** Catch errors **/

    default:
        AckCode = stat_UnknownCommand;
        AckArg0 = command->opcode;
        break;

    }
}

/* Function: enqueueCommand =====================================================
 * Abstract:
 *  Appends command to the current macro sequence.
 */
static void enqueueCommand(Command * command)
{

    /* Reset */
    macro_execution_counter = 0;
    macro_repeat_start_id = 0;
    macro_repeat_count = 0;

    /* Copy command to memory */
    macro[++macro_length] = *command;
}

/* Function: receiveCommand =====================================================
 * Abstract:
 *  Handles receiving of a command from the client and decides whether it should be stored
 *  in a macro for later execution or instead should be immediately run.
 */
static void receiveCommand(void)
{
    Command comm; /* Create a command */

    /* Get the op-code. */
    blockingIO(RECEIVE, 6);

    /* Fill in memory */
    comm.opcode = buff[0];
    comm.arg0 = (((unsigned long) buff[1]) & 0xFF) | ((((unsigned long) buff[2]) & 0xFF) << 8) | ((((unsigned long) buff[3]) & 0xFF) << 16) | ((((unsigned long) buff[4]) & 0xFF) << 24);

    /* Check checksum */
    if (buff[5] != (buff[0] ^ buff[1] ^ buff[2] ^ buff[3] ^ buff[4]))
    {
        AckCode = stat_CommandChecksum_Fail;
        AckArg0 = comm.opcode;
        sendAck();
        return;
    }

    /* Decide where to redirect the newly received command */
    switch ( comm.opcode )
    {

        /** Commands that have to be executed immediately **/

    case op_shutdown:
    case op_macro_run:
    case op_blockingReceive_test:
    case op_getBufferSize:

        executeCommand(&comm); /* Immediately execute */
        break;

        /** Commands that need to be queued **/

    case op_macro_blockRcvSlow:
    case op_macro_blockRcvFast:
    case op_macro_repeat_start:
    case op_macro_repeat:
    case op_macro_blockSendSlow:
    case op_macro_blockSendFast:
    case op_macro_pause:
    case op_macro_sendWakeup:

        enqueueCommand(&comm); /* Add to queue */
        break;

        /** Catch errors **/

    default:

        AckCode = stat_UnknownCommand;
        AckArg0 = comm.opcode;
    }

    /* Send ACK. This is always executed after a command has finished and sends
     * the AckCode and AckArg0 set by the execution back to the client. */
    sendAck();
}

/* Function: handshake =====================================================
 * Abstract: Does the initialization of the communication between the client
 *  and the target. It allows the client to deduce smallest addressable memory
 *  chunk size, Endianess and size of different data types of the target.
 *
 *  The target starts broadcasting a beacon. It contains enough information
 *  to make it possible to deduce the chunk size and Endianess of the target.
 *  The beacon is one byte with value 1, 2, 4 or 8 for little Endian targets or
 *  101, 102, 104, 108 for big Endian targets where last digit indicates the
 *  number of bytes per memUnit (per char). It is up to the client to ignore 0s
 *  introduced by sending data from non-byte addressable target.
 *
 *  After beacon is received, the target waits for the client to free its buffer
 *  from any leftover beacons and then sends 7 chunks containing the sizes of
 *  the different datatypes.
 */
static void handshake(void)
{
    MemUnit_T beacon;
    unsigned int param_isLittleEndian; /* 1 for LE, 0 for BE */
    int stopReceived = 0;
    int invalidDataReceived = 0;
    int memUnitScaling;

    /* Detect Endianess */
    union
    {
        long lnum;
        unsigned char cnum;
    } test;
    test.lnum = 1;
    param_isLittleEndian = test.cnum == 1;
#ifdef BIG_ENDIAN_TESTING
    param_isLittleEndian = 0;
#endif

    /* Send beacon */
    beacon = param_isLittleEndian ? MEM_UNIT_BYTES : 100 + MEM_UNIT_BYTES;
    while (!stopReceived) {
        /* send data to MATLAB */
        buff[0] = beacon;
        blockingIO(SEND, 1); /* assume sends complete without error during handshake */

        /* Receive instruction from MATLAB */
        blockingIO(RECEIVE, 1);
        if (AckCode == stat_RTIOSTREAM_ERROR) {
           /* forward unrecoverable rtiostream error to MATLAB */
           beacon = HS_RTIOSTREAM_RECV_ERROR;
           AckCode = stat_OK; /* clear error status */
        }
        else {
           /* Process instruction */
           switch (buff[0] & 0xFF) {
              case HS_REPEAT:
                 /* repeat */
                 break;
              case HS_STOP:
                 /* stop byte received - we're done */
                 stopReceived = 1;
                 break;
              default:
                 /* flag receipt of invalid data
                  * this may be owing to incorrect driver initialization
                  * behaviour */
                 invalidDataReceived = 1;
                 break;
           }
        }
    }
    /* send ack */
    if (invalidDataReceived) {
       buff[0] = HS_INCORRECT_DATA_RECVD;
    }
    else {
       buff[0] = HS_STOP_ACK;
    }
    blockingIO(SEND, 1);

    /* Send sizes of other types to client */
#ifdef HOST_WORD_ADDRESSABLE_TESTING
   memUnitScaling = 1;
#else
   memUnitScaling = MEM_UNIT_BYTES;
#endif

    buff[0] = (MemUnit_T) (sizeof(char)*memUnitScaling);
    buff[1] = (MemUnit_T) (sizeof(long)*memUnitScaling);
    buff[2] = (MemUnit_T) (sizeof(short)*memUnitScaling);
    buff[3] = (MemUnit_T) (sizeof(float)*memUnitScaling);
    buff[4] = (MemUnit_T) (sizeof(void *)*memUnitScaling);
    buff[5] = (MemUnit_T) (sizeof(int)*memUnitScaling);
    buff[6] = (MemUnit_T) (sizeof(double)*memUnitScaling);
    blockingIO(SEND, 7);
}

/* Function: openServer =====================================================
 * Abstract:
 *  Passes the input arguments to the rtiostream implementation and sets the
 *  streamID. After this call, if successful, the server is assumed to be up
 *  and running.
 */
static int openServer(int rtArgc, void * rtArgv [])
{

    /* Initialize stream */
    streamID = rtIOStreamOpen(rtArgc, rtArgv);

    /*  Check for errors */
    if (streamID == RTIOSTREAM_ERROR)
    {
        return streamID;
    }
    return RTIOSTREAM_NO_ERROR;
}

/* Function: closeServer =====================================================
 * Abstract:
 *  Properly shuts down the server and the current program.
 */
static int closeServer(void)
{
    const int errorOccurred = rtIOStreamClose(streamID);
    /*  Check for errors */
    if (errorOccurred == RTIOSTREAM_ERROR)
    {
        return errorOccurred;
    }
    return RTIOSTREAM_NO_ERROR;
}


/* Function: blockingIO =====================================================
 * Abstract: Sends or receives numMemUnits memory chunks of data.
 */
static void blockingIO(int send, unsigned long numMemUnits)
{
    size_t sizeToTransfer = (size_t) numMemUnits;
    size_t sizeTransferred;
    IOUnit_T * ioPtr = (IOUnit_T *) &buff[0];
    int status;

    /* Detect possible buffer overflow and complain */
    if (numMemUnits > BUFFER_SIZE)
    {
        AckCode = stat_notEnoughSpace;
        AckArg0 = BUFFER_SIZE;
        return;
    }

#ifdef HOST_WORD_ADDRESSABLE_TESTING
   /* map to bytes */
   sizeToTransfer *= MEM_UNIT_BYTES;
#endif

   while (sizeToTransfer > 0) {
      sizeTransferred = 0;
      /* Do the low level call */
      status = send ?
         rtIOStreamSend(streamID, ioPtr, sizeToTransfer, &sizeTransferred) :
         rtIOStreamRecv(streamID, ioPtr, sizeToTransfer, &sizeTransferred);

      /* Throw RTIOSTREAM error */
      if (status != RTIOSTREAM_NO_ERROR) {
         if (AckCode == stat_OK) {
            AckCode = stat_RTIOSTREAM_ERROR;
            AckArg0 = data_counter;
         }
         return;
      }
      else {
         sizeToTransfer -= sizeTransferred;
         ioPtr += sizeTransferred;
      }
   }
}

/* Function: sendAck =====================================================
 * Abstract:
 *  Sends an acknowledgment to the server.
 */
static void sendAck(void)
{
    /* Send the op-code and the arguments */
    buff[0] = AckCode;
    buff[1] = AckArg0 & 0xFF;
    buff[2] = (AckArg0 >> 8) & 0xFF;
    buff[3] = (AckArg0 >> 16) & 0xFF;
    buff[4] = (AckArg0 >> 24) & 0xFF;
    buff[5] = buff[0] ^ buff[1] ^ buff[2] ^ buff[3] ^ buff[4];

    /* Send */
    blockingIO(SEND, 6);

    /* Reset ACK data */
    AckCode = stat_OK;
    AckArg0 = 0;
}

/* Function: oneSleepTic =====================================================
 * Abstract: Does a tic - the smallest pause that could be done.
 */
static void oneSleepTic(void)
{
    volatile int k, j;

    for(k=0; k<SLEEP_COEFF; k++)
        for(j=0; j<SLEEP_COEFF; j++);
}

/* Function: sleep =====================================================
 * Abstract: Runs the sleeping loop certain amount of times
 */
static void sleep(unsigned long amount)
{
    volatile unsigned long iterations = amount;

    while (--iterations) oneSleepTic();
}

/* Function: generatePacket =====================================================
 * Abstract: Gets a byteValue given byteId
 *
 * _Z_____Z+1___Z+2___Z+3___Z+4___ _ (byteId   )
 *  | 255 |  x1 |  x2 |  x3 |  x4 |  (byteValue)
 * _|_____|_____|_____|_____|_____|_
 *
 * x1, x2, x3, x4 are numbers between 0 and 254 inclusive
 *
 * if you calculate (254^0)*x1+(254^1)*x2+(254^2)*x3+(254^3)*x4 = Z
 * Z should be the id of the byte 255. Z must have the property Z % 5 == 0
 */
static void generateCounter(unsigned long Z, MemUnit_T array[5])
{
    int i;

    array[4] = 255; /* The padding */

    for (i = 0; i < 4; i++, Z /= 255)
        array[i] = Z % 255;
}

/* Function: parseCounter =====================================================
 * Abstract: Reads a counter from a given array. array[0] should be 255. For
 * array structure, refer to generatePacket. Z = 0 for the incoming array.
 */
static unsigned long parseCounter(MemUnit_T array[5])
{
    unsigned long value = array[0];
    unsigned long multiplier = 1;
    int i;

    /* Check whether the values are valid */
    if (array[4] != 255) return FIRST_INVALID_COUNTER_VALUE + 4;
    if (array[0] == 255) return FIRST_INVALID_COUNTER_VALUE;

    for (i = 1; i < 4; value += array[i]*(multiplier *= 255), i++)
        if (array[i] == 255)  /* Check whether value is valid */
            return FIRST_INVALID_COUNTER_VALUE + i;

    return value;
}

/* Function: validateDataitem =====================================================
 * Abstract: A function that accepts one byte at a time of sequential data that must
 *  contain a 5 byte (slow) counter (generated with a function similar to generateCounter).
 *  It detects errors because of dropped/scrambled packets at end or at beginning of
 *  the stream. It relies on the variable "data_counter" to provide the id of the chunk
 *  that is being passed as an argument. It uses different variables to keep track
 *  of what value is expected next and check whether it was actually received.
 */
static void validateDataitem(const MemUnit_T data)
{
    /* This works a bit like a finite state machine. It has three states:
        1. It waits for a "padding". Normally the first state would be executed only once, if the
        stream begins with a 255 as it should be. If it does not then data was dropped from the beginning of the stream
        but in order to get how much data was dropped, we need to have a counter value. That's where step 2 comes into place.
        2. The "memory" array is being filled up with data until a "padding" is received. The array is then parsed. Normally
        the first counter will read "0". If it doesn't then data was dropped in the beginning of the stream. If it reads "0", then we
        know that after "0" comes "5" so we fill the "memory" array with the "predicted" counter values that should follow next.
        3. We check whether the incoming data matches the prediction. When a "padding" byte is received, a new prediction is made.
    */

    /* Initialize */
    if (last_sync_pos == FIRST_INVALID_COUNTER_VALUE) /* If we start from scratch */
    {
        last_sync_pos = 0; /* The next counter should be 0 */
        dataCheckState = 1; /* Set state 0 */
        storage_counter = 0; /* We are at position 0 */
    }

    switch (dataCheckState)
    {
    case 1: /* In this state we wait for our first padding */

        /* No sync byte found */
        if (storage_counter >= SIZE_OF_COUNTER)
        {
            AckCode = stat_DataIntegrity_Fail;
            AckArg0 = 0; /* We really have no clue on what happened */
            return;
        }

        storage_counter++; /* Count number of ignored bytes */

        if (data  == SYNC_BYTE_VALUE)
        {
            storage_counter = 0;
            dataCheckState = 2; /* Skip to state 2 */
        }
        return;

    case 2: /* Here we wait for our first counter */

        /* If no sync byte found ever */
        if (storage_counter >= SIZE_OF_COUNTER)
        {
            AckCode = stat_DataIntegrity_Fail;
            AckArg0 = 0; /* We really have no clue on what happened */
            return;
        }

        counter_storage_arr[storage_counter++] = data;

        if (data == SYNC_BYTE_VALUE)
        {
            /* We have received our first packet */
            unsigned long current_sync_pos = parseCounter(counter_storage_arr);

            /* If there was an error parsing the value */
            if (current_sync_pos >= FIRST_INVALID_COUNTER_VALUE)
            {
                AckCode = stat_DataIntegrity_Fail;
                AckArg0 = current_sync_pos - FIRST_INVALID_COUNTER_VALUE + 1;
                return;
            }

            /* The first ever number received was not 0! Data was dropped from the beginning of the stream! */
            if (current_sync_pos != 0)
            {
                AckCode = stat_DataIntegrity_Fail;
                AckArg0 = current_sync_pos - data_counter + SIZE_OF_COUNTER - 1;
                return;
            }

            /* Calculate the predicted next burst of data */
            last_sync_pos += SIZE_OF_COUNTER;
            generateCounter(last_sync_pos, counter_storage_arr);

            storage_counter = 0; /* Reset counter */

            dataCheckState = 3; /* Do normal checking */
        }
        return;

    case 3: /* Here we compare received values against predicted values */

        if (data != counter_storage_arr[storage_counter++])
        {
            /* If unexpected byte received */
            AckCode = stat_DataIntegrity_Fail;
            AckArg0 = data_counter + 1;
            return;
        }

        if (data == SYNC_BYTE_VALUE)
        {
            /* Generate next expected byte */
            last_sync_pos += SIZE_OF_COUNTER;
            generateCounter(last_sync_pos, counter_storage_arr);

            storage_counter = 0; /* Reset counter */
        }
        return;
    }
}

/* Function: checkIncomingDataSlow =====================================================
 * Abstract: Splits the data into individual words to be checked by validateDataitem and
 *  takes care of incrementing the data_counter so that the validateDataitem functions properly.
 */
/* Data types that would be used */
static void checkIncomingDataSlow(MemUnit_T * start, int length)
{
    int i;

    for (i = 0; i < length; i++)
    {
        const MemUnit_T data = start[i];

        /* Validate current byte */
        validateDataitem(data);

        /* Throw error if needed */
        if (AckCode != stat_OK)
            return;

        /* Calculate the position of the next memunit in the stream */
        data_counter++;
    }

}
