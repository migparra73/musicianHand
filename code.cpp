#include <rt-bridge.h>

#include <stdio.h>
#include <quickDAQ.h>

int main(void)
{

    void *context = zmq_ctx_new();
    void *pub = rtb_initPub(context, (char *)"tcp://*:5555");
    // void* sub = rtb_connectSub(context, (char*)"tcp://169.254.115.243:5557"); //Tim Laptop
    void *sub = rtb_connectSub(context, (char *)"tcp://169.254.101.187:5558"); // Hesam's Laptop
    messagefloat received;

    quickDAQinit();

    uInt32 DO1 = 0x000000ff;

    float64 MTR[3] = {0.0, 0.0, 0.0};                                                    // motors
    float64 encoderValues[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};                           // Set 6 encoders, but reads from second encoder o
    float EncValSend[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // RT bridge

    const float64 muscleTone = 0.2;
    const float maxNewt = 3.1;
    const float maxVolt = 1;

    pinMode(5, ANALOG_IN, 0); //
    int i = 0;
    for (i = 0; i < 2; i++)
    {
        pinMode(2, ANALOG_OUT, i); //  out of computer to motor PinMode(A,B,C) A: slot or card number, B: output, C: pin number
    }
    for (i = 0; i < 6; i++)
    {
        pinMode(3, CTR_ANGLE_IN, i); // into computer from the encoders
    }
    pinMode(2, DIGITAL_OUT, 0); // open the channel to motor

    setSampleClockTiming((samplingModes)HW_CLOCKED, DAQmxSamplingRate, DAQmxClockSource, (triggerModes)DAQmxTriggerEdge, DAQmxNumDataPointsPerSample, TRUE);
    printf("\nIO timeout is %f\n", DAQmxDefaults.IOtimeout);

    signal(SIGINT, rtb_signalHandler);
    // float x = 0.0;
    // float y = 0.0;
    // float z = 0.0;
    // float a = 0.0;

    float64 AI;

    printf("Press enter to start DAQ\n\n");
    getchar();

    quickDAQstart();
    syncSampling();

    writeDigital(2, &DO1);
    writeAnalog(2, &(MTR[0]));
    printf("Motor Enabled");

    readAnalog(5, &AI);
    for (i = 0; i < 4; i++)
    {
        MTR[i] = muscleTone;
    }
    writeAnalog(2, &(MTR[0]));

    printf("\n\n");

    while (rtb_isLooping() == 1)
    {
        rtb_receiveMsg(sub);
        received = getLast();
        MTR[0] = received.x;
        MTR[1] = received.y;
        MTR[2] = received.z;
        // MTR[3] = received.a;

        for (i = 0; i < 3; i++)
        {
            MTR[i] = (MTR[i] * maxVolt); // +muscleTone;
        }

        writeAnalog(2, &(MTR[0]));
        printf("Motors: %2.3f %2.3f %2.3f %2.3f\r", MTR[0], MTR[1], MTR[2], MTR[3]);

        for (i = 2; i < 5; i++)
        {
            readCounterAngle(3, i, &(encoderValues[i]));
            EncValSend[i - 2] = encoderValues[i];
        }
        rtb_publishMsg(pub, EncValSend);
        // printf("Angle: %5.2f %5.2f %5.2f %5.2f\r", encoderValues[2], encoderValues[3], encoderValues[4], encoderValues[5]);
        readAnalog(5, &AI);
        syncSampling();
    }
    printf("Closing out");
    for (i = 0; i < 4; i++)
    {
        MTR[i] = 0;
    }
    writeAnalog(2, &(MTR[0]));
    printf("Motors wound down\n");

    DO1 = 0x00000000;
    writeDigital(2, &DO1);
    printf("Motors disabled\n\n");

    syncSampling();
    readAnalog(5, &AI);
    quickDAQstop();

    quickDAQTerminate();
    return 0;
}