#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
using namespace std;

#include <NIDAQmx.h>
#include <MCSControl.h>
#include <time.h>
#include <Windows.h>

inline void handleError(int err)
{
	if (err == 0)
		return;

	char error[1024];
	DAQmxGetErrorString(err, error, 1024);
	cout << error << endl;
}

void PrintMcsError(SA_STATUS st)
{
	printf("MCS error %u\n", st);
}

void ExitIfError(SA_STATUS st)
{
	if (st != SA_OK) {
		PrintMcsError(st);
		exit(1);
	}
}

void setup(SA_INDEX& mcsHandle_d, unsigned int& numOfChannels_d, SA_INDEX& channel1_d, SA_INDEX& channel2_d)
{
	/* open the MCS with USB interface in asyncronous communication mode. We use async mode to allow multiple channels to receive and execute commands simultaneously */
	ExitIfError(SA_OpenSystem(&mcsHandle_d, "usb:ix:0", "async"));

	/*Senses number of channels with actuators attached*/
	ExitIfError(SA_GetNumberOfChannels(mcsHandle_d, &numOfChannels_d));
	printf("Number of Channels: %u\n", numOfChannels_d);

	ExitIfError(SA_SetSensorEnabled_A(mcsHandle_d, SA_SENSOR_ENABLED));
	printf("Sensors are enabled\n");

	//ExitIfError(SA_FindReferenceMark_A(mcsHandle_d, channel1_d, SA_BACKWARD_DIRECTION, 0, SA_AUTO_ZERO));
	//printf("Channel u1: Reference mark found\n");
	//ExitIfError(SA_FindReferenceMark_A(mcsHandle_d, channel2_d, SA_BACKWARD_DIRECTION, 0, SA_AUTO_ZERO));
	//printf("Channel u2: Reference mark found\n");

	ExitIfError(SA_Stop_A(mcsHandle_d, channel1_d));
	ExitIfError(SA_Stop_A(mcsHandle_d, channel2_d));
	printf("All stopped\n");

	ExitIfError(SA_SetClosedLoopMoveSpeed_A(mcsHandle_d, channel1_d, 2000000));
	ExitIfError(SA_SetClosedLoopMoveSpeed_A(mcsHandle_d, channel2_d, 2000000));
	printf("Speed control set\n");

	ExitIfError(SA_SetClosedLoopMoveAcceleration_A(mcsHandle_d, channel1_d, 20000));
	ExitIfError(SA_SetClosedLoopMoveAcceleration_A(mcsHandle_d, channel2_d, 20000));
	printf("Acceleration control set\n");

	ExitIfError(SA_SetChannelProperty_A(mcsHandle_d, channel1_d, SA_EPK(SA_GENERAL, SA_LOW_VIBRATION, SA_OPERATION_MODE), SA_ENABLED));
	ExitIfError(SA_SetChannelProperty_A(mcsHandle_d, channel2_d, SA_EPK(SA_GENERAL, SA_LOW_VIBRATION, SA_OPERATION_MODE), SA_ENABLED));
	printf("Low vibration mode on\n");

	ExitIfError(SA_SetBufferedOutput_A(mcsHandle_d, SA_BUFFERED_OUTPUT));
	ExitIfError(SA_FlushOutput_A(mcsHandle_d));
}

void close(SA_INDEX& mcsHandle_d, unsigned int& numOfChannels_d, SA_INDEX& channel1_d,
	int& stop_d, SA_PACKET& packet_d, int chanXcount_d, int chanXStopped_d)
{
	ExitIfError(SA_Stop_A(mcsHandle_d, channel1_d));
	ExitIfError(SA_CloseSystem(mcsHandle_d));
	system("pause");
}

void delay(int num_sec)
{
	int mil_sec = 1000 * num_sec;
	int start_time = clock();
	while (clock() < start_time + mil_sec)
		;
}

// Gets current u1 position
int getu1Pos(SA_INDEX& mcsHandle_d, SA_INDEX& channel1_d, SA_PACKET& packet_d)
{
	ExitIfError(SA_GetPosition_A(mcsHandle_d, channel1_d));
	ExitIfError(SA_ReceiveNextPacket_A(mcsHandle_d, 1000, &packet_d));
	//std::cout << packet_d.data2 << "\n";
	return packet_d.data2;
	
}

int main()
{
	/*Assigns names to each channel 0 through 2*/
	SA_INDEX mcsHandle;
	unsigned int numOfChannels = 2;
	SA_INDEX channel1 = 0;
	SA_INDEX channel2 = 1;
	int stop = 0;
	int a = 0;
	int chanXStopped = 0;
	int chanXcount = 0;
	SA_PACKET packet;

	setup(mcsHandle, numOfChannels, channel1, channel2);
	//	DAQmxGetSysDevNames( devNameOut, 1024 );
	//	DAQmxAddNetworkDevice( "192.168.1.50", devName.c_str(), true, 1.0, devNameOut, bufferSize );

	TaskHandle handle;
	handleError(DAQmxCreateTask("Task1", &handle));
	//	handleError( DAQmxCreateLinScale( "Scaling1", 0.3, 0.5, DAQmx_Val_Volts, "Volts" ) );
	//	handleError( DAQmxCreateLinScale( "Scaling2", 0.7, 1.1, DAQmx_Val_Volts, "Volts" ) );
	handleError(DAQmxCreateLinScale("Scaling1", 1, 0, DAQmx_Val_Volts, "Volts"));
	handleError(DAQmxCreateAIVoltageChan(handle, "Dev1/ai0", "channel1", DAQmx_Val_RSE, 0.0, 9.0, DAQmx_Val_FromCustomScale, "Scaling1"));
	handleError(DAQmxCfgSampClkTiming(handle, 0, 100.0, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 100));
	handleError(DAQmxStartTask(handle));

	std::ofstream voltageFile;
	voltageFile.open("voltages.csv");
	voltageFile << "Voltage,Distance, \n";

	float64 pos = 0;

	float64 value;

	time_t time1 = time(NULL);

	ExitIfError(SA_GotoPositionAbsolute_A(mcsHandle, channel1, -1000000000, 1));

	//get rid of garbage data
	for (int i = 0; i < 3; i++)
		getu1Pos(mcsHandle, channel1, packet);

	ExitIfError(SA_FlushOutput_A(mcsHandle));
	
	int count = 0;
	int zero = 0;
	int dist = 0;

	while (dist < 20000000)
	{
		handleError(DAQmxReadAnalogScalarF64(handle, 10.0, &value, 0));
		pos = getu1Pos(mcsHandle, channel1, packet);
		if (count == 0) {
			zero = pos;
		}

		ExitIfError(SA_FlushOutput_A(mcsHandle));

		dist = -(pos - zero);
		std::cout << value << ", " << dist << "\n";

		voltageFile << to_string(value) + "," + to_string(dist) + ",\n";

		count++;
	}
	
	ExitIfError(SA_Stop_A(mcsHandle, channel1));
	ExitIfError(SA_FlushOutput_A(mcsHandle));
	cout << "*" << getu1Pos(mcsHandle, channel1, packet) << "," << pos;

	close(mcsHandle, numOfChannels, channel1, stop, packet, chanXcount, chanXStopped);
	voltageFile.close();

	return 0;
}