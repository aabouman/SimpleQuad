/* 
Copyright © 2012 NaturalPoint Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */


/*

SampleClient.cpp

This program connects to a NatNet server, receives a data stream, and writes that data stream
to an ascii file.  The purpose is to illustrate using the NatNetClient class.

Usage [optional]:

    SampleClient [ServerIP] [LocalIP] [OutputFilename]

    [ServerIP]			IP address of the server (e.g. 192.168.0.107) ( defaults to local machine)
    [OutputFilename]	Name of points file (pts) to write out.  defaults to Client-output.pts

*/

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#   include <conio.h>
#else
#   include <unistd.h>
#   include <termios.h>
#endif

#include <vector>

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

#ifndef _WIN32
// char getch();
#endif
// void _WriteHeader(FILE* fp, sDataDescriptions* pBodyDefs);
// void _WriteFrame(FILE* fp, sFrameOfMocapData* data);
// void _WriteFooter(FILE* fp);
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);    // receives data from the server
void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg);      // receives NatNet error messages
void resetClient();
int ConnectClient();

static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;

NatNetClient* g_pClient = NULL;

// Globals
static const char* SERVER_ADDR = "192.168.1.5";
static const char* LOCAL_ADDR = "192.168.1.63";
sNatNetClientConnectParams g_connectParams;
int g_analogSamplesPerMocapFrame = 0;
sServerDescription g_serverDescription;


int main( int argc, char* argv[] )
{
    // print version info
    unsigned char ver[4];
    NatNet_GetVersion( ver );
    printf( "NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3] );

    // Install logging callback
    NatNet_SetLogCallback( MessageHandler );

    // create NatNet client
    g_pClient = new NatNetClient();

    // set the frame callback handler
    g_pClient->SetFrameReceivedCallback( DataHandler, g_pClient );	// this function will receive data from the server

    // Connect to Server
    if ( argc == 1 ) {
        g_connectParams.serverAddress = SERVER_ADDR; 
        g_connectParams.localAddress = LOCAL_ADDR; 
    } else {
        g_connectParams.connectionType = kDefaultConnectionType;

        if ( argc >= 2 )
        {
            g_connectParams.localAddress = argv[2];
        }

        if ( argc >= 3 )
        {
            g_connectParams.serverAddress = argv[1];
        }
    }

    int iResult;

    // Connect to Motive
    iResult = ConnectClient();
    if (iResult != ErrorCode_OK)
    {
        printf("Error initializing client. See log for details. Exiting.\n");
        return 1;
    }
    else
    {
        printf("Client initialized and ready.\n");
    }


    // Send/receive test request
    void* response;
    int nBytes;
    printf("[SampleClient] Sending Test Request\n");
    iResult = g_pClient->SendMessageAndWait("TestRequest", &response, &nBytes);
    if (iResult == ErrorCode_OK)
    {
        printf("[SampleClient] Received: %s\n", (char*)response);
    }

    // Ready to receive marker stream!
    printf("\nClient is connected to server and listening for data...\n");
    bool bExit = false;
    while ( const int c = getchar() )
    {
        switch(c)
        {
            case 'q':
                bExit = true;		
                break;	
            case 'r':
                resetClient();
                break;	
            case 'p':
                sServerDescription ServerDescription;
                memset(&ServerDescription, 0, sizeof(ServerDescription));
                g_pClient->GetServerDescription(&ServerDescription);
                if(!ServerDescription.HostPresent)
                {
                    printf("Unable to connect to server. Host not present. Exiting.");
                    return 1;
                }
                break;
            case 's':
                {
                printf("\n\n[SampleClient] Requesting Data Descriptions...");
                sDataDescriptions* pDataDefs = NULL;
                iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
                if (iResult != ErrorCode_OK || pDataDefs == NULL)
                {
                    printf("[SampleClient] Unable to retrieve Data Descriptions.");
                }
                else
                {
                    printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions);
                }
                }
                break;
            case 'm':	                        // change to multicast
                g_connectParams.connectionType = ConnectionType_Multicast;
                iResult = ConnectClient();
                if(iResult == ErrorCode_OK)
                    printf("Client connection type changed to Multicast.\n\n");
                else
                    printf("Error changing client connection type to Multicast.\n\n");
                break;
            case 'u':	                        // change to unicast
                g_connectParams.connectionType = ConnectionType_Unicast;
                iResult = ConnectClient();
                if(iResult == ErrorCode_OK)
                    printf("Client connection type changed to Unicast.\n\n");
                else
                    printf("Error changing client connection type to Unicast.\n\n");
                break;
            case 'c' :                          // connect
                iResult = ConnectClient();
                break;
            case 'd' :                          // disconnect
                // note: applies to unicast connections only - indicates to Motive to stop sending packets to that client endpoint
                iResult = g_pClient->SendMessageAndWait("Disconnect", &response, &nBytes);
                if (iResult == ErrorCode_OK)
                    printf("[SampleClient] Disconnected");
                break;
            default:
                break;
        }
        if(bExit)
            break;
    }

    // Done - clean up.
    if (g_pClient)
    {
        g_pClient->Disconnect();
        delete g_pClient;
        g_pClient = NULL;
    }

    return ErrorCode_OK;
}


// Establish a NatNet Client connection
int ConnectClient()
{
    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams );
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting.\n", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // connection succeeded

        void* pResult;
        int nBytes = 0;
        ErrorCode ret = ErrorCode_OK;

        // print server info
        memset( &g_serverDescription, 0, sizeof( g_serverDescription ) );
        ret = g_pClient->GetServerDescription( &g_serverDescription );
        if ( ret != ErrorCode_OK || ! g_serverDescription.HostPresent )
        {
            printf("Unable to connect to server. Host not present. Exiting.\n");
            return 1;
        }
        printf("\n[SampleClient] Server application info:\n");
        printf("Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp, g_serverDescription.HostAppVersion[0],
            g_serverDescription.HostAppVersion[1], g_serverDescription.HostAppVersion[2], g_serverDescription.HostAppVersion[3]);
        printf("NatNet Version: %d.%d.%d.%d\n", g_serverDescription.NatNetVersion[0], g_serverDescription.NatNetVersion[1],
            g_serverDescription.NatNetVersion[2], g_serverDescription.NatNetVersion[3]);
        printf("Client IP:%s\n", g_connectParams.localAddress );
        printf("Server IP:%s\n", g_connectParams.serverAddress );
        printf("Server Name:%s\n", g_serverDescription.szHostComputerName);

        // get mocap frame rate
        ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            float fRate = *((float*)pResult);
            printf("Mocap Framerate : %3.2f\n", fRate);
        }
        else
            printf("Error getting frame rate.\n");

        // get # of analog samples per mocap frame of data
        ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            g_analogSamplesPerMocapFrame = *((int*)pResult);
            printf("Analog Samples Per Mocap Frame : %d\n", g_analogSamplesPerMocapFrame);
        }
        else
            printf("Error getting Analog frame rate.\n");
    }

    return ErrorCode_OK;
}

// DataHandler receives data from the server
// This function is called by NatNet when a frame of mocap data is available
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData)
{
    NatNetClient* pClient = (NatNetClient*) pUserData;

    // Software latency here is defined as the span of time between:
    //   a) The reception of a complete group of 2D frames from the camera system (CameraDataReceivedTimestamp)
    // and
    //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
    //
    // This figure may appear slightly higher than the "software latency" reported in the Motive user interface,
    // because it additionally includes the time spent preparing to stream the data via NatNet.
    const uint64_t softwareLatencyHostTicks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
    const double softwareLatencyMillisec = (softwareLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);

    // Transit latency is defined as the span of time between Motive transmitting the frame of data, and its reception by the client (now).
    // The SecondsSinceHostTimestamp method relies on NatNetClient's internal clock synchronization with the server using Cristian's algorithm.
    const double transitLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->TransmitTimestamp ) * 1000.0;

    int i=0;

    printf("FrameID : %d\n", data->iFrame);
    printf("Timestamp : %3.2lf\n", data->fTimestamp);
    printf("Software latency : %.2lf milliseconds\n", softwareLatencyMillisec);

    // Only recent versions of the Motive software in combination with ethernet camera systems support system latency measurement.
    // If it's unavailable (for example, with USB camera systems, or during playback), this field will be zero.
    const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;

    if ( bSystemLatencyAvailable )
    {
        // System latency here is defined as the span of time between:
        //   a) The midpoint of the camera exposure window, and therefore the average age of the photons (CameraMidExposureTimestamp)
        // and
        //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
        const uint64_t systemLatencyHostTicks = data->TransmitTimestamp - data->CameraMidExposureTimestamp;
        const double systemLatencyMillisec = (systemLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);

        // Client latency is defined as the sum of system latency and the transit time taken to relay the data to the NatNet client.
        // This is the all-inclusive measurement (photons to client processing).
        const double clientLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->CameraMidExposureTimestamp ) * 1000.0;

        // You could equivalently do the following (not accounting for time elapsed since we calculated transit latency above):
        //const double clientLatencyMillisec = systemLatencyMillisec + transitLatencyMillisec;

        printf( "System latency : %.2lf milliseconds\n", systemLatencyMillisec );
        printf( "Total client latency : %.2lf milliseconds (transit time +%.2lf ms)\n", clientLatencyMillisec, transitLatencyMillisec );
    }
    else
    {
        printf( "Transit latency : %.2lf milliseconds\n", transitLatencyMillisec );
    }

    // FrameOfMocapData params
    bool bIsRecording = ((data->params & 0x01)!=0);
    bool bTrackedModelsChanged = ((data->params & 0x02)!=0);
    if(bIsRecording) {
        printf("RECORDING\n");
    }
    if(bTrackedModelsChanged) {
        printf("Models Changed.\n");
    }
    
    // Rigid Bodies
    printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
    for(i=0; i < data->nRigidBodies; i++)
    {
        // params
        // 0x01 : bool, rigid body was successfully tracked in this frame
        bool bTrackingValid = data->RigidBodies[i].params & 0x01;

        printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
        printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
        printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
            data->RigidBodies[i].x,
            data->RigidBodies[i].y,
            data->RigidBodies[i].z,
            data->RigidBodies[i].qx,
            data->RigidBodies[i].qy,
            data->RigidBodies[i].qz,
            data->RigidBodies[i].qw);
    }

}


// MessageHandler receives NatNet error/debug messages
void NATNET_CALLCONV MessageHandler( Verbosity msgType, const char* msg )
{
    // Optional: Filter out debug messages
    if ( msgType < Verbosity_Info )
    {
        return;
    }

    printf( "\n[NatNetLib]" );

    switch ( msgType )
    {
        case Verbosity_Debug:
            printf( " [DEBUG]" );
            break;
        case Verbosity_Info:
            printf( "  [INFO]" );
            break;
        case Verbosity_Warning:
            printf( "  [WARN]" );
            break;
        case Verbosity_Error:
            printf( " [ERROR]" );
            break;
        default:
            printf( " [?????]" );
            break;
    }

    printf( ": %s\n", msg );
}

void resetClient()
{
    int iSuccess;

    printf("\n\nre-setting Client\n\n.");

    iSuccess = g_pClient->Disconnect();
    if(iSuccess != 0)
        printf("error un-initting Client\n");

    iSuccess = g_pClient->Connect( g_connectParams );
    if(iSuccess != 0)
        printf("error re-initting Client\n");
}
