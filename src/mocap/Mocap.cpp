#include "Mocap.hpp"
#include <NatNetTypes.h>

namespace rexlab {

void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg);      // receives NatNet error messages

Mocap::Mocap() {
    unsigned char ver[4];
    NatNet_GetVersion( ver );
    printf( "NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3] );

    // Install logging callback
    NatNet_SetLogCallback( MessageHandler );

    // create NatNet client
    _pClient = new NatNetClient();
}

Mocap::~Mocap() {
    if (_pClient) {
        _pClient->Disconnect();
        delete _pClient;
        _pClient = NULL;
    }
}

void Mocap::SetLocalAddress(const std::string &addr) {
    _local_address = addr;
}

void Mocap::SetServerAddress(const std::string &addr) {
    _server_address = addr;
}

int Mocap::ConnectClient() {
    _connectParams.serverAddress = _server_address.c_str(); 
    _connectParams.localAddress = _local_address.c_str();

    // Release previous server
    _pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = _pClient->Connect( _connectParams );
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
        memset( &_serverDescription, 0, sizeof( _serverDescription ) );
        ret = _pClient->GetServerDescription( &_serverDescription );
        if ( ret != ErrorCode_OK || ! _serverDescription.HostPresent )
        {
            printf("Unable to connect to server. Host not present. Exiting.\n");
            return 1;
        }
        printf("\n[SampleClient] Server application info:\n");
        printf("Application: %s (ver. %d.%d.%d.%d)\n", _serverDescription.szHostApp, _serverDescription.HostAppVersion[0],
            _serverDescription.HostAppVersion[1], _serverDescription.HostAppVersion[2], _serverDescription.HostAppVersion[3]);
        printf("NatNet Version: %d.%d.%d.%d\n", _serverDescription.NatNetVersion[0], _serverDescription.NatNetVersion[1],
            _serverDescription.NatNetVersion[2], _serverDescription.NatNetVersion[3]);
        printf("Client IP:%s\n", _connectParams.localAddress );
        printf("Server IP:%s\n", _connectParams.serverAddress );
        printf("Server Name:%s\n", _serverDescription.szHostComputerName);

        // get mocap frame rate
        ret = _pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            float fRate = *((float*)pResult);
            printf("Mocap Framerate : %3.2f\n", fRate);
        }
        else
            printf("Error getting frame rate.\n");

        // get # of analog samples per mocap frame of data
        ret = _pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            _analogSamplesPerMocapFrame = *((int*)pResult);
            printf("Analog Samples Per Mocap Frame : %d\n", _analogSamplesPerMocapFrame);
        }
        else
            printf("Error getting Analog frame rate.\n");
    }

    return ErrorCode_OK;
}

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

void Mocap::ResetClient()
{
    int iSuccess;

    printf("\n\nre-setting Client\n\n.");

    iSuccess = _pClient->Disconnect();
    if(iSuccess != 0)
        printf("error un-initting Client\n");

    iSuccess = _pClient->Connect( _connectParams );
    if(iSuccess != 0)
        printf("error re-initting Client\n");
}

void Mocap::SendTestRequest() {
    void* response;
    int nBytes;
    printf("[SampleClient] Sending Test Request\n");
    int iResult = _pClient->SendMessageAndWait("TestRequest", &response, &nBytes);
    if (iResult == ErrorCode_OK)
    {
        printf("[SampleClient] Received: %s\n", (char*)response);
    }
}

int Mocap::Run() {
    bool bExit = false;
    int iResult;
    void* response;
    int nBytes;
    while ( const int c = getchar() )
    {
        switch(c)
        {
            case 'q':
                bExit = true;		
                break;	
            case 'r':
                ResetClient();
                break;	
            case 'p':
                sServerDescription ServerDescription;
                memset(&ServerDescription, 0, sizeof(ServerDescription));
                _pClient->GetServerDescription(&ServerDescription);
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
                iResult = _pClient->GetDataDescriptionList(&pDataDefs);
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
                _connectParams.connectionType = ConnectionType_Multicast;
                iResult = ConnectClient();
                if(iResult == ErrorCode_OK)
                    printf("Client connection type changed to Multicast.\n\n");
                else
                    printf("Error changing client connection type to Multicast.\n\n");
                break;
            case 'u':	                        // change to unicast
                _connectParams.connectionType = ConnectionType_Unicast;
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
                iResult = _pClient->SendMessageAndWait("Disconnect", &response, &nBytes);
                if (iResult == ErrorCode_OK)
                    printf("[SampleClient] Disconnected");
                break;
            default:
                break;
        }
        if(bExit)
            break;
    }
    return ErrorCode_OK;
}

}  // namespace rexlab