#include "Mocap.hpp"

namespace rexlab {

void Mocap::SetLocalAddress(const std::string &addr) {
    _local_address = addr;
}

void Mocap::SetServerAddress(const std::string &addr) {
    _server_address = addr;
}

int Mocap::ConnectClient() {
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

}  // namespace rexlab