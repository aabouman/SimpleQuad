#pragma once

#include "string"

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>
#include <pthread.h>

namespace rexlab {

class Mocap {
   public:

    Mocap();
    ~Mocap();
    void SetServerAddress(const std::string& addr);
    void SetLocalAddress(const std::string& addr);
    int ConnectClient();
    void SendTestRequest();
    void ResetClient();
    NatNetClient* GetClient() { return _pClient; };
    const sServerDescription& GetServerDescription() const;
    int Run();

   private:
    std::string _local_address;
    std::string _server_address;
    NatNetClient* _pClient;  
    sNatNetClientConnectParams _connectParams;
    sServerDescription _serverDescription;
    int _analogSamplesPerMocapFrame = 0;

};

}  // namespace rexlab