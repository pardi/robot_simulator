// Author: Tommaso Pardi
// Date: 07/03/2020

#ifndef _simClientSocket_header_
#define _simClientSocket_header_

// Socket Lib
#include <iostream>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>
#include <RKD/simServerSocket.h>

// for convenience
using json = nlohmann::json;

using namespace boost::asio;
using ip::tcp;

namespace RKD{

class simClientSocket{
public:
	simClientSocket(const std::string& address = "127.0.0.1", int port = 8305);
	~simClientSocket() = default;

	bool sendMSG(const std::string&);

protected:
	//Storage
	boost::asio::io_service service_;
	bool verbose_{VERBOSE};
    int port_{8305};
    std::string address_{"127.0.0.1"};

	//Methods
	void waitingForConnection();
	std::unique_ptr<tcp::socket> socketPtr_{nullptr};

};
}


#endif

