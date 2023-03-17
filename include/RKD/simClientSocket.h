// Author: Tommaso Pardi
// Date: 07/03/2020

#ifndef _simClientSocket_header_
#define _simClientSocket_header_


#define PORT 8305
#define LOCALHOST "127.0.0.1"
#define END_MSG "-EndMSG"
#define VERBOSE false

// Socket Lib
#include <iostream>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>

// for convenience
using json = nlohmann::json;

using namespace boost::asio;
using ip::tcp;

namespace RKD{

class simClientSocket{
public:
	simClientSocket();
	~simClientSocket(){}

	bool sendMSG(const std::string&);

protected:
	//Storage
	boost::asio::io_service service_;
	bool verbose_{VERBOSE};

	//Methods
	void waitingForConnection();
	tcp::socket* socketPtr_;

};
}


#endif

