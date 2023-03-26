// Author: Tommaso Pardi
// Date: 07/03/2020

#ifndef _simServerSocket_header_
#define _simServerSocket_header_


#define PORT 8305
#define END_MSG "-EndMSG"
#define ACK_MSG "ACK-EndMSG"
#define NOACK_MSG "NOACK-EndMSG"
#define VERBOSE false


// Socket Lib
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <thread>
#include <memory>
#include <list>

using namespace boost::asio;
using ip::tcp;

namespace RKD{

class simServerSocket{
public:
	simServerSocket();
	~simServerSocket(){};

protected:
    //Storage
    boost::asio::io_service service_;
    std::unique_ptr<boost::asio::ip::tcp::acceptor> acceptorPtr_{nullptr}; 
    bool verbose_{VERBOSE};
    std::list<std::string> msg_in_;
    boost::mutex mux_;

    //Methods
    // Wait for a new connection
    void waitingForConnection();

    // Virtual function for handling the socket message
    virtual void HandleRequest(boost::asio::ip::tcp::socket&&);

    // Read from the socket a msg
    std::string readFromSocket(tcp::socket&);

    // Send to the socket a msg
    void sendToSocket(tcp::socket&, const std::string&);

    // Send an Acknowledgement
    void sendACK(tcp::socket &, const std::string&);

};
}


#endif
