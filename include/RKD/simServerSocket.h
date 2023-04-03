// Author: Tommaso Pardi
// Date: 07/03/2020

#ifndef _simServerSocket_header_
#define _simServerSocket_header_

// Socket Lib
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <thread>
#include <memory>
#include <list>
#include <unordered_map>
#include <mutex>

using namespace boost::asio;
using ip::tcp;

namespace RKD{

enum class MsgType{
    ACK,
    NOACK,
    END};

class simServerSocket{
public:
	simServerSocket();
	~simServerSocket() = default;

protected:

    // Attributes
    boost::asio::io_service service_;
    std::unique_ptr<boost::asio::ip::tcp::acceptor> acceptorPtr_{nullptr}; 
    bool verbose_{false};
    std::list<std::string> msg_in_;
    std::mutex mux_;
    const int port_{8305};
    std::unordered_map<const MsgType, const std::string> msgTypeMap_ = {{MsgType::ACK, "ACK-EndMSG"}, {MsgType::NOACK, "NOACK-EndMSG"}, {MsgType::END, "-EndMSG"}};

    //Methods
    /* /brief Function for waiting for a connection
     *
     * This function is used to wait for a connection from a client.
     *
     * \return void
     *
     */

    void waitingForConnection();

    // Virtual function for handling the socket message
    /* /brief Virtual function for handling the socket message
     *
     * This function is used to handle the message received from the socket.
     *
     * \param clientSock The socket used to communicate with the client
     * \return void
     *
     * */
    virtual void HandleRequest(boost::asio::ip::tcp::socket&& clientSock);

    /* /brief Read from the socket a msg
     *
     * This function is used to read a message from the socket.
     *
     * \param socket The socket used to communicate with the client
     * \return std::string The message read from the socket
     *
     * */
    std::string readFromSocket(tcp::socket& socket);

    /* /brief Send to the socket a msg
     *
     * This function is used to send a message to the socket.
     *
     * \param socket The socket used to communicate with the client
     * \param msg The message to send to the socket
     * \return void
     *
     * */
    void sendToSocket(tcp::socket& socket, const std::string& msg);

    /* /brief Send an Acknowledgement
     *
     * This function is used to send an Acknowledgement to the socket.
     *
     * \param socket The socket used to communicate with the client
     * \param msg The message to send to the socket
     * \return void
     *
     * */
    void sendACK(tcp::socket& socket, const std::string& msg);

};
}


#endif
