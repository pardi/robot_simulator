#include <RKD/simServerSocket.h>


using namespace RKD;

simServerSocket::simServerSocket(){

	acceptorPtr_ = std::make_unique<boost::asio::ip::tcp::acceptor>(service_, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port_));

	// Start the thread
    std::thread{std::bind(&simServerSocket::waitingForConnection, this)}.detach();
}

void simServerSocket::HandleRequest(boost::asio::ip::tcp::socket&& clientSock){

	std::string	msg_in = readFromSocket(clientSock);

	if (msg_in.empty()){
		sendToSocket(clientSock, c_msgTypeMap[MsgType::NOACK]);
	}
	else{
		sendToSocket(clientSock, c_msgTypeMap[MsgType::ACK]);
        {
            const std::lock_guard <std::mutex> lock(mux_);
            // Store new message
            msg_in_.push_back(msg_in);
        }
	}
}

void simServerSocket::waitingForConnection(){

	boost::asio::ip::tcp::socket clientSock (service_);
	acceptorPtr_->accept(clientSock); //new socket accepted

   	while(true){
		// Check if the Socket is available
		if (clientSock.available()){
			HandleRequest(std::move(clientSock));

		    std::this_thread::sleep_for (std::chrono::microseconds(100));
		}
	}	
}

std::string simServerSocket::readFromSocket(tcp::socket& socket) {

	boost::asio::streambuf buf;

	boost::system::error_code error;

	boost::asio::read_until(socket, buf, c_msgTypeMap[MsgType::END], error);

	std::string msg = boost::asio::buffer_cast<const char*>(buf.data());

	//  Erase the last character 
  	msg.erase(msg.end() - 7, msg.end());

	if( error && error != boost::asio::error::eof ) {
        msg.clear();
    }

	return msg;
}

void simServerSocket::sendToSocket(tcp::socket & socket, const std::string& message) {
	boost::asio::write(socket, boost::asio::buffer(message + "\n"));
}

void simServerSocket::sendACK(tcp::socket& socket, const std::string& msg){
	if (msg == "") {
        sendToSocket(socket, c_msgTypeMap[MsgType::NOACK]);
    }
	else {
        sendToSocket(socket, c_msgTypeMap[MsgType::ACK]);
    }
}