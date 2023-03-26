#include <RKD/simServerSocket.h>


using namespace RKD;

simServerSocket::simServerSocket(){

	acceptorPtr_ = std::make_unique<boost::asio::ip::tcp::acceptor>(service_, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), PORT));

	// Start the thread
    std::thread{std::bind(&simServerSocket::waitingForConnection, this)}.detach();
}

void simServerSocket::HandleRequest(boost::asio::ip::tcp::socket&& clientSock){


	std::string	msg_in = readFromSocket(clientSock);

	if (msg_in.empty()){
		sendToSocket(clientSock, NOACK_MSG);
	}
	else{
		sendToSocket(clientSock, ACK_MSG);
	
		mux_.lock();	
		// Store new message
		msg_in_.push_back(msg_in);
		mux_.unlock();
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

std::string simServerSocket::readFromSocket(tcp::socket & socket) {

	boost::asio::streambuf buf;

	boost::system::error_code error;

	boost::asio::read_until( socket, buf, END_MSG, error);

	std::string msg = boost::asio::buffer_cast<const char*>(buf.data());

	//  Erase the last character 
  	msg.erase(msg.end() - 7, msg.end());

	if( error && error != boost::asio::error::eof )
		msg.clear();


	return msg;
}

void simServerSocket::sendToSocket(tcp::socket & socket, const std::string& message) {
	const std::string msg = message + "\n";
	boost::asio::write( socket, boost::asio::buffer(message) );
}


void simServerSocket::sendACK(tcp::socket & socket, const std::string& msg){
	if (msg == "")
		sendToSocket(socket, NOACK_MSG);
	else
		sendToSocket(socket, ACK_MSG);
}