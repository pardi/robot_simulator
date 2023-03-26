#include <RKD/simClientSocket.h>


using namespace RKD;

bool simClientSocket::sendMSG(const std::string& msg) {

	/*
	*	Send the MSG
	*/

	boost::system::error_code error;

	boost::asio::write( *socketPtr_, boost::asio::buffer(msg + END_MSG), error );
	
	if( error ) {
		if (verbose_)
			std::cout << "Send failed: " << error.message() << std::endl;
		return false;
	}
	
	/*
	*	Read the Acknowledgement
	*/

	boost::asio::streambuf ack_buf;

	boost::asio::read_until( *socketPtr_, ack_buf, END_MSG, error);

	if( error && error != boost::asio::error::eof ) {
		if (verbose_)
			std::cout << "receive failed: " << error.message() << std::endl;
	}
	else {
		std::string ack_msg = boost::asio::buffer_cast<const char*>(ack_buf.data());

		//  Erase the last character 
		ack_msg.erase(ack_msg.end() - 7, ack_msg.end());

		if(ack_msg.compare("ACK") != 0)
			return false;
	}

	return true;
}


simClientSocket::simClientSocket(){
	/*
	*	DEFINE THE SOCKET
	*/

	//socket creation
	socketPtr_ = std::make_unique<tcp::socket>(service_);
	//connection
	socketPtr_->connect( boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(LOCALHOST), PORT));
}
