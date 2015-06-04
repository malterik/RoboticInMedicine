#include "TcpClient.h"


TcpClient::TcpClient()
	:socket_(io_service_), is_connected_(false)
{
	delimiter_ = '\n';

	
}


TcpClient::~TcpClient()
{
	disconnect();
}

void TcpClient::connect(const std::string &ip, unsigned short port) 
{
	try {
		const boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(ip), port);

		connect(endpoint);
	}
	catch (const std::exception &e) {
		std::cout << "Server exception:" << e.what() << std::endl;
	}
}


void TcpClient::connect(const boost::asio::ip::tcp::endpoint& endpoint) {
	
	endPoint_ = endpoint;
	boost::system::error_code error = boost::asio::error::host_not_found;
	socket_.connect(endpoint,error);
	if (error) {
		throw boost::system::system_error(error);
		is_connected_ = false;
	} else {
		is_connected_ = true;
	}
}

void TcpClient::disconnect()
{
	socket_.close();
	io_service_.stop();
	is_connected_ = false;
}




const std::string TcpClient::read()  {
	buffer_.assign(0);
	for (;;) {
		boost::system::error_code error;
		size_t len = socket_.read_some(boost::asio::buffer(buffer_), error);

		if (error)
			throw boost::system::system_error(error); // Some error.
		
		if (len > 0){
			if (buffer_.at(len - 1) == delimiter_) {
				return buffer_.data();
			}
		}
		else
			return "else";
		
	}
}

void TcpClient::write(const std::string &msg) {
	boost::system::error_code error;
	socket_.write_some(boost::asio::buffer(msg),error);
	if (error)
		throw boost::system::system_error(error);

}

std::string TcpClient::command(const std::string &msg) {
	write(msg);
	return read();
}




bool TcpClient::isConnected(){
	return is_connected_;
}

