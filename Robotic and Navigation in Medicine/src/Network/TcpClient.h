#pragma once


#ifdef WIN32
	#include <sdkddkver.h>
#endif

#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost\array.hpp> 
class TcpClient
{
public:
	TcpClient();
	~TcpClient();
	
	void connect(const std::string &ip, unsigned short port);
	void connect(const boost::asio::ip::tcp::endpoint& endpoint);
	void disconnect();


	void TcpClient::update();
	
	std::string command(const std::string &msg);
	
	void write(const std::string &msg);
	const std::string read();

	bool isConnected();
	

private:
	bool is_connected_;

	boost::asio::ip::tcp::endpoint	endPoint_;
	boost::asio::io_service			io_service_;

	boost::asio::ip::tcp::socket	socket_;
	std::array<char,128>	buffer_;

	char						delimiter_;

};

