#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <memory>
#include <optional>
#include <stdio.h>
#include <cstdlib>

using boost::asio::ip::address;
using boost::asio::ip::tcp;

#define PORT 1234

uint32_t keepalive = 0;

uint32_t inline make_uint32(uint8_t* buf) {
    return (buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3];
}

class session : public std::enable_shared_from_this<session>
{
public:
  session (boost::asio::ip::tcp::socket &&socket)
  : socket (std::move (socket)) {}

  void start () {
    socket.async_read_some(boost::asio::buffer(data_, max_length),
        boost::bind(&session::handle_read, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }

private:
  boost::asio::ip::tcp::socket socket;
  boost::asio::streambuf streambuf;
  enum { max_length = 1024 };
  uint8_t data_[max_length];
  uint32_t recvd_count;
  uint32_t send_count;

  void handle_read(const boost::system::error_code& error,
      size_t bytes_transferred)
  {
    if (!error)
    {
      std::cout << "NEW MESSAGE RECEIVED!" << std::endl;
      recvd_count++;
      std::cout << "bytes_transferred = " << bytes_transferred << std::endl;
      for (int i = 0; i < bytes_transferred; i++){
        printf("[%u]", data_[i]);
      }
      std::cout << "\nKEEPALIVE = " << make_uint32(data_ + 4) << std::endl;
      std::cout << "\n\033[1;32mRECEIVED \033[0m= " << recvd_count << std::endl;
      boost::asio::async_write(socket,
          boost::asio::buffer(data_, bytes_transferred),
          boost::bind(&session::handle_write, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
      std::cout << "here1\n";
    }
    else
    {
      delete this;
    }
  }

  void handle_write(const boost::system::error_code& error,
      size_t bytes_transferred)
  {
    if (!error && bytes_transferred > 0)
    {
      std::cout << "\nSEND TO TCP: ";
      for (int i = 0; i < bytes_transferred; i++){
          printf("[%u]", data_[i]);
      }
      std::cout << std::endl;
      send_count++;
      std::cout << "\033[1;32msend_count\033[0m = " << send_count << "\n";
      socket.async_read_some(boost::asio::buffer(data_, max_length),
          boost::bind(&session::handle_read, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
    }
    else
    {
      delete this;
    }
  }
};

class TCPServer{
public:
  TCPServer(boost::asio::io_context& io_context) 
  : io_context(io_context),
    acceptor(io_context, tcp::endpoint(tcp::v4(), PORT))
  {
    std::cout << "TCP SERVER IS RUNNING\n";
    async_accept();
  }

private:
  boost::asio::io_context &io_context;
  boost::asio::ip::tcp::acceptor acceptor;

  void async_accept (){
    auto socket = std::make_shared<boost::asio::ip::tcp::socket> (io_context);
    acceptor.async_accept (*socket, [&, socket] (boost::system::error_code error) {
      std::make_shared<session> (std::move (*socket))->start();
      async_accept();
    });
  }
};

int main()
{
  boost::asio::io_context io_context;
  TCPServer tcpServer(io_context);
  io_context.run();
  return 0;
}