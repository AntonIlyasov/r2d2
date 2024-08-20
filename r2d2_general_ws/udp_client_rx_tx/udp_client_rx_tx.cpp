#include <cstdlib>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <stdio.h>
#include <string.h>
#include "umba_crc_table.h"
using boost::asio::ip::udp;
using boost::asio::ip::address;

#define MY_CLIENT_PORT 1234
#define MY_CLIENT_IP "127.0.0.1"

const uint8_t DATA_LEN = 9;
uint32_t keepalive = 0;
uint8_t kpl_bytes[4] = {0};
bool array = false;

uint32_t inline make_uint32(uint8_t* buf) {
    return (buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3];
}

inline void int_to_bytes(uint32_t &val, uint8_t *bytes) {
  // printf("OUTPUT = %d\n", val);
  bytes[0] = val & 0xFF;                /*0-7 bits*/
  bytes[1] = (val & 0xFF00) >> 8;       /*8-15 bits*/
  bytes[2] = (val & 0xFF0000) >> 16;    /*16-23 bits*/
  bytes[3] = (val & 0xFF000000) >> 24;  /*24-31 bits*/
  // printf("bytes\n[%u][%u][%u][%u]\n", bytes[0],bytes[1],bytes[2],bytes[3]);
}

class UDPClient
{
public:
    UDPClient(boost::asio::io_context& io_context)
    : io_context_(io_context), socket_(io_context){
        std::cout << "UDP CLIENT IS RUNNING\n";
        sender_endpoint_ = udp::endpoint(address::from_string(MY_CLIENT_IP), MY_CLIENT_PORT);
        socket_.open(udp::v4());
        boost::bind(&UDPClient::udp_handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred);
        read_msg_udp();
    }

    void sendMsg(uint8_t num) {
        auto start = std::chrono::high_resolution_clock::now();
        int_to_bytes(keepalive, kpl_bytes);
        uint8_t msg_raw[] = {0xAA, 0xBB, DATA_LEN, num, kpl_bytes[3], kpl_bytes[2], kpl_bytes[1], kpl_bytes[0]};
        uint8_t msg[] = {0xAA, 0xBB, DATA_LEN, num, kpl_bytes[3], kpl_bytes[2], kpl_bytes[1], kpl_bytes[0], umba_crc8_table(msg_raw, DATA_LEN - 1)};

        std::cout << "\nInput... ";
        
        static uint32_t send_count = 0;
        boost::system::error_code err;
        
        auto sent = socket_.send_to(boost::asio::buffer(msg), sender_endpoint_, 0, err);
        if (!err && sent > 0){
            std::cout << "\nSEND TO UDP: ";
            for (int i = 0; i < DATA_LEN; i++){
                printf("[%u]", msg[i]);
            }
            std::cout << std::endl;
            send_count++;
            std::cout << "Sent Payload = " << sent << "\n";
            std::cout << "\033[1;32msend_count\033[0m = " << send_count << "\n";
            //printf("[crc8 = %u\n]", crc8);
            std::this_thread::sleep_for(std::chrono::microseconds(100000));
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration = end - start;
        float mls = duration.count() * 1000;
        std::cout << "mls: " << mls << std::endl;
    }

    ~UDPClient() {
        socket_.close();
    }

private:
    boost::asio::io_context& io_context_;
    udp::socket socket_;
    udp::endpoint sender_endpoint_;
    enum {max_length = 600};
    uint8_t data_[max_length] = {0};
    uint32_t recvd_count = 0;

    void udp_handle_receive(const boost::system::error_code& error, size_t bytes_transferred) {
        if (error) {
          std::cout << "Receive failed: " << error.message() << "\n";
          return;
        } else {
          std::cout << "\nNEW MESSAGE RECEIVED!" << std::endl;
          recvd_count++;
          std::cout << "bytes_transferred = " << bytes_transferred << std::endl;
          for (int i = 0; i < bytes_transferred; i++){
            printf("[%u]", data_[i]);
          }
          std::cout << "\nKEEPALIVE = " << make_uint32(data_ + 4) << std::endl;
          std::cout << "\033[1;32mRECEIVED \033[0m= " << recvd_count << std::endl;
          read_msg_udp();
        }
    }
    void read_msg_udp(){
        socket_.async_receive_from(boost::asio::buffer(data_, sizeof(data_)), sender_endpoint_,
          boost::bind(&UDPClient::udp_handle_receive, this, boost::asio::placeholders::error, 
          boost::asio::placeholders::bytes_transferred));
    }
};

int main(int argc, char* argv[])
{
    if (argc < 2){
        std::cerr << "Usage: " << argv[0] << " cmd" << std::endl;
        return 1;
    }

    try{
        boost::asio::io_context io_context;
        UDPClient udpClient(io_context);
        uint8_t cmd  = (uint8_t)std::stoi(argv[1]);
        while(1) {
            udpClient.sendMsg(cmd);
            keepalive++;
            io_context.poll_one();
        }
    } catch (std::exception e){
        std::cerr << "Exeption: " << e.what() << std::endl;
    }
    return 0;
}
