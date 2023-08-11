#pragma once
#include "Utils/Math.hpp"
#include "Utils/FileIO.hpp"
#include "iostream"
#include "UDP_Socket.hpp"

namespace stmotion_controller
{
namespace udp
{
struct header_pack{
    unsigned int a;
    unsigned int b;
};

struct send_pack{
    unsigned int packet_type = 1;
    unsigned int version_no = 1;
    unsigned int seq_no;
    unsigned char last_data;
    unsigned char read_io_type = 0;
    unsigned short read_io_idx = 0;
    unsigned short read_io_mask = 0;
    unsigned char data_style;
    unsigned char write_io_type =0;
    unsigned short write_io_idx = 0;
    unsigned short write_io_mask = 0;
    unsigned short write_io_val = 0;
    unsigned short unused = 0;
    float J1;
    float J2;
    float J3;
    float J4;
    float J5;
    float J6;
    float ext_axis1;
    float ext_axis2;
    float ext_axis3;
};


struct recv_pack{
    unsigned int seq_no;
    unsigned char status;
    unsigned int timestamp;
    unsigned char write_io_type =0;
    unsigned short write_io_idx = 0;
    unsigned short write_io_mask = 0;
    unsigned short write_io_val = 0;
    unsigned short unused = 0;
    float X;
    float Y;
    float Z;
    float Roll;
    float Pitch;
    float Yaw;
    float ext_axis1;
    float ext_axis2;
    float ext_axis3;
    float J1;
    float J2;
    float J3;
    float J4;
    float J5;
    float J6;
    float J7;
    float J8;
    float J9;
};

class UDP_Interface
{
    /* -------------------------------------------------------------------------- */
    /*                                   pointer                                  */
    /* -------------------------------------------------------------------------- */
    public:
        typedef std::shared_ptr<UDP_Interface> Ptr;
        typedef std::shared_ptr<UDP_Interface const> ConstPtr;

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:
        std::string IP_addr = "192.168.1.100";
        int port = 60015;
        unsigned int timeout_us = 100000; // us

    /* -------------------------------------------------------------------------- */
    /*                                  functions                                 */
    /* -------------------------------------------------------------------------- */
    private:
        UDP_Socket::Ptr socket;
        char com_buffer[132];

        // Conversion between Little and Big Endian for unsigned int
        inline void SwapEndian(unsigned int &val)
        {
            val = (val<<24) | ((val<<8) & 0x00ff0000) |
                ((val>>8) & 0x0000ff00) | (val>>24);
        }

        // Conversion between Little and Big Endian for unsigned short
        inline void SwapEndian(unsigned short &val)
        {
            val = (val<<8) | (val>>8);
        }

        // Conversion between Little and Big Endian for float
        float ReverseFloat(const float inFloat)
        {
            float retVal;
            char *floatToConvert = ( char* ) & inFloat;
            char *returnFloat = ( char* ) & retVal;

            returnFloat[0] = floatToConvert[3];
            returnFloat[1] = floatToConvert[2];
            returnFloat[2] = floatToConvert[1];
            returnFloat[3] = floatToConvert[0];

            return retVal;
        }

    public:
        UDP_Interface();
        ~UDP_Interface(){}

        // setter
        void set_IP(const std::string& ip) {IP_addr = ip;}
        void set_port(const int& port_in) {port = port_in;}
        void set_timeout(const int& timeout) {timeout_us = timeout;}
        
        // getter
        std::string get_IP() {return IP_addr;}
        int get_port() {return port;}
        unsigned int get_timeout_us() {return timeout_us;}

        // Operations
        void Setup();
        void Send(const math::VectorJd& q, const int seq_no, bool last, bool command_type);
        recv_pack Recv();
        void Shutdown();
        void SendInitPack();
        void SendEndPack();
};
}
}