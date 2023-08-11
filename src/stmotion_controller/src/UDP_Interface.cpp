#include "UDP_Interface.hpp"

namespace stmotion_controller
{
namespace udp
{
UDP_Interface::UDP_Interface()
{
}

void UDP_Interface::Setup()
{
    try
    {
        socket = std::make_shared<UDP_Socket>();
        socket->Setup(IP_addr, port, timeout_us);
        std::cout << "UDP Connection to Robot Established!" << std::endl;
        std::cout << "IP address: " << IP_addr << std::endl;
        std::cout << "Port: " << port << std::endl;
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

void UDP_Interface::Send(const math::VectorJd& q, const int seq_no, bool last, bool command_type)
{
    send_pack packet;
    packet.seq_no = seq_no;
    packet.last_data = last;
    packet.data_style = command_type;
    packet.J1 = ReverseFloat(q(0));
    packet.J2 = ReverseFloat(q(1));
    packet.J3 = ReverseFloat(q(2) - q(1));
    packet.J4 = ReverseFloat(q(3));
    packet.J5 = ReverseFloat(q(4));
    packet.J6 = ReverseFloat(q(5));
    packet.ext_axis1 = ReverseFloat(0.0);
    packet.ext_axis2 = ReverseFloat(0.0);
    packet.ext_axis3 = ReverseFloat(0.0);
    SwapEndian(packet.packet_type);
    SwapEndian(packet.version_no);
    SwapEndian(packet.seq_no);
    socket->SendTo(&packet, sizeof(packet));
}


recv_pack UDP_Interface::Recv()
{
    memset(com_buffer, 0, 132);
    socket->RecvFrom(com_buffer, 132);
    
    recv_pack packet;
    char array_4[4];
    char array_1[1];

    strncpy(array_4, com_buffer+8, 1);
    strncpy(array_4+1, com_buffer+9, 1);
    strncpy(array_4+2, com_buffer+10, 1);
    strncpy(array_4+3, com_buffer+11, 1);
    packet.seq_no = *(unsigned int*)&array_4;
    SwapEndian(packet.seq_no);

    strncpy(array_1, com_buffer+12, 1);
    packet.status = array_1[0];

    strncpy(array_4, com_buffer+20, 1);
    strncpy(array_4+1, com_buffer+21, 1);
    strncpy(array_4+2, com_buffer+22, 1);
    strncpy(array_4+3, com_buffer+23, 1);
    packet.timestamp = *(unsigned int*)&array_4;
    SwapEndian(packet.timestamp);

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+24, 4);
    packet.X = ReverseFloat(*(float*)&array_4);

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+28, 4);
    packet.Y = ReverseFloat(*(float*)&array_4);

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+32, 4);
    packet.Z = ReverseFloat(*(float*)&array_4);

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+36, 4);
    packet.Roll = ReverseFloat(*(float*)&array_4);

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+40, 4);
    packet.Pitch = ReverseFloat(*(float*)&array_4);

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+44, 4);
    packet.Yaw = ReverseFloat(*(float*)&array_4);

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+48, 4);
    packet.ext_axis1 = ReverseFloat(*(float*)&array_4);

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+52, 4);
    packet.ext_axis2 = ReverseFloat(*(float*)&array_4);

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+56, 4);
    packet.ext_axis3 = ReverseFloat(*(float*)&array_4);

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+60, 4);
    packet.J1 = ReverseFloat(*(float*)&array_4);

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+64, 4);
    packet.J2 = ReverseFloat(*(float*)&array_4);

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+68, 4);
    packet.J3 = ReverseFloat(*(float*)&array_4) + packet.J2;

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+72, 4);
    packet.J4 = ReverseFloat(*(float*)&array_4);

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+76, 4);
    packet.J5 = ReverseFloat(*(float*)&array_4);

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+80, 4);
    packet.J6 = ReverseFloat(*(float*)&array_4);

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+84, 4);
    packet.J7 = ReverseFloat(*(float*)&array_4);

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+88, 4);
    packet.J8 = ReverseFloat(*(float*)&array_4);

    memset(array_4, 0, 4);
    strncpy(array_4, com_buffer+92, 4);
    packet.J9 = ReverseFloat(*(float*)&array_4);

    return packet;
}

void UDP_Interface::Shutdown()
{
    socket->Close();
}

void UDP_Interface::SendInitPack()
{
    header_pack init_packet;
    init_packet.a = 0;
    init_packet.b = 1;
    SwapEndian(init_packet.a);
    SwapEndian(init_packet.b);
    socket->SendTo(&init_packet, sizeof(init_packet));
}


void UDP_Interface::SendEndPack()
{
    header_pack end_packet;
    end_packet.a = 2;
    end_packet.b = 1;
    SwapEndian(end_packet.a);
    SwapEndian(end_packet.b);
    socket->SendTo(&end_packet, sizeof(end_packet));
}

}
}