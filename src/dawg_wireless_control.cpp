#include "unitree_legged_sdk/unitree_wireless.h"


// Singleton class for maintaining one udp instance
WIRE_LESS_CONTROL* WIRE_LESS_CONTROL :: UdpSingleton(uint8_t level){
    if (wirelesscontrol == nullptr)
        wirelesscontrol = new WIRE_LESS_CONTROL(level);
    return wirelesscontrol;
}

void WIRE_LESS_CONTROL :: UdpDeleteInstance(){
    if (wirelesscontrol != nullptr)
        delete wirelesscontrol;
        wirelesscontrol = nullptr;
}


WIRE_LESS_CONTROL* WIRE_LESS_CONTROL :: GetUdpInstance(uint8_t level){
    return UdpSingleton(level);
}

void WIRE_LESS_CONTROL :: UDPSend(){
    wirelesscontrol->udp.Send();
}

void WIRE_LESS_CONTROL :: UDPRecv(){
    wirelesscontrol->udp.Recv();
}

void WIRE_LESS_CONTROL ::  UDPCont(){

    wirelesscontrol->udp.GetRecv(wirelesscontrol->state);
    wirelesscontrol->safe.PowerProtect(wirelesscontrol->cmd, wirelesscontrol->state, 1);
    wirelesscontrol->udp.SetSend(wirelesscontrol->cmd);

}

void WIRE_LESS_CONTROL :: UDPLoop(){

    LoopFunc loop_control("control_loop", wirelesscontrol->dt,    boost::bind(&WIRE_LESS_CONTROL::UDPCont,      wirelesscontrol));
    LoopFunc loop_udpSend("udp_send",     wirelesscontrol->dt, 3, boost::bind(&WIRE_LESS_CONTROL::UDPSend,      wirelesscontrol));
    LoopFunc loop_udpRecv("udp_recv",     wirelesscontrol->dt, 3, boost::bind(&WIRE_LESS_CONTROL::UDPRecv,      wirelesscontrol));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

}


