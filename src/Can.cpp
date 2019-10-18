#include <hal_ttcontrol_communication/Can.h>

Can::Can()
{
    _iFileDesc = 0;
}

int Can::iInitialize(char* pchCanChannel)
{
    struct sockaddr_can Addr;
    int iRet = 0;
    can_err_mask_t ErrMask = CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF;
    struct ifreq _ifr;
    
    _iFileDesc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    
    if( _iFileDesc < 0 )
    {
        std::cerr << "Error while opening socket" <<std::endl;
        return -1;
    }

    strcpy(_ifr.ifr_name, pchCanChannel);
    ioctl(_iFileDesc, SIOCGIFINDEX, &_ifr);

    Addr.can_family  = AF_CAN;
    Addr.can_ifindex = _ifr.ifr_ifindex; 
    
    if(bind(_iFileDesc, (struct sockaddr *)&Addr, sizeof(Addr)) < 0) 
    {
        std::cerr << "Error in socket bind" << std::endl;
        return -2;
    }
    
    iRet = fcntl(_iFileDesc,F_SETFL,O_NONBLOCK);
    if (iRet!=0)
    {
        std::cerr << "Fcntl fail" << std::endl;
        return -3;
    }

    iRet = setsockopt(_iFileDesc, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &ErrMask, sizeof(ErrMask));    
    if (iRet!=0)
    {
        std::cerr << "Setsockopt fail" << std::endl;
        return -4;
    }
    return 0;
}

bool Can::bReceiveFrame(can_frame* frame)
{
    if(read(_iFileDesc, frame, sizeof((*frame))) > 0)
    {
        if ((*frame).can_id & CAN_ERR_FLAG)
        {
            std::cerr<< "Error frame" << std::endl;
        }
        else
        {
            return true;
        }
    }
    return false;
}

//IF we use extended frames, then we need to do (*frame).can_id = (*frame).can_id | CAN_EFF_FLAG;;
int Can::iSendFrame(can_frame* frame)
{
    (*frame).can_id = (*frame).can_id | CAN_EFF_FLAG;
    return write(_iFileDesc, frame, sizeof(struct can_frame));
}
