#ifndef COMMUNICATe_H
#define COMMUNICATe_H

#include "cmsis_os.h"
#include "main.h"
#include "Can_receive.h"

class Communicate
{
public:
    void init();
};

extern Can_receive can_receive;
extern Communicate communicate;

#endif