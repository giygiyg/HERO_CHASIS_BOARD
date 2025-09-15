#ifndef  _BSP_CAN_H
#define  _BSP_CAN_H

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++
extern "C"
{

#include "struct_typedef.h"
#include "fdcan.h"
void can_filter_init(void);

}
#endif
#endif
