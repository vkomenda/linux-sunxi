#ifndef 	__NAND_OAL__
#define  	__NAND_OAL__

#include "../../include/type_def.h"
#include "../../nfd/nand_user_cfg.h"
#include <linux/string.h>
#include <linux/slab.h>
//#include "../../sys_include/epdk.h"

//define the memory set interface
#define MEMSET(x,y,z)            			memset((x),(y),(z))

//define the memory copy interface
#define MEMCPY(x,y,z)                   	memcpy((x),(y),(z))

//define the memory alocate interface
#define MALLOC(x)                       	kmalloc((x), GFP_KERNEL)

//define the memory release interface
#define FREE(x,size)                    	kfree((x))
//define the message print interface
#define PRINT(...)							printk(__VA_ARGS__)

#endif
