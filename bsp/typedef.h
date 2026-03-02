#ifndef TYPEDEF_H
#define TYPEDEF_H

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"

//#define __PACKED __attribute__((packed))
#define __COUNT(arr) sizeof(arr)/sizeof(arr[0])

typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

typedef union __PACKED{
	fp32 data;
	uint8_t bytes[4];
} union_fp32;

#endif
