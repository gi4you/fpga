/*
 * type.h
 *
 *  Created on: Jun 14, 2024
 *      Author: kha
 */

#ifndef SRC_TYPES_H_
#define SRC_TYPES_H_

#include "xbasic_types.h"		/* contains basic types for Xilinx software IP */
//#include "xil_types.h"


typedef unsigned int		WORD;
typedef unsigned char    	BYTE;
//typedef int					BOOL;

#define HIBYTE(w)   ((BYTE) (((WORD) (w) >> 8) & 0xFF))
#define LOBYTE(w)   ((BYTE) (w))
#define HIWORD(l)   ((WORD) (((DWORD) (l) >> 16) & 0xFFFF))
#define LOWORD(l)   ((WORD) (l))

// Hardware memory access
typedef volatile Xfloat32 	DDR2_FLOAT32_REG;
typedef volatile Xint32 	DDR2_I32BIT_REG;		// Hardware register definition
typedef volatile Xuint32 	DDR2_UI32BIT_REG;		// Hardware register definition
typedef volatile Xint16 	DDR2_I16BIT_REG;
typedef volatile Xuint16 	DDR2_UI16BIT_REG;


//
typedef int             int16;
typedef long            int32;
typedef unsigned int    Uint16;
typedef unsigned long   Uint32;
typedef float           float32;
typedef long double     float64;

#define DDR2_MEM_CAST(a) (a)




#endif /* SRC_TYPES_H_ */
