/*****************************************************************************
 @Project	: Common include header for embbeded system
 @File 		: Common.h
 @Details  	: Common share macro, type         
 @Author	: Gigasitron X-Cross
 @Hardware	: Generic
 
 --------------------------------------------------------------------------
 @Revision	:
  Ver  	Author    	Date        	Changes
 --------------------------------------------------------------------------
   1.0  Name     XXXX-XX-XX  		Initial Release
   
******************************************************************************/

#ifndef __COMMON_DOT_H__
#define __COMMON_DOT_H__

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>


/*****************************************************************************
 Define
******************************************************************************/
#define TRUE		1
#define FALSE		0

#define PI			3.1415926535897932384626433832795
#define RAD_TO_DEG	57.295779513082320876798154814105
#define UNUSE( a )	(void)(a)
#define ABS( a )	(((a)<0)? -(a) : (a))

#ifndef BIT
	#define BIT( x )	(1U<<(x))
#endif

#define MIN( a, b )	(((a)<(b))? (a) : (b))
#define MAX( a, b )	(((a)<(b))? (b) : (a))

#define CONCAT (a,b) a ## b				/* concatenate  */



/*****************************************************************************
 typedef
******************************************************************************/
typedef int BOOL;




		
#endif /* __COMMON_DOT_H__ */














