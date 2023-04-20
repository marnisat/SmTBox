/*
 * GlobalDefines.h
 *
 *  Created on: 23-May-2016
 *      Author: radhakrishna
 */

#ifndef GLOBALDEFINES_H_
#define GLOBALDEFINES_H_

/* typedef enum{SUCCESS = 0x05,FAILURE = -1} Err_t; */
typedef enum{PASS = 0x09,FAIL = -1} Test_t;
typedef enum{YES = 0x0A,NO = -1} Sts_t;

typedef Test_t Check_t;
/* typedef Err_t Result_t; */


#define VALID       (0)
#define INVALID     (-1)

#ifndef	NULL
	#define NULL        (0u)
#endif
#define NULLPTR     ((void *)(0))


#define SET         1U
#define CLEAR       0U
#define RESET       0U



#endif /* GLOBALDEFINES_H_ */
