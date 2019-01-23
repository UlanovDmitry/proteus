#ifndef _DEFS_H
#define _DEFS_H

#include <avr/io.h>

#if !defined (F_CPU)
#define F_CPU 4000000
#endif

#define sleep()

#define set_bit(adress,bit)  (adress |=  (1<<bit))
#define clr_bit(adress,bit)  (adress &=~ (1<<bit))
#define toogle_bit(adress,bit) (adress ^=(1<<bit))
#define is_high(x,y) ((x & _BV(y)) == _BV(y))

#endif
