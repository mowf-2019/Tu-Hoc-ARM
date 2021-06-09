#ifndef  PS2_H_
#define	 PS2_H_
#include "main.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
 

#define PS2_Left            ((uint8_t)0x01U)
#define PS2_Down            ((uint8_t)0x02U)
#define PS2_Right           ((uint8_t)0x04U)
#define PS2_Up              ((uint8_t)0x08U)
#define PS2_Square          ((uint8_t)0x10U)
#define PS2_Cross           ((uint8_t)0x20U)
#define PS2_Circle          ((uint8_t)0x40U)
#define PS2_Trigle          ((uint8_t)0x80U)


#define PS2_R1              ((uint8_t)0x01U)
#define PS2_L1              ((uint8_t)0x02U)
#define PS2_R2              ((uint8_t)0x04U)
#define PS2_L2              ((uint8_t)0x08U)
#define PS2_Start           ((uint8_t)0x10U)
#define PS2_Select          ((uint8_t)0x20U)
#define PS2_R3              ((uint8_t)0x40U)
#define PS2_L3              ((uint8_t)0x80U)

typedef struct {
  uint8_t L:1 ; //left
  uint8_t D:1; //down
  uint8_t R:1; //right
  uint8_t U:1; //up
  uint8_t square:1;
  uint8_t X:1;
  uint8_t O:1;
  uint8_t trigle:1;
  uint8_t L1:1;
  uint8_t L2:1;
  uint8_t R1:1;
  uint8_t R2:1;
  uint8_t Start:1;
  uint8_t Select:1;
  uint8_t L3:1;
  uint8_t R3:1;
  uint8_t LX,LY,RX,RY;
}PS2_stat;

void ps2_read(uint8_t Data[7]);

#endif