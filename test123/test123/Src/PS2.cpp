#include "PS2.h"
PS2_stat ps2;
void ps2_read(uint8_t Data[7])
{
	
		if ((Data[1] & PS2_Left) == PS2_Left)
		{
			ps2.L = 1;
		}
		else
			ps2.L = 0;
		if ((Data[1] & PS2_Down) == PS2_Down)
		{
			ps2.D = 1;
		}
		else
			ps2.D = 0;
		if ((Data[1] & PS2_Right) == PS2_Right)
		{
			ps2.R = 1;
		}
		else
			ps2.R = 0;
		if ((Data[1] & PS2_Up) == PS2_Up)
		{
			ps2.U = 1;
		}
		else
			ps2.U = 0;
		if ((Data[1] & PS2_Square) == PS2_Square)
		{
			ps2.square = 1;
		}
		else
			ps2.square = 0;
		if ((Data[1] & PS2_Cross) == PS2_Cross)
		{
			ps2.X = 1;
		}
		else
			ps2.X = 0;
		if ((Data[1] & PS2_Circle) == PS2_Circle)
		{
			ps2.O = 1;
		}
		else
			ps2.O = 0;
		if ((Data[1] & PS2_Trigle) == PS2_Trigle)
		{
			ps2.trigle = 1;
		}
		else
			ps2.trigle = 0;
                
                
		if ((Data[2] & PS2_R1) == PS2_R1)
		{
			ps2.R1 = 1;
		}
		else
			ps2.R1 = 0;
		if ((Data[2] & PS2_L1) == PS2_L1)
		{
			ps2.L1 = 1;
		}
		else
			ps2.L1 = 0;
		if ((Data[2] & PS2_R2) == PS2_R2)
		{
			ps2.R2 = 1;
		}
		else
			ps2.R2 = 0;
		if ((Data[2] & PS2_L2) == PS2_L2)
		{
			ps2.L2 = 1;
		}
		else
			ps2.L2 = 0;
		if ((Data[2] & PS2_Start) == PS2_Start)
		{
			ps2.Start = 1;
		}
		else
			ps2.Start = 0;
		if ((Data[2] & PS2_Select) == PS2_Select)
		{
			ps2.Select = 1;
		}
		else
			ps2.Select = 0;
		if ((Data[2] & PS2_R3) == PS2_R3)
		{
			ps2.R3 = 1;
		}
		else
			ps2.R3 = 0;
		if ((Data[2] & PS2_L3) == PS2_L3)
		{
			ps2.L3 = 1;
		}
		else
			ps2.L3 = 0;
}