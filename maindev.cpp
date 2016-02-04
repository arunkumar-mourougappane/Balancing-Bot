#include <mbed.h>
#include <stdio.h>
#include "MPU60501.cpp"

int main()
{

	 
  		wait(0.5);
       i2c.read( 0x6B, &v, 1 );
     wait(0.5);
		 pc.printf("Ans = %c",v);
			}

