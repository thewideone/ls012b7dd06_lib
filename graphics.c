#include "graphics.h"

// Swap two uint16 macro:
#define swap( a, b ) { uint16_t t = a; a = b; b = t; }

// 
// Draw a line using Bresenham's algorithm.
// Coordinates as signed ints to make drawing large objects easier.
// x0, y0 - one end of the line
// x0, y0 - the other end of the line
// colour - colour of the line
// 
void rlcd_drawLine( int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t colour ){

	int steep = abs( y1 - y0 ) > abs( x1 - x0 );

	if(steep){
		swap( x0, y0 );
		swap( x1, y1 );
	}

	if( x0 > x1 ){
		swap( x0, x1 );
		swap( y0, y1 );
	}

	int dx, dy;
	dx = x1 - x0;
	dy = abs( y1 - y0 );

	int err = dx / 2;
	int ystep;

	if( y0 < y1 )
		ystep = 1;
	else
		ystep = -1;

	for( ; x0 <= x1; x0++ ){
		if(steep)
			rlcd_putPixel( y0, x0, colour );
		else
			rlcd_putPixel( x0, y0, colour );

		err -= dy;

		if( err < 0 ){
			y0 += ystep;
			err += dx;
		}
	}
}