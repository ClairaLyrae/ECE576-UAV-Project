/*
 * extrause.h
 *
 *  Created on: May 4, 2016
 *      Author: Claira Safi
 */

#ifndef PHYSICS_UTIL_EXTRAUSE_H_
#define PHYSICS_UTIL_EXTRAUSE_H_

float halfToFloat(uint16_t h) {
	uint32_t temp;
	temp = ((h&0x8000)<<16) | (((h&0x7c00)+0x1C000)<<13) | ((h&0x03FF)<<13);
	return *((float*)&temp);
}

uint16_t floatToHalf(float f) {
	uint32_t x = *((uint32_t*)&f);
	return (uint16_t)(((x>>16)&0x8000)|((((x&0x7f800000)-0x38000000)>>13)&0x7c00)|((x>>13)&0x03ff));
}

#endif /* PHYSICS_UTIL_EXTRAUSE_H_ */
