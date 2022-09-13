/*
 ADNS2083.cpp - Part of optical mouse sensor library for Arduino
 Copyright (c) 2008 Martijn The.  All right reserved.
 http://www.martijnthe.nl/
 Conversion to Arduino 1.x by zapmaker (zapmaker.org)
 
 Based on sketches by Beno�� Rousseau.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/******************************************************************************
 * Includes
 ******************************************************************************/

#include "Arduino.h"
#include "OptiMouse.h"
#include "ADNS2083.h"

/******************************************************************************
 * Definitions
 ******************************************************************************/

#define Delta_Y				0x04
#define Delta_X				0x03

/******************************************************************************
 * Constructor
 ******************************************************************************/


ADNS2083::ADNS2083(uint8_t sclkPin, uint8_t sdioPin) : OptiMouse::OptiMouse(sclkPin, sdioPin)
{

}

/******************************************************************************
 * User API
 ******************************************************************************/

signed char ADNS2083::dx(void)
{
	return (signed char) readRegister(Delta_X);
}

signed char ADNS2083::dy(void)
{
	return (signed char) readRegister(Delta_Y);
}
signed char ADNS2083::mo(void)
{
	return (signed char) readRegister(0x02);
}
signed char ADNS2083::ms(void)
{
	return (signed char) readRegister(0x06);
}
// Private Methods /////////////////////////////////////////////////////////////

