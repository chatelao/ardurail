/* Ardurail 1.0

   Copyright (C) 2012 Jan Weller jan.weller@gmx.de Rewritten
   and reorganized 2013 by Holger Wirtz <dcoredump@googlemail.com>

   This program is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the Free 
   Software Foundation; either version 3 of the License, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful, but WITHOUT 
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

 */

#include "Ardurail_P50.h"

Ardurail_P50 P50_CENTRAL;


void setup(void)
{		
  P50_CENTRAL.init(6,2,8);
}

void loop(void)
{
  P50_CENTRAL.p50();
}
