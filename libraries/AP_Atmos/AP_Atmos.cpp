/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Atmos.h"
#include "AP_Atmos_UAVCAN.h"


void AP_Atmos::init(void)  
{

    sensor = AP_Atmos_UAVCAN::probe(*this, 1); 
    printf("-----------------------AP_Atmos::init(void)\n");


}


AP_Atmos::AP_Atmos()
{
    
}

bool AP_Atmos::get_humidity(float &humidity)
{
     
    if (sensor) {
        return sensor->get_humidity(humidity);
      
    }


    return false;
}

 bool AP_Atmos::get_temperature(float &temperature)
{
     
    if (sensor) {
        return sensor->get_temperature(temperature);
        printf("AP_Atmos::temperature = %f\n",temperature);
    }

    return false;
}
