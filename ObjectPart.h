/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    SimoxCGAL
* @author     Nikolaus Vahrenkamp
* @copyright  2017 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/

#ifndef _SimoxCGAL_ObjectPart_h_
#define _SimoxCGAL_ObjectPart_h_

#include "SimoxCGAL.h"


namespace SimoxCGAL
{
    /*!

    */
    class SIMOX_CGAL_IMPORT_EXPORT ObjectPart
    {
    public:

        ObjectPart();

        /*!
        */
        virtual ~ObjectPart();


        void addParameterDouble(const std::string &key, double value);
        bool hasParamterDouble(const std::string &key);
        bool getParameterDouble(const std::string &key, double &storeValue);

        void addParameterString(const std::string &key, const std::string &value);
        bool hasParamterString(const std::string &key);
        bool getParameterString(const std::string &key, std::string  &storeValue);

    protected:

        std::map<std::string, double> parametersDouble;
        std::map<std::string, std::string> parametersString;

    };

    typedef boost::shared_ptr<ObjectPart> ObjectPartPtr;
}

#endif // _SimoxCGAL_ObjectPart_h_
