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

#ifndef _SimoxCGAL_CGALMeshIO_h_
#define _SimoxCGAL_CGALMeshIO_h_

#include "SimoxCGAL.h"
#include "CGALSurfaceMesh.h"
#include "CGALPolyhedronMesh.h"

namespace SimoxCGAL
{
    /*!

    */
    class SIMOX_CGAL_IMPORT_EXPORT CGALMeshIO
    {
    public:

        static CGALSurfaceMeshPtr LoadSurfaceMesh(const std::string &filename);
        static CGALPolyhedronMeshPtr LoadPolyhedronMesh(const std::string &filename);
        static bool Save(CGALSurfaceMeshPtr o, const std::string &filename);
        static bool Save(CGALPolyhedronMeshPtr o, const std::string &filename);

    private:
        // no need to instanciate this class
        CGALMeshIO();
        virtual ~CGALMeshIO();
    };

}

#endif // _SimoxCGAL_CGALMeshIO_h_
