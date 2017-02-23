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

#ifndef _SimoxCGAL_CGALPolyhedronMesh_h_
#define _SimoxCGAL_CGALPolyhedronMesh_h_

#include "SimoxCGAL.h"


namespace SimoxCGAL
{
    /*!

    */
    class SIMOX_CGAL_IMPORT_EXPORT CGALPolyhedronMesh
    {
    public:

        /*!
        */
        CGALPolyhedronMesh(PolyhedronMeshPtr m);

        /*!
        */
        virtual ~CGALPolyhedronMesh();

        PolyhedronMeshPtr getMesh();

        unsigned int getNrOfVertices();
        unsigned int getNrOfEdges();
        unsigned int getNrOfFaces();

        void print();

        std::string toXML(int nrTabs = 1);

        static boost::shared_ptr<CGALPolyhedronMesh> fromXML(const std::string &xml);

    protected:
        PolyhedronMeshPtr mesh;

    };

    typedef boost::shared_ptr<CGALPolyhedronMesh> CGALPolyhedronMeshPtr;
}

#endif
