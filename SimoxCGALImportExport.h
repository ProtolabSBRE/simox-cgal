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
#ifndef _SimoxCGALImportExportSymbols_h
#define _SimoxCGALImportExportSymbols_h

namespace SimoxCGAL
{
#ifdef WIN32
#  pragma warning ( disable : 4251 )
#  if defined(SimoxCGAL_EXPORTS)
#    define SIMOX_CGAL_IMPORT_EXPORT __declspec(dllexport)
#  else
#    define SIMOX_CGAL_IMPORT_EXPORT __declspec(dllimport)
#  endif
#else
#  define SIMOX_CGAL_IMPORT_EXPORT
#endif
}

#endif

