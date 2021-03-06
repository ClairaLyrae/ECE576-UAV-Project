// GMTL is (C) Copyright 2001-2010 by Allen Bierbaum
// Distributed under the GNU Lesser General Public License 2.1 with an
// addendum covering inlined code. (See accompanying files LICENSE and
// LICENSE.addendum or http://www.gnu.org/copyleft/lesser.txt)

// This file was originally part of PyJuggler.

// PyJuggler is (C) Copyright 2002, 2003 by Patrick Hartling
// Distributed under the GNU Lesser General Public License 2.1.  (See
// accompanying file COPYING.txt or http://www.gnu.org/copyleft/lesser.txt)

// Includes ====================================================================
#include <boost/python.hpp>
#include <gmtl/VecBase.h>
#include <gmtl/Output.h>
#include <gmtl-wrappers.h>
#include <gmtl-array-wrappers.h>
#include <gmtl-getData-wrappers.h>
#include <gmtl-pickle.h>

// Using =======================================================================
using namespace boost::python;

// Module ======================================================================
void _Export_VecBase_double_4()
{
    scope* gmtl_VecBase_double_4_scope = new scope(
    class_< gmtl::VecBase<double,4> >("VecBase4d", init<  >())
        .def(init< const gmtl::VecBase<double,4> & >())
        .def(init< const double &, const double &, const double &, const double & >())
        .def("set", (void (gmtl::VecBase<double,4>::*)(const double &, const double &, const double &, const double &) )&gmtl::VecBase<double,4>::set)
        .def("set", (void (*)(gmtl::VecBase<double,4>*, list))&gmtlWrappers::VecBase_set)
        .def("set",
             (gmtl::VecBase<double, 4>& (gmtl::VecBase<double, 4>::*)(const gmtl::VecBase<double, 4>&)) &gmtl::VecBase<double, 4>::operator=,
             return_internal_reference<1>())
        .def("getData", (tuple (*)(gmtl::VecBase<double,4>*)) &gmtlWrappers::VecBase_4_getData)
        .add_property("data", (tuple (*)(gmtl::VecBase<double,4>*)) &gmtlWrappers::VecBase_4_getData)
        .def("__getitem__", (double& (gmtl::VecBase<double,4>::*)(const unsigned) ) &gmtl::VecBase<double,4>::operator[], return_value_policy<copy_non_const_reference>())
        .def("__setitem__", (void (*)(gmtl::VecBase<double,4>*, const unsigned, double)) &gmtlWrappers::setArrayElement)
        .def_pickle(gmtlPickle::VecBase4_pickle<double>())
        .def(-self)
        .def(self += self)
        .def(self + self)
        .def(self -= self)
        .def(self - self)
        .def(self *= double())
        .def(self * double())
        .def(double() * self)
        .def(self /= double())
        .def(self / double())
        .def(self == self)
        .def(self != self)
        .def(self_ns::str(self))
    );

    enum_< gmtl::VecBase<double,4>::Params >("Params")
        .value("Size", gmtl::VecBase<double,4>::Size)
    ;

    delete gmtl_VecBase_double_4_scope;

}
