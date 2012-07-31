// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "edge_pointxyz_pointxyz.h"
#include "parameter_se3_offset.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#endif

#include <iostream>

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#endif

namespace g2o {
  using namespace std;

  EdgePointXYZPointXYZ::EdgePointXYZPointXYZ() : BaseBinaryEdge<1, double, VertexPointXYZ, VertexPointXYZ>() 
  {
  }

  bool EdgePointXYZPointXYZ::read(std::istream& is) 
  {
    double measurement;
    is >> measurement;
    setMeasurement(measurement);
    information().setIdentity();
    is >> information()(0,0);
    return true;
  }

  bool EdgePointXYZPointXYZ::write(std::ostream& os) const 
  {
    os << measurement() << " " << information()(0,0);
    return os.good();
  }


#ifdef G2O_HAVE_OPENGL
  EdgePointXYZPointXYZDrawAction::EdgePointXYZPointXYZDrawAction(): DrawAction(typeid(EdgePointXYZPointXYZ).name()){}

  HyperGraphElementAction* EdgePointXYZPointXYZDrawAction::operator()(HyperGraph::HyperGraphElement* element,
               HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;

    if (_show && !_show->value())
      return this;

    EdgePointXYZPointXYZ* e = static_cast<EdgePointXYZPointXYZ*>(element);
    VertexPointXYZ* from = static_cast<VertexPointXYZ*>(e->vertex(0));
    VertexPointXYZ* to   = static_cast<VertexPointXYZ*>(e->vertex(1));
    glColor3f(0.2f,0.3f,0.3f);
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f((float)from->estimate().x(),(float)from->estimate().y(),(float)from->estimate().z());
    glVertex3f((float)to->estimate().x(),(float)to->estimate().y(),(float)to->estimate().z());
    glEnd();
    glPopAttrib();
    return this;
  }
#endif

} // end namespace
