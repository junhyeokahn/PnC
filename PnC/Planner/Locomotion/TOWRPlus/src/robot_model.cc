/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/******************************************************************************
Modified by Junhyeok Ahn (junhyeokahn91@gmail.com) for towr+
******************************************************************************/

#include <stdio.h>
#include <towr_plus/models/examples/atlas_model.h>
#include <towr_plus/models/examples/nao_model.h>
#include <towr_plus/models/examples/valkyrie_model.h>
#include <towr_plus/models/robot_model.h>

#include <iostream>

namespace towr_plus {

RobotModel::RobotModel(Robot robot) {
  switch (robot) {
  case Atlas:
    dynamic_model_ = std::make_shared<AtlasDynamicModel>();
    kinematic_model_ = std::make_shared<AtlasKinematicModel>();
    break;
  case Valkyrie:
    dynamic_model_ = std::make_shared<ValkyrieDynamicModel>();
    kinematic_model_ = std::make_shared<ValkyrieKinematicModel>();
    break;
  case Nao:
    dynamic_model_ = std::make_shared<NaoDynamicModel>();
    kinematic_model_ = std::make_shared<NaoKinematicModel>();
    break;
  default:
    assert(false); // Error: Robot model not implemented.
    break;
  }
}

} // namespace towr_plus
