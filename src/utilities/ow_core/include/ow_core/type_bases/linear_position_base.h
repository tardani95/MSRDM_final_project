/*! \file
 *
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author J. Rogelio Guadarrama-Olvera
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 14.02.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */


#ifndef OPEN_WALKER_CORE_LINEAR_POSITION_BASE_H
#define OPEN_WALKER_CORE_LINEAR_POSITION_BASE_H

#include <ow_core/conversions.h>
#include <ow_core/type_bases/vector3_base.h>

namespace ow_core{

/*!
 * \brief The LinearPositionBase class.
 */
template <typename _Derived>
class LinearPositionBase :
    public Vector3Base<_Derived>
{
public:
  typedef _Derived Derived;
  typedef Vector3Base<Derived> Base;

public:
  using Base::operator=;

};

}  // namespace ow_core

#endif  // OPEN_WALKER_CORE_LINEAR_POSITION_BASE_H
