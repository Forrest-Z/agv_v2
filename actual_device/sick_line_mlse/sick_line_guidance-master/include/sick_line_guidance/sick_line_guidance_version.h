/*
 * sick_line_guidance_version handles the version of sick_line_guidance ros driver.
 * The version concates major version, minor version and patch level, each with three digits.
 * 
 * Copyright (C) 2019 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2019 SICK AG, Waldkirch
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of SICK AG nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 *  Copyright 2019 SICK AG
 *  Copyright 2019 Ing.-Buero Dr. Michael Lehning
 * 
 */
#ifndef __SICK_LINE_GUIDANCE_VERSION_H_INCLUDED
#define __SICK_LINE_GUIDANCE_VERSION_H_INCLUDED

#define SICK_LINE_GUIDANCE_MAJOR_VER   1
#define SICK_LINE_GUIDANCE_MINOR_VER   0
#define SICK_LINE_GUIDANCE_PATCH_LEVEL 0

namespace sick_line_guidance
{

  /*
   * class Version handles the version of sick_line_guidance ros driver.
   */
  class Version
  {
  public:
  
    /*
     * @brief returns the version as a string, f.e. "001.002.003".
     *
     * @return version string.
     */
    static std::string getVersionInfo(void)
    {
      std::stringstream str;
      str << std::uppercase << std::setfill('0') << std::setw(3) << SICK_LINE_GUIDANCE_MAJOR_VER << ".";
      str << std::uppercase << std::setfill('0') << std::setw(3) << SICK_LINE_GUIDANCE_MINOR_VER << ".";
      str << std::uppercase << std::setfill('0') << std::setw(3) << SICK_LINE_GUIDANCE_PATCH_LEVEL;
      return str.str();
    }
   
  }; // class Version
} // namespace sick_line_guidance
#endif // __SICK_LINE_GUIDANCE_VERSION_H_INCLUDED
