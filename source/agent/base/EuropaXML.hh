

/* -*- C++ -*-
 * $Id$
 */
/** @file "EuropaXML.hh"
 * @brief Xml serialization utilities for Europa
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#ifndef _EUROPAXML_HH
#define _EUROPAXML_HH

/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2007, MBARI.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the TREX Project nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/

#include "TREXDefs.hh"

#include "XMLUtils.hh"
#include "AbstractDomain.hh"

#include <typeinfo>


namespace TREX {

  std::string domain_val_to_str(AbstractDomain const &domain, double value, 
				bool symbolic = false);

  //!@{
  /** @brief Utility for Serialization in XML
   *
   * @param Ty the type of the source object
   * @param obj The object to serialize
   *
   * @return The XML structure corresponding to @e obj
   * 
   * @author Frederic Py <fpy@mbari.org>
   */
  template<class Ty>
  TiXmlElement *to_xml(Ty const &obj);


  TiXmlElement *to_xml(AbstractDomain const &domain);
  //!@}
  
  //!@{
  /** @brief Utility for Serialization in XML
   *
   * @param Ty the type of the source object
   * @param out ther output file
   * @param obj The object to serialize
   *
   * Writes the XML form of @e obj
   * 
   * @author Frederic Py <fpy@mbari.org>
   */
//   template<class Ty>
//   void print_xml(FILE *out, Ty const &obj);

  void print_xml(FILE *out, AbstractDomain const &domain);
  //!@}

  
  /** @brief Path extraction in XML
   *
   * @param elem an XML node
   * @param path A Path of tags ala UNIX directory
   *
   * @return The first node corresponding to @e path in @e elem or NULL if @e path does not exist.
   *
   * @author Frederic Py <fpy@mbari.org>
   */
  TiXmlNode const *firstPath(TiXmlNode const *elem, std::string const &path);

} // TREX

#endif // _EUROPAXML_HH
