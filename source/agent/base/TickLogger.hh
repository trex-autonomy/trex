/* -*- C++ -*-
 * $Id$
 */
/** @file "TickLogger.hh"
 * @brief Definition of TickLogger class
 *
 * @author Frederic Py <fpy@mbari.org>
 */
#ifndef _TICKLOGGER_HH
#define _TICKLOGGER_HH

/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2007. MBARI.
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

#include <list>
#include <map>
#include <string>
#include <fstream>

#include "TREXDefs.hh"

namespace TREX {
  
  class LogManager;

  /** @brief Structured periodic data logger.
   *
   * This class is used to store data in a structured way into a log file.
   * 
   * @author Frederic Py <fpy@mbari.org>
   *
   * @note For now the data are stored in a tab separated file. Next step would be to store
   * it in a binary format similar to what is done on MBARI auv code.
   *
   * @bug This class is not thread safe for now it may need to be corrected.
   */
  class TickLogger {
  private:
    /** @brief Abstract data field management.
     *
     * Thsi class gives an abstract interface to be able to write data fields
     * dynamically on the TickLogger file.
     *
     * @author Frederic Py <fpy@mbari.org>
     */
    class AbstractField {
    public:
      /** @brief Constructor */
      AbstractField() {}
      /** @brief Destructor */
      virtual ~AbstractField() {}
      /** @brief Stream printing method.
       *
       * This method is called by LogField::handleNewTick to print the value of
       * the field into the corresponding output stream.
       *
       * @param out The destination output stream
       *
       * @retur nout after the operation.
       */
      virtual std::ostream &print(std::ostream &out) const =0;
    }; // TickLogger::AbstractField

    /** @brief Typed data field management class.
     *
     * @param Ty type of the field.
     *
     * This class offers an encapsulation of one data field with type Ty.
     * It stores a reference to the real data and will be able to read
     * its value each time it is needed.
     *
     * @bug This class is not taking care about thread safety and it is the
     * main reason why TickLogger class is not thread safe. It may be usefull
     * to have a better thread protection limiting logged data to SharedVar or
     * having one specialization of this class for SharedVar to offer the ability
     * to not have mutexes for data that are not shared by different threads.
     *
     * @authore Frederic Py <fpy@mbari.org>
     */
    template<class Ty>
    class LogField :public AbstractField {
    public:
      /** @brief Constructor.
       * 
       * @param ref The varaible to log.
       *
       * This mtehod creates a new LogField which will keep
       * track of the value of @e ref
       */
      explicit LogField(Ty const &ref)
	:m_ref(&ref) {}
      /** @brief Destructor */ 
      ~LogField() {}

      /** @brief Derefernce operator.
       *
       * @note In practice I think that this method is useless.
       */
      Ty const &operator* () const {
	return *m_ref;
      }

      std::ostream &print(std::ostream &out) const {
	return out<<*m_ref; // For now we log into binary format
      }
    private:
      /** @brief reference to tracked variable */ 
      Ty const *m_ref;
    }; // TickLogger::LogField<>

  public:
    /** @brief Add a new field.
     *
     * @param field Field name
     * @param value referenced variable.
     *
     * This method adds a new columns on the datalog file. The newly created column
     * will be named @e field and will keep track of @e value.
     *
     * @pre The method printHeader has not been called.
     *
     * @retval true the field was newly created.
     * @retval false a field with this name already exist. The newly requested field was not created.
     *
     * @bug No control is done to check if we are still is in the header.
     *
     * @sa TickLogger::exist
     */
    template<class Ty>
    bool addField(std::string const &field, Ty const &value) {
      fields_type::value_type to_ins(field, NULL);
      std::pair<fields_type::iterator, bool> ret = m_fields.insert(to_ins);

      if( ret.second ) {
	ret.first->second = new LogField<Ty>(value);
	m_print_order.push_back(ret.first);
      }
      return ret.second;
    }

    /** @brief Check for field existence
     *
     * @param field Field name
     *
     * @retval true the field exists.
     * @retval false else.
     */
    bool exist(std::string const &field) const;

    /** @{
     * @brief Field access
     *
     * @param name A field name
     * @param Ty type of the field managed varaiable
     *
     * @pre field @e name exists
     * 
     * @return a reference to the field managed varaible
     *
     * @throw std::bad_cast Ty is not the type of this field.
     * @bug This function ds  not check properly for field existence.
     *
     * @note I don't think that this method is usefull on nominal TickLogger usage.
     */
    template<class Ty>
    Ty &get(std::string const &name) {
      LogField<Ty> &field = dynamic_cast<LogField<Ty> &>(getField(name));
      return *field;
    }

    template<class Ty>
    Ty const &get(std::string const &name) const {
      LogField<Ty> const &field = dynamic_cast<LogField<Ty> const &>(getField(name));
      return *field;
    }
    /** @} */

    /** @brief TickLogger file name.
     *
     * @return The name of this file has it was given at construction.
     *
     * @note As this file uses LogManager to create its file this name is a symbolic
     * name for the file. The real filename may also points to the directory created by LogManager
     */
    std::string const &getName() {
      return m_baseName;
    }
    
    /** @brief Print file header.
     *
     * Thsi method is called by LogManager::handleInit to fill the header of the datalog file.
     *
     * After this call no more fields should be added to this file.
     */
    void printHeader();
    /** @brief Print new datalog line
     *
     * @param current Actual tick value.
     *
     * This method is called by LogManager::handleNewTick to indicate that a new tick has started in the agent.
     * It is used to create e new datalog line attached to @e current tick.
     */
    void handleNewTick(TICK current);

  protected:
    /** @brief Constructor.
     *
     * @brief A short file name
     *
     * This constructor creates a new file on LogManager created directory with the name @e fileName
     */
    TickLogger(std::string const &fileName);
    /** @brief Destructor */
    ~TickLogger();
    
    /** @{
     * @brief Direct field access
     *
     * @param name A field name
     *
     * @pre @e name must exists
     *
     * @return The field referenced by @e name
     */
    AbstractField &getField(std::string const &name);
    AbstractField const &getField(std::string const &name) const;
    /** @} */
  private:
    /** @brief Short file name */
    std::string m_baseName;
    /** @brief datalog file */
    std::ofstream m_file;
    bool m_inHeader, //!< @brief header flag
      m_closed; //!< @brief file closed flag

    typedef std::map<std::string, AbstractField *> fields_type;
    typedef std::list<fields_type::const_iterator> order_type;

    /** @brief Fields access map
     *
     * This attribute allows to access easily to the field managed
     * by this class using their symbolic name.
     */
    fields_type m_fields;
    /** @brief Fields display order queue.
     *
     * This attribute is used to know the order in which the
     * fields has to be displayed. The actual policy is a FIFO one
     * (ie The first declared field will be the first column).
     */
    order_type m_print_order;

    friend class LogManager;

    // Following methods are not implemented in purpose
    void operator= (TickLogger const &);
    TickLogger(TickLogger const &);
  }; // TickLogger

} // TREX

#endif // _TICKLOGGER_HH
