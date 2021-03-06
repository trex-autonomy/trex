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

/* -*- C++ -*-
 */

#include <cstring>

#include "RStat.hh"

namespace {
  
  char const mem_sizes[] = {' ', 'k', 'M', 'G', 'T', '\0'};
  
  std::ostream &mem_print(std::ostream &out, long val, char unit='b') {
    char const *iter = mem_sizes;

    while( iter[1] && (val>>10) ) {
      ++iter;
      val >>= 10;
    }
    out<<val;
    if( *iter!=' ' )
      out.put(*iter);
    return out.put(unit);
  }
  
} // unnamed 

using namespace TREX;

/*
 * class RStat
 */ 
// Structors :

RStat::RStat(RStat::who_type who) 
  :_kind(who) {
  reset();
}

// Modifiers :

void RStat::reset(RStat::who_type wt) {
  if( wt!=unknown )
    _kind = wt;
  
  switch( _kind ) {
  case zeroed:
  case unknown:
    memset(&_snapshot, 0, sizeof(rusage));
    break;
  default:
    if( getrusage(_kind, &_snapshot)<0 )
      throw ErrnoExcept("RStat");
  }
}

// Manipulators :

RStat RStat::operator- (RStat const &other) const {
  RStat result(_kind==other._kind?_kind:unknown, false);
  
  result._snapshot.ru_utime = _snapshot.ru_utime-other._snapshot.ru_utime;
  result._snapshot.ru_stime = _snapshot.ru_stime-other._snapshot.ru_stime;

  result._snapshot.ru_maxrss = _snapshot.ru_maxrss-other._snapshot.ru_maxrss;
  result._snapshot.ru_ixrss = _snapshot.ru_ixrss-other._snapshot.ru_ixrss;
  result._snapshot.ru_idrss = _snapshot.ru_idrss-other._snapshot.ru_idrss;
  result._snapshot.ru_isrss = _snapshot.ru_isrss-other._snapshot.ru_isrss;

  result._snapshot.ru_minflt = _snapshot.ru_minflt-other._snapshot.ru_minflt;
  result._snapshot.ru_majflt = _snapshot.ru_majflt-other._snapshot.ru_majflt;
  result._snapshot.ru_nswap = _snapshot.ru_nswap-other._snapshot.ru_nswap;

  result._snapshot.ru_inblock = _snapshot.ru_inblock-other._snapshot.ru_inblock;
  result._snapshot.ru_oublock = _snapshot.ru_oublock-other._snapshot.ru_oublock;

  result._snapshot.ru_msgsnd = _snapshot.ru_msgsnd-other._snapshot.ru_msgsnd;
  result._snapshot.ru_msgrcv = _snapshot.ru_msgrcv-other._snapshot.ru_msgrcv;

  result._snapshot.ru_nsignals = _snapshot.ru_nsignals-other._snapshot.ru_nsignals;

  result._snapshot.ru_nvcsw = _snapshot.ru_nvcsw-other._snapshot.ru_nvcsw;
  result._snapshot.ru_nivcsw = _snapshot.ru_nivcsw-other._snapshot.ru_nivcsw;

  return result;
}

RStat RStat::operator+ (RStat const &other) const {
  RStat result(_kind==other._kind?_kind:unknown, false);
  
  result._snapshot.ru_utime = _snapshot.ru_utime+other._snapshot.ru_utime;
  result._snapshot.ru_stime = _snapshot.ru_stime+other._snapshot.ru_stime;

  result._snapshot.ru_maxrss = _snapshot.ru_maxrss+other._snapshot.ru_maxrss;
  result._snapshot.ru_ixrss = _snapshot.ru_ixrss+other._snapshot.ru_ixrss;
  result._snapshot.ru_idrss = _snapshot.ru_idrss+other._snapshot.ru_idrss;
  result._snapshot.ru_isrss = _snapshot.ru_isrss+other._snapshot.ru_isrss;

  result._snapshot.ru_minflt = _snapshot.ru_minflt+other._snapshot.ru_minflt;
  result._snapshot.ru_majflt = _snapshot.ru_majflt+other._snapshot.ru_majflt;
  result._snapshot.ru_nswap = _snapshot.ru_nswap+other._snapshot.ru_nswap;

  result._snapshot.ru_inblock = _snapshot.ru_inblock+other._snapshot.ru_inblock;
  result._snapshot.ru_oublock = _snapshot.ru_oublock+other._snapshot.ru_oublock;

  result._snapshot.ru_msgsnd = _snapshot.ru_msgsnd+other._snapshot.ru_msgsnd;
  result._snapshot.ru_msgrcv = _snapshot.ru_msgrcv+other._snapshot.ru_msgrcv;

  result._snapshot.ru_nsignals = _snapshot.ru_nsignals+other._snapshot.ru_nsignals;

  result._snapshot.ru_nvcsw = _snapshot.ru_nvcsw+other._snapshot.ru_nvcsw;
  result._snapshot.ru_nivcsw = _snapshot.ru_nivcsw+other._snapshot.ru_nivcsw;

  return result;
}

RStat RStat::operator* (long n) const {
  RStat result(_kind, false);
  
  result._snapshot.ru_utime = _snapshot.ru_utime*n;
  result._snapshot.ru_stime = _snapshot.ru_stime*n;

  result._snapshot.ru_maxrss = _snapshot.ru_maxrss*n;
  result._snapshot.ru_ixrss = _snapshot.ru_ixrss*n;
  result._snapshot.ru_idrss = _snapshot.ru_idrss*n;
  result._snapshot.ru_isrss = _snapshot.ru_isrss*n;

  result._snapshot.ru_minflt = _snapshot.ru_minflt*n;
  result._snapshot.ru_majflt = _snapshot.ru_majflt*n;
  result._snapshot.ru_nswap = _snapshot.ru_nswap*n;

  result._snapshot.ru_inblock = _snapshot.ru_inblock*n;
  result._snapshot.ru_oublock = _snapshot.ru_oublock*n;

  result._snapshot.ru_msgsnd = _snapshot.ru_msgsnd*n;
  result._snapshot.ru_msgrcv = _snapshot.ru_msgrcv*n;

  result._snapshot.ru_nsignals = _snapshot.ru_nsignals*n;

  result._snapshot.ru_nvcsw = _snapshot.ru_nvcsw*n;
  result._snapshot.ru_nivcsw = _snapshot.ru_nivcsw*n;

  return result;
}

RStat RStat::operator/ (long n) const {
  RStat result(_kind, false);
  
  result._snapshot.ru_utime = _snapshot.ru_utime/n;
  result._snapshot.ru_stime = _snapshot.ru_stime/n;

  result._snapshot.ru_maxrss = _snapshot.ru_maxrss/n;
  result._snapshot.ru_ixrss = _snapshot.ru_ixrss/n;
  result._snapshot.ru_idrss = _snapshot.ru_idrss/n;
  result._snapshot.ru_isrss = _snapshot.ru_isrss/n;

  result._snapshot.ru_minflt = _snapshot.ru_minflt/n;
  result._snapshot.ru_majflt = _snapshot.ru_majflt/n;
  result._snapshot.ru_nswap = _snapshot.ru_nswap/n;

  result._snapshot.ru_inblock = _snapshot.ru_inblock/n;
  result._snapshot.ru_oublock = _snapshot.ru_oublock/n;

  result._snapshot.ru_msgsnd = _snapshot.ru_msgsnd/n;
  result._snapshot.ru_msgrcv = _snapshot.ru_msgrcv/n;

  result._snapshot.ru_nsignals = _snapshot.ru_nsignals/n;

  result._snapshot.ru_nvcsw = _snapshot.ru_nvcsw/n;
  result._snapshot.ru_nivcsw = _snapshot.ru_nivcsw/n;

  return result;
}



// Observers :

std::ostream &RStat::long_desc(std::ostream &out) const {
  out<<"Resource Stats ("<<_kind<<"):\n"
    "\t- user time        : "<<_snapshot.ru_utime<<" s\n"
    "\t- system time      : "<<_snapshot.ru_stime<<" s\n"
    "\t- Max resident set : ";
  mem_print(out, _snapshot.ru_maxrss)<<"\n"
    "\t- Shared text mem  : ";
  mem_print(out, _snapshot.ru_ixrss)<<"\n"
    "\t- Unshared data    : ";
  mem_print(out, _snapshot.ru_idrss)<<"\n"
    "\t- Unshared stack   : ";
  mem_print(out, _snapshot.ru_isrss)<<"\n"
    "\t- Page reclaims    : "<<_snapshot.ru_minflt<<"\n"
    "\t- Page Faults      : "<<_snapshot.ru_majflt<<"\n"
    "\t- Swaps            : "<<_snapshot.ru_nswap<<"\n"
    "\t- Block inputs     : "<<_snapshot.ru_inblock<<"\n"
    "\t- Block outputs    : "<<_snapshot.ru_oublock<<"\n"
    "\t- Messages sent    : "<<_snapshot.ru_msgsnd<<"\n"
    "\t- Messages recvd   : "<<_snapshot.ru_msgrcv<<"\n"
    "\t- Signals received : "<<_snapshot.ru_nsignals<<"\n"
    "\t- Voluntary ctxt switches : "<<_snapshot.ru_nvcsw<<"\n"
    "\t- Unvol. context switches : "<<_snapshot.ru_nivcsw<<std::endl;
  return out;
}

std::string RStat::compact_header() {
  return "UserTime\tSystemTime\tMaxRSS\tSharedTxtMem\t"
    "UnsharedData\tUnsharedStack\tPageReclaim\tPageFaults";
}


std::ostream &RStat::compact_desc(std::ostream &out) const {
  out<<_snapshot.ru_utime<<"\t"<<_snapshot.ru_stime<<"\t"
     << _snapshot.ru_maxrss<<"\t"<<_snapshot.ru_ixrss<<"\t"
     << _snapshot.ru_idrss<<"\t"<<_snapshot.ru_isrss<<"\t"
     <<_snapshot.ru_minflt<<"\t"<<_snapshot.ru_majflt;
  return out;
}
