// -*-c++-*-

/************************************************************
 * Crash.hh
 *
 * Geraint Paul Bevan <geraint.bevan@gcu.ac.uk>
 * Initial Revision : <2005-06-23>
 * Latest Time-stamp: <2021/02/26 02:32:42 geraint>
 *
 ***********************************************************/

#ifndef _CRASH_H_
#define _CRASH_H_

/// An exception that can be thrown.
class Crash
{
public:

  /** can be called with a message which can be read by the
   * error handling routine.
   */
  Crash(const char *message) {
    std::cerr << message << std::endl;
  }
};

#endif // _CRASH_H_
