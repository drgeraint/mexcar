// -*-c++-*-

/************************************************************
 * Crash.hh
 *
 * Geraint Paul Bevan <g.bevan@mech.gla.ac.uk>
 * Initial Revision : <2005-06-23>
 * Latest Time-stamp: <2007-08-05 21:59:13 geraint>
 *
 * $Id: Crash.hh,v 1.1 2008-01-09 14:21:13 gbevan Exp $
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
