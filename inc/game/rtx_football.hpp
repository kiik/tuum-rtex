/** @file rtx_football.hpp
 *  Robotex 1vs1 football logic module.
 *
 *  @authors Meelik Kiik
 *  @version 0.1
 *  @date 17. November 2015
 */

#ifndef RTX_FOOTBALL_LOGIC_H
#define RTX_FOOTBALL_LOGIC_H

#include "application.hpp"
#include "STM.hpp"

namespace rtx {
namespace Football {

  extern STM stm;

  void setup();
  void process();

}}

#endif
