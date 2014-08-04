/* -*- c++ -*- */
/* 
 * Copyright 2014 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */


#ifndef INCLUDED_MST_ROUTE_H
#define INCLUDED_MST_ROUTE_H

#include <MST/api.h>
#include <gnuradio/sync_block.h>
#include <string>

namespace gr {
  namespace MST {

    /*!
     * \brief <+description of block+>
     * \ingroup MST
     *
     */
    class MST_API route : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<route> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of MST::route.
       *
       * To avoid accidental use of raw pointers, MST::route's
       * constructor is in a private implementation
       * class. MST::route::make is the public interface for
       * creating new instances.
       */
      static sptr make(std::string routing,
                       std::string hostIp,
                       bool repair, 
                       bool ack, 
                       bool destOnly, 
                       bool gratutiousRrep, 
                       unsigned char rreqRetries, 
                       unsigned char ttlThreshold, 
                       unsigned char ttlIncrement, 
                       unsigned char ttlMax, 
                       unsigned char ttlStart, 
                       unsigned char netDiameter, 
                       unsigned int nodeTraversalTime, 
                       unsigned int activeRouteTimeout);
    };

  } // namespace MST
} // namespace gr

#endif /* INCLUDED_MST_ROUTE_H */

