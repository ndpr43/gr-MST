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

#ifndef INCLUDED_MST_ROUTE_IMPL_H
#define INCLUDED_MST_ROUTE_IMPL_H

#include <MST/route.h>
#include "MST/aodv.h"
#include <iostream>
#include <vector>
#include <gnuradio/thread/thread.h>
#include <gnuradio/blocks/pdu.h>
#include <pmt/pmt.h>

namespace gr {
  namespace MST {

    class route_impl : public route
    {
     private:
      //User inputs
      std::string routing;
      bool repair;
      //Variables
      std::vector<rTbEntry> rTbl;
      std::vector<rreqTblEntry> rreqTbl;
      std::vector<pmt::pmt_t> d_messages;   
      
      //Functions
      void rx_msg_mac(pmt::pmt_t msg);
      void rx_msg_host(pmt::pmt_t msg);

     public:
      route_impl(std::string routing, bool repair);
      ~route_impl();
      
      

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace MST
} // namespace gr

#endif /* INCLUDED_MST_ROUTE_IMPL_H */

