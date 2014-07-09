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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "route_impl.h"

namespace gr {
  namespace MST {

    route::sptr
    route::make(std::string routing, bool repair)
    {
      return gnuradio::get_initial_sptr
        (new route_impl(routing, repair));
    }

    /*
     * The private constructor
     */
    route_impl::route_impl(std::string routing, bool repair)
      : gr::sync_block("route",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(0, 0, 0))
    {
      // User input parameter initializations
      this->routing = routing;
      this->repair = repair;
      
      // Message Port initializations
      message_port_register_in(pmt::mp("from_mac"));
      set_msg_handler(pmt::mp("from_mac"),
      boost::bind(&route_impl::rx_msg, this, _1));
      
      message_port_register_in(pmt::mp("from_host"));
      set_msg_handler(pmt::mp("from_host"),
      boost::bind(&route_impl::tx_msg, this, _1));
      
      message_port_register_out(pmt::mp("to_mac"));
      message_port_register_out(pmt::mp("to_host"));
    }

    /*
     * Our virtual destructor.
     */
    route_impl::~route_impl()
    {
    }

    int route_impl::work(int noutput_items, 
      gr_vector_const_void_star &input_items,
      gr_vector_void_star &output_items)
    {
       // Work function not required. Likely to be removed if possible.
      return noutput_items;
    }
    
    /*
     * Message handlers
     */
     void rx_msg(pmt::pmt_t msg)
     {
     }
     
     void tx_msg(pmt::pmt_t msg)
     {
     }

  } /* namespace MST */
} /* namespace gr */

