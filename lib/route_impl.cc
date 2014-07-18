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
#include <cstdio>
#include <iostream>

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
      
      // Debug user input
      std::cout << "Repair == " << repair << std::endl;
      std::cout << "Routing == " << routing << std::endl;
      
      // Message Port initializations
      message_port_register_in(pmt::mp("from_mac"));
      message_port_register_in(pmt::mp("from_host"));
      
      set_msg_handler(pmt::mp("from_mac"),
      boost::bind(&route_impl::rx_msg_mac, this, _1));
      set_msg_handler(pmt::mp("from_host"),
      boost::bind(&route_impl::rx_msg_host, this, _1));
      
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
     void route_impl::rx_msg_mac(pmt::pmt_t msg)
     {
    	 /*size_t offset(0);
    	 pmt::pmt_t meta(pmt::car(msg));
    	 pmt::pmt_t aodvPacket(pmt::cdr(msg)); // Extract aodvPacket from msg
    	 unsigned const char * aodvPacketPtr = (const uint8_t*) uniform_vector_elements(aodvPacket, offset); // Get pointer to aodvPacket
    	 pmt::pmt_t ipPacket = pmt::make_u8vector(pmt::length(aodvPacket)-24, 0x00); // Make empty IP packet vector
    	 unsigned char * ipPacketPtr = (uint8_t*) uniform_vector_writable_elements(ipPacket, offset); // Get pointer to ipPacket
    	 // Copy contents of IP packet into packet to be sent
    	 for(int i=24; i < (pmt::length(aodvPacket)); i++)
    	 {
    		 ipPacketPtr[i] = aodvPacketPtr[i];
    	 }
    	 
    	 pmt::pmt_t msg_out = pmt::cons(meta, ipPacket);
    	 message_port_pub(pmt::mp("to_host"), msg_out);*/
    	 message_port_pub(pmt::mp("to_host"), msg);
     }
     
     void route_impl::rx_msg_host(pmt::pmt_t msg)
     {
    	//size_t offset(0);
    	pmt::pmt_t meta(pmt::car(msg)); 
    	//pmt::pmt_t ipPacket(pmt::cdr(msg));
    	pmt::pmt_t aodvPacket(pmt::cdr(msg)); //Remove me
    	/*unsigned const char * ipPacketPtr = (const uint8_t*) uniform_vector_elements(ipPacket, offset); // Get read only pointer to IP packet
    	pmt::pmt_t aodvHeader = pmt::make_u8vector(24, 0x00); // Construct null aodvHeader pmt vector
    	pmt::pmt_t aodvPacket = pmt::make_u8vector(pmt::length(aodvHeader) + pmt::length(ipPacket), 0x00); // Construct null aodvPacket pmt vector
    	unsigned char * aodvPacketPtr = (uint8_t*) uniform_vector_writable_elements(aodvPacket, offset); // Get pointer aodvPacket
    	unsigned char * aodvHeaderPtr = (uint8_t*) uniform_vector_writable_elements(aodvHeader, offset); // Get pointer to aodv header
    	
    	// Copy AODV header into packet to be sent
    	for(int i=0; i<pmt::length(aodvHeader); i++)
    	{
    		aodvPacketPtr[i] = aodvHeaderPtr[i];
    	}
    	// Copy contents of IP packet into packet to be sent
    	for(int i=pmt::length(aodvHeader); i < (pmt::length(aodvHeader)+pmt::length(ipPacket)); i++)
    	{
    	 	aodvPacketPtr[i] = ipPacketPtr[i];
    	}
    	*/
    	meta = dict_add(meta, pmt::string_to_symbol("EM_DEST_ADDR"), pmt::from_long(255)); // Set dest ID
    	meta = dict_add(meta, pmt::string_to_symbol("EM_USE_ARQ"), pmt::from_bool(true));  // Set ARQ
    	 	
    	pmt::pmt_t msg_out = pmt::cons(meta, aodvPacket);
    	message_port_pub(pmt::mp("to_mac"), msg_out);
     }

  } /* namespace MST */
} /* namespace gr */

