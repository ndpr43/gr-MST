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
      pmt::pmt_t meta(pmt::car(msg)); // Get msg metadata
	  pmt::pmt_t vect(pmt::cdr(msg)); // Get msg data
	  char IHL; // Internet Header Length
	  unsigned short updDestPort;
      if(routing=="AODV")
	  {
        // TODO: Optimize memory usage!!!!
		// TODO: Set some values to constant
        std::vector<uint8_t> ipPacket = pmt::u8vector_elements(vect);
        if(ipPacket[9]==138) // MANET Control Packet
        {
          IHL = ipPacket[0] & 0x0F; // Read Internet Header Length
          std::vector<unsigned char> udpPacket(ipPacket.begin() + 4*IHL,ipPacket.end());
          updDestPort = static_cast<unsigned short>(static_cast<unsigned short>(udpPacket[2])<<8
            | static_cast<unsigned short>(udpPacket[3]));
          if(updDestPort==654) // AODV Control Packet
          {
            std::vector<uint8_t> aodvPacket(udpPacket.begin()+(2*4),udpPacket.end());
            unsigned char aodvType = aodvPacket[0];
            switch ( aodvType ) 
            {
              case 1:
              {
			    // RREQ
                // Parse the packet
                bool joinFlag = static_cast<bool>(aodvPacket[2] & (1<<7));
                bool repairFlag = static_cast<bool>(aodvPacket[2] & (1<<6));
                bool gratuitousRREPFlag = static_cast<bool>(aodvPacket[2] & (1<<5));
                bool destOnlyFlag = static_cast<bool>(aodvPacket[2] & (1<<4));
                bool unkownSeqNum = static_cast<bool>(aodvPacket[2] & (1<<3));
                unsigned char hopCnt = aodvPacket[3];
                int rreqId = static_cast<unsigned int>(aodvPacket[4])<<(8*3) 
                  | static_cast<unsigned int>(aodvPacket[5])<<(8*2) 
                  | static_cast<unsigned int>(aodvPacket[6])<<(8*1)
		          | static_cast<unsigned int>(aodvPacket[7]);
                unsigned int destIp = static_cast<unsigned int>(aodvPacket[8])<<(8*3) 
                  | static_cast<unsigned int>(aodvPacket[9])<<(8*2) 
				  | static_cast<unsigned int>(aodvPacket[10])<<(8*1)
				  | static_cast<unsigned int>(aodvPacket[11]);
			    int destSeqNum = static_cast<int>(static_cast<unsigned int>(aodvPacket[12])<<(8*3) 
			      | static_cast<unsigned int>(aodvPacket[13])<<(8*2) 
			      | static_cast<unsigned int>(aodvPacket[14])<<(8*1)
			      | static_cast<unsigned int>(aodvPacket[15]));
			    unsigned int origIp = static_cast<unsigned int>(aodvPacket[16])<<(8*3) 
			      | static_cast<unsigned int>(aodvPacket[17])<<(8*2) 
			      | static_cast<unsigned int>(aodvPacket[18])<<(8*1)
		          | static_cast<unsigned int>(aodvPacket[19]);
		        int origSeqNum = static_cast<int>(static_cast<unsigned int>(aodvPacket[20])<<(8*3) 
		          | static_cast<unsigned int>(aodvPacket[21])<<(8*2) 
		          | static_cast<unsigned int>(aodvPacket[22])<<(8*1) 
		          | static_cast<unsigned int>(aodvPacket[23]));
		        // TODO: route the packet
		        // 
		        //	   
		        break;
              }
			  case 2:
			  {
			    // RREP
		        bool repairFlag = static_cast<bool>(aodvPacket[2] & (1<<7));
		        bool ackFlag = static_cast<bool>(aodvPacket[2] & (1<<6));
		        unsigned char preFixSz = aodvPacket[2];
		        unsigned char hopCnt = aodvPacket[3];
		        unsigned int destIp = static_cast<unsigned int>(aodvPacket[4])<<(8*3)
		          | static_cast<unsigned int>(aodvPacket[5])<<(8*2)
		          | static_cast<unsigned int>(aodvPacket[6])<<(8*1)
		          | static_cast<unsigned int>(aodvPacket[7]);
		        int destSeqNum = static_cast<int>(static_cast<unsigned int>(aodvPacket[8])<<(8*3) 
                  | static_cast<unsigned int>(aodvPacket[9])<<(8*2) 
                  | static_cast<unsigned int>(aodvPacket[10])<<(8*1)
                  | static_cast<unsigned int>(aodvPacket[11]));
		        unsigned int origIp = static_cast<unsigned int>(aodvPacket[12])<<(8*3)
		          | static_cast<unsigned int>(aodvPacket[13])<<(8*2) 
		          | static_cast<unsigned int>(aodvPacket[14])<<(8*1)
		          | static_cast<unsigned int>(aodvPacket[15]);
		        int lifetime = static_cast<int>(static_cast<unsigned int>(aodvPacket[8])<<(8*3)
		          | static_cast<unsigned int>(aodvPacket[9])<<(8*2)
		          | static_cast<unsigned int>(aodvPacket[10])<<(8*1)
		          | static_cast<unsigned int>(aodvPacket[11]));
		        // TODO: route the packet
		        // 
		        //
		        break;
			  }
		      case 3:
		      {
			    // RERR 
		        bool noDeleteFlag = static_cast<bool>(aodvPacket[2] & (1<<7));
		        unsigned char destCnt = aodvPacket[3];
		        std::vector<unsigned int> unreachableDestIp(destCnt);
		        std::vector<int> unreachableSeqNum(destCnt);
		        int i=0;
		        int j=4;
		        while(i < destCnt)
		        {
		          unreachableDestIp[i] = static_cast<unsigned int>(aodvPacket[j])<<(8*3) 
		            | static_cast<unsigned int>(aodvPacket[j+1])<<(8*2) 
		            | static_cast<unsigned int>(aodvPacket[j+2])<<(8*1)
		            | static_cast<unsigned int>(aodvPacket[j+3]);
		          unreachableSeqNum[i] = static_cast<int>(static_cast<unsigned int>(aodvPacket[j+4])<<(8*3) 
		            | static_cast<unsigned int>(aodvPacket[j+5])<<(8*2) 
		            | static_cast<unsigned int>(aodvPacket[j+6])<<(8*1)
		            | static_cast<unsigned int>(aodvPacket[j+7]));
		          i++;
		          j+=8;
		        }
		        // TODO: route the packet
		        // 
		        //
		        break;
		      }
		      case 4:
		      {
                // RREP-ACK
		        // TODO: route the packet
			    // 
		        //
		        break;
		      }
		      default:
		      {
		        // Code
                std::cout << "Error : AODV : Unkown type" << std::endl;
                break;
			  }
            }
          }
          else // MANET != AODV
          {
			// Use data packet forwarding logic
	      }
        }
		else // Data Packet
		{ 
		  // AODV Forwarding logic here
		}
			 /*pmt::pmt_t outVect = pmt::init_u8vector (ipPacket.size(), ipPacket);
			 //pmt::pmt_t outVect = pmt::init_u8vector (ethFrame.size(), ethFrame);
			 
			 meta = dict_add(meta, pmt::string_to_symbol("EM_DEST_ADDR"), pmt::from_long(255)); // Set dest ID
			 meta = dict_add(meta, pmt::string_to_symbol("EM_USE_ARQ"), pmt::from_bool(true));  // Set ARQ
			 
				
			 pmt::pmt_t msg_out = pmt::cons(meta, outVect);
			 message_port_pub(pmt::mp("to_host"), msg_out);
			 */
	  }
      else // Default case with no routing
	  {
        message_port_pub(pmt::mp("to_host"),msg);
	  }
    }
    
    void route_impl::rx_msg_host(pmt::pmt_t msg)
    {
      pmt::pmt_t meta(pmt::car(msg)); // Get msg metadata
      pmt::pmt_t vect(pmt::cdr(msg)); // Get msg data
      if(routing=="AODV")
      {
        std::vector<uint8_t> ipPacket = pmt::u8vector_elements(vect);
    	std::vector<uint8_t> aodvHeader(24, 2);
    	std::vector<uint8_t> aodvPacket;
    	aodvPacket.reserve(ipPacket.size() + aodvHeader.size());
    	aodvPacket.insert(aodvPacket.end(), aodvHeader.begin(), aodvHeader.end());
    	aodvPacket.insert(aodvPacket.end(), ipPacket.begin(), ipPacket.end());
    	  
    	pmt::pmt_t outVect = pmt::init_u8vector (aodvPacket.size(), aodvPacket);
    	meta = dict_add(meta, pmt::string_to_symbol("EM_DEST_ADDR"), pmt::from_long(255)); // Set dest ID
    	meta = dict_add(meta, pmt::string_to_symbol("EM_USE_ARQ"), pmt::from_bool(true));  // Set ARQ
    	  
    	pmt::pmt_t msg_out = pmt::cons(meta, outVect);
    	message_port_pub(pmt::mp("to_mac"), msg_out);
      }
    }
    
    unsigned short route_impl::ip4_checksum(std::vector<int> &ipPacket)
    {
      int i=0;
      int j=0;
      unsigned char IHL = ipPacket[0] & 0x0F;
      unsigned int accum = 0;
      unsigned short checksum = 0;
      unsigned short carry;
      while(i<(IHL*4))
      {
    	if(i!=10) //Don't include the checksum itself
    	{
    	  accum += static_cast<unsigned int>((static_cast<unsigned short>(ipPacket[i]<<8) | static_cast<unsigned short>(ipPacket[i+1])));
    	}
    	i+=2;
      }
      carry = static_cast<unsigned short>((accum)>>16);
      checksum = ~(static_cast<unsigned short>(accum & 0x0000FFFF) + carry);
      return checksum;
    }
  } /* namespace MST */
} /* namespace gr */

