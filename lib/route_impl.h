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
#include <iostream>
#include <chrono>
#include <ctime>

namespace gr {
  namespace MST {

    class route_impl : public route
    {
     private:
      //User inputs
      std::string routing;
      unsigned int HOST_IP;
      //AODV user inputs
      bool ROUTE_REPAIR;
      bool ROUTE_ACK;
      bool DEST_ONLY;
      bool GRATUITOUS_RREP;
      unsigned char RREQ_RETRIES;
      unsigned char TTL_THRESHOLD;
      unsigned char TTL_INCREMENT;
      unsigned char TTL_MAX;
      unsigned char TTL_START;
      unsigned char NET_DIAMETER;
      std::chrono::milliseconds NODE_TRAVERSAL_TIME; //Amount of time it takes for a packet to be stored-and-forwarded
      std::chrono::milliseconds ACTIVE_ROUTE_TIMEOUT; //Period of inactivity before a route is marked as invalid
      
     
      
      
      //Variables
      unsigned int hostRreqId;
      int hostSeqNum;
      std::vector<rTblEntry> rTbl;
      std::vector<rreqTblEntry> rreqTbl;
      std::vector<pmt::pmt_t> txBuffer; //From host
      std::vector<pmt::pmt_t> rxBuffer; //From MAC. For use during route repair.
      
      
      //Functions
      void rx_msg_mac(pmt::pmt_t msg);
      void rx_msg_host(pmt::pmt_t msg);
      unsigned short ip4_checksum(std::vector<unsigned char> &ipPkt);
      void decTTL(std::vector<unsigned char> &ipPacket);
      void routeInvalid(int j/*rTbl Index*/, unsigned int destIp);
      void newRoute(unsigned int destIp);
      void sendRERR(unsigned int destIp, unsigned int unreachableIp, bool N = false);
      void sendRREQ(unsigned int destIp, unsigned char ttl, bool J, bool R, bool U, unsigned int destSeqNum );
      std::vector<unsigned char> makeIP4Pkt(unsigned int sourceIp=0,
                                            unsigned int destIp=0,
                                            unsigned int ttl=64,
                                            unsigned char version=4,
                                            unsigned char ihl=4,
                                            unsigned char dscp=0,
                                            unsigned char ecn=0,
                                            unsigned char flags=0,
                                            unsigned char protocol=138,
                                           unsigned short totalLength=20,
                                           unsigned short fragmentOffset=0,
                                           unsigned short identfication=0,
                                           unsigned short headerChecksum=0);
      
      std::vector<unsigned char> makeUDPPkt (unsigned short srcPort=0,
                                             unsigned short destPort=654,
                                             unsigned short length=8+24,
                                             unsigned short checksum=0);
      std::vector<unsigned char> makeRREQPkt (unsigned int rreqId, 
                                              unsigned int destIp,
                                                      bool J=false,
                                                      bool R=false,
                                                      bool G=false,
                                                      bool D=false,
                                                      bool U=true,
                                              unsigned int destSeqNum=0,
                                              unsigned int origIp=0,
                                              unsigned int origSeqNum=0);
      std::vector<unsigned char> makeRERRPkt(std::vector<std::vector<unsigned int>> pair, bool N=false);
      

     public:
      route_impl(std::string routing,
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
      ~route_impl();
      
      // Where none of the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace MST
} // namespace gr

#endif /* INCLUDED_MST_ROUTE_IMPL_H */

