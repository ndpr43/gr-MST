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
#include <chrono>
#include <ctime>

namespace gr {
  namespace MST {
    
    route::sptr
    route::make(std::string routing,
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
                unsigned int activeRouteTimeout)
    {
      return gnuradio::get_initial_sptr
        (new route_impl(routing,
                        repair, 
                        ack, 
                        destOnly, 
                        gratutiousRrep, 
                        rreqRetries, 
                        ttlThreshold, 
                        ttlIncrement, 
                        ttlMax, 
                        ttlStart, 
                        netDiameter, 
                        nodeTraversalTime, 
                        activeRouteTimeout));
    }

    /*
     * The private constructor
     */
    route_impl::route_impl(std::string routing,
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
                           unsigned int activeRouteTimeout)
      : gr::sync_block("route",
      gr::io_signature::make(0, 0, 0),
      gr::io_signature::make(0, 0, 0))
    {
      // User input parameter initializations
      this->routing = routing;
      this-> HOST_IP;
      if(routing=="AODV")
      {
        ROUTE_REPAIR = repair;
        ROUTE_ACK = ack;
        DEST_ONLY = destOnly;
        GRATUITOUS_RREP = gratutiousRrep;
        RREQ_RETRIES = static_cast<unsigned char>(gratutiousRrep);
        TTL_THRESHOLD = static_cast<unsigned char>(ttlThreshold);
        TTL_INCREMENT = static_cast<unsigned char>(ttlIncrement);
        TTL_MAX = static_cast<unsigned char>(ttlMax);
        TTL_START = static_cast<unsigned char>(ttlStart);
        NET_DIAMETER = static_cast<unsigned char>(netDiameter);
        NODE_TRAVERSAL_TIME = std::chrono::milliseconds(nodeTraversalTime);
        ACTIVE_ROUTE_TIMEOUT = std::chrono::milliseconds(activeRouteTimeout);
      }
      
      // Debug user input
      //std::cout << "Repair == " << ROUTE_REPAIR << std::endl;
      //std::cout << "Routing == " << routing << std::endl;
      
      // Message Port initializations
      message_port_register_in(pmt::mp("from_mac"));
      message_port_register_in(pmt::mp("from_host"));
      
      set_msg_handler(pmt::mp("from_mac"),
      boost::bind(&route_impl::rx_msg_mac, this, _1)); // Setting message handler for incoming messages from mac layer
      set_msg_handler(pmt::mp("from_host"),
      boost::bind(&route_impl::rx_msg_host, this, _1)); // Setting message handler for incoming messages from host
      
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
        if (pmt::is_null(vect) && pmt::dict_has_key(meta, pmt::mp("EM_UNREACHABLE_DEST_ADDR"))) // Link failure notification from mac layer
        {
          unsigned char unreachableDest;
          pmt::pmt_t deadNode = pmt::dict_ref ( meta, pmt::mp("EM_UNREACHABLE_DEST_ADDR"), pmt::PMT_NIL );
          if(pmt::is_symbol(deadNode))
          {
            unreachableDest = static_cast<unsigned char>(std::stoi(pmt::symbol_to_string(deadNode)));
          }
          else if(pmt::is_number(deadNode))
          {
            unreachableDest = static_cast<unsigned char>(pmt::to_long(deadNode));
          }
          
          //unsigned int destIp = (unreachableDest & 0x000000FF) | (HOST_IP & 0xFFFFFF00) // Kludge assumes 255.255.255.0 Subnet mask
          if(ROUTE_REPAIR)
          {
            //perform local repair
            //set status of route to invalid
            //set other status to being repair
            //send rreq w/ repair flag
              
          }
          else // Send RERR to affected nodes and Routes
          {
            for(int i=0; i<rTbl.size(); i++)
            {
              // Search for affected routes
              if(rTbl[i].nxtHop==unreachableDest)
              {
                // Mark route(s) invalid
                rTbl[i].valid = false;
                // Look through precursors list
                for(int j=0; j<rTbl[i].precursors.size();j++)
                {
                  // Find the reverse route for each precursor node in the table
                  for(int k=0; k<rTbl.size();k++)
                  {
                    // If the reverse route for the affected precursor is found send RERR
                    if(rTbl[k].destIp == rTbl[i].precursors[j] && rTbl[k].valid)
                    {
                      sendRERR(rTbl[i].precursors[j], rTbl[i].destIp, rTbl[k].nxtHop);
                    }
                  }
                }
              }
            }
          }
        }
        else // Packet Routing
        {
          // TODO: Optimize memory usage!!!!
          // TODO: Set some values to constant
          std::vector<uint8_t> ipPacket = pmt::u8vector_elements(vect);
          if(ipPacket[9]==138) // MANET Control Packet
          {
            pmt::pmt_t srcMac_t = pmt::dict_ref ( meta, pmt::mp("EM_SRC_ID"), pmt::PMT_NIL );
            unsigned char srcMac = static_cast<unsigned char>(pmt::to_long(srcMac_t));
            
            IHL = ipPacket[0] & 0x0F; // Read Internet Header Length
            unsigned int srcIp = static_cast<unsigned int>(ipPacket[12])<<(3*8) 
              | static_cast<unsigned int>(ipPacket[13])<<(2*8) 
              | static_cast<unsigned int>(ipPacket[14])<<(8) 
              | static_cast<unsigned int>(ipPacket[15]);
            unsigned char ttl=ipPacket[8];
            std::vector<unsigned char> udpPacket(ipPacket.begin() + 4*IHL,ipPacket.end());
            updDestPort = static_cast<unsigned short>(static_cast<unsigned short>(udpPacket[2])<<8
              | static_cast<unsigned short>(udpPacket[3]));
            if(ttl>=0)
            {
              if(updDestPort==654) // AODV Control Packet
              {
                std::vector<uint8_t> aodvPacket(udpPacket.begin()+(2*4),udpPacket.end());
                unsigned char aodvType = aodvPacket[0];
                switch ( aodvType ) 
                {
                  case 1: // TODO: RREQ
                  {
                    // Parse the packet
                    bool joinFlag = static_cast<bool>(aodvPacket[2] & (1<<7));
                    bool repairFlag = static_cast<bool>(aodvPacket[2] & (1<<6));
                    bool gratuitousRREPFlag = static_cast<bool>(aodvPacket[2] & (1<<5));
                    bool destOnlyFlag = static_cast<bool>(aodvPacket[2] & (1<<4));
                    bool unkownSeqNum = static_cast<bool>(aodvPacket[2] & (1<<3));
                    unsigned char hopCnt = aodvPacket[3];
                    unsigned int rreqId = static_cast<unsigned int>(aodvPacket[4])<<(8*3) 
                      | static_cast<unsigned int>(aodvPacket[5])<<(8*2) 
                      | static_cast<unsigned int>(aodvPacket[6])<<(8*1)
                      | static_cast<unsigned int>(aodvPacket[7]);
                    unsigned int destIp = static_cast<unsigned int>(aodvPacket[8])<<(8*3) 
                      | static_cast<unsigned int>(aodvPacket[9])<<(8*2) 
                      | static_cast<unsigned int>(aodvPacket[10])<<(8*1)
                      | static_cast<unsigned int>(aodvPacket[11]);
                    unsigned int destSeqNum = static_cast<unsigned int>(static_cast<unsigned int>(aodvPacket[12])<<(8*3) 
                      | static_cast<unsigned int>(aodvPacket[13])<<(8*2) 
                      | static_cast<unsigned int>(aodvPacket[14])<<(8*1)
                      | static_cast<unsigned int>(aodvPacket[15]));
                    unsigned int origIp = static_cast<unsigned int>(aodvPacket[16])<<(8*3) 
                      | static_cast<unsigned int>(aodvPacket[17])<<(8*2) 
                      | static_cast<unsigned int>(aodvPacket[18])<<(8*1)
                      | static_cast<unsigned int>(aodvPacket[19]);
                    unsigned int origSeqNum = static_cast<int>(static_cast<unsigned int>(aodvPacket[20])<<(8*3) 
                      | static_cast<unsigned int>(aodvPacket[21])<<(8*2) 
                      | static_cast<unsigned int>(aodvPacket[22])<<(8*1) 
                      | static_cast<unsigned int>(aodvPacket[23]));
                    
                    // Have we seen this rreq before?
                    // If (no)
                      // Make an rreq entry
                      // Update reverse route
                    // Is it me?
                      // Update HOST SEQ NUM
                      // Send RREP to source
                    // else not me
                      // If(D)
                        // Forward
                      // else
                        // Do I have a route?
                          // Update forward route
                          // If(G)
                            // Send RREP to dest
                          // Send RREP to source
                        // else
                          // Forward   
                    
                    // Check to see if we've forwarded this rreq before
                    int i=0;
                    bool reqFound = false;
                    while( i<rreqTbl.size() && !reqFound)
                    {
                      // If src IP and rreq ID match
                      if(rreqTbl[i].srcIp==srcIp && rreqTbl[i].rreqId==rreqId)
                        reqFound = true;
                      else
                        i++;
                    }
                    if(reqFound)
                    {
                      // Do nothing. We've forwarded this rreq before
                    }
                    else // Haven't seen it before
                    {
                      // Make an rreq entry
                      rreqTblEntry entry = {destIp, rreqId, srcIp, ttl, 0, std::chrono::system_clock::now() + PATH_DISCOVER_TIME, PATH_DISCOVER_TIME, repairFlag};
                      rreqTbl.push_back(entry);
                      // Make/Update reverse route
                      addRoute(origIp,// rev route
                               HOST_IP,// rev route
                               origSeqNum,// rev route
                               true, // Spec
                               !ROUTE_ACK, // If ack is needed don't mark as a valid route yet
                               false,
                               false,
                               hopCnt+1,
                               srcMac);
                      // Is it me?
                      if(destIp == HOST_IP)
                      {
                        // Update my Seq Number
                        if(destSeqNum > hostSeqNum)
                          hostSeqNum++;
                        // Send rrep
                        sendRREP(repairFlag, destIp, origIp, 0);
                      }
                      else if(!destOnlyFlag)
                      {
                        int j=0;
                        bool found = false;
                        while (!found && j<rTbl.size())
                        {
                          if(destIp==rTbl[j].destIp && rTbl[j].valid==true)
                            found = true;
                          else
                            j++;
                        }
                        if(found)
                        {
                          // Update my dest Seq Number
                          if(destSeqNum > rTbl[j].destSeqNum)
                            rTbl[j].destSeqNum++;
                          // Update forward route
                          rTbl[j].lifetime = std::chrono::system_clock::now() + ACTIVE_ROUTE_TIMEOUT;
                          // Add node to precusors list
                          int k=0;
                          found = false;
                          // Ensure src node is not already on the list
                          while(!found && k<rTbl[j].precursors.size())
                          {
                            if(rTbl[j].precursors[k]==srcIp)
                              found = true;
                            else
                              k++;
                          }
                          if(!found)
                            rTbl[j].precursors.push_back(srcIp);
                          
                          if(gratuitousRREPFlag)
                          {
                            // Send rrep to destination
                            sendGRREP(repairFlag, destIp, origIp, rTbl[j].hopCnt);
                          }
                          // Send rrep
                          sendRREP(repairFlag, destIp, origIp, rTbl[j].hopCnt);
                        }
                        else // Not found
                        {
                          // Forward
                          // We're sending the original packet
                          // There's no reason to reconstruct it
                          decTTL(ipPacket);
                          ipPacket[31]++; // Increment Hop Count
                          //forward(ipPacket,BROADCAST_IP, static_cast<unsigned char>(BROADCAST_IP*0x000000FF));
                          pmt::pmt_t outVect = pmt::init_u8vector (pkt.size(), pkt);     
                          meta = dict_add(meta, pmt::string_to_symbol("EM_DEST_ADDR"), pmt::from_long(static_cast<long>(255)));
                          meta = dict_add(meta, pmt::string_to_symbol("EM_USE_ARQ"), pmt::from_bool(true));  // Set ARQ   
                          pmt::pmt_t msg_out = pmt::cons(meta, outVect);
                          message_port_pub(pmt::mp("to_mac"), msg_out);
                        }
                      }
                      else // Forward
                      {
                        // Forward
                        // We're sending the original packet
                        // There's no reason to reconstruct it
                        decTTL(ipPacket);
                        ipPacket[31]++; // Increment Hop Count
                        //forward(ipPacket,BROADCAST_IP, static_cast<unsigned char>(BROADCAST_IP*0x000000FF));
                        pmt::pmt_t outVect = pmt::init_u8vector (pkt.size(), pkt);     
                        meta = dict_add(meta, pmt::string_to_symbol("EM_DEST_ADDR"), pmt::from_long(static_cast<long>(255)));
                        meta = dict_add(meta, pmt::string_to_symbol("EM_USE_ARQ"), pmt::from_bool(true));  // Set ARQ   
                        pmt::pmt_t msg_out = pmt::cons(meta, outVect);
                        message_port_pub(pmt::mp("to_mac"), msg_out);
                      }
                    }
                    break;
                  }
                  case 2: // TODO: RREP
                  {
                    bool repairFlag = static_cast<bool>(aodvPacket[2] & (1<<7));
                    bool ackFlag = static_cast<bool>(aodvPacket[2] & (1<<6));
                    unsigned char preFixSz = aodvPacket[2];
                    unsigned char hopCnt = aodvPacket[3];
                    unsigned int destIp = static_cast<unsigned int>(aodvPacket[4])<<(8*3)
                      | static_cast<unsigned int>(aodvPacket[5])<<(8*2)
                      | static_cast<unsigned int>(aodvPacket[6])<<(8*1)
                      | static_cast<unsigned int>(aodvPacket[7]);
                    unsigned int destSeqNum = static_cast<unsigned int>(static_cast<unsigned int>(aodvPacket[8])<<(8*3) 
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
                  case 3: // TODO: RERR 
                  {
                    bool noDeleteFlag = static_cast<bool>(aodvPacket[2] & (1<<7));
                    unsigned char destCnt = aodvPacket[3];
                    std::vector<unsigned int> unreachableDestIp(destCnt);
                    std::vector<unsigned int> unreachableSeqNum(destCnt);
                    int i=0;
                    int j=4;
                    while(i < destCnt)
                    {
                      unreachableDestIp[i] = static_cast<unsigned int>(aodvPacket[j])<<(8*3) 
                        | static_cast<unsigned int>(aodvPacket[j+1])<<(8*2) 
                        | static_cast<unsigned int>(aodvPacket[j+2])<<(8*1)
                        | static_cast<unsigned int>(aodvPacket[j+3]);
                      unreachableSeqNum[i] = static_cast<unsigned int>(static_cast<unsigned int>(aodvPacket[j+4])<<(8*3) 
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
                  case 4: // TODO: RREP-ACK
                  {
                    // TODO: route the packet
                    // 
                    //
                    break;
                  }
                  default: // TODO: ERROR
                  {
                    std::cout << "Error: Unkown AODV type" << std::endl;
                    std::cout << "       Dropping packet" << std::endl;
                    break;
                  }
                }
              }
              else // MANET != AODV
              {
                // Use data packet forwarding logic
              }
            }
            else // TODO: Data Packet
            { 
              // AODV Forwarding logic here
            }
          }
        }
        else // Default case with no routing
        {
          message_port_pub(pmt::mp("to_host"),msg);
          //meta = dict_add(meta, pmt::string_to_symbol("EM_DEST_ADDR"), pmt::from_long(255)); // Set dest ID
          //meta = dict_add(meta, pmt::string_to_symbol("EM_USE_ARQ"), pmt::from_bool(true));  // Set ARQ
        }
      }
    }
    
    void route_impl::rx_msg_host(pmt::pmt_t msg)
    {
      txBuffer.push_back(msg);
      pmt::pmt_t top;
      for(int i=0; i < txBuffer.size(); i++)
      {
        top = txBuffer.front();
        pmt::pmt_t meta(pmt::car(top)); // Get msg metadata
        pmt::pmt_t vect(pmt::cdr(top)); // Get msg data
        std::vector<uint8_t> ipPacket = pmt::u8vector_elements(vect);          
        unsigned int destIp = static_cast<unsigned int>(ipPacket[16])<<3*8 
          | static_cast<unsigned int>(ipPacket[17])<<2*8 
          | static_cast<unsigned int>(ipPacket[18])<<8 
          | static_cast<unsigned int>(ipPacket[19]);
        //Check for loopback
        // TODO: Add filter loopback control in the future
        if(destIp == HOST_IP)
        {
          message_port_pub(pmt::mp("to_host"), top);
          txBuffer.erase(txBuffer.begin());
        }
        else //Not a loopback
        {
          if(routing=="AODV")
          {
            int j = 0;
            bool routeFound = false;
            // Search Routing table for route
            while (j<rTbl.size() && !routeFound)
            {
              if(rTbl[j].destIp==destIp)
                routeFound = true;
              else
                j++;
            }
            
            if(routeFound)
            {
              if(rTbl[j].valid) // if(Route is Valid)
              {
                if(rTbl[j].lifetime > std::chrono::system_clock::now()) // Route is fresh
                {
                  // Reset route lifetime
                  rTbl[j].lifetime = std::chrono::system_clock::now() + ACTIVE_ROUTE_TIMEOUT;
                  // Reset reverse route lifetime
                  for(int k=0; k<rTbl.size(); k++) // Search table for reverse route(s)
                  {
                    // Check rtbl[k] to see if its destination matches any
                    // nodes in the precursors list of current active route
                    for(int l=0; l<rTbl[j].precursors.size(); l++)  // TODO: only refresh the precursor we recieved from
                    {
                      // If match found reset the lifetime of that reverse route
                      if(rTbl[k].destIp==rTbl[j].precursors[l])
                        rTbl[k].lifetime = std::chrono::system_clock::now() + ACTIVE_ROUTE_TIMEOUT;
                    }
                  }
                  // Send message
                  meta = dict_add(meta, pmt::string_to_symbol("EM_DEST_ADDR"), pmt::from_long(static_cast<unsigned char>(rTbl[j].nxtHop&0x000000FF))); // Set dest ID
                  meta = dict_add(meta, pmt::string_to_symbol("EM_USE_ARQ"), pmt::from_bool(true));  // Set ARQ
                  pmt::pmt_t msg_out = pmt::cons(meta, vect);
                  message_port_pub(pmt::mp("to_mac"), msg_out);
                  message_port_pub(pmt::mp("to_mac"), top);
                  // Delete message from queue
                  txBuffer.erase(txBuffer.begin());
                }
                else if(rTbl[j].lifetime - std::chrono::system_clock::now() > DELETE_PERIOD) // Route is older than delete period
                {
                  rTbl.erase(rTbl.begin()+j); // Erase old route
                  newRoute(destIp); // Start new route procedure
                }
                else // Route has expired, but is not old enough to delete
                {
                  // Set status to invalid
                  rTbl[j].valid=false;
                  if(ROUTE_REPAIR)
                  {
                    // TODO: Route repair procedure 
                  }
                  else // Send RERR to precursers list 
                  {
                    // Unicast Route Error to every route in precursors list
                    for( int k = 0; rTbl[j].precursors.size(); k++)
                    {
                      sendRERR(rTbl[j].precursors[k], destIp);
                    }
                    routeInvalid(j, destIp); // Invalid route procedure
                  }
                }
              }
              else // Route invalid
              {                
                routeInvalid(j, destIp);
              }
            }
            else // Route not found. Start new
            {
              newRoute(destIp);
            }
          }
          else // Routing = None
          {
            meta = dict_add(meta, pmt::string_to_symbol("EM_DEST_ADDR"), pmt::from_long(255)); // Set dest ID
            meta = dict_add(meta, pmt::string_to_symbol("EM_USE_ARQ"), pmt::from_bool(true));  // Set ARQ
                          
            pmt::pmt_t msg_out = pmt::cons(meta, vect);
            message_port_pub(pmt::mp("to_mac"), msg_out);
            txBuffer.erase(txBuffer.begin());
          }
        }
      }
    }
    
    std::vector<unsigned char> route_impl::makeIP4Pkt(unsigned int sourceIp,
                                          unsigned int destIp,
                                          unsigned int ttl,
                                         unsigned char version,
                                         unsigned char ihl,
                                         unsigned char dscp,
                                         unsigned char ecn,
                                         unsigned char flags,
                                         unsigned char protocol,
                                        unsigned short totalLength,
                                        unsigned short fragmentOffset,
                                        unsigned short identfication,
                                        unsigned short headerChecksum)
    {
      unsigned short checksum;
      std::vector<unsigned char> pkt;
      pkt[0]=((version<<4) | (ihl&0x0F));
      pkt[1]=((dscp<<2) | (ecn&0x03));
      pkt[2]=static_cast<unsigned char>((totalLength & 0xFF00)>>8);
      pkt[3]=static_cast<unsigned char>(totalLength & 0x00FF);
      pkt[4]=static_cast<unsigned char>((identfication & 0xFF00)>>8);
      pkt[5]=static_cast<unsigned char>(identfication & 0x00FF);
      pkt[6]=((flags<<5)|static_cast<unsigned char>((fragmentOffset & 0x1F00)>>8));
      pkt[7]=static_cast<unsigned char>(fragmentOffset & 0x00FF);
      pkt[8]=ttl;
      pkt[9]=protocol;
      pkt[10]=0;
      pkt[11]=0;
      pkt[12]=static_cast<unsigned char>((sourceIp & 0xFF000000)>>24);
      pkt[13]=static_cast<unsigned char>((sourceIp & 0x00FF0000)>>16);
      pkt[14]=static_cast<unsigned char>((sourceIp & 0x0000FF00)>>8);
      pkt[15]=static_cast<unsigned char>(sourceIp & 0x000000FF);
      pkt[16]=static_cast<unsigned char>((destIp & 0xFF000000)>>24);
      pkt[17]=static_cast<unsigned char>((destIp & 0x00FF0000)>>16);
      pkt[18]=static_cast<unsigned char>((destIp & 0x0000FF00)>>8);
      pkt[19]=static_cast<unsigned char>(destIp & 0x000000FF);
      
      checksum=ip4_checksum(pkt);
      pkt[10]=static_cast<unsigned char>((checksum & 0xFF00)>>8);
      pkt[11]=static_cast<unsigned char>(checksum & 0x00FF);
      return pkt;
    }
    
    void route_impl::decTTL(std::vector<unsigned char> &ipPacket)
    {
      unsigned short checksum;
      ipPacket[8]--;
      checksum=ip4_checksum(ipPacket);
      ipPacket[10]=static_cast<unsigned char>((checksum&0xFF00)>>8);
      ipPacket[11]=static_cast<unsigned char>(checksum&0x00FF);
      return;
    }
          
    std::vector<unsigned char> makeUDPPkt (unsigned short srcPort,
                                           unsigned short destPort,
                                           unsigned short length,
                                           unsigned short checksum)
    {
      std::vector<unsigned char> pkt;
      pkt[0]=static_cast<unsigned char>((srcPort&0xFF00)>>8);
      pkt[1]=static_cast<unsigned char>(srcPort&0x00FF);
      pkt[2]=static_cast<unsigned char>((destPort&0xFF00)>>8);
      pkt[3]=static_cast<unsigned char>(destPort&0x00FF);
      pkt[4]=static_cast<unsigned char>((length&0xFF00)>>8);
      pkt[5]=static_cast<unsigned char>(length&0x00FF);
      pkt[6]=static_cast<unsigned char>((checksum&0xFF00)>>8);
      pkt[7]=static_cast<unsigned char>(checksum&0x00FF);
      return pkt;
    }
    
    std::vector<unsigned char> route_impl::makeRREQPkt (unsigned int rreqId, 
                                                        unsigned int destIp,
                                                                bool J, 
                                                                bool R,
                                                                bool G,
                                                                bool D,
                                                                bool U,
                                                        unsigned int destSeqNum,
                                                        unsigned int origIp,
                                                        unsigned int origSeqNum)
    {
      std::vector<unsigned char>(6*4) pkt;
      pkt[0]=1;
      pkt[1]= 0x00 | static_cast<unsigned char>(J)<<7 
          | static_cast<unsigned char>(R)<<6
          | static_cast<unsigned char>(G)<<5
          | static_cast<unsigned char>(D)<<4
          | static_cast<unsigned char>(U)<<3;
      pkt[2] = 0x00;
      pkt[3] = 0;
      pkt[4] = static_cast<unsigned char>((rreqId&0xFF000000)>>24);
      pkt[5] = static_cast<unsigned char>((rreqId&0x00FF00000>>16);
      pkt[6] = static_cast<unsigned char>((rreqId&0x0000FF00)>>8);
      pkt[7] = static_cast<unsigned char>(rreqId&0x000000FF);
      pkt[8] = static_cast<unsigned char>((destIp&0xFF000000)>>24);
      pkt[9] = static_cast<unsigned char>((destIp&0x00FF0000)>>16);
      pkt[10] = static_cast<unsigned char>((destIp&0x0000FF00)>>8);
      pkt[11] = static_cast<unsigned char>(destIp&0x000000FF);
      pkt[12] = static_cast<unsigned char>((destSeqNum&0xFF000000)>>24);
      pkt[13] = static_cast<unsigned char>((destSeqNum&0x00FF0000)>>16);
      pkt[14] = static_cast<unsigned char>((destSeqNum&0x0000FF00)>>8);
      pkt[15] = static_cast<unsigned char>(destSeqNum&0x000000FF);
      pkt[16] = static_cast<unsigned char>((origIp&0xFF000000)>>24);
      pkt[17] = static_cast<unsigned char>((origIp&0x00FF0000)>>16);
      pkt[18] = static_cast<unsigned char>((origIp&0x0000FF00)>>8);
      pkt[19] = static_cast<unsigned char>(origIp&0x000000FF);
      pkt[20] = static_cast<unsigned char>((origSeqNum&0xFF000000)>>24);
      pkt[21] = static_cast<unsigned char>((origSeqNum&0x00FF0000)>>16);
      pkt[22] = static_cast<unsigned char>((origSeqNum&0x0000FF00)>>8);
      pkt[23] = static_cast<unsigned char>(origSeqNum&0x000000FF);
      return pkt;
    }
    
    void forward(std::vector<unsigned char> pkt, unsigned int destIp, unsigned char nxtHop)
    {
      pmt::pmt_t outVect = pmt::init_u8vector (pkt.size(), pkt);
      pmt::pmt_t meta = pmt::make_dict();      
      meta = dict_add(meta, pmt::string_to_symbol("EM_DEST_ADDR"), pmt::from_long(static_cast<long>(nxtHop))); // Set dest ID
      meta = dict_add(meta, pmt::string_to_symbol("EM_USE_ARQ"), pmt::from_bool(true));  // Set ARQ   
      pmt::pmt_t msg_out = pmt::cons(meta, outVect);
      message_port_pub(pmt::mp("to_mac"), msg_out);
    }
    
    unsigned short route_impl::ip4_checksum(std::vector<unsigned char> &ipPacket)
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
      carry = static_cast<unsigned short>((accum& 0xFFFF0000)>>16);
      checksum = ~(static_cast<unsigned short>(accum & 0x0000FFFF) + carry);
      return checksum;
    }
    
    void route_impl::routeInvalid(int j/*rTbl Index*/, unsigned int destIp)
    {
      int k = 0;
      bool rreqFound = false;
      // Look for existing RREQ
      while(k<rreqTbl.size() && !rreqFound)
      {
        if(rreqTbl[k].destIp==destIp)
        {
          rreqFound = true;
        }
        else
        {
          k++;
        }
      }
      if(rreqFound) // Found RREQ. Could be old.
      {
        if(rreqTbl[k].lifetime > std::chrono::system_clock::now()) // RREQ is new
        {
          // Wait patiently for RREP
        }
        else // Route invalid // RREQ expired // resend
        {
          if(rreqTbl[k].retryCnt < RREQ_RETRIES) // Retries left try again
          {
            // Increment retryCnt
            rreqTbl[k].retryCnt++; 
            // Increase TTL using expanding ring search
            if(rreqTbl[k].ttl < TTL_THRESHOLD)
              rreqTbl[k].ttl += TTL_INCREMENT;
            else
              rreqTbl[k].ttl = TTL_MAX; 
            // Reset/increase search lifetime
            rreqTbl[k].waitTime *=2;
            rreqTbl[k].lifetime = std::chrono::system_clock::now() + rreqTbl[k].waitTime;
            // Increment rreqId
            hostRreqId++;
            // Update rreq ID
            rreqTbl[k].rreqId = hostRreqId;
            sendRREQ(destIp, rreqTbl[k].ttl, false, false, false, rTbl[j].destSeqNum);
          }
          else // Out of retries
          {
            std::cout << "Error: RREQ retry limit reached" << std::endl;
            std::cout << "       Dropping packet" << std::endl;
            rreqTbl.erase(rreqTbl.begin() + k); // Erase old RREQ
          }
        }
      }
      else // Invalid route. RREQ !found
      {
        // Update rTbl entry
        rTbl[j].destSeqNum++;
        // Make rreqTbl entry
        hostRreqId++; // check
        hostSeqNum++; // CHECK THESE
        rreqTblEntry entry = {destIp,hostRreqId, HOST_IP, rTbl[j].hopCnt, 0, std::chrono::system_clock::now() + NET_TRAVERSAL_TIME, NET_TRAVERSAL_TIME,false};
        rreqTbl.push_back(entry);
        sendRREQ(destIp, rTbl[j].hopCnt, false, false, false, rTbl[j].destSeqNum );
      }
      return;
    }
    
    void route_impl::sendRREQ(unsigned int destIp, unsigned char ttl, bool J, bool R,  bool U, unsigned int destSeqNum )
    {
      // Make rreq packet
      // Build IPv4 packet
      std::vector<uint8_t> pkt = makeIP4Pkt(HOST_IP,destIp,ttl);
      // Build UDP packet
      std::vector<uint8_t> udp = makeUDPPkt();
      // Build AODV RREQ packet
      std::vector<uint8_t> rreq = makeRREQPkt (
          hostRreqId, 
          destIp, 
          J, 
          R, 
          GRATUITOUS_RREP, 
          DEST_ONLY, 
          U,
          destSeqNum,
          HOST_IP,
          hostSeqNum);
      // Assemble packet
      pkt.insert(pkt.end(), udp.begin(), udp.end());
      pkt.insert(pkt.end(), rreq.begin(), rreq.end());
      // Publish to port
      pmt::pmt_t outVect = pmt::init_u8vector (pkt.size(), pkt);
      pmt::pmt_t meta = pmt::make_dict();
      meta = dict_add(meta, pmt::string_to_symbol("EM_DEST_ADDR"), pmt::from_long(static_cast<unsigned char>(255))); // Set dest ID
      meta = dict_add(meta, pmt::string_to_symbol("EM_USE_ARQ"), pmt::from_bool(true));  // Set ARQ
      pmt::pmt_t msg_out = pmt::cons(meta, outVect);
      message_port_pub(pmt::mp("to_mac"), msg_out);
      return;
    }
    
    void route_impl::sendRERR(unsigned int destIp, unsigned int unreachableIp, unsigned char nxtHop, bool N)
    {
      std::vector<std::vector<unsigned int>> list;
      std::vector<unsigned int> pair; 
      pair.push_back(destIp);
      pair.push_back(unreachableIp);
      list.push_back(pair);
      // Build RERR
      std::vector<unsigned char> rerr = makeRERRPkt(list, N);
      // Build IPv4 packet
      std::vector<uint8_t> pkt = makeIP4Pkt(HOST_IP,destIp,TTL_MAX);
      // Build UDP packet
      std::vector<uint8_t> udp = makeUDPPkt();
      // Assemble packet
      pkt.insert(pkt.end(), udp.begin(), udp.end());
      pkt.insert(pkt.end(), rerr.begin(), rerr.end());
      pmt::pmt_t outVect = pmt::init_u8vector (pkt.size(), pkt);
      pmt::pmt_t meta = pmt::make_dict();
      meta = dict_add(meta, pmt::string_to_symbol("EM_DEST_ADDR"), pmt::from_long(static_cast<long>(nxtHop))); // Set dest ID
      meta = dict_add(meta, pmt::string_to_symbol("EM_USE_ARQ"), pmt::from_bool(true));  // Set ARQ
      pmt::pmt_t msg_out = pmt::cons(meta, outVect);
      message_port_pub(pmt::mp("to_mac"), msg_out);
      return;
    }
    
    void route_impl::newRoute(unsigned int destIp)
    {
      // Make a new routing table entry
      std::vector<unsigned int> precursors;
      rTblEntry route = {destIp,0,false,false,ROUTE_REPAIR,false,0,0,precursors,(std::chrono::system_clock::now()+ACTIVE_ROUTE_TIMEOUT)};
      rTbl.push_back(route);
      // Increment host rreq id
      hostRreqId++;
      // Increment host sequence number
      hostSeqNum++;
      // Make a new rreq table entry
      rreqTblEntry rreqEntry = { destIp,
          hostRreqId,
          HOST_IP,
          TTL_START,
          0,
          std::chrono::system_clock::now()+NET_TRAVERSAL_TIME,
          NET_TRAVERSAL_TIME,
          false};
      rreqTbl.push_back(rreqEntry);
      sendRREQ(destIp, TTL_START, false, false, true, 0);
      return;
    }
    
    void route_impl::addRoute(unsigned int destIp,
                   unsigned int srcIp,
                   unsigned int destSeqNum,
                   bool validDestSeq,
                   bool valid,
                   bool repairable,
                   bool beingRepaired,
                   unsigned char hopCnt,
                   unsigned char nxtHop)
    {
      int i=0;
      bool routeFound = false;
      // Find  route
      while(i<rTbl.size() && !routeFound)
      {
        if(rTbl[i].destIp==destIp)
          routefound = true;
        else
          i++;
      }
      if(routeFound) // Route found
      {
        // Add srcIp to precursors list
        if(srcIp != HOST_IP) // Don't add your own IP as this will cause consfusion
        {
          int j=0;
          bool found = false;
          // Search for srcIp;
          while(j<rTbl.precursors.size() && !found)
          {
            if(rTbl[i].precursors[j]==srcIp)
              found = true;
            else
              j++;
          }
          if(!found) // Add to list if not present
            rTbl[i].precursors.push_back(srcIp);
        }
        // Refresh the route lifetime
        rTbl[i].lifetime = std::chrono::system_clock::now() + ACTIVE_ROUTE_TIMEOUT;
        // Update the reverse Sequence number
        if(destSeqNum > Tbl[i].destSeqNum)
        {
          rTbl[i].destSeqNum = destSeqNum;
          rTbl[i].hopCnt = hopCnt;
          rTbl[i].nxtHop = nxtHop;
        }
        else if(destSeqNum == Tbl[i].destSeqNum && hopCnt < Tbl[i].hopCnt)// Update the hop count
        {
          rTbl[i].hopCnt = hopCnt;
          rTbl[i].destSeqNum = origSeqNum;
          rTbl[i].nxtHop = nxtHop;
        }
        else
        {
          // Do nothing
        }
      }
      else // Route not found make new entry
      {
        std::vector<unsigned int> temp;
        rTblEntry revRoute = {destIp,destSeqNum,validDestSeq, valid, repairable, beingRepaired, hopCnt,nxtHop,temp,std::chrono::system_clock::now() + ACTIVE_ROUTE_TIMEOUT};
        rTbl.push_back(revRoute)
      }
    }
    
    void sendRREP(bool repair, 
                  unsigned int destIp,  
                  unsigned int origIp,
                  unsigned char hopCnt)
    {
      int i=0;
      bool found = false;
      // Find  route
      while(i<rTbl.size() && !found)
      {
        if(rTbl[i].destIp==origIp) // May need verification
          found = true;
        else
          i++;
      }
      if(found)
      {
        // Refresh route lifetime
        rTbl[i].lifetime = std::chrono::system_clock::now() + ACTIVE_ROUTE_TIMEOUT;
        // Build RREP
        std::vector<unsigned char> rrep = makeRREPPkt(repair,ROUTE_ACK,hopCnt,HOST_IP,HOST_SEQ_NUM,origIp,static_cast<unsigned int>(ACTIVE_ROUTE_TIMEOUT));
        // Build IPv4 packet
        std::vector<uint8_t> pkt = makeIP4Pkt(HOST_IP,origIp,TTL_MAX);
        // Build UDP packet
        std::vector<uint8_t> udp = makeUDPPkt();
        // Assemble packet
        pkt.insert(pkt.end(), udp.begin(), udp.end());
        pkt.insert(pkt.end(), rrep.begin(), rrep.end());
        pmt::pmt_t outVect = pmt::init_u8vector (pkt.size(), pkt);
        pmt::pmt_t meta = pmt::make_dict();
        meta = dict_add(meta, pmt::string_to_symbol("EM_DEST_ADDR"), pmt::from_long(static_cast<long>(rTbl[i].nxtHop))); // Set dest ID
        meta = dict_add(meta, pmt::string_to_symbol("EM_USE_ARQ"), pmt::from_bool(true));  // Set ARQ
        pmt::pmt_t msg_out = pmt::cons(meta, outVect);
        message_port_pub(pmt::mp("to_mac"), msg_out);
      }
    }
    
    void sendGRREP(unsigned int destIp, unsigned char ttl, bool J, bool R, bool U, unsigned int destSeqNum )
    {
      int i=0;
      int j=0;
      bool found = false;
      // Find  route to destination
      while(i<rTbl.size() && !found)
      {
        if(rTbl[i].destIp==destIp)
          found = true;
        else
          i++;
      }
      // Find route to source
      found = false;
      while(j<rTbl.size() && !found)
      {
        if(rTbl[j].destIp==origIp)
          found = true;
        else
          j++;
      }
      
      if(found)
      {
        // Refresh route lifetime
        rTbl[i].lifetime = std::chrono::system_clock::now() + ACTIVE_ROUTE_TIMEOUT;
        // Build RREP
        std::vector<unsigned char> rrep = makeRREPPkt(R,ROUTE_ACK,0,rTbl[j].hopCnt,destIp,rTbl[i].destSeqNum,origIp,static_cast<unsigned int>(ACTIVE_ROUTE_TIMEOUT));
        // Build IPv4 packet
        std::vector<uint8_t> pkt = makeIP4Pkt(HOST_IP,destIp,TTL_MAX);
        // Build UDP packet
        std::vector<uint8_t> udp = makeUDPPkt();
        // Assemble packet
        pkt.insert(pkt.end(), udp.begin(), udp.end());
        pkt.insert(pkt.end(), rrep.begin(), rrep.end());
        pmt::pmt_t outVect = pmt::init_u8vector (pkt.size(), pkt);
        pmt::pmt_t meta = pmt::make_dict();
        meta = dict_add(meta, pmt::string_to_symbol("EM_DEST_ADDR"), pmt::from_long(static_cast<long>(rTbl[i].nxtHop))); // Set dest ID
        meta = dict_add(meta, pmt::string_to_symbol("EM_USE_ARQ"), pmt::from_bool(true));  // Set ARQ
        pmt::pmt_t msg_out = pmt::cons(meta, outVect);
        message_port_pub(pmt::mp("to_mac"), msg_out);
      }
    }
    
    std::vector<unsigned char> makeRERRPkt(std::vector<std::vector<unsigned int>> pair, bool N)
    {
      std::vector<unsigned char>(1+8*pair.size()) pkt;
      pkt[0] = 3;
      pkt[1] = static_cast<unsigned char>(N)<<7;
      pkt[2] = 0;
      pkt[3] = static_cast<unsigned char>(pair.size());
      for(int i=0; i<pair.size(); i++)
      {
        // Unreachable dest IP (i)
        pkt[(i*8)+4] = static_cast<unsigned char>((pair[i][0] & 0xFF000000)>>(3*8));
        pkt[(i*8)+5] = static_cast<unsigned char>((pair[i][0] & 0x00FF0000)>>(2*8));
        pkt[(i*8)+6] = static_cast<unsigned char>((pair[i][0] & 0x0000FF00)>>(8));
        pkt[(i*8)+7] = static_cast<unsigned char>((pair[i][0] & 0x000000FF));
        // Unreachable dest Sequence Number (i)
        pkt[(i*8)+8] = static_cast<unsigned char>((pair[i][1] & 0xFF000000)>>(3*8));
        pkt[(i*8)+9] = static_cast<unsigned char>((pair[i][1] & 0x00FF0000)>>(2*8));
        pkt[(i*8)+10] = static_cast<unsigned char>((pair[i][1] & 0x0000FF00)>>(8));
        pkt[(i*8)+11] = static_cast<unsigned char>((pair[i][1] & 0x000000FF));
      }
    }
    
    std::vector<unsigned char> makeRREPPkt(bool repair, 
                                           bool ack, 
                                           unsigned char prefixSz, 
                                           unsigned char hopCnt, 
                                           unsigned int destIp, 
                                           unsigned int destSeqNum, 
                                           unsigned int origIp, 
                                           unsigned int lifetime)
    {
      std::vector<unsigned char>(20) pkt;
      pkt[0] = 2;
      pkt[1] = static_cast<unsigned char>(repair)<<7 | static_cast<unsigned char>(ack)<<7;
      pkt[2] = prefixSz * 0x1F;
      pkt[3] = hopCnt;
      pkt[4] = static_cast<unsigned char>((destIp&0xFF000000)>>24);
      pkt[5] = static_cast<unsigned char>((destIp&0x00FF0000)>>16);
      pkt[6] = static_cast<unsigned char>((destIp&0x0000FF00)>>8);
      pkt[7] = static_cast<unsigned char>(destIp&0x000000FF);
      pkt[8] = static_cast<unsigned char>((destSeqNum&0xFF000000)>>24);
      pkt[9] = static_cast<unsigned char>((destSeqNum&0x00FF0000)>>16);
      pkt[10] = static_cast<unsigned char>((destSeqNum&0x0000FF00)>>8);
      pkt[11] = static_cast<unsigned char>(destSeqNum&0x000000FF);
      pkt[12] = static_cast<unsigned char>((origIp&0xFF000000)>>24);
      pkt[13] = static_cast<unsigned char>((origIp&0x00FF0000)>>16);
      pkt[14] = static_cast<unsigned char>((origIp&0x0000FF00)>>8);
      pkt[15] = static_cast<unsigned char>(origIp&0x000000FF);
      pkt[16] = static_cast<unsigned char>((lifetime&0xFF000000)>>24);
      pkt[17] = static_cast<unsigned char>((lifetime&0x00FF0000)>>16);
      pkt[18] = static_cast<unsigned char>((lifetime&0x0000FF00)>>8);
      pkt[19] = static_cast<unsigned char>(lifetime&0x000000FF);
      return pkt;
    }
    
  } /* namespace MST */
} /* namespace gr */

