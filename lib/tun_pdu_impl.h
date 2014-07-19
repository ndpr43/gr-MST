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

#ifndef INCLUDED_MST_TUN_PDU_IMPL_H
#define INCLUDED_MST_TUN_PDU_IMPL_H

#include <gnuradio/thread/thread.h>
#include <pmt/pmt.h>
#include <MST/tun_pdu.h>

#if (defined(linux) || defined(__linux) || defined(__linux__))
#include <linux/if_tun.h>
#endif

namespace gr {
  namespace MST {

    class tun_pdu_impl : public tun_pdu
    {
#if (defined(linux) || defined(__linux) || defined(__linux__))
    private:
      std::string d_dev;
      int tun_alloc(char *dev, int flags = IFF_TAP | IFF_NO_PI);
      ///////////////
      int d_fd;
      bool d_started;
      bool d_finished;
      std::vector<uint8_t> d_rxbuf;
      gr::thread::thread d_thread;

      pmt::pmt_t d_port;
      basic_block *d_blk;

      void run();
      void send(pmt::pmt_t msg);
      bool wait_ready();
      void start_rxthread(basic_block *blk, pmt::pmt_t rxport);
      void stop_rxthread();
      /////////////////
     public:
      tun_pdu_impl(std::string dev, int MTU);
      ~tun_pdu_impl();
#endif
    };

  } // namespace MST
} // namespace gr

#endif /* INCLUDED_MST_TUN_PDU_IMPL_H */

