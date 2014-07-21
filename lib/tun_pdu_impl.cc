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

#ifdef HAVE_IO_H
#include <io.h>
#endif

#include "tun_pdu_impl.h"
#include <gnuradio/io_signature.h>
#include <gnuradio/blocks/pdu.h>
#include <boost/format.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#if (defined(linux) || defined(__linux) || defined(__linux__))
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <linux/if.h>

#endif

static const long timeout_us = 100*1000; //100ms

namespace gr {
  namespace MST {

    tun_pdu::sptr
    tun_pdu::make(std::string dev, int MTU)
    {
#if (defined(linux) || defined(__linux) || defined(__linux__))
      return gnuradio::get_initial_sptr(new tun_pdu_impl(dev, MTU));
#else
      throw std::runtime_error("tun_pdu not implemented on this platform");
#endif
    }

#if (defined(linux) || defined(__linux) || defined(__linux__))
    tun_pdu_impl::tun_pdu_impl(std::string dev, int MTU)
      : gr::block("tun_pdu",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(0, 0, 0)),
              d_fd(-1), d_started(false), d_finished(false), d_dev(dev)
    {
      // reserve space for rx buffer 
      d_rxbuf.resize(MTU,0);
      // make the tuntap
      char dev_cstr[1024];
      memset(dev_cstr, 0x00, 1024);
      strncpy(dev_cstr, dev.c_str(), std::min(sizeof(dev_cstr), dev.size()));

      d_fd = tun_alloc(dev_cstr,IFF_TUN | IFF_NO_PI);
      if (d_fd <= 0)
        throw std::runtime_error("gr::tun_pdu::make: tun_alloc failed (are you running as root?)");

      std::cout << boost::format(
	"Allocated virtual ethernet interface: %s\n"
        "You must now use ifconfig to set its IP address. E.g.,\n"
        "  $ sudo ifconfig %s 192.168.200.1\n"
        "Be sure to use a different address in the same subnet for each machine.\n"
        ) % dev % dev << std::endl;

      // set up output message port
      message_port_register_out(PDU_PORT_ID);
      start_rxthread(this, PDU_PORT_ID);
    
      // set up input message port
      message_port_register_in(PDU_PORT_ID);
      set_msg_handler(PDU_PORT_ID, 
    		  boost::bind(&tun_pdu_impl::send, this, _1));
    }

    tun_pdu_impl::~tun_pdu_impl()
    {
    	stop_rxthread();
    }
    
    int
    tun_pdu_impl::tun_alloc(char *dev, int flags)
    {
      struct ifreq ifr;
      int fd, err;
      const char *clonedev = "/dev/net/tun";

      /* Arguments taken by the function:
       *
       * char *dev: the name of an interface (or '\0'). MUST have enough
       *   space to hold the interface name if '\0' is passed
       * int flags: interface flags (eg, IFF_TUN etc.)
       */

      /* open the clone device */
      if ((fd = open(clonedev, O_RDWR)) < 0)
        return fd;

      /* preparation of the struct ifr, of type "struct ifreq" */
      memset(&ifr, 0, sizeof(ifr));

      ifr.ifr_flags = flags;   /* IFF_TUN or IFF_TAP, plus maybe IFF_NO_PI */

      /* if a device name was specified, put it in the structure; otherwise,
       * the kernel will try to allocate the "next" device of the
       * specified type
       */
      if (*dev)
        strncpy(ifr.ifr_name, dev, IFNAMSIZ);

      /* try to create the device */
      if ((err = ioctl(fd, TUNSETIFF, (void *) &ifr)) < 0) {
        close(fd);
        return err;
      }

      /* if the operation was successful, write back the name of the
       * interface to the variable "dev", so the caller can know
       * it. Note that the caller MUST reserve space in *dev (see calling
       * code below)
       */
      strcpy(dev, ifr.ifr_name);

      /* this is the special file descriptor that the caller will use to talk
       * with the virtual interface
       */
      return fd;
    }
    
    void
        tun_pdu_impl::start_rxthread(basic_block *blk, pmt::pmt_t port)
        {
          d_blk = blk;
          d_port = port;
          d_thread = gr::thread::thread(boost::bind(&tun_pdu_impl::run, this));
          d_started = true;
        }

        void
        tun_pdu_impl::stop_rxthread()
        {
          d_finished = true;

          if (d_started) {
            d_thread.interrupt();
            d_thread.join();
          }
        }

        void
        tun_pdu_impl::run()
        {
          while(!d_finished) {
            if (!wait_ready())
    	  continue;

            const int result = read(d_fd, &d_rxbuf[0], d_rxbuf.size());
            if (result <= 0)
    	  throw std::runtime_error("tun_pdu_impl, bad socket read!");

            pmt::pmt_t vector = pmt::init_u8vector(result, &d_rxbuf[0]);       
            pmt::pmt_t pdu = pmt::cons(pmt::PMT_NIL, vector);

            d_blk->message_port_pub(d_port, pdu);
          } 
        }

        bool
        tun_pdu_impl::wait_ready()
        {
          //setup timeval for timeout
          timeval tv;
          tv.tv_sec = 0;
          tv.tv_usec = timeout_us;
        
          //setup rset for timeout
          fd_set rset;
          FD_ZERO(&rset);
          FD_SET(d_fd, &rset);
          
          //call select with timeout on receive socket
          return ::select(d_fd+1, &rset, NULL, NULL, &tv) > 0;
        }

        void
        tun_pdu_impl::send(pmt::pmt_t msg)
        {
          pmt::pmt_t vector = pmt::cdr(msg);
          size_t offset(0);
          //size_t itemsize(gr::blocks::pdu::itemsize(gr::blocks::pdu::type_from_pmt(vector)));
          size_t itemsize = sizeof(char);
          int len(pmt::length(vector)*itemsize);
        
          const int rv = write(d_fd, pmt::uniform_vector_elements(vector, offset), len);
          if (rv != len) {
            std::cerr << boost::format("WARNING: tun_pdu_impl::send(pdu) write failed! (d_fd=%d, len=%d, rv=%d)")
    	  % d_fd % len % rv << std::endl;
          }
        }
#endif

  } /* namespace MST */
} /* namespace gr */

