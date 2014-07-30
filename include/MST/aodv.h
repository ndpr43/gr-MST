#ifndef AODV_H
#define AODV_H

#include <chrono>
#include <ctime>
 
struct rTblEntry
{
  unsigned int destIp;
  int destSeqNum;
  bool validDestSeq;
  bool valid;
  bool repairable;
  bool beingRepaired;
  //unsigned char nic;
  unsigned char hopCnt;
  unsigned int nxtHop; // MAC Address
  std::vector<unsigned int> precursors;
  std::chrono::time_point<std::chrono::system_clock> lifetime; // in msec
};

struct rreqTblEntry
{
 unsigned int destIp;
 unsigned int rreqId;
 unsigned int srcIp;
 unsigned char ttl;
 char retryCnt;
 std::chrono::time_point<std::chrono::system_clock> lifetime;
 std::chrono::milliseconds waitTime;
 bool repair;
};

/*
struct rxrreqTblEntry
{
  unsigned int rreqId;
  unsigned int destIp;
  unsigned int srcIp;
  int destSeqNum;
  int time;
};
*/

// #defines
#define BLACKLIST_TIMEOUT (RREQ_RETRIES * NET_TRAVERSAL_TIME)
#define MAX_REPAIR_TTL (0.3 * NET_DIAMETER)
#define MY_ROUTE_TIMEOUT (2 * ACTIVE_ROUTE_TIMEOUT)
//#define NEXT_HOP_WAIT NODE_TRAVERSAL_TIME +10
#define NET_TRAVERSAL_TIME (2 * NODE_TRAVERSAL_TIME * NET_DIAMETER) //Period of time to wait before resending a RREQ
#define PATH_DISCOVER_TIME (2 * NET_TRAVERSAL_TIME) //Period of time to wait before deleting a RREQ
#define RING_TRAVERSAL_TIME (2 * NODE_TRAVERSAL_TIME * (TTL_VALUE + TIMEOUT_BUFFER))//FIXME

//Constants--Make sure to include extern calls
// FLAG CONSTANTS
//RREQ
const unsigned char RREQ_JOIN_FLAG = 1<<7; //Reserved for multicast;
const unsigned char RREQ_REPAIR_FLAG = 1<<6; //Reserved for multicast;
const unsigned char RREQ_GRATUITIOUS_FLAG = 1<<5;
const unsigned char RREQ_DEST_ONLY_FLAG = 1<<4;
const unsigned char RREQ_UNK_SEQ_NUM_FLAG = 1<<3;

//RREP
const unsigned char RREP_REPAIR_FLAG = 1<<7; //Reserved for multicast;
const unsigned char RREP_ACK_FLAG = 1<<6;

//RERR
const unsigned char	RERR_NO_DEL_FLAG = 1<<7;

// PKT INDEX CONSTANTS
//gr-mac packet indexes
const unsigned char PKT_INDEX_CNT = 0;
const unsigned char PKT_INDEX_SRC = 1;
const unsigned char PKT_INDEX_DEST = 2;
const unsigned char PKT_INDEX_PROT_ID = 3;
const unsigned char PKT_INDEX_CTRL = 4;
const unsigned char PKT_INDEX_MAX = 5 ; // Packet length

//AODV packet indexes
// Uniform first byte
const unsigned char PKT_INDEX_TYPE = 0;
const unsigned char PKT_INDEX_FLAGS = 1;
const unsigned char PKT_INDEX_PREFIX_SZ = 2;
const unsigned char PKT_INDEX_HOP_CNT = 3;
// RREQ
const unsigned int PKT_INDEX_RREQ_RREQ_ID = 1;
const unsigned int PKT_INDEX_RREQ_DEST_IP = 2;
const unsigned int PKT_INDEX_RREQ_DEST_SEQ_NUM = 3;
const unsigned int PKT_INDEX_RREQ_ORIG_IP = 4;
const unsigned int PKT_INDEX_RREQ_ORIG_SEQ_NUM = 5;
// RREP
const unsigned int PKT_INDEX_RREP_DEST_IP = 1;
const unsigned int PKT_INDEX_RREP_DEST_SEQ_NUM = 2; 
const unsigned int PKT_INDEX_RREP_ORIG_IP = 3;
const unsigned int PKT_INDEX_RREP_LIFETIME = 4;
// RERR
const unsigned int PKT_INDEX_RERR_UDEST_IP = 1; 
const unsigned int PKT_INDEX_RERR_UDEST_SEQ_NUM = 2;
// DATA
const unsigned int PKT_INDEX_DATA_DEST_ID = 1; 
const unsigned int PKT_INDEX_DATA_ORIG_ID = 2;

 
#endif /* AODV_H */
