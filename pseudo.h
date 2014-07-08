// Data structures
/* Routing table
struct RTbEntry
{
  char dest;
  int destSeqNum;
  bool validDestSeq;
  bool validR;
  char nic;
  char hopCnt;
  char nxtHop;
  char precursors[8];
  int lifetime;
};

struct rreqTblEntry
{
 char dest;
 char retryCnt;
 int lifetime;
};
*/


//event handler receive packet from virtual encoder
/*
  Read pdu meta
  Read pdu data
  Construct AODV header
  prepend AODV header with data packet
  Buffer packet
*/

/* Standard tx subroutine
read top packet from buffer
read adov header
if (route) {
	modify mac header's destination
	tag with appropriate meta
	send pdu arq route
	}
else {
	if(RREQ)
	{
		Update lifetime
		if(expired)
		{
			if(retryCnt < Max )
			{
				retryCnt++
				increase ttl
				reset lifetime
				create rreq
				tag with appropriate meta (bcast)
				send pdu
			}
			else
			{
				Drop data
				Print warning
			}
		}
		else {
			// Do nothing and wait for rrep
		}
	}
	else // NO RREQ
	{ 
		create table entry
		create rreq
		tag with appropriate meta (bcast)
		send pdu
	}
}
*/

//event handler receive packet from channel
/*
Read meta data
Update route table
Read AODV header
//Is it me?
if(AODV.dest == self.addr)
{
	if(AODV.type == RREQ)
	{
		RREQ handling procedure //TODO		
	}
	else if(AODV.type == RREP)
	{
		RREP handling procedure //TODO
	}
	else if(AODV.type == RERR)
	{
		RERR handling procedure //TODO
	}
	else if (AODV.type == RREP-ACK)
	{
		RREP-ACK handling procedure //TODO
	}
	else if (AODV.type == DATA)
	{
		strip AODV header
		construct pdu
		append meta data
		send pdu
	}
	else
	{
		throw error unrecognised AODV.type
	}
}
else // AODV.dest!=self.addr) forward
{
	TTL--;
	if(TTL > 0)
	{
		update AODV header
		Buffer
	}
	
}
	
*/