ifconfig tun1 192.168.1.2 promisc 

arp -sD 192.168.1.1 tun1 pub

