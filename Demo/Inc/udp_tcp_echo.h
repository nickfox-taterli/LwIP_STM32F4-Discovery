#ifndef __UDP_TCP_ECHO_SERVER__
#define __UDP_TCP_ECHO_SERVER__

#define UDPECHO_THREAD_PRIO  ( tskIDLE_PRIORITY + 4 )
#define TCPECHO_THREAD_PRIO  ( tskIDLE_PRIORITY + 4 )

void udpecho_thread(void *arg);
void tcpecho_thread(void *arg);
void udplite_thread(void *arg);
	
#endif
