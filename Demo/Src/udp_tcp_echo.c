#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/sys.h"

static struct netconn *udpconn;
static struct netbuf *udpbuf;
static ip_addr_t *udpaddr;
static unsigned short udpport;

void udpecho_thread(void *arg)
{
  err_t err, recv_err;
  
  LWIP_UNUSED_ARG(arg);

  udpconn = netconn_new(NETCONN_UDP);
  if (udpconn!= NULL)
  {
    err = netconn_bind(udpconn, IP_ADDR_ANY, 7001);
    if (err == ERR_OK)
    {
      while (1) 
      {
        recv_err = netconn_recv(udpconn, &udpbuf);
      
        if (recv_err == ERR_OK) 
        {
          udpaddr = netbuf_fromaddr(udpbuf);
          udpport = netbuf_fromport(udpbuf);
          netconn_connect(udpconn, udpaddr, udpport);
          udpbuf->addr.addr = 0;
          netconn_send(udpconn,udpbuf);
          netbuf_delete(udpbuf);
        }
      }
    }
    else
    {
      netconn_delete(udpconn);
    }
  }
}

void tcpecho_thread(void *arg)
{
  struct netconn *conn, *newconn;
  err_t err, accept_err;
  struct netbuf *buf;
  void *data;
  u16_t len;
      
  LWIP_UNUSED_ARG(arg);

  /* Create a new connection identifier. */
  conn = netconn_new(NETCONN_TCP);
  
  if (conn!=NULL)
  {  
    /* Bind connection to well known port number 7002. */
    err = netconn_bind(conn, NULL, 7002);
    
    if (err == ERR_OK)
    {
      /* Tell connection to go into listening mode. */
      netconn_listen(conn);
    
      while (1) 
      {
        /* Grab new connection. */
         accept_err = netconn_accept(conn, &newconn);
    
        /* Process the new connection. */
        if (accept_err == ERR_OK) 
        {

          while (netconn_recv(newconn, &buf) == ERR_OK) 
          {
            do 
            {
              netbuf_data(buf, &data, &len);
              netconn_write(newconn, data, len, NETCONN_COPY);
          
            } 
            while (netbuf_next(buf) >= 0);
          
            netbuf_delete(buf);
          }
        
          /* Close connection and discard connection identifier. */
          netconn_close(newconn);
          netconn_delete(newconn);
        }
      }
    }
    else
    {
      netconn_delete(newconn);
    }
  }
}
