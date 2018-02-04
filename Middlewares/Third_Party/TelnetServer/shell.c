#include "shell.h"

#include "lwip/opt.h"

#include <string.h>
#include <stdio.h>

#include "lwip/mem.h"
#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/api.h"
#include "lwip/stats.h"

#define TELNET_BUFSIZE             128
static unsigned char telnet_buffer[TELNET_BUFSIZE];

struct command {
  struct netconn *conn;
  int8_t (* exec)(struct command *);
  u8_t nargs;
  char *args[10];
};

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#define ESUCCESS 0
#define ESYNTAX -1
#define ETOOFEW -2
#define ETOOMANY -3
#define ECLOSED -4

static void shell_sendstr(const char *str, struct netconn *conn)
{
  netconn_write(conn, (void *)str, strlen(str), NETCONN_COPY);
}

static int8_t shell_com_hello(struct command *com)
{
  shell_sendstr("Hello,Server by TaterLi.type \"quit\" for close.\r\n", com->conn); /* Say Hello ~ */
  return ESUCCESS;
}

static int8_t shell_parse_command(struct command *com, u32_t len)
{
  uint16_t i;
  uint16_t bufp;
  
	if (strncmp((const char *)telnet_buffer, "hello", 5) == 0) {
    com->exec = shell_com_hello;
    com->nargs = 0;
  } else if (strncmp((const char *)telnet_buffer, "quit", 4) == 0) {
    return ECLOSED;
  } else {
    return ESYNTAX;
  }

  if (com->nargs == 0) {
    return ESUCCESS;
  }
  bufp = 0;
  for(; bufp < len && telnet_buffer[bufp] != ' '; bufp++);
  for(i = 0; i < 10; i++) {
    for(; bufp < len && telnet_buffer[bufp] == ' '; bufp++);
    if (telnet_buffer[bufp] == '\r' ||
			telnet_buffer[bufp] == '\n') { /* 不是一直回车 */
      telnet_buffer[bufp] = 0;
      if (i < com->nargs - 1) {
        return ETOOFEW;
      }
      if (i > com->nargs - 1) {
        return ETOOMANY;
      }
      break;
    }    
    if (bufp > len) {
      return ETOOFEW;
    }    
    com->args[i] = (char *)&telnet_buffer[bufp];
    for(; bufp < len && telnet_buffer[bufp] != ' ' && telnet_buffer[bufp] != '\r' &&
      telnet_buffer[bufp] != '\n'; bufp++) {
      if (telnet_buffer[bufp] == '\\') {
        telnet_buffer[bufp] = ' ';
      }
    }
    if (bufp > len) {
      return ESYNTAX;
    }
    telnet_buffer[bufp] = 0;
    bufp++;
    if (i == com->nargs - 1) {
      break;
    }

  }

  return ESUCCESS;
}

static void shell_error(s8_t err, struct netconn *conn)
{
  switch (err) {
  case ESYNTAX:
    shell_sendstr("## Command not found!\r\n", conn);
    break;
  case ETOOFEW:
    shell_sendstr("## Too few arguments to command given!\r\n", conn);
    break;
  case ETOOMANY:
    shell_sendstr("## Too many arguments to command given!\r\n", conn);
    break;
  case ECLOSED:
    shell_sendstr("## Connection closed!\r\n", conn);
    break;
  default:
    break;
  }
}

void shell_thread(void *arg)
{
  struct netconn *conn, *newconn;
  err_t err,ret;
  struct pbuf *p;
  uint16_t len = 0, cur_len;
  struct command com;

  conn = netconn_new(NETCONN_TCP); /* Telnet 是 TCP 方式,作为服务器来Listen一下. */
  netconn_bind(conn, NULL, 23);
  netconn_listen(conn);

  while (1) {
    err = netconn_accept(conn, &newconn);
    if (err == ERR_OK) {
			
			  do {
    ret = netconn_recv_tcp_pbuf(newconn, &p);
    if (ret == ERR_OK) {
      pbuf_copy_partial(p, &telnet_buffer[len], TELNET_BUFSIZE - len, 0);
      cur_len = p->tot_len;
      len += cur_len;
			
      pbuf_free(p);
      if (((len > 0) && ((telnet_buffer[len-1] == '\r') || (telnet_buffer[len-1] == '\n'))) ||
          (len >= TELNET_BUFSIZE)) {
        if (telnet_buffer[0] != 0xff && 
           telnet_buffer[1] != 0xfe) {
          err = shell_parse_command(&com, len);
          if (err == ESUCCESS) {
            com.conn = newconn;
            err = com.exec(&com);
          }
          if (err == ECLOSED) {
            shell_error(err, newconn);
            goto close;
          }
          if (err != ESUCCESS) {
            shell_error(err, newconn);
          }
        } else {
          shell_sendstr("Welcome to STM32 Telnet Server by TaterLi.type \"hello\" for Hello World.\r\n", newconn);
        }
        if (ret == ERR_OK) {
          shell_sendstr("STM32 Telnet > ", newconn);
        }
        len = 0;
      }
    }
  } while (ret == ERR_OK);
				
	close:
  netconn_close(conn);
				
      netconn_delete(newconn);
    }
  }
}
