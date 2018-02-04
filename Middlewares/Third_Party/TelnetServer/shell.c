/**
  ******************************************************************************
  * @file    shell.c
  * @author  TaterLi
  * @brief   LwIP Netconn Telnet Server
  ******************************************************************************
  */

#include "shell.h"

#include "lwip/opt.h"

#include <string.h>
#include <stdio.h>

#include "lwip/mem.h"
#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/api.h"
#include "lwip/stats.h"

#define TELNET_BUFSIZE             128 /* 大则不溢出,小则节约,最理想是刚好容纳一条指令. */
static unsigned char *telnet_buffer;

struct command
{
    struct netconn *conn;
    int8_t (* exec)(struct command *);
    uint8_t nargs;
    char *args[5]; /* 最多5参数 */
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

static int8_t shell_com_echo(struct command *com)
{
    char *str = pvPortMalloc(TELNET_BUFSIZE - 4);
    sprintf(str, "echo %s - %s\r\n", com->args[0], com->args[1]); /* 参数0直接回去. */
    shell_sendstr(str, com->conn);
    return ESUCCESS;
}

static int8_t shell_parse_command(struct command *com, u32_t len)
{
    uint16_t i;
    uint16_t bufp;

    if (strncmp((const char *)telnet_buffer, "echo", 4) == 0)
    {
        com->exec = shell_com_echo;
        com->nargs = 2; /* 所需参数总量 */
    }
    else if (strncmp((const char *)telnet_buffer, "hello", 5) == 0) /* 反复比较字符串 */
    {
        com->exec = shell_com_hello;
        com->nargs = 0; /* 如果有参数还要填arg */
    }
    else if (strncmp((const char *)telnet_buffer, "quit", 4) == 0)
    {
        return ECLOSED;
    }
    else
    {
        return ESYNTAX;
    }

    if (com->nargs == 0)
    {
        return ESUCCESS;
    }

    /* 找参数同时给Buf不足容错,但是目前参数不够会莫名其妙卡死. */

    for(bufp = 0; bufp < len && telnet_buffer[bufp] != ' '; bufp++);
    for(i = 0; i < 5; i++) /* 遍历5个参数 */
    {
        for(; bufp < len && telnet_buffer[bufp] == ' '; bufp++); /* 把指针移动到空格位置 */
        if (telnet_buffer[bufp] == '\r' ||
                telnet_buffer[bufp] == '\n')   /* 遇到回车,命令完成.途径多少个i,就是有多少个参数. */
        {
            telnet_buffer[bufp] = 0;
            if (i < com->nargs - 1)
            {
                return ETOOFEW; /* 参数过少 */
            }
            if (i > com->nargs - 1)
            {
                return ETOOMANY; /* 参数过多 */
            }
            break;
        }
        if (bufp > len)
        {
            return ETOOFEW;
        }
        com->args[i] = (char *)&telnet_buffer[bufp];
        for(; bufp < len && telnet_buffer[bufp] != ' ' && telnet_buffer[bufp] != '\r' &&
                telnet_buffer[bufp] != '\n'; bufp++)
        {
            if (telnet_buffer[bufp] == '\\')
            {
                telnet_buffer[bufp] = ' ';
            }
        }
        if (bufp > len)
        {
            return ESYNTAX;
        }
        telnet_buffer[bufp] = 0;
        bufp++;
        if (i == com->nargs - 1) /* 加到参数足够,跳出就好. */
        {
            break;
        }

    }

    return ESUCCESS;
}

static void shell_error(s8_t err, struct netconn *conn)
{
    switch (err)
    {
    case ESYNTAX:
        /* shell_sendstr("## Command not found!\r\n", conn); */
        /* 通常只是因为回车等未输入完整. */
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
    err_t err, ret;
    struct pbuf *p;
    uint16_t len = 0, cur_len;
    struct command com;

    conn = netconn_new(NETCONN_TCP); /* Telnet 是 TCP 方式,作为服务器来Listen一下. */
    netconn_bind(conn, NULL, 23);
    netconn_listen(conn);

    while (1)
    {
        err = netconn_accept(conn, &newconn);
        telnet_buffer = pvPortMalloc(TELNET_BUFSIZE);
        shell_sendstr("Welcome to STM32 Telnet Server by TaterLi.type \"hello\" for Hello World.\r\n\r\nSTM32 Telnet > ", newconn);
        if (err == ERR_OK)
        {
            do
            {
                ret = netconn_recv_tcp_pbuf(newconn, &p); /* 直接获取到pbuf */
                if (ret == ERR_OK)
                {
                    pbuf_copy_partial(p, &telnet_buffer[len], TELNET_BUFSIZE - len, 0); /* 复制收到的内容,如果输入很长还不回车,len不会重置就溢出. */
                    cur_len = p->tot_len;
                    len += cur_len;

                    pbuf_free(p);
                    if (((len > 0) && ((telnet_buffer[len - 1] == '\r') || (telnet_buffer[len - 1] == '\n'))) ||
                            (len >= TELNET_BUFSIZE))
                    {
                        if (telnet_buffer[0] != 0xff &&
                                telnet_buffer[1] != 0xfe) /* 判断下,不是首次握手. */
                        {
                            err = shell_parse_command(&com, len); /* 解释命令,命令正在buf的开头写着. */
                            if (err == ESUCCESS)
                            {
                                com.conn = newconn;
                                err = com.exec(&com);
                            }
                            if (err == ECLOSED)
                            {
                                shell_error(err, newconn);
                                goto close;
                            }
                            if (err != ESUCCESS)
                            {
                                shell_error(err, newconn);
                            }
                        }
                        shell_sendstr("STM32 Telnet > ", newconn); /* 输出命令提示符. */
                        len = 0; /* 处理完指针归零. */
                    }
                }
            }
            while (ret == ERR_OK);

close:
            netconn_close(conn);
            netconn_delete(newconn);
            vPortFree(telnet_buffer);
        }
    }
}
