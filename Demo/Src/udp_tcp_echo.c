#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/sys.h"
#include "lwip/dns.h"

#include "lwip/netif.h"

#include <string.h>
#include <stdio.h>

extern struct netif gnetif;

void udpecho_thread(void *arg)
{
    struct netconn *udpconn;
    struct netbuf *udpbuf;
    struct netbuf *recv_udpbuf;
    ip_addr_t udpaddr;
    uint8_t udpdemo_buf[18] =
    {
        0xAA, 0x55, 0xFF, 0x5A, 0xA5, 0xAA, 0x55, 0xFF, 0x5A, 0xA5,
        0xAA, 0x55, 0xFF, 0x5A, 0xA5, 0xAA, 0x55, 0xFF
    };
    err_t err;

    /* 新建一个连接块 */
    udpconn = netconn_new(NETCONN_UDP);
    udpconn->recv_timeout = 1000; /* 1000毫秒收不到东西,接收函数也不会堵塞. */
    if (udpconn != NULL) /* 间接申请了内存 */
    {
        /* 绑定本地所有地址(开发板是本地) */
        err = netconn_bind(udpconn, IP_ADDR_ANY, 49152);
        /* 写目标地址 */
        IP4_ADDR(&udpaddr, 10, 0, 1, 35);
        /* 连接目标端口,UDP是无状态协议,肯定能连接成功的. */
        netconn_connect(udpconn, &udpaddr, 49152);

        if (err == ERR_OK)
        {
            while (1)
            {
                while (netconn_recv(udpconn, &recv_udpbuf) == ERR_OK)
                {
                    if(recv_udpbuf->p->len >= 1)  udpdemo_buf[4] = ((uint8_t *)recv_udpbuf->p->payload)[0];
                    netbuf_delete(recv_udpbuf);
                }

                /* UDP 缓冲区申请 */
                udpbuf = netbuf_new();
                /* 申请内存 */
                netbuf_alloc(udpbuf, strlen((char *)udpdemo_buf));
                /* 把数据复制到payload里去. */
                memcpy(udpbuf->p->payload, (void *)udpdemo_buf, strlen((const char *)udpdemo_buf));
                /* payload 其实也可以直接修改. */
                ((uint32_t *)udpbuf->p->payload)[0] = xTaskGetTickCount();
                /* 这一步把数据发送出去. */
                err = netconn_send(udpconn, udpbuf);
                /* UDP总会发成功的. */
                netbuf_delete(udpbuf);
                /* 延迟等下一次再发. */
                vTaskDelay(1000);
            }
        }
        else
        {
            /* 如果本地地址绑定不了,那么失败. */
            netconn_delete(udpconn);
        }
    }
    vTaskDelete(NULL);
}

void tcpecho_thread(void *arg)
{
    struct netconn *conn, *newconn;
    err_t err, accept_err;
    struct netbuf *buf;
    uint8_t *data;
    u16_t len;

    /* 新建一个连接块 */
    conn = netconn_new(NETCONN_TCP);

    if (conn != NULL)
    {
        /* 绑定本地7002端口 */
        err = netconn_bind(conn, NULL, 80);

        if (err == ERR_OK)
        {
            /* 进入监听模式 */
            netconn_listen(conn);

            while (1)
            {
                /* TCP是有状态的,这里遇到连接就会解除阻塞. */
                accept_err = netconn_accept(conn, &newconn);

                /* 当传入有效的时候,这里就可以继续. */
                if (accept_err == ERR_OK)
                {
                    /* 阻塞等数据来,只要连接一直保持,这个就不会出来. */
                    while (netconn_recv(newconn, &buf) == ERR_OK)
                    {
                        do
                        {
                            netbuf_data(buf, (void *)&data, &len); /* 等到数据长度. */

                            if(data[0] == 0xAA && len >= 2) data[1] = ~data[0]; /* 把读取到的数据进行判断 */

                            netconn_write(newconn, data, len, NETCONN_COPY); /* 把数据回写. */
                        }
                        while (netbuf_next(buf) >= 0);

                        netbuf_delete(buf);
                    }

                    /* 关闭连接(不建议长连接) */
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
    vTaskDelete(NULL);
}

#define HTTP_OK	0
#define HTTPS_NOT_SUPPORT -1
#define HTTP_AUTH_NOT_SUPPORT -2
#define HTTP_SERVER_REFUSE	-3
#define HTTP_ROUTE_NOT_FOUND	-4
#define HTTP_OUT_OF_RAM -5
#define HTTP_NOT_200OK	-6
#define HTTP_NO_CONTENT -7

uint8_t *pageBuf;

int get_webpage(const char *url)
{

    uint16_t i, j, k;
    char *server_addr = NULL;
    char *web_addr = NULL;
    ip_addr_t server_ip;
    struct netconn *conn;
    struct netbuf *inBuf;
    struct pbuf *q;
    char *request = NULL;
    err_t err;
    uint16_t recvPos = 0;
    uint8_t *recvBuf;


    while(gnetif.ip_addr.addr == 0x00)
    {
        return HTTP_ROUTE_NOT_FOUND;
    }

    if(strncmp((const char *)url, "http://", 7) == 0) 		/* 只处理http协议 */
    {
		server_addr = pvPortMalloc(strlen(url) - 7);
        /* 1)提取服务器部分 */
        for(i = 4; url[i]; ++i)
        {
            if (url[i] == ':' && url[i + 1] == '/' && url[i + 2] == '/')
            {
                for (i = i + 3, j = 0; url[i] > 32 && url[i] < 127 && url[i] != '/';
                        ++i, ++j)
                {
                    server_addr[j] = url[i];
                    if (url[i] == '@') /* 服务器基础认证符号,我们做不了,遇到就错误. */
                    {
                        return HTTP_AUTH_NOT_SUPPORT;
                    }
                }
                server_addr[j] = '\0';

                web_addr = pvPortMalloc(strlen(url) - 7 - strlen(server_addr));

                for (k = 7 + j; k < (strlen(url)); k++) /* 后半部分提取 */
                {
                    web_addr[k - 7 - j] = url[k];
                }

                web_addr[k - 7 - j] = 0x20; /* 末尾加截断 */

                while (--j)
                {
                    if (server_addr[j] == ':')
                    {
                        server_addr[j] = '\0';
                    }
                }
            }

        }

        /* 2)查询IP */
        netconn_gethostbyname(server_addr, &server_ip);


        /* 3)构造访问头 */
        request = pvPortMalloc(strlen(url) + 128); /* 头所需内存大小. */
        if(request == NULL) return HTTP_OUT_OF_RAM;
        sprintf(request, "GET %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: Mozilla/5.0 (lwip;STM32) TaterLi\r\n\r\n12", web_addr, server_addr);
        vPortFree(web_addr);
        vPortFree(server_addr);

        /* 4)开始访问 */
        conn = netconn_new(NETCONN_TCP);
        err = netconn_connect(conn, &server_ip, 80); /* 目前也只能支持80端口,比较常用,不考虑特殊情况. */

        if (err == ERR_OK)
        {
            netconn_write(conn, request, strlen((char *)request), NETCONN_COPY);
            vPortFree(request);
            inBuf = netbuf_new();
            conn->recv_timeout = 3000;
            recvPos = 0;

            if(netconn_recv(conn, &inBuf) == ERR_OK)   /* HTTP 1.0 天然不拆包 */
            {
                recvBuf = pvPortMalloc(inBuf->p->tot_len);
                if(recvBuf == NULL)
                {
                    return HTTP_OUT_OF_RAM;
                }
                for(q = inBuf->p; q != NULL; q = q->next) //遍历完整个pbuf链表
                {
                    memcpy(recvBuf + recvPos, q->payload, q->len);
                    recvPos += q->len;
                }
            }
            else
            {
                return HTTP_OUT_OF_RAM;
            }

            if(inBuf != NULL) netbuf_delete(inBuf);
            netconn_close(conn);
            netconn_delete(conn);

            /* 5)分析数据(分析HTTP头,暂时不打算支持301之类的)	*/
            for(i = 0; recvBuf[i]; ++i)
            {
                if (recvBuf[i] == '2' && recvBuf[i + 1] == '0' && recvBuf[i + 2] == '0')
                {
                    /* 证明200 OK */
                    for(; recvBuf[i]; ++i)
                    {
                        /* 响应头的结束也是两个回车 */
                        if(recvBuf[i] == '\r' && recvBuf[i + 1] == '\n' && recvBuf[i + 2] == '\r' && recvBuf[i + 3] == '\n')
                        {
                            /* 6)复制正文内容 */
                            i = i + 5;
                            k = strlen((const char *)recvBuf) - i;
                            if(k == 0) return HTTP_NO_CONTENT;
                            pageBuf = pvPortMalloc(k);
                            if(pageBuf == NULL)
                            {
                                vPortFree(recvBuf);
                                return HTTP_OUT_OF_RAM;
                            }
                            memcpy((char *)pageBuf, (const char *)recvBuf + i, k); /* 用HTTP1.0是没http chunked response的.方便处理,否则还要分段接收网页,大的网页反正MCU也接不下. */
                            vPortFree(recvBuf);
                            return HTTP_OK;
                        }
                    }
                }
            }
            return HTTP_NOT_200OK;
        }
        else
        {
            return HTTP_SERVER_REFUSE;
        }

    }
    else
    {
        return HTTPS_NOT_SUPPORT;
    }
}

void tcpget_thread(void *arg)
{

    while(gnetif.ip_addr.addr == 0x00)
    {
        vTaskDelay(1000);
    }
    while(1)
    {

        get_webpage("http://ticks.applinzi.com/test.php?price=123&hello=world");
        if(pageBuf != NULL)
        {
            vPortFree(pageBuf);
            vTaskDelay(100);
        }
        vTaskDelay(1000);
    }
}

void udplite_thread(void *arg)
{
    struct netconn *udpconn;
    struct netbuf *udpbuf;
    ip_addr_t udpaddr;
    uint8_t udpdemo_buf[5] = {0xAA, 0x55, 0xFF, 0x5A, 0xA5};

    /* 新建一个连接块,根UDP的区别只在这里.但是UDP-Lite属于那种传了不管,也不校验. */
    /* 类似的有NETCONN_UDPNOCHKSUM,但是这个跟Lite的Type不同,主机依然可能因为校验和不对直接丢掉. */
    /* UDP只是丢包,收不到,主机没反应,UDP-Lite是可以错传包,而主机还不管. */
    udpconn = netconn_new(NETCONN_UDPLITE);
    if (udpconn != NULL) /* 间接申请了内存 */
    {
        /* 写目标地址 */
        IP4_ADDR(&udpaddr, 10, 0, 1, 35);
        /* 连接目标端口,UDP是无状态协议,肯定能连接成功的. */
        netconn_connect(udpconn, &udpaddr, 7800);
        while (1)
        {
            /* UDP 缓冲区申请 */
            udpbuf = netbuf_new();
            /* 申请内存 */
            netbuf_alloc(udpbuf, strlen((char *)udpdemo_buf));
            /* 把数据复制到payload里去. */
            memcpy(udpbuf->p->payload, (void *)udpdemo_buf, strlen((const char *)udpdemo_buf));
            /* payload 其实也可以直接修改. */
            ((uint32_t *)udpbuf->p->payload)[0] = xTaskGetTickCount();
            /* 这一步把数据发送出去. */
            netconn_send(udpconn, udpbuf);
            /* UDP总会发成功的. */
            netbuf_delete(udpbuf);
            /* 延迟等下一次再发. */
            vTaskDelay(1000);
        }
    }
    netconn_delete(udpconn);
    vTaskDelete(NULL);
}
