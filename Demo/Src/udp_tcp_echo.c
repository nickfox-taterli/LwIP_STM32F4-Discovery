#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/sys.h"

#include <string.h>
#include <stdio.h>

void udpecho_thread(void *arg)
{
    struct netconn *udpconn;
    struct netbuf *udpbuf;
    struct netbuf *recv_udpbuf;
    ip_addr_t udpaddr;
    uint8_t udpdemo_buf[5] = {0xAA, 0x55, 0xFF, 0x5A, 0xA5};
    err_t err;
    err_t recv_err;

    /* 新建一个连接块 */
    udpconn = netconn_new(NETCONN_UDP);
    udpconn->recv_timeout = 1000; /* 1000毫秒收不到东西,接收函数也不会堵塞. */
    if (udpconn != NULL) /* 间接申请了内存 */
    {
        /* 绑定本地所有地址(开发板是本地) */
        err = netconn_bind(udpconn, IP_ADDR_ANY, 7001);
        /* 写目标地址 */
        IP4_ADDR(&udpaddr, 10, 0, 1, 35);
        /* 连接目标端口,UDP是无状态协议,肯定能连接成功的. */
        netconn_connect(udpconn, &udpaddr, 7800);
        if (err == ERR_OK)
        {
            while (1)
            {

                recv_err = netconn_recv(udpconn, &recv_udpbuf);
                if (recv_err == ERR_OK)
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
        err = netconn_bind(conn, NULL, 7002);

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

                            if(data[0] == 0xAA && len >= 2) data[1] ^= data[0]; /* 把读取到的数据进行判断 */

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

void udplite_thread(void *arg)
{
    struct netconn *udpconn;
    struct netbuf *udpbuf;
    ip_addr_t udpaddr;
    uint8_t udpdemo_buf[5] = {0xAA, 0x55, 0xFF, 0x5A, 0xA5};
    err_t err;

    /* 新建一个连接块,根UDP的区别只在这里.但是UDP-Lite属于那种传了不管,也不校验. */
    /* 类似的有NETCONN_UDPNOCHKSUM,但是这个跟Lite的Type不同,主机依然可能因为校验和不对直接丢掉. */
    /* UDP只是丢包,收不到,主机没反应,UDP-Lite是可以错传包,而主机还不管. */
    udpconn = netconn_new(NETCONN_UDPLITE);
    if (udpconn != NULL) /* 间接申请了内存 */
    {
        /* 绑定本地所有地址(开发板是本地) */
        err = netconn_bind(udpconn, IP_ADDR_ANY, 7003);
        /* 写目标地址 */
        IP4_ADDR(&udpaddr, 10, 0, 1, 35);
        /* 连接目标端口,UDP是无状态协议,肯定能连接成功的. */
        netconn_connect(udpconn, &udpaddr, 7800);
        if (err == ERR_OK)
        {
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
