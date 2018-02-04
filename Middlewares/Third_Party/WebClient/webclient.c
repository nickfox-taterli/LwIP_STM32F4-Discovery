#include "webclient.h"

extern struct netif gnetif;

int WebClient(const char *url, const char *post, uint8_t **pageBuf)
{

    uint16_t i, j, k;
    char *server_addr = NULL;
    char *web_addr = NULL;
    ip_addr_t server_ip;
    struct netconn *conn;
    struct netbuf *inBuf;
    struct pbuf *q;
    char *request = NULL;
    uint16_t recvPos = 0;
    uint8_t *recvBuf;
    err_t err_msg;

    while(gnetif.ip_addr.addr == 0x00)
    {
        return HTTP_ROUTE_NOT_FOUND;
    }

    if(strncmp((const char *)url, "http://", 7) == 0) 		/* 只处理http协议 */
    {
        server_addr = pvPortMalloc(strlen(url) - 7);
        if(server_addr == NULL) return HTTP_OUT_OF_RAM;
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
                if(web_addr == NULL) return HTTP_OUT_OF_RAM;

                for (k = 7 + j; k < (strlen(url)); k++) /* 后半部分提取 */
                {
                    web_addr[k - 7 - j] = url[k];
                }

                web_addr[k - 7 - j] = '\0';

                while (--j)
                {
                    if (server_addr[j] == ':')
                    {
                        server_addr[j] = '\0';
                    }
                }
            }

        }

        if(strlen(server_addr) < 2) /* 最短网址3.cn */
        {
            vPortFree(server_addr);
            if(web_addr == NULL) vPortFree(web_addr); /* 这么短,还不一定提取到了这个. */
            return HTTP_SERVER_ADDR_ERROR;
        }

        /* 2)查询IP */
        netconn_gethostbyname(server_addr, &server_ip);

        /* 3)构造访问头 */
        request = pvPortMalloc(strlen(url) + 1024); /* 头所需内存大小. */
        if(request == NULL) return HTTP_OUT_OF_RAM;
        
				if(post != NULL)
					sprintf(request, "POST %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: Mozilla/5.0 (lwip;STM32) TaterLi\r\nContent-Length: %d\r\nContent-Type: application/x-www-form-urlencoded\r\n\r\n%s", web_addr, server_addr,strlen(post),post);
        else
					sprintf(request, "GET %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: Mozilla/5.0 (lwip;STM32) TaterLi\r\n\r\n", web_addr, server_addr);
				vPortFree(server_addr);
        if(web_addr != NULL)
        {
            /* 万一没提取到,就是NULL,如果是NULL,那么也不用继续了. */
            vPortFree(web_addr);
        }
        else
        {
            vPortFree(request);
            return HTTP_NOT_VALID_ADDR;
        }

        /* 4)开始访问 */
        conn = netconn_new(NETCONN_TCP);
        err_msg = netconn_connect(conn, &server_ip, 80); /* 目前也只能支持80端口,比较常用,不考虑特殊情况. */

        if (err_msg == ERR_OK)
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
                            i += 4;
                            k = strlen((const char *)recvBuf) - i;
                            if(k == 0) return HTTP_NO_CONTENT;
                            *pageBuf = pvPortMalloc(k);
                            if(*pageBuf == NULL)
                            {
                                vPortFree(recvBuf);
                                return HTTP_OUT_OF_RAM;
                            }
                            memcpy((char *)*pageBuf, (const char *)recvBuf + i, k); /* 用HTTP1.0是没http chunked response的.方便处理,否则还要分段接收网页,大的网页反正MCU也接不下. */
                            vPortFree(recvBuf);
                            return HTTP_OK;
                        }
                    }
                }
            }

            if(recvBuf != NULL)	vPortFree(recvBuf);

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
