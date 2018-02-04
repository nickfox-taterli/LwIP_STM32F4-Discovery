#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/sys.h"
#include "lwip/dns.h"

#include "lwip/netif.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "webclient.h"

extern struct netif gnetif;

uint8_t *abuf;
cJSON *root;
cJSON *origin_price, *real_price;

void web_view(void *arg)
{
    int err = HTTP_OK;

    while(gnetif.ip_addr.addr == 0x00)
    {
        vTaskDelay(1000);
    }
    while(1)
    {

        err = get_webpage("http://p.3.cn/prices/mgets?type=1&skuIds=1379747", &abuf);
        if(abuf != NULL && err == HTTP_OK)
        {
            root = cJSON_Parse((const char *)abuf + 1); /* 部分JSON一开始字符是[ 结尾字符无关 */

            origin_price = cJSON_GetObjectItem( root , "op" );
            real_price = cJSON_GetObjectItem( root , "p" );
            cJSON_Delete(root);
            vPortFree(abuf);
						if(origin_price != NULL) vPortFree(origin_price);
						if(real_price != NULL) vPortFree(real_price);
            vTaskDelay(100);
        }
        vTaskDelay(1000);
    }
}
