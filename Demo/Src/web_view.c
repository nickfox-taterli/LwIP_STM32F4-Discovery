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

cJSON *root;
char *origin_price, *real_price;
uint8_t *abuf;

uint32_t free_ram = 0;

void web_view(void *arg)
{
    int err = HTTP_OK;
    const char postVar[] = "origin_price=1&real_price=3";

    while(gnetif.ip_addr.addr == 0x00)
    {
        vTaskDelay(1000);
    }
    while(1)
    {			
        err = WebClient("http://10.0.1.64/lwip/post.php", postVar, &abuf);
        if(abuf != NULL && err == HTTP_OK)
        {
            root = cJSON_Parse((const char *)abuf);

            origin_price = cJSON_GetObjectItem( root , "origin_price" )->valuestring;
            real_price = cJSON_GetObjectItem( root , "real_price" )->valuestring;
            vPortFree(abuf);
            cJSON_Delete(root);
        }

        err = WebClient("http://10.0.1.64/lwip/get.php?origin_price=2&real_price=6", NULL, &abuf);
        if(abuf != NULL && err == HTTP_OK)
        {
            root = cJSON_Parse((const char *)abuf);

            origin_price = cJSON_GetObjectItem( root , "origin_price" )->valuestring;
            real_price = cJSON_GetObjectItem( root , "real_price" )->valuestring;
            vPortFree(abuf);
            cJSON_Delete(root);
        }

        err = WebClient("http://10.0.1.64/lwip/mixed.php?origin_price=5", postVar, &abuf);
        if(abuf != NULL && err == HTTP_OK)
        {
            root = cJSON_Parse((const char *)abuf);
            origin_price = cJSON_GetObjectItem( cJSON_GetObjectItem( root , "get" ) , "origin_price" )->valuestring;
						real_price = cJSON_GetObjectItem( cJSON_GetObjectItem( root , "post" ) , "real_price" )->valuestring;
            vPortFree(abuf);
            cJSON_Delete(root);
        }
				
				vTaskDelay(30 * 1000);
    }
}
