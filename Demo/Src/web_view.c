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
cJSON *origin_price, *real_price;
uint8_t *abuf;

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

        err = WebClient("http://ticks.applinzi.com/lwip/post.php", postVar, &abuf);
        if(abuf != NULL && err == HTTP_OK)
        {
            root = cJSON_Parse((const char *)abuf);

            origin_price = cJSON_GetObjectItem( root , "origin_price" );
            real_price = cJSON_GetObjectItem( root , "real_price" );
            vPortFree(abuf);
            cJSON_Delete(root);
            vTaskDelay(100);
					
        }

        err = WebClient("http://ticks.applinzi.com/lwip/get.php?origin_price=2&real_price=6", NULL, &abuf);
        if(abuf != NULL && err == HTTP_OK)
        {
					
            root = cJSON_Parse((const char *)abuf);

            origin_price = cJSON_GetObjectItem( root , "origin_price" );
            real_price = cJSON_GetObjectItem( root , "real_price" );
            vPortFree(abuf);
						cJSON_Delete(root);
            vTaskDelay(100);
					
        }
				
				err = WebClient("http://ticks.applinzi.com/lwip/mixed.php?get_origin_price=5", postVar, &abuf);
        if(abuf != NULL && err == HTTP_OK)
        {
					
            root = cJSON_Parse((const char *)abuf);

            origin_price = cJSON_GetObjectItem( root , "get_origin_price" );
            real_price = cJSON_GetObjectItem( root , "real_price" );
            vPortFree(abuf);
						cJSON_Delete(root);
            vTaskDelay(100);
					
        }
        vTaskDelay(1000);
    }
}
