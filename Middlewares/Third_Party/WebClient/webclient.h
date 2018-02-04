#ifndef _TATER_WEB_CLIENT_H_
#define _TATER_WEB_CLIENT_H_

#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/sys.h"
#include "lwip/dns.h"

#include "lwip/netif.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HTTP_OK	0
#define HTTPS_NOT_SUPPORT -1
#define HTTP_AUTH_NOT_SUPPORT -2
#define HTTP_SERVER_REFUSE	-3
#define HTTP_ROUTE_NOT_FOUND	-4
#define HTTP_OUT_OF_RAM -5
#define HTTP_NOT_200OK	-6
#define HTTP_NO_CONTENT -7
#define HTTP_SERVER_ADDR_ERROR	-8
#define HTTP_NOT_VALID_ADDR -9

int8_t WebClient(const char *url, const char *post, uint8_t **pageBuf);

#endif
