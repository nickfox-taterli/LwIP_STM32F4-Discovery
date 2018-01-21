#ifndef __HTTPSERVER_SOCKET_H__
#define __HTTPSERVER_SOCKET_H__

#define WEBSERVER_THREAD_PRIO  ( tskIDLE_PRIORITY + 4 )

void http_server_socket_thread(void *arg);
void DynWebPage(int conn);

#endif /* __HTTPSERVER_SOCKET_H__ */
