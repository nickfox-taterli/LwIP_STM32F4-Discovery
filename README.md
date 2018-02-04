# STM32F4 LwIP Demo

------

引脚按照默认,PHY随意,外部晶振,实现的主要功能:

> * UDP Echo
> * TCP Echo
> * UDP-Lite Sender
> * HTTP Server
> * HTTP Client
> * Telnet Server

主要组件:

> * FreeRTOS 10.0
> * LwIP 2.0.3
> * cJson 1.7.1
> * 自己重写的ETH库,包含锁机制,一定程度防丢包.
> * 很懒,其他自己探索吧.