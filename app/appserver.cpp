#ifndef WIN32
   #include <unistd.h>
   #include <cstdlib>
   #include <cstring>
   #include <netdb.h>
#else
   #include <winsock2.h>
   #include <ws2tcpip.h>
   #include <wspiapi.h>
#endif
#include <iostream>
#include <udt.h>
#include "cc.h"
#include "test_util.h"
#include<sys/time.h>

using namespace std;
struct timeval st,ct;
double time0;
double sumpkt,sumrate;
#ifndef WIN32
void* recvdata(void*);
void* recv_monitor(void* s);
#else
DWORD WINAPI recvdata(LPVOID);
#endif

int main(int argc, char* argv[])
{
   gettimeofday(&st,NULL);
   if ((1 != argc) && ((2 != argc) || (0 == atoi(argv[1]))))
   {
      cout << "usage: appserver [server_port]" << endl;
      return 0;
   }

   // Automatically start up and clean up UDT module.
   UDTUpDown _udt_;

   addrinfo hints;
   addrinfo* res;

   memset(&hints, 0, sizeof(struct addrinfo));

   hints.ai_flags = AI_PASSIVE;
   hints.ai_family = AF_INET;
   hints.ai_socktype = SOCK_STREAM;
   //hints.ai_socktype = SOCK_DGRAM;

   string service("9000");
   if (2 == argc)
      service = argv[1];

   if (0 != getaddrinfo(NULL, service.c_str(), &hints, &res))
   {
      cout << "illegal port number or port is busy.\n" << endl;
      return 0;
   }

   UDTSOCKET serv = UDT::socket(res->ai_family, res->ai_socktype, res->ai_protocol);

   // UDT Options
   //UDT::setsockopt(serv, 0, UDT_CC, new CCCFactory<CUDPBlast>, sizeof(CCCFactory<CUDPBlast>));
   //UDT::setsockopt(serv, 0, UDT_MSS, new int(9000), sizeof(int));
   //UDT::setsockopt(serv, 0, UDT_RCVBUF, new int(10000000), sizeof(int));
   //UDT::setsockopt(serv, 0, UDP_RCVBUF, new int(10000000), sizeof(int));
   UDT::setsockopt(serv, 0, UDT_CC, new CCCFactory<PCC>, sizeof(CCCFactory<PCC>));
   if (UDT::ERROR == UDT::bind(serv, res->ai_addr, res->ai_addrlen))
   {
      cout << "bind: " << UDT::getlasterror().getErrorMessage() << endl;
      return 0;
   }

   freeaddrinfo(res);

   cout << "server is ready at port: " << service << endl;

   if (UDT::ERROR == UDT::listen(serv, 10))
   {
      cout << "listen: " << UDT::getlasterror().getErrorMessage() << endl;
      return 0;
   }

   sockaddr_storage clientaddr;
   int addrlen = sizeof(clientaddr);

   UDTSOCKET recver;

   while (true)
   {
      if (UDT::INVALID_SOCK == (recver = UDT::accept(serv, (sockaddr*)&clientaddr, &addrlen)))
      {
         cout << "accept: " << UDT::getlasterror().getErrorMessage() << endl;
         return 0;
      }

      char clienthost[NI_MAXHOST];
      char clientservice[NI_MAXSERV];
      getnameinfo((sockaddr *)&clientaddr, addrlen, clienthost, sizeof(clienthost), clientservice, sizeof(clientservice), NI_NUMERICHOST|NI_NUMERICSERV);
      cout << "new connection: " << clienthost << ":" << clientservice << endl;

      #ifndef WIN32
         pthread_t rcvthread;
         pthread_create(&rcvthread, NULL, recvdata, new UDTSOCKET(recver));
         pthread_detach(rcvthread);
      #else
         CreateThread(NULL, 0, recvdata, new UDTSOCKET(recver), 0, NULL);
      #endif
   }

   UDT::close(serv);

   return 0;
}

#ifndef WIN32
void* recvdata(void* usocket)
#else
DWORD WINAPI recvdata(LPVOID usocket)
#endif
{
   UDTSOCKET recver = *(UDTSOCKET*)usocket;
   delete (UDTSOCKET*)usocket;
   pthread_create(new pthread_t, NULL, recv_monitor, &recver);
   char* data;
   int size = 100000;
   data = new char[size];

   while (true)
   {
      int rsize = 0;
      int rs;
      while (rsize < size)
      {
         int rcv_size;
         int var_size = sizeof(int);
         UDT::getsockopt(recver, 0, UDT_RCVDATA, &rcv_size, &var_size);
         if (UDT::ERROR == (rs = UDT::recv(recver, data + rsize, size - rsize, 0)))
         {
            cout << "recv:" << UDT::getlasterror().getErrorMessage() << endl;
            break;
         }
         gettimeofday(&ct,NULL); //??????????????????
         time0=(double)(ct.tv_sec-st.tv_sec)+(double)(ct.tv_usec-st.tv_usec)/1000.0;
         if(time0>=30)
         {
         cout<<"tp:"<<sumpkt/sumrate<<"Mbps"<<endl<<endl<<endl;
         sumpkt=0;
         sumrate=0;
         st=ct;
         }
         rsize += rs;
      }

      if (rsize < size)
         break;
   }

   delete [] data;

   UDT::close(recver);

   #ifndef WIN32
      return NULL;
   #else
      return 0;
   #endif
}

#ifndef WIN32
void* recv_monitor(void* s)
#else
DWORD WINAPI recv_monitor(LPVOID s)
#endif
{
  UDTSOCKET u = *(UDTSOCKET*)s;
  int i = 0;

  UDT::TRACEINFO perf;

  cout << "Recv Rate(Mb/s)\tRTT(ms)\tPackets Recvd" << endl;

  while (true) {
    ++i;
#ifndef WIN32
    sleep(1);
#else
    Sleep(1000);
#endif

    if (UDT::ERROR == UDT::perfmon(u, &perf)) {
      cout << "perfmon: " << UDT::getlasterror().getErrorMessage() << endl;
      break;
    }

    cout << perf.mbpsRecvRate << "\t\t" << perf.msRTT << "\t"
         << perf.pktRecv << "\t\t" << endl;
    sumpkt++;
    sumrate+=perf.mbpsRecvRate;
    cout<<"sumpkt:"<<sumpkt<<endl;
    cout<<"sumrate"<<sumrate<<endl;
  }

#ifndef WIN32
  return NULL;
#else
  return 0;
#endif
}
