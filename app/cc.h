#include <udt.h>
#include <ccc.h>
#include <ctime>
#include <cmath>
#include<algorithm>
#include<sys/time.h>
using namespace std;

class CTCP: public CCC
{
public:
   void init()
   {
      m_bSlowStart = true;
      m_issthresh = 83333;

      m_dPktSndPeriod = 0.0;
      m_dCWndSize = 2.0;

      setACKInterval(2);
      setRTO(1000000);
   }

   virtual void onACK(const int& ack)
   {
      if (ack == m_iLastACK)
      {
         if (3 == ++ m_iDupACKCount)
            DupACKAction();
         else if (m_iDupACKCount > 3)
            m_dCWndSize += 1.0;
         else
            ACKAction();
      }
      else
      {
         if (m_iDupACKCount >= 3)
            m_dCWndSize = m_issthresh;

         m_iLastACK = ack;
         m_iDupACKCount = 1;

         ACKAction();
      }
   }

   virtual void onTimeout()
   {
      m_issthresh = getPerfInfo()->pktFlightSize / 2;
      if (m_issthresh < 2)
         m_issthresh = 2;

      m_bSlowStart = true;
      m_dCWndSize = 2.0;
   }

protected:
   virtual void ACKAction()
   {
      if (m_bSlowStart)
      {
         m_dCWndSize += 1.0;

         if (m_dCWndSize >= m_issthresh)
            m_bSlowStart = false;
      }
      else
         m_dCWndSize += 1.0/m_dCWndSize;
   }

   virtual void DupACKAction()
   {
      m_bSlowStart = false;

      m_issthresh = getPerfInfo()->pktFlightSize / 2;
      if (m_issthresh < 2)
         m_issthresh = 2;

      m_dCWndSize = m_issthresh + 3;
   }

protected:
   int m_issthresh;
   bool m_bSlowStart;

   int m_iDupACKCount;
   int m_iLastACK;
};


class CUDPBlast: public CCC
{
public:
   void init()
   {
      m_dPktSndPeriod = 1000000; 
      m_dCWndSize = 83333.0;
   }

public:
   void setRate(double mbps)
   {
      m_dPktSndPeriod = (m_iMSS * 8.0) / mbps;
   }
};



class PCC: public CCC
{
public:
   void init()
   {
      m_dCWndSize = 83333.0; //将拥塞窗口开的非常大，表示不使用窗口进行拥塞控制，而是完全使用速率控制
      m_iRTT=30; //初始化RTT为30ms
      rate=8; //初始速率为3KB/RTT
      bestRate=rate;
      pktInMI=0;
      lossInMI=0;
      utility=-1e9;
      DMS_flag=0;
      pmin=0.01;
      pmax=0.05;
      p=pmin;
      setRate(rate); 
      state=1;
      MI=2*m_iRTT;
      gettimeofday(&st,NULL);
   }

   virtual void onPktSent(const CPacket*) //在任意包发送完毕后讨论它后面包的速率调控问题
   {
      pktInMI++;
      gettimeofday(&ct,NULL); //记录当前时刻
      time=(double)(ct.tv_sec-st.tv_sec)*1000.0+(double)(ct.tv_usec-st.tv_usec)/1000.0;
      if(time>=MI)
      {
         st=ct;
         if(state==1) //开始状态SS
         {
            double u0=func_u(); //计算上一次速率造成的价值
            if(u0<utility) //如果价值函数减少就恢复原来的速率并进入DMS状态
            {
               setRate(bestRate);
               state=2;
            }
            else
            {
               bestRate=rate;
               setRate(2*rate);
               utility=u0;
            }
         }
         else if(state==2) //决策状态DMS
         {
            if(DMS_flag>0)U[DMS_flag]=func_u(); //计算上一个MI的U
            DMS_flag++;
            if(DMS_flag!=5&&DMS_flag%2==1)setRate(bestRate*(1+p));
            else if(DMS_flag!=5&&DMS_flag%2==0)setRate(bestRate*(1-p));
            if(DMS_flag==5) //四个MI算好之后
            {
               setRate(bestRate); //当完成四个MI的微实验后短暂的将速率调回r
               if(U[1]>U[2]&&U[3]>U[4])
               {
                  setRate(bestRate*(1+p));
                  state=3;
               }
               else if(U[1]<U[2]&&U[3]<U[4])
               {
                  setRate(bestRate*(1-p));
                  state=3;
               }
               else 
               {
                  setRate(bestRate);
                  p=min(pmax,p+pmin);
               }
               DMS_flag=0;
            }
         }
         else if(state==3) //DMS状态调整完速率后判断进入RAS状态
            {
               double u0=func_u();
               if(u0<utility)
               {
                  setRate(bestRate);
                  state=2;
               }
               else 
               {
                  state=0; 
                  utility=u0;
                  bestRate=rate;
               }
            }
         pktInMI=0;
         lossInMI=0;
      }
   }

   virtual void onLoss(const int32_t*, int)
   {
      lossInMI++;
   }
protected:
   virtual void setRate(double Mbps)
   {
      rate=Mbps; //在本地记录当前速率
      m_dPktSndPeriod = (m_iMSS * 8.0) / Mbps;
   }
   virtual double func_u()
   {
      double u= (((pktInMI-lossInMI))/time*(1-1/(1+exp(-100*(lossInMI/pktInMI-0.05))))-1*(lossInMI)/time);
      //double u= ((pktInMI-lossInMI)/time)*(1/(1+exp(100*(lossInMI/pktInMI-0.05))))-lossInMI/time;
      return u;
   }
protected:
   int state; //记录算法处于什么状态，1为SS、2为DMS、3为RAS
   double MI; //Monitor Interval
   double rate; //当前传输速率
   double bestRate;//存储前面轮中最优速率用于慢启动回滚
   double utility; //当前价值函数的计算值
   double pktInMI; //当前MI内共传输了多少个包
   double lossInMI; //当前MI内共丢失了多少个包
   int DMS_flag; //表示DMS状态下讨论到了哪个MI，变量等于0表示未处于或即将进入DMS状态
   int p; //步长ε
   int pmin;
   int pmax;
   double time; 
   double U[5]; //记录DMS状态时四个MI的u值，角标从1开始
   struct timeval st,ct; //StartTime,CurrentTime
};
