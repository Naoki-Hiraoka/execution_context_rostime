#include "ROSTimePeriodicExecutionContext.h"
#include <rtm/ECFactory.h>
#include <ros/ros.h>

ROSTimePeriodicExecutionContext::ROSTimePeriodicExecutionContext()
    : PeriodicExecutionContext()
{
  std::cerr << "gen" <<std::endl;
}

ROSTimePeriodicExecutionContext::~ROSTimePeriodicExecutionContext()
{
}

int ROSTimePeriodicExecutionContext::svc(void)
{
  RTC_TRACE(("svc()"));
  int count(0);

  if(!ros::isInitialized()){
      int argc = 0;
      char* argv[0];
      ros::init(argc,argv,"ROSTimePeriodicExecutionContext",ros::init_options::AnonymousName);
  }
  ros::NodeHandle nh;

  do
    {
      m_worker.mutex_.lock();
      while (!m_worker.running_)
        {
          m_worker.cond_.wait();
        }
      ros::Time t0_ = ros::Time::now();
      coil::TimeValue t0(t0_.sec,t0_.nsec/1000);
      if (m_worker.running_)
        {
          std::for_each(m_comps.begin(), m_comps.end(), invoke_worker());
        }
      m_worker.mutex_.unlock();
      ros::Time t1_ = ros::Time::now();
      coil::TimeValue t1(t1_.sec,t1_.nsec/1000);
      {
        RTC_PARANOID(("Period:    %f [s]", (double)m_period));
        RTC_PARANOID(("Execution: %f [s]", (double)(t1 - t0)));
        RTC_PARANOID(("Sleep:     %f [s]", (double)(m_period - (t1 - t0))));
      }
      ros::Time t2_ = ros::Time::now();
      coil::TimeValue t2(t2_.sec,t2_.nsec/1000);

      // ROS::Timeは/clockが届いたときにしか変化しない. /clockの周期と制御周期が近い場合、単純なcoil::sleep(m_period - (t1 - t0))では誤差が大きい
      while (!m_nowait && m_period > (t1 - t0))
        {
          coil::sleep((coil::TimeValue)(double(m_period) / 100));
          t1_ = ros::Time::now();
          t1 = coil::TimeValue(t1_.sec,t1_.nsec/1000);
        }
      if (count > 1000)
        {
          ros::Time t3_ = ros::Time::now();
          coil::TimeValue t3(t3_.sec,t3_.nsec/1000);
          RTC_PARANOID(("Slept:     %f [s]", (double)(t3 - t2)));
          count = 0;
        }
      std::cerr << count << std::endl;
      ++count;
    } while (m_svc);

  return 0;
}

extern "C"
{
    void ROSTimePeriodicExecutionContextInit(RTC::Manager* manager)
    {
        manager->registerECFactory("ROSTimePeriodicExecutionContext",
                                   RTC::ECCreate<ROSTimePeriodicExecutionContext>,
                                   RTC::ECDelete<ROSTimePeriodicExecutionContext>);
        std::cerr << "ROSTimePeriodicExecutionContext is registered" << std::endl;
    }
};
 
