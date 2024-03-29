#include "ROSTimePeriodicExecutionContext.h"
#include <rtm/ECFactory.h>
#include <ros/ros.h>

ROSTimePeriodicExecutionContext::ROSTimePeriodicExecutionContext()
    : PeriodicExecutionContext()
{
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

      std::vector<double> processTimes(m_comps.size());
      if (m_worker.running_)
        {
          for(int i=0;i<m_comps.size();i++){
            coil::TimeValue before(coil::gettimeofday());
            invoke_worker()(m_comps[i]);
            coil::TimeValue after(coil::gettimeofday());
            processTimes[i] = double(after - before);
          }
        }
      m_worker.mutex_.unlock();
      ros::Time t1_ = ros::Time::now();
      coil::TimeValue t1(t1_.sec,t1_.nsec/1000);
      if((double)(t1 - t0) < 0) t0 = t1; //巻き戻り

      if ((double)(t1 - t0) > m_period){
        std::cerr<<"[ROSTimeEC] Timeover: processing time = "<<(double)(t1 - t0)<<"[s]"<<std::endl;
        for(int i=0;i<m_comps.size();i++){
          RTC::ComponentProfile_var profile = RTC::RTObject::_narrow(m_comps[i]._ref)->get_component_profile(); // get_component_profile() はポインタを返すので、呼び出し側で release するか、_var型変数で受ける必要がある
          std::cerr << profile->instance_name<<"("<<processTimes[i]<<"), ";
        }
        std::cerr << std::endl;
      }

      // ROS::Timeは/clockが届いたときにしか変化しない. /clockの周期と制御周期が近い場合、単純なcoil::sleep(m_period - (t1 - t0))では誤差が大きい
      // ros::Rate::sleepは内部で1ms間隔でsleepするため、粗すぎる
      while (!m_nowait && m_period > (t1 - t0))
        {
          coil::sleep((coil::TimeValue)(double(m_period) / 100));
          t1_ = ros::Time::now();
          t1 = coil::TimeValue(t1_.sec,t1_.nsec/1000);
          if((double)(t1 - t0) < 0) t0 = t1; //巻き戻り
        }

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
 
