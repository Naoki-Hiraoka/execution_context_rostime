#ifndef ROSTIMEPERIODICEXECUTIONCONTEXT_H
#define ROSTIMEPERIODICEXECUTIONCONTEXT_H

#include <rtm/RTC.h>
#include <rtm/Manager.h>

#include <rtm/PeriodicExecutionContext.h>


class ROSTimePeriodicExecutionContext : public virtual RTC::PeriodicExecutionContext
{
public:
    ROSTimePeriodicExecutionContext();
    virtual ~ROSTimePeriodicExecutionContext(void);
    virtual int svc(void);
};

extern "C"
{
  void ROSTimePeriodicExecutionContextInit(RTC::Manager* manager);
};

#endif
