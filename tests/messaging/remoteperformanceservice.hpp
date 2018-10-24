/*
**
** Copyright (C) 2018 Softbank Robotics Europe
*/
#pragma once
#include <chrono>
#include <qi/anyobject.hpp>

struct RemotePerformanceService
{

  void setObject(qi::AnyObject obj)
  {
    objects.push_back(obj);
  }

  void setObjectList(std::vector<qi::AnyObject> objs)
  {
    objects = std::move(objs);
  }

  // Returns millisecs taken to call the object's functoin
  std::int64_t measureCallDuration(const std::string& functionToCall)
  {
    using namespace std::chrono;

    if (objects.empty())
      throw std::runtime_error("No object stored to call");

    qi::AnyObject& o = objects.back();

    ++totalCallCount;

    const auto startTime = high_resolution_clock::now();

    o.call<void>(functionToCall);

    const auto endTime = high_resolution_clock::now();
    const auto discussDuration = endTime - startTime;

    qiLogInfo("TEST") << "Test Call " << totalCallCount << " call " << " UID{" << o.uid() << "} : "
                      << duration_cast<milliseconds>(discussDuration).count() << " ms ("
                      << duration_cast<nanoseconds>(discussDuration).count() << " ns)"
                      ;
    return static_cast<std::int64_t>(duration_cast<milliseconds>(discussDuration).count());
  }

  qi::AnyObject getMeasuredObject()
  {
    if (objects.empty())
      throw std::runtime_error("No object stored to call");
    return objects.back();
  }

  void clear()
  {
    objects.clear();
  }
private:

  std::vector<qi::AnyObject> objects;
  int totalCallCount = 0;
};

QI_REGISTER_OBJECT(RemotePerformanceService, setObject, setObjectList, measureCallDuration, clear, getMeasuredObject)
