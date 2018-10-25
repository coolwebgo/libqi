/*
**  Copyright (C) 2014 Aldebaran Robotics
**  See COPYING for the license
*/

#include <boost/algorithm/string.hpp>

#include <mutex>

#include "streamcontext.hpp"

#include "remoteobject_p.hpp"
#include "boundobject.hpp"

namespace qi
{
  namespace capabilityname
  {
    char const * const clientServerSocket    = "ClientServerSocket";
    char const * const metaObjectCache       = "MetaObjectCache";
    char const * const messageFlags          = "MessageFlags";
    char const * const remoteCancelableCalls = "RemoteCancelableCalls";
    char const * const objectPtrUid          = "ObjectPtrUID";
    char const * const directMessageDispatch  = "DirectMessageDispatch";
  }

  namespace {
    // Adds or update values in a map with values from another map.
    // If an exception is thrown, the state of 'map' is undetermined.
    // Requires: Map and AnotherMap matches the AssociativeContainer concept.
    // Requires: newValues.empty() || map[newValues.begin()->first] = newValues.begin()->second
    template<class Map, class AnotherMap>
    void updateMap(Map& map, const AnotherMap& newValues)
    {
      // TODO: replace par std::map::merge() in C++17
      for (auto& slot : newValues)
      {
        map[slot.first] = slot.second;
      }
    }
  }

StreamContext::StreamContext()
{
  _localCapabilityMap = StreamContext::defaultCapabilities();
}

StreamContext::~StreamContext()
{
}

void StreamContext::advertiseCapability(const std::string& key, const AnyValue& value)
{
  Mutex::scoped_lock lock(_contextMutex);
  _localCapabilityMap[key] = value;
  invalidateCapabilityCache();
}

void StreamContext::advertiseCapabilities(const CapabilityMap &map)
{
  Mutex::scoped_lock lock(_contextMutex);
  updateMap(_localCapabilityMap, map);
  invalidateCapabilityCache();
}

boost::optional<AnyValue> StreamContext::remoteCapability(const std::string& key) const
{
  Mutex::scoped_lock loc(_contextMutex);
  const auto it = _remoteCapabilityMap.find(key);
  if (it != _remoteCapabilityMap.end())
    return it->second;
  else
    return {};
}

void StreamContext::updateRemoteCapabilities(const CapabilityMap& remoteCaps)
{
  Mutex::scoped_lock lock(_contextMutex);
  updateMap(_remoteCapabilityMap, remoteCaps);
  invalidateCapabilityCache();
}


bool StreamContext::hasReceivedRemoteCapabilities() const
{
  Mutex::scoped_lock lock(_contextMutex);
  return _remoteCapabilityMap.size() != 0;
}


CapabilityMap StreamContext::remoteCapabilities() const
{
  Mutex::scoped_lock loc(_contextMutex);
  return _remoteCapabilityMap;
}

CapabilityMap StreamContext::localCapabilities() const
{
  Mutex::scoped_lock loc(_contextMutex);
  return _localCapabilityMap;
}

boost::optional<AnyValue> StreamContext::localCapability(const std::string& key) const
{
  Mutex::scoped_lock loc(_contextMutex);
  const auto it = _localCapabilityMap.find(key);
  if (it != _localCapabilityMap.end())
    return it->second;
  else
    return {};
}

MetaObject StreamContext::receiveCacheGet(unsigned int uid) const
{
  // Return by value, as map is by value.
  Mutex::scoped_lock lock(_contextMutex);
  const auto it = _receiveMetaObjectCache.find(uid);
  if (it == _receiveMetaObjectCache.end())
    throw std::runtime_error("MetaObject not found in cache");
  return it->second;
}

void StreamContext::receiveCacheSet(unsigned int uid, const MetaObject& mo)
{
  Mutex::scoped_lock lock(_contextMutex);
  _receiveMetaObjectCache[uid] = mo;
}

std::pair<unsigned int, bool> StreamContext::sendCacheSet(const MetaObject& mo)
{
  Mutex::scoped_lock lock(_contextMutex);
  SendMetaObjectCache::iterator it = _sendMetaObjectCache.find(mo);
  if (it == _sendMetaObjectCache.end())
  {
    unsigned int v = ++_cacheNextId;
    _sendMetaObjectCache[mo] = v;
    return std::make_pair(v, true);
  }
  else
    return std::make_pair(it->second, false);
}

namespace
{
  CapabilityMap applyCapabilitiesFromEnv(CapabilityMap capabilities)
  {
    std::string capstring = qi::os::getenv("QI_TRANSPORT_CAPABILITIES");
    std::vector<std::string> capsNames;
    boost::algorithm::split(capsNames, capstring, boost::algorithm::is_any_of(":"));
    for (unsigned i = 0; i < capsNames.size(); ++i)
    {
      const std::string& c = capsNames[i];
      if (c.empty())
        continue;
      size_t p = c.find_first_of("=");
      if (p == std::string::npos)
      {
        if (c[0] == '-')
          capabilities.erase(c.substr(1, c.npos));
        else if (c[0] == '+')
          capabilities[c.substr(1, c.npos)] = AnyValue::from(true);
        else
          capabilities[c] = AnyValue::from(true);
      }
      else
        capabilities[c.substr(0, p)] = AnyValue::from(c.substr(p + 1, c.npos));
    }
    return capabilities;
  }
}

const CapabilityMap& StreamContext::defaultCapabilities()
{
  static const CapabilityMap defaultCapabilities = applyCapabilitiesFromEnv(
    { { capabilityname::clientServerSocket    , AnyValue::from(true)  }
    , { capabilityname::messageFlags          , AnyValue::from(true)  }
    , { capabilityname::metaObjectCache       , AnyValue::from(false) }
    , { capabilityname::remoteCancelableCalls , AnyValue::from(true)  }
    , { capabilityname::objectPtrUid          , AnyValue::from(true)  }
    , { capabilityname::directMessageDispatch , AnyValue::from(true)  }
    });
  return defaultCapabilities;
}

void StreamContext::invalidateCapabilityCache() const
{
  Mutex::scoped_lock lock(_contextMutex);
  _isDirectDispatchAllowed = boost::none;
}
bool StreamContext::isDirectDispatchAllowed() const
{
  Mutex::scoped_lock lock(_contextMutex);
  if (_isDirectDispatchAllowed == boost::none)
  {
    const bool hasObjectPtrUidCapability = sharedCapability(capabilityname::objectPtrUid, false);
    const bool hasDirectMessageRoutageCapability = sharedCapability(capabilityname::directMessageDispatch, false);
    _isDirectDispatchAllowed = hasObjectPtrUidCapability && hasDirectMessageRoutageCapability;
  }

  return _isDirectDispatchAllowed.value();
}

}
