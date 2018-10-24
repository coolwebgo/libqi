#pragma once
/*
**  Copyright (C) 2018 Softbank Robotics Europe
**  See COPYING for the license
*/

#include <unordered_map>

#include <boost/thread/synchronized_value.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <ka/memory.hpp>
#include <qi/objectuid.hpp>
#include <qi/log.hpp>
#include <qi/strand.hpp>

namespace qi {
  class Message;
  class StreamContext;
  class RemoteObject;
  class BoundObject;
  class MessageSocket;
  using MessageSocketPtr = boost::shared_ptr<MessageSocket>;
  using RemoteObjectPtr = boost::shared_ptr<RemoteObject>;
  using BoundObjectPtr = boost::shared_ptr<BoundObject>;

  namespace detail
  {
    namespace {
      const auto logCategory = "ObjectNetworkInterfaceRegistry";
    }

    bool canBeDirectlyDispatched(const Message& message, const StreamContext& context);

    // Used to register and later find instances of types representing an object exposed online.
    // Relies on ObjectUid to identify and find objects.
    // @tparam NetworkObject Requires:
    //     With NetworkObject x, Message m, SocketPtr s, the following is valid:
    //          boost::weak_ptr<NetworkObject> p = x.getWeakPtr();
    //       && x.onMessage(m, s);
    template<class NetworkObject>
    class ObjectNetworkInterfaceRegistry
    {
    public:
      using WeakPtr = boost::weak_ptr<NetworkObject>;
      using SharedPtr = boost::shared_ptr<NetworkObject>;

      // Record a weak pointer to the object associated to the provided uid.
      // Preconditions: this->find(uid).get() == nullptr || this->find(uid).get() == &object
      // Postconditions: this->find(uid).get() == &object
      void add(const ObjectUid& uid, NetworkObject& object)
      {
        const auto weakObject = object.getWeakPtr();

        bool insertionSucceeded = false;
        auto insertedIt = registry.end();
        std::tie(insertedIt, insertionSucceeded) = registry.emplace(std::make_pair(uid, weakObject));

        QI_ASSERT_TRUE(insertionSucceeded || insertedIt->second.lock().get() == &object); // Only one object should be registered by ObjectUid.
        qiLogDebug(logCategory) << "Registered in " << this << " : { " << uid << " }"
          << " AS " << typeid(object).name() << " "
          << (insertionSucceeded ? "" : " - skipped");
      }

      // Postconditions: this->find(uid).get() == nullptr
      void remove(const ObjectUid& id)
      {
        qiLogDebug(logCategory) << "Unregistered remoteobject from " << this << " : { " << id << " }";
        registry.erase(id);
      }

      // Postconditions: With `SharedPtr object` and `this->add(uid, *object)` the following is true:
      //     object == this->find(uid)
      SharedPtr find(const ObjectUid& id) const
      {
        const auto find_it = registry.find(id);
        if (find_it != end(registry))
        {
          return acquireOrRemove(find_it);
        }

        return {};
      }

    private:
      using Registry = std::unordered_map<ObjectUid, WeakPtr>;
      mutable Registry registry;

      SharedPtr acquireOrRemove(typename Registry::iterator it) const
      {
        const SharedPtr object = it->second.lock();
        if (!object)
        {
          registry.erase(it);
        }
        return object;
      }

    };
  }

  /* Provides "direct message dispatch" mechanism to be used on reception of network messages.
     "Direct message dispatch" refers to passing a network message directly to the recipient,
     which is an object handling network interfacing for another object
     (@see RemoteObject and BoundObject).

     Registered objects will be receive messages dispatched through `dispatchMessage` function
     if they match the recipient identifier in the message.

  */
  class DirectDispatchRegistry
  {
    template<class T>
    using Registry = detail::ObjectNetworkInterfaceRegistry<T>;
    template<class T>
    using ThreadSafeRegistry = boost::synchronized_value<Registry<T>, boost::recursive_mutex>;

    ThreadSafeRegistry<RemoteObject> _remoteObjectRegistry;
    ThreadSafeRegistry<BoundObject> _boundObjectRegistry;
  public:
    DirectDispatchRegistry() = default;
    DirectDispatchRegistry(const DirectDispatchRegistry&) = delete;
    DirectDispatchRegistry& operator=(const DirectDispatchRegistry&) = delete;

    void registerRecipient(RemoteObject& object) BOOST_NOEXCEPT;
    void registerRecipient(BoundObject & object) BOOST_NOEXCEPT;
    void unregisterRecipient(const RemoteObject& object) BOOST_NOEXCEPT;
    void unregisterRecipient(const BoundObject & object) BOOST_NOEXCEPT;

    RemoteObjectPtr findRemoteObject(const ObjectUid& id) const BOOST_NOEXCEPT;
    BoundObjectPtr findBoundObject(const ObjectUid& id) const BOOST_NOEXCEPT;

    // Requires:
    //  - message.destinationUid() is empty
    //  - canBeDirectlyDispatched(message)
    //  - socket != nullptr
    bool dispatchMessage(Message& message, const MessageSocketPtr& socket) const BOOST_NOEXCEPT;

    auto lockRemoteObjectRegistry() -> decltype(_remoteObjectRegistry.synchronize())
    {
      return _remoteObjectRegistry.synchronize();
    }

    auto lockBoundObjectRegistry() -> decltype(_boundObjectRegistry.synchronize())
    {
      return _boundObjectRegistry.synchronize();
    }

  };


}

