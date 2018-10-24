/*
**  Copyright (C) 2018 Softbank Robotics Europe
**  See COPYING for the license
*/
#include "directdispatch.hpp"

#include <boost/optional.hpp>

#include <ka/errorhandling.hpp>

#include "remoteobject_p.hpp"
#include "boundobject.hpp"
#include "messagesocket.hpp"

qiLogCategory("qimessaging.directdispatch");

namespace qi {

  namespace detail {

    bool canBeDirectlyDispatched(const Message& message, const StreamContext& context)
    {
      // For now we handle only calls, not replies or others. TODO: apply this workaround to other messages too.
      return message.type() == Message::Type::Type_Call // TODO: handle other types of messages.
          && message.object() != qi::Message::GenericObject_Main // TODO: remove this once services can be identified
          && message.object() != qi::Message::GenericObject_None
          && message.service() != qi::Message::Service_Server // TODO: remove this once we can identify servers
          && context.isDirectDispatchAllowed()
          ;
    }

  }

  namespace {
    template<class String, class Func, class Result = decltype(std::declval<Func>()())>
    Result invokeLogOnError(String&& location, Func&& f)
    {
      auto errorLogger = ka::compose(ka::constant_function_t<Result>{}
                                    , exceptionLogError("qi.directdispatch", std::string("Failed in ") + ka::fwd<String>(location))
                                    );
      return ka::invoke_catch(std::move(errorLogger), ka::fwd<Func>(f));
    }
  }

  void DirectDispatchRegistry::registerRecipient(RemoteObject& object) BOOST_NOEXCEPT
  {
    invokeLogOnError(__FUNCTION__, [&]{
      _remoteObjectRegistry->add(object.remoteObjectUid(), object);
    });
  }

  void DirectDispatchRegistry::registerRecipient(BoundObject & object) BOOST_NOEXCEPT
  {
    invokeLogOnError(__FUNCTION__, [&] {
      _boundObjectRegistry->add(object.uid(), object);
    });
  }

  void DirectDispatchRegistry::unregisterRecipient(const RemoteObject& object) BOOST_NOEXCEPT
  {
    invokeLogOnError(__FUNCTION__, [&] {
      _remoteObjectRegistry->remove(object.remoteObjectUid());
    });
  }

  void DirectDispatchRegistry::unregisterRecipient(const BoundObject & object) BOOST_NOEXCEPT
  {
    invokeLogOnError(__FUNCTION__, [&] {
      _boundObjectRegistry->remove(object.uid());
    });
  }

  RemoteObjectPtr DirectDispatchRegistry::findRemoteObject(const ObjectUid& uid) const BOOST_NOEXCEPT
  {
    return invokeLogOnError(__FUNCTION__, [&] {
      return _remoteObjectRegistry->find(uid);
    });

  }

  BoundObjectPtr DirectDispatchRegistry::findBoundObject(const ObjectUid& uid) const BOOST_NOEXCEPT
  {
    return invokeLogOnError(__FUNCTION__, [&] {
      return _boundObjectRegistry->find(uid);
    });
  }


  namespace
  {
    boost::optional<ObjectUid> extractObjectUid(const Message& message)
    {
      ObjectUid uid;
      const auto& buffer = message.buffer();
      if (buffer.size() < size(uid))
      {
        return {};
      }
      // Here we assume that the last thing in the buffer is the uid.
      const size_t uidOffset = buffer.size() - size(uid);
      const auto readCount = buffer.read(begin(uid), uidOffset, size(uid));
      QI_ASSERT_TRUE(readCount == size(uid));
      QI_ASSERT_FALSE(uid == qi::ObjectUid{});
      return uid;
    }


    template<class NetworkObject>
    bool dispatchMessageToObject(const ObjectUid& id, const Message& message,
                  const detail::ObjectNetworkInterfaceRegistry<NetworkObject>& registry,
                  const MessageSocketPtr& socket)
    {
      const auto object = registry.find(id);
      if (!object)
        return false;

      object->onMessage(message, socket);

      return true;
    }

  }

  bool DirectDispatchRegistry::dispatchMessage(Message& message, const MessageSocketPtr& socket) const BOOST_NOEXCEPT
  {
    return invokeLogOnError(__FUNCTION__, [&] {
      QI_ASSERT_FALSE(message.recipientUid()); // Keep this assert to detect this situation when developing.
      if(message.recipientUid()) // We still implement a predictable behavior to avoid random crashes in the end product.
        return false;

      const auto uid = extractObjectUid(message);
      if(!uid)
        return false;
      message.setRecipientUid(uid);
      QI_ASSERT_TRUE(message.recipientUid());

      qiLogDebug() << "Direct dispatch in " << this << " : message id:" << message.id() << " for " << uid;
      bool success = dispatchMessageToObject(uid.get(), message, *_boundObjectRegistry.synchronize(), socket);
      if(!success)
        success = dispatchMessageToObject(uid.get(), message, *_remoteObjectRegistry.synchronize(), socket);

      if (!success)
      {
        qiLogWarning() << "Failed direct dispatch in " << this << " (fallback to legacy/slow dispatching system) : " << message;
      }

      return success;
    });
  }

}


