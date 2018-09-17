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

  void DirectDispatchRegistry::registerDestination(RemoteObject& object) BOOST_NOEXCEPT
  {
    invokeLogOnError(__FUNCTION__, [&]{
      _remoteObjectRegistry->add(object.remotePtrUid(), object);
    });
  }

  void DirectDispatchRegistry::registerDestination(BoundObject & object) BOOST_NOEXCEPT
  {
    invokeLogOnError(__FUNCTION__, [&] {
      _boundObjectRegistry->add(object.ptrUid(), object);
    });
  }

  void DirectDispatchRegistry::unregisterDestination(const RemoteObject& object) BOOST_NOEXCEPT
  {
    invokeLogOnError(__FUNCTION__, [&] {
      _remoteObjectRegistry->remove(object.remotePtrUid());
    });
  }

  void DirectDispatchRegistry::unregisterDestination(const BoundObject & object) BOOST_NOEXCEPT
  {
    invokeLogOnError(__FUNCTION__, [&] {
      _boundObjectRegistry->remove(object.ptrUid());
    });
  }

  RemoteObjectPtr DirectDispatchRegistry::findRemoteObject(const PtrUid& id) const BOOST_NOEXCEPT
  {
    return invokeLogOnError(__FUNCTION__, [&] {
      return _remoteObjectRegistry->find(id);
    });

  }

  BoundObjectPtr DirectDispatchRegistry::findBoundObject(const PtrUid& id) const BOOST_NOEXCEPT
  {
    return invokeLogOnError(__FUNCTION__, [&] {
      return _boundObjectRegistry->find(id);
    });
  }


  namespace
  {
    boost::optional<PtrUid> extractPtrUid(const Message& message)
    {
      PtrUid id;
      const auto& buffer = message.buffer();
      if (buffer.size() < size(id))
      {
        return {};
      }
      // Here we assume that the last thing in the buffer is the uid.
      const size_t ptrUidOffset = buffer.size() - size(id);
      const auto readCount = buffer.read(begin(id), ptrUidOffset, size(id));
      QI_ASSERT_TRUE(readCount == size(id));
      QI_ASSERT_FALSE(id == qi::PtrUid{});
      return id;
    }


    template<class NetworkObject>
    bool dispatchMessageToObject(const PtrUid& id, const Message& message,
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
      QI_ASSERT_FALSE(message.destinationUID()); // Keep this assert to detect this situation when developing.
      if(message.destinationUID()) // We still implement a predictable behavior to avoid random crashes in the end product.
        return false;

      const auto id = extractPtrUid(message);
      if(!id)
        return false;
      message.setDestinationId(id);
      QI_ASSERT_TRUE(message.destinationUID());

      qiLogDebug() << "Direct dispatch in " << this << " : message id:" << message.id() << " for " << id;
      bool success = dispatchMessageToObject(id.get(), message, *_boundObjectRegistry.synchronize(), socket);
      if(!success)
        success = dispatchMessageToObject(id.get(), message, *_remoteObjectRegistry.synchronize(), socket);

      if (!success)
      {
        qiLogWarning() << "Failed direct dispatch in " << this << " (fallback to legacy/slow dispatching system) : " << message;
      }

      return success;
    });
  }

}


