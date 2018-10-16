#include <qi/log.hpp>
#include <qi/messaging/sock/option.hpp>
#include "messagesocket.hpp"
#include "tcpmessagesocket.hpp"

// Disable "'this': used in base member initializer list"
#if BOOST_COMP_MSVC
# pragma warning(push)
# pragma warning(disable: 4355)
#endif


qiLogCategory(qi::sock::logCategory());

namespace qi
{
  MessageSocket::~MessageSocket()
  {
    qiLogDebug() << "Destroying transport socket";
    _signalsStrand.join();
  }

  bool MessageSocket::isConnected() const
  {
    return status() == qi::MessageSocket::Status::Connected;
  }

  MessageSocketPtr makeMessageSocket(const std::string &protocol, qi::EventLoop *eventLoop)
  {
    return makeTcpMessageSocket(protocol, eventLoop);
  }

  namespace {
    // Direct Message Routage capability: Append the destination id at the back of the buffer.
    void extendDirectMessageRoutageCapability(MessageSocket& socket, Message& msg)
    {
      if (detail::canBeDirectlyDispatched(msg, socket))
      {
        const auto maybeRecipientUid = msg.recipientUid();
        QI_ASSERT_TRUE(maybeRecipientUid); // TODO: replace by ka::empty once available
        const auto recipientUid = maybeRecipientUid.get();
        Buffer recipientUidBuffer;
        recipientUidBuffer.write(begin(recipientUid), size(recipientUid));
        auto msgBuffer = msg.extractBuffer();
        msgBuffer.addSubBuffer(std::move(recipientUidBuffer));
        msg.setBuffer(std::move(msgBuffer));
      }
    }
  }

  bool MessageSocket::send(Message msg)
  {
    extendDirectMessageRoutageCapability(*this, msg);
    qiLogDebug() << "Sending " << msg;
    return sendImpl(msg);
  }

}

#if BOOST_COMP_MSVC
# pragma warning(pop)
#endif
