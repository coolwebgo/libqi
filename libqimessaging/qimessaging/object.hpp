/*
** Author(s):
**  - Cedric GESTES <gestes@aldebaran-robotics.com>
**
** Copyright (C) 2012 Aldebaran Robotics
*/

#pragma once
#ifndef _QIMESSAGING_OBJECT_HPP_
#define _QIMESSAGING_OBJECT_HPP_

#include <map>
#include <string>
#include <qimessaging/details/makefunctor.hpp>
#include <qimessaging/signature.hpp>

namespace qi {

  class MetaMethod {
  public:
    MetaMethod(const std::string &name, const std::string &sig, const qi::Functor *functor);
    MetaMethod();

    const std::string &name() const { return _name; }
    const std::string &signature() const { return _signature; }

  protected:
  public:
    std::string  _name;
    std::string  _signature;
    const qi::Functor *_functor;
    unsigned int _idx;
  };

  qi::DataStream &operator<<(qi::DataStream &stream, const MetaMethod &meta);
  qi::DataStream &operator>>(qi::DataStream &stream, MetaMethod &meta);

  class MetaObject {
  public:
    MetaObject()
      : _methodsNumber(0)
    {
    };
    /*
     * When a member is added, serialization and deserialization
     * operators _MUST_ be updated.
     */
    std::map<std::string, unsigned int> _methodsNameToIdx;
    std::vector<MetaMethod>             _methods;
    unsigned int                        _methodsNumber;
    // std::map<std::string, MethodInfo>   _signals;
    // std::map<std::string, MethodInfo>   _slots;
    // std::map<std::string, PropertyInfo> _properties;
  };
  qi::DataStream &operator<<(qi::DataStream &stream, const MetaObject &meta);
  qi::DataStream &operator>>(qi::DataStream &stream, MetaObject &meta);


  class Object {
  public:
    Object();
    virtual ~Object();

    MetaObject &metaObject();

    template <typename OBJECT_TYPE, typename METHOD_TYPE>
    inline unsigned int advertiseMethod(const std::string& name, OBJECT_TYPE object, METHOD_TYPE method);

    template <typename FUNCTION_TYPE>
    inline unsigned int advertiseMethod(const std::string& name, FUNCTION_TYPE function);


    void callVoid(const std::string& methodName);
    template <typename RETURN_TYPE>
    RETURN_TYPE call(const std::string& methodName);

    template <typename P0>
    void callVoid(const std::string& methodName, const P0 &p0);
    template <typename RETURN_TYPE, typename P0>
    RETURN_TYPE call(const std::string& methodName, const P0 &p0);

    template <typename P0, typename P1>
    void callVoid(const std::string& methodName, const P0 &p0, const P1 &p1);
    template <typename RETURN_TYPE, typename P0, typename P1>
    RETURN_TYPE call(const std::string& methodName, const P0 &p0, const P1 &p1);

    template <typename P0, typename P1, typename P2>
    void callVoid(const std::string& methodName, const P0 &p0, const P1 &p1, const P2 &p2);
    template <typename RETURN_TYPE, typename P0, typename P1, typename P2>
    RETURN_TYPE call(const std::string& methodName, const P0 &p0, const P1 &p1, const P2 &p2);

    template <typename P0, typename P1, typename P2, typename P3>
    void callVoid(const std::string& methodName, const P0 &p0, const P1 &p1, const P2 &p2, const P3 &p3);
    template <typename RETURN_TYPE, typename P0, typename P1, typename P2, typename P3>
    RETURN_TYPE call(const std::string& methodName, const P0 &p0, const P1 &p1, const P2 &p2, const P3 &p3);

    template <typename P0, typename P1, typename P2, typename P3, typename P4>
    void callVoid(const std::string& methodName, const P0 &p0, const P1 &p1, const P2 &p2, const P3 &p3, const P4 &p4);
    template <typename RETURN_TYPE, typename P0, typename P1, typename P2, typename P3, typename P4>
    RETURN_TYPE call(const std::string& methodName, const P0 &p0, const P1 &p1, const P2 &p2, const P3 &p3, const P4 &p4);

    template <typename P0, typename P1, typename P2, typename P3, typename P4, typename P5>
    void callVoid(const std::string& methodName, const P0 &p0, const P1 &p1, const P2 &p2, const P3 &p3, const P4 &p4, const P5 &p5);
    template <typename RETURN_TYPE, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5>
    RETURN_TYPE call(const std::string& methodName, const P0 &p0, const P1 &p1, const P2 &p2, const P3 &p3, const P4 &p4, const P5 &p5);

    template <typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
    void callVoid(const std::string& methodName, const P0 &p0, const P1 &p1, const P2 &p2, const P3 &p3, const P4 &p4, const P5 &p5, const P6 &p6);
    template <typename RETURN_TYPE, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6>
    RETURN_TYPE call(const std::string& methodName, const P0 &p0, const P1 &p1, const P2 &p2, const P3 &p3, const P4 &p4, const P5 &p5, const P6 &p6);

    template <typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
    void callVoid(const std::string& methodName, const P0 &p0, const P1 &p1, const P2 &p2, const P3 &p3, const P4 &p4, const P5 &p5, const P6 &p6, const P7 &p7);
    template <typename RETURN_TYPE, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7>
    RETURN_TYPE call(const std::string& methodName, const P0 &p0, const P1 &p1, const P2 &p2, const P3 &p3, const P4 &p4, const P5 &p5, const P6 &p6, const P7 &p7);

    template <typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
    void callVoid(const std::string& methodName, const P0 &p0, const P1 &p1, const P2 &p2, const P3 &p3, const P4 &p4, const P5 &p5, const P6 &p6, const P7 &p7, const P8 &p8);
    template <typename RETURN_TYPE, typename P0, typename P1, typename P2, typename P3, typename P4, typename P5, typename P6, typename P7, typename P8>
    RETURN_TYPE call(const std::string& methodName, const P0 &p0, const P1 &p1, const P2 &p2, const P3 &p3, const P4 &p4, const P5 &p5, const P6 &p6, const P7 &p7, const P8 &p8);

    virtual void metaCall(unsigned int method, const std::string &sig, DataStream &in, DataStream &out);

  protected:
    unsigned int xAdvertiseMethod(const std::string &name, const std::string& signature, const Functor *functor);

  protected:
    MetaObject *_meta;
  };



template <typename OBJECT_TYPE, typename METHOD_TYPE>
inline unsigned int Object::advertiseMethod(const std::string& name, OBJECT_TYPE object, METHOD_TYPE method)
{
  std::string signature(name);
  signature += "::";
  signatureFromObject::value(method, signature);
  return xAdvertiseMethod(name, signature, makeFunctor(object, method));
}

template <typename FUNCTION_TYPE>
inline unsigned int Object::advertiseMethod(const std::string& name, FUNCTION_TYPE function)
{
  std::string signature(name);
  signature += "::";
  signatureFromObject::value(function, signature);
  return xAdvertiseMethod(name, signature, makeFunctor(function));
}



};

#include <qimessaging/object.hxx>
#endif  // _QIMESSAGING_OBJECT_HPP_
