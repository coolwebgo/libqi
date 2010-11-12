/*
** Author(s):
**  - Cedric GESTES <gestes@aldebaran-robotics.com>
**
** Copyright (C) 2010 Aldebaran Robotics
*/

#include <gtest/gtest.h>
#include <qi/signature.hpp>
//#include <qi/perf/dataperftimer.hpp>

#include <vector>
#include <map>


static const int gLoopCount   = 1000000;

static int gGlobalResult = 0;

void vfun0()                                                                                      { gGlobalResult = 0; }
void vfun1(const int &p0)                                                                         { gGlobalResult = p0; }
void vfun2(const int &p0,const int &p1)                                                           { gGlobalResult = p0 + p1; }
void vfun3(const int &p0,const int &p1,const int &p2)                                             { gGlobalResult = p0 + p1 + p2; }
void vfun4(const int &p0,const int &p1,const int &p2,const int &p3)                               { gGlobalResult = p0 + p1 + p2 + p3; }
void vfun5(const int &p0,const int &p1,const int &p2,const int &p3,const  int &p4)                { gGlobalResult = p0 + p1 + p2 + p3 + p4; }
void vfun6(const int &p0,const int &p1,const int &p2,const int &p3,const  int &p4,const  int &p5) { gGlobalResult = p0 + p1 + p2 + p3 + p4 + p5; }

int fun0()                                                                                      { return 0; }
int fun1(const int &p0)                                                                         { return p0; }
int fun2(const int &p0,const int &p1)                                                           { return p0 + p1; }
int fun3(const int &p0,const int &p1,const int &p2)                                             { return p0 + p1 + p2; }
int fun4(const int &p0,const int &p1,const int &p2,const int &p3)                               { return p0 + p1 + p2 + p3; }
int fun5(const int &p0,const int &p1,const int &p2,const int &p3,const  int &p4)                { return p0 + p1 + p2 + p3 + p4; }
int fun6(const int &p0,const int &p1,const int &p2,const int &p3,const  int &p4,const  int &p5) { return p0 + p1 + p2 + p3 + p4 + p5; }


struct Foo {
  void voidCall()                                          { return; }
  int intStringCall(const std::string &plouf)              { return plouf.size(); }

  int fun0()                                                                                      { return 0; }
  int fun1(const int &p0)                                                                         { return p0; }
  int fun2(const int &p0,const int &p1)                                                           { return p0 + p1; }
  int fun3(const int &p0,const int &p1,const int &p2)                                             { return p0 + p1 + p2; }
  int fun4(const int &p0,const int &p1,const int &p2,const int &p3)                               { return p0 + p1 + p2 + p3; }
  int fun5(const int &p0,const int &p1,const int &p2,const int &p3,const  int &p4)                { return p0 + p1 + p2 + p3 + p4; }
  int fun6(const int &p0,const int &p1,const int &p2,const int &p3,const  int &p4,const  int &p5) { return p0 + p1 + p2 + p3 + p4 + p5; }

  void vfun0()                                                                                      { gGlobalResult = 0; }
  void vfun1(const int &p0)                                                                         { gGlobalResult = p0; }
  void vfun2(const int &p0,const int &p1)                                                           { gGlobalResult = p0 + p1; }
  void vfun3(const int &p0,const int &p1,const int &p2)                                             { gGlobalResult = p0 + p1 + p2; }
  void vfun4(const int &p0,const int &p1,const int &p2,const int &p3)                               { gGlobalResult = p0 + p1 + p2 + p3; }
  void vfun5(const int &p0,const int &p1,const int &p2,const int &p3,const  int &p4)                { gGlobalResult = p0 + p1 + p2 + p3 + p4; }
  void vfun6(const int &p0,const int &p1,const int &p2,const int &p3,const  int &p4,const  int &p5) { gGlobalResult = p0 + p1 + p2 + p3 + p4 + p5; }
};

TEST(TestSignature, BasicTypeSignature) {
  EXPECT_EQ("b",    qi::signature<bool>::value());
  EXPECT_EQ("i",    qi::signature<int>::value());
  EXPECT_EQ("f",    qi::signature<float>::value());
  EXPECT_EQ("d",    qi::signature<double>::value());
  EXPECT_EQ("s",    qi::signature<std::string>::value());
  EXPECT_EQ("[i]",  qi::signature< std::vector<int> >::value());
  typedef std::map<int,int> MapInt;
  EXPECT_EQ("{ii}", qi::signature< MapInt >::value() );

  EXPECT_EQ("b",    qi::signature<const bool>::value());
  EXPECT_EQ("i",    qi::signature<const int>::value());
  EXPECT_EQ("f",    qi::signature<const float>::value());
  EXPECT_EQ("d",    qi::signature<const double>::value());
  EXPECT_EQ("s",    qi::signature<const std::string>::value());
  EXPECT_EQ("[i]",  qi::signature<const std::vector< int > >::value());
  EXPECT_EQ("{ii}", qi::signature<const MapInt >::value());

  EXPECT_EQ("b*",   qi::signature<const bool*>::value());
  EXPECT_EQ("i*",   qi::signature<const int*>::value());
  EXPECT_EQ("f*",   qi::signature<const float*>::value());
  EXPECT_EQ("d*",   qi::signature<const double*>::value());
  EXPECT_EQ("s*",   qi::signature<const std::string*>::value());
  EXPECT_EQ("[i]*", qi::signature<const std::vector< int >* >::value());
  EXPECT_EQ("{ii}*",qi::signature<const MapInt* >::value());

  EXPECT_EQ("b",    qi::signature<const bool&>::value());
  EXPECT_EQ("i",    qi::signature<const int&>::value());
  EXPECT_EQ("f",    qi::signature<const float&>::value());
  EXPECT_EQ("d",    qi::signature<const double&>::value());
  EXPECT_EQ("s",    qi::signature<const std::string&>::value());
  EXPECT_EQ("[i]",  qi::signature<const std::vector< int >& >::value());
  EXPECT_EQ("{ii}", qi::signature<const MapInt& >::value());

  //ERROR
  EXPECT_EQ("UNKNOWN", qi::signature<short>::value());
}

TEST(TestSignature, ComplexTypeSignature) {
  typedef std::map<int,int> MapInt;
  //{ii}
  typedef std::map<MapInt,MapInt> MapInt2;
  //{{ii}{ii}}
  typedef std::map<std::vector<MapInt2>, std::vector<const std::vector<MapInt2&> > > FuckinMap;
  //{[{{ii}{ii}}][[{{ii}{ii}}&]#]}
  //and obama said: Yes We Can!

  EXPECT_EQ("{[{{ii}{ii}}][[{{ii}{ii}}]]}"      , qi::signature<FuckinMap>::value());

}

TEST(TestSignature, FunctionType) {
  EXPECT_EQ("v:ii", qi::signature<void (int, int)>::value());
  EXPECT_EQ("v:ii", qi::signature<void (*)(int, int)>::value());
  //exit(1);
}

TEST(TestSignature, BasicVoidFunctionSignature) {
  EXPECT_EQ("v:"      , qi::signatureFromObject::value(&vfun0));
  EXPECT_EQ("v:i"     , qi::signatureFromObject::value(&vfun1));
  EXPECT_EQ("v:ii"    , qi::signatureFromObject::value(&vfun2));
  EXPECT_EQ("v:iii"   , qi::signatureFromObject::value(&vfun3));
  EXPECT_EQ("v:iiii"  , qi::signatureFromObject::value(&vfun4));
  EXPECT_EQ("v:iiiii" , qi::signatureFromObject::value(&vfun5));
  EXPECT_EQ("v:iiiiii", qi::signatureFromObject::value(&vfun6));
  EXPECT_EQ("v:"      , qi::signatureFromObject::value(vfun0));
  EXPECT_EQ("v:i"     , qi::signatureFromObject::value(vfun1));
  EXPECT_EQ("v:ii"    , qi::signatureFromObject::value(vfun2));
  EXPECT_EQ("v:iii"   , qi::signatureFromObject::value(vfun3));
  EXPECT_EQ("v:iiii"  , qi::signatureFromObject::value(vfun4));
  EXPECT_EQ("v:iiiii" , qi::signatureFromObject::value(vfun5));
  EXPECT_EQ("v:iiiiii", qi::signatureFromObject::value(vfun6));

  EXPECT_EQ("v:ii", qi::signature<void (int, int)>::value());
}

TEST(TestSignature, FromObject) {
  int myint;
  EXPECT_EQ("i*"     , qi::signatureFromObject::value(&myint));
}

TEST(TestSignature, BasicFunctionSignature) {
  EXPECT_EQ("i:"      , qi::signatureFromObject::value(&fun0));
  EXPECT_EQ("i:i"     , qi::signatureFromObject::value(&fun1));
  EXPECT_EQ("i:ii"    , qi::signatureFromObject::value(&fun2));
  EXPECT_EQ("i:iii"   , qi::signatureFromObject::value(&fun3));
  EXPECT_EQ("i:iiii"  , qi::signatureFromObject::value(&fun4));
  EXPECT_EQ("i:iiiii" , qi::signatureFromObject::value(&fun5));
  EXPECT_EQ("i:iiiiii", qi::signatureFromObject::value(&fun6));
  EXPECT_EQ("i:"      , qi::signatureFromObject::value(fun0));
  EXPECT_EQ("i:i"     , qi::signatureFromObject::value(fun1));
  EXPECT_EQ("i:ii"    , qi::signatureFromObject::value(fun2));
  EXPECT_EQ("i:iii"   , qi::signatureFromObject::value(fun3));
  EXPECT_EQ("i:iiii"  , qi::signatureFromObject::value(fun4));
  EXPECT_EQ("i:iiiii" , qi::signatureFromObject::value(fun5));
  EXPECT_EQ("i:iiiiii", qi::signatureFromObject::value(fun6));
}

TEST(TestSignature, BasicVoidMemberSignature) {
  EXPECT_EQ("v:"      , qi::signatureFromObject::value(&Foo::vfun0));
  EXPECT_EQ("v:i"     , qi::signatureFromObject::value(&Foo::vfun1));
  EXPECT_EQ("v:ii"    , qi::signatureFromObject::value(&Foo::vfun2));
  EXPECT_EQ("v:iii"   , qi::signatureFromObject::value(&Foo::vfun3));
  EXPECT_EQ("v:iiii"  , qi::signatureFromObject::value(&Foo::vfun4));
  EXPECT_EQ("v:iiiii" , qi::signatureFromObject::value(&Foo::vfun5));
  EXPECT_EQ("v:iiiiii", qi::signatureFromObject::value(&Foo::vfun6));
}

TEST(TestSignature, BasicMemberSignature) {
  EXPECT_EQ("i:"      , qi::signatureFromObject::value(&Foo::fun0));
  EXPECT_EQ("i:i"     , qi::signatureFromObject::value(&Foo::fun1));
  EXPECT_EQ("i:ii"    , qi::signatureFromObject::value(&Foo::fun2));
  EXPECT_EQ("i:iii"   , qi::signatureFromObject::value(&Foo::fun3));
  EXPECT_EQ("i:iiii"  , qi::signatureFromObject::value(&Foo::fun4));
  EXPECT_EQ("i:iiiii" , qi::signatureFromObject::value(&Foo::fun5));
  EXPECT_EQ("i:iiiiii", qi::signatureFromObject::value(&Foo::fun6));
}
