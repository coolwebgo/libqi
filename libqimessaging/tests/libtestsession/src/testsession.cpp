/*
**
** Author(s):
**  - Pierre Roullon <proullon@aldebaran-robotics.com>
**
** Copyright (C) 2012 Aldebaran Robotics
*/

#include <testsession/testsession.hpp>
#include "testsession_p.hpp"

TestSession::TestSession(const std::string &serviceDirectoryUrl, bool listen, TestMode::Mode mode)
{
  _p = new TestSessionPrivate(serviceDirectoryUrl, mode, listen);
}

TestSession::~TestSession()
{
  delete _p;
}

qi::Session* TestSession::session()
{
  return _p->session();
}
