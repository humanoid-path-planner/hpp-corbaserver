#ifndef HPPCI_EXCEPTION_HANDLING_MACROS_H
#define HPPCI_EXCEPTION_HANDLING_MACROS_H

/**
   \brief CORBA exception handling

   catch various exceptions and return an error message on std::cerr.
*/

#define HPPCI_CATCH(msg, ret)			\
  catch(CORBA::SystemException&) {\
    std::cerr << "hppCorbaServer: CORBA::SystemException: " << msg << std::endl;\
    return ret;\
  }\
  catch(CORBA::Exception&) {\
    std::cerr << "hppCorbaServer: CORBA::Exception: " << msg << std::endl;\
    return ret;\
  }\
  catch(omniORB::fatalException& fe) {\
    std::cerr << "hppCorbaServer: CORBA::fatalException: " << msg << std::endl;\
    return ret;\
  }\
  catch(...) {\
    std::cerr << "hppCorbaServer: unknown exception: " << msg << std::endl;\
    return ret;\
  }\


#endif
