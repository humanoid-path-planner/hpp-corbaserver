#include "hppCorbaServer/hppciServer.h"
#include "KineoModel/kppLicense.h"

#if DEBUG==2
#define ODEBUG2(x) std::cout << "hppCorbaServer:" << x << std::endl
#define ODEBUG1(x) std::cerr << "hppCorbaServer:" << x << std::endl
#elif DEBUG==1
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "hppCorbaServer:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

int main(int argc, char** argv)
{
  if (CkppLicense::initialize() != true) {
    ODEBUG1(" failed to get a Kineo license");
    exit(-1);
  }
  ChppPlanner* hppPlanner = new ChppPlanner;
  ChppciServer server(hppPlanner, argc, argv, true);

  if (server.startCorbaServer() == -1){
    ODEBUG1(" failed to start CORBA server.");
    exit(-1);
  }
  server.processRequest(true);
}
