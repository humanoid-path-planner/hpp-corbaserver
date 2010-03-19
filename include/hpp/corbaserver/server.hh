// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of the hpp-corbaserver.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORBASERVER_SERVER_HH
# define HPP_CORBASERVER_SERVER_HH
# include <hpp/corbaserver/fwd.hh>
# include <hpp/corbaserver/config.hh>
# include <hppCore/hppPlanner.h>

/**

   The Corba server of Library Hpp is mainly implemented by class
   ChppciServer. This class basically:

   \li initializes a Corba server that implements several interfaces
   defined by the following modules,

   \li instantiate an object ChppPlanner a pointer to which is stored
   in ChppciServer::hppPlanner.

   The Corba interfaces trigger actions processed by ChppPlanner
   object. More information is provided in each interface
   documentation.
*/

/** \brief Implementation of Hpp module Corba server.
 *
 This class initializes the Corba server and starts the following Corba interface implementations.
 \li hppCorbaServer::ChppciRobot: to build a ChppDevice and to insert it in a ChppPlanner object,
 \li hppCorbaServer::ChppciObstacle: to build obstacles and insert them in a ChppPlanner object,
 \li hppCorbaServer::ChppciProblem: to define a path planning problem and solve it.

 To use this object, call the constructor

 \code
 int argc=1;
 char *argv[1] = {"program"};
 ChppPlanner* hppPlanner = new ChppPlanner;
 ChppciServer server(ChppPlanner* hppPlanner, argc, argv, isMultiThread);
 \endcode
 where \c isMultiThread specifies whether the server should process
 requests using multi-thread policy of not.

 After starting a name server and configuring your Corba implementation, start the servers.
 \code
 server.startCorbaServer();
 \endcode
 Then, enter in the loop that handle the Corba requests
 \code
 server.processRequest(true);
 \endcode
 You can then send request to the servers.
*/

namespace hpp
{
  namespace corbaServer
  {
    class HPP_CORBASERVER_DLLAPI Server
    {
    public:
      /**
	 \brief Constructor
	 \param inHppPlanner the object that will handle Corba requests.
	 \param argc, argv parameter to feed ORB initialization.
	 \param inMultiThread whether the server may process request using multithred policy.

	 \note It is recommended to configure your Corba implementation through environment
	 variables and to set argc to 1 and argv to any string.

	 \note It is highly recommended not to enable multi-thread policy in CORBA request processing when using
	 hppCorbaServer with KPP interface, since OpenGL is not compatible with multi-threading.

	 \note If multi-thread policy is not selected, request hppCorbaServer::ChppciProblem::interruptPathPlanning
	 will have no effect.
      */
      Server (ChppPlanner* inHppPlanner, int argc, const char* argv[], bool multiThread = false);

      /// \brief Shutdown CORBA server
      ~Server ();

      /// \brief Initialize CORBA server to process requests from clients to hpp module
      /// \return 0 if success, -1 if failure.
      int startCorbaServer ();

      /// \brief If ORB work is pending, process it
      /// \param loop if true, the function never returns; if false, the function processes pending requests and returns.
      int processRequest (bool loop);

      const ChppPlanner* planner () const;
      ChppPlanner* planner ();

      /**
	 \name Steering method management
	 @{
      */

      /**
	 \brief Indicates whether a name already corresponds to a steering method factory.
	 \param inName Name of the steering method factory.
	 \return true if name already corresponds to a steering method factory.
      */
      bool steeringMethodFactoryAlreadySet (std::string inName);

      /**
	 \brief Add a steering method factory
	 \param inName Name of the steering method built by factory.
	 \param inSteeringMethodFactory Steering method factory
	 \return true if success, false if name already used
      */
      bool addSteeringMethodFactory (std::string inName,
				     CkwsPlusSteeringMethodFactory* inSteeringMethodFactory);

      /**
	 \brief Get steering method from name.
      */
      CkwsSteeringMethodShPtr createSteeringMethod (std::string inName, bool inOriented);

      /**
	 @}
      */

      /**
	 \name Distance function management
      */
      /**
	 \brief Indicates whether a name already corresponds to a distance factory.
	 \param inName Name of the distance function factory.
	 \return true if name already corresponds to a distance function factory.
      */
      bool distanceFactoryAlreadySet (std::string inName);

      /**
	 \brief Add a distance function factory
	 \param inName Name of the distance function built by factory.
	 \param inDistanceFactory distance function factory
	 \return true if success, false if name already used
      */
      bool addDistanceFactory (std::string inName,
			      CkwsPlusDistanceFactory* inDistanceFactory);

      /**
	 \brief Get distance function from name.
      */
      CkwsDistanceShPtr createDistanceFunction (std::string inName, bool inOriented);

      /**
	 @}
      */

      /**
	 \name Diffusion node picker management
      */
      /**
	 \brief Indicates whether a name already corresponds to a diffusion node picker factory.
	 \param inName Name of the diffusion node picker factory.
	 \return true if name already corresponds to a diffusion node picker factory.
      */
      bool diffusionNodePickerFactoryAlreadySet (std::string inName);

      /**
	 \brief Add a diffusion node picker factory
	 \param inName Name of the distance function built by factory.
	 \param inDiffusionNodePickerFactory diffusion node picker factory
	 \return true if success, false if name already used
      */
      bool addDiffusionNodePickerFactory (std::string inName,
					 CkwsPlusDiffusionNodePickerFactory* inDiffusionNodePickerFactory);

      /**
	 \brief Get an instance of diffusion node picker from name.
	 \param inName name of the diffusion node picker.
      */
      CkwsDiffusionNodePickerShPtr createDiffusionNodePicker (std::string inName);

      /**
	 @}
      */

      /**
	 \name diffusion shooter management
      */
      /**
	 \brief Indicates whether a name already corresponds to a diffusion shooter factory.
	 \param inName Name of the diffusion shooter factory.
	 \return true if name already corresponds to a diffusion shooter factory.
      */
      bool diffusionShooterFactoryAlreadySet (std::string inName);

      /**
	 \brief Add a diffusion shooter factory
	 \param inName Name of the distance function built by factory.
	 \param inDiffusionShooterFactory diffusion shooter factory
	 \return true if success, false if name already used
      */
      bool addDiffusionShooterFactory (std::string inName,
				      CkwsPlusDiffusionShooterFactory* inDiffusionShooterFactory);

      /**
	 \brief Get an instance of diffusion shooter from name.
	 \param inName name of the diffusion shooter.
	 \param inStandardDeviation standard deviation of Gaussian sampling
      */
      CkwsDiffusionShooterShPtr createDiffusionShooter (std::string inName,
						       double inStandardDeviation);

      /**
	 @}
      */

    private:

      /**
	 \name CORBA server initialization
      */

      /**
	 \brief Initialize ORB and CORBA servers.

	 \param argc, argv parameter to feed ORB initialization.
	 \param inMultiThread whether the server may process request using multithred policy.
      */
      ktStatus initORBandServers (int argc, const char* argv[], bool inMultiThread);

      /**
	 @}
      */

      /**
	 \name Steering method factories
	 @{
      */
      /**
	 \brief Initialize map of steering method factories.

	 Insert default factories:
	 \li "linear" factory building linear steering methods,
	 \li "rs" factory building Reeds and Shepp steering methods (radius = 1),
	 \li "flic" factory building flat interpolation based steering methods.
      */
      void initMapSteeringMethodFactory ();

      /**
	 \brief Destroy each steering method factory stored in the map.
      */
      void destroySteeringMethodFactory ();

      /**
	 @}
      */

      /**
	 \name Steering method factories
	 @{
      */
      /**
	 \brief Associative array of Steering method factories.
      */
      std::map<std::string, CkwsPlusSteeringMethodFactory*> attMapSteeringMethodFactory;

      /**
	 @}
      */
      /**
	 \name Distance function factories
	 @{
      */
      /**
	 \brief Initialize map of distance function factories.

	 Insert default factories:
	 \li "linear" factory building linear distance functions,
	 \li "rs" factory building Reeds and Shepp distance functions (radius = 1),
	 \li "flic" factory building flat interpolation based distance functions.
      */
      void initMapDistanceFunctionFactory();

      /**
	 \brief Destroy each distance function factory stored in the map.
      */
      void destroyDistanceFunctionFactory ();

      /**
	 @}
      */
      /**
	 \name Distance function factories
	 @{
      */
      /**
	 \brief Associative array of distance function factories.
      */
      std::map<std::string, CkwsPlusDistanceFactory*> attMapDistanceFunctionFactory;

      /**
	 @}
      */
      /**
	 \name Diffusion node picker factories
	 @{
      */
      /**
	 \brief Initialize map of diffusion node picker factories.

	 Insert default factories:
	 \li "basic" factory building basic diffusion node pickers,
	 \li "smallestTree" factory building smallest tree diffusion node pickers,
      */
      void initMapDiffusionNodePickerFactory ();

      /**
	 \brief Destroy each diffusion node picker factory stored in the map.
      */
      void destroyDiffusionNodePickerFactory ();

      /**
	 @}
      */
      /**
	 \name Diffusion node picker factories
	 @{
      */
      /**
	 \brief Associative array of diffusion node picker factories.
      */
      std::map<std::string, CkwsPlusDiffusionNodePickerFactory*> attMapDiffusionNodePickerFactory;

      /**
	 @}
      */
      /**
	 \name diffusion shooter factories
	 @{
      */
      /**
	 \brief Initialize map of diffusion shooter factories.

	 Insert default factories:
	 \li "basic" factory building basic diffusion shooters,
	 \li "smallestTree" factory building smallest tree diffusion shooters,
      */
      void initMapDiffusionShooterFactory ();

      /**
	 \brief Destroy each diffusion shooter factory stored in the map.
      */
      void destroyDiffusionShooterFactory ();

      /**
	 @}
      */
      /**
	 \name diffusion shooter factories
	 @{
      */
      /**
	 \brief Associative array of diffusion shooter factories.
      */
      std::map<std::string, CkwsPlusDiffusionShooterFactory*> attMapDiffusionShooterFactory;

      /**
	 @}
      */

      impl::Server* attPrivate;

      /// \brief pointer to ChppPlanner Object.
      ///
      /// At initialization, the constructor creates a ChppPlanner
      /// object and keeps a pointer to it. All Corba requests are
      /// processed by this object. Notice that this pointer is passed
      /// to each constructor of implementation classes of the server
      /// Corba interface.
      ChppPlanner* hppPlanner;
    };

  } // end of namespace corbaServer.
} // end of namespace hpp.
#endif
