/// \page hpp_corbaserver_porting_notes Porting notes
///
/// \section hpp_corbaserver_porting_4_to_5 Porting your code from version 4 to 5
/// \subsection hpp_corbaserver_porting_4_to_5_1 Modification of idl API
///
/// \subsubsection hpp_corbaserver_porting_4_to_5_1_1 Method
/// hpp::corbaserver::Obstacle::loadObstacleModel has been split in two methods
///
/// \li hpp::corbaserver::Obstacle::loadObstacleModel (in string filename, in string prefix), and
/// \li hpp::corbaserver::Obstacle::loadObstacleModelFromString (in string urdfString, in string prefix).
///
/// Arguments to loadObstacleModel are now
/// \li a filename and a prefix, instead of
/// \li a package name, a filename radical and a prefix.
///
/// As in many other methods and functions, the filename now includes reference
/// to the package: "package://...".
///
/// \subsubsection hpp_corbaserver_porting_4_to_5_1_2 Methods
/// hpp::corbaserver::Robot::loadRobotModel and hpp::corbaserver::Robot::loadHumanoidModel
///
/// Arguments
/// \li in string packageName, in string modelName, in string urdfSuffix, in string srdfSuffix have been replaced by
/// \li in string urdfName, in string srdfName.
///
/// As in the previous section these latter arguments contain the full filenames
/// to the urdf and srdf files.
///
/// 
/// \subsection hpp_corbaserver_porting_4_to_5_2 Modification of python API
///
/// \subsubsection hpp_corbaserver_porting_4_to_5_2_1 Method
/// hpp.corbaserver.problem_solver.ProblemSolver.loadObstacleFromUrdf
///
/// Arguments
/// \li package, filename have been replaced by
/// \li filename.
/// Calling the method with 3 arguments interpretes arguments as (package, filename, prefix) and raises a warning.
/// Note that it is not possible to call arguments by names.
