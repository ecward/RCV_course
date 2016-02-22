# RCV_course

Code and documentation for DD3356-RCV2016.

This project on the RCV should is about integrating a number of modules to achieve some level of autonomy for the vehicle.

Course description:

<http://www.kth.se/student/kurser/kurs/DD3356?l=en>

Course group page:

<https://www.kth.se/social/group/dd3356-rcv2016/>

## Project proposal

> INL1: in the group write a project proposal including an explanation of the work and how each student contributes to it, the delegation between the students of the group, and how the work should be carried out. The proposal should also describe the expected learning outcomes for each student and how they should be achieved. Finally, the proposal should list the deliverables that are connected to the project. Each student should present and defend the proposal individually orally.

### Project members

* Nils Bore, CVAP KTH
* Xi Chen, CVAP KTH
* Silvia Cruciani, CVAP KTH
* Johan Ekekrantz, CVAP KTH
* Niclas Evestedt, Automaic Control LiU
* Rui Oliveira, Automatic Control KTH
* Erik Ward, CVAP KTH

### System

The Research Concept Vehicle (RCV) is a electric car developed at the
Integrated Transport Research Lab (ITRL) at KTH that is instrumented
with drive-by-wire capability and different sensors.

*TODO*

### Scenario descriptions

In this project the aim is to provide sufficient autonomy for two
types of scenarios involving the RCV and other traffic participants.

1. Parking lot (open area) navigation. The RCV should handle
navigating, at low speeds, to a defined pose, e.g. a
specific parking space, while avoiding collisions with static obstacles
and pedestrians.

2. Junction navigation. The RCV should successfully handle a T-junction
scenario where another car is driving through the junction and the
goal of the RCV is to make a left or right turn through the
junction. Here the speeds of the RCV and the other car is
significantly higher than in scenario 1., e.g. 10 m/s rather than
almost walking pace as in scenario 1.

The most fundamental requirements for these scenarios is perception
algorithms that can estimate the driveable surface relative to the RCV,
localization with respect to a world coordinate frame where goal
way-points are described (where the left or right turn leads to in
scenario 2. and where the specific pose is in scenario 1.), motion
planing that avoids static obstacles and path-following control of the
vehicle. Both these scenarios requires detection and tracking of moving
objects, in scenario 2. another car and scenario 1. pedestrians, using
sensors mounted on the vehicle. For scenario 1. the detection range
requirement is shorter than in scenario 2.  however we need to be able
to detect much smaller targets.

Scenario 1. requires a model of where roads are and topology of the
road network, e.g. that the RCV should be in the right lane and this
leads to the right lane of the connecting road after the turn. A map
of the road network, such as an RNDF file or a Lanelet file, containing
the lon,lat of center of lanes is assumed to be available a-priori
although it might not be very accurate. We thus need to estimate the
RCVs position relative to the off-line road map, e.g. the distance to
the right edge of the road.

The motion planning in scenario 1. will at least be able to find a
route that does not collide with any static obstacle.  Consideration
of dynamic obstacles will be handled either by slowing down or by
considering dynamic obstacles during planning. Motion planning in
scenario 2.  is mostly concerned with small corrections with respect
the route given by the lane center in order to achieve a smooth
ride. Scenario 1. and 2. presents two different types of problems,
where in scenario 1. we need to perform precise maneuvering at low
speeds where in scenario 2. we need to follow the road, rather than
finding a route, at higher speeds. In addition to avoiding obstacles
and dynamic objects generated speed profiles for the RCV car should
take into account comfort constraints such as maximal acceleration and
jerk.

While the route planning and speed planning might only consider a
simplified model of the vehicle, the employed lateral and longitudinal
control algorithms must provide stable and robust control and interface
with low-level control components of the vehicle.

Scenario 2. requires inference about the behavior of the other car,
what rules of the road applies and finding speed profiles that
correspond to safe, natural behavior of the RCV, e.g. break if the
other car has the right of way and only proceeding through the
intersection when the collision risk of doing so is negligible.

There will have to be a high level behavior module of the robot that
can figure out what kind of actions are appropriate, e.g. driving
slower on parking lots and according to speed limits on roads. This
high level behavior module should be able to take as input a mission
way-point and switch behaviors appropriately until the way point is
reached.

Most of the functionally that is required is reliant on accurate
state estimation of the RCV.

### Integration and implementation

As far as possible we will use existing components to provide the
required functionality. These components either come from previous
work on the RCV, previous research code by project members or open
source components. The components will be integrated using ROS as middle-ware.

The perception requirements for the two scenarios represents the
majority of the work required, and 4 out of the 7 project members will
be focused on this.

For localization and mapping, we will rely both on GPS position and the
odometry and velodyne measurements from the RCV. Both the GPS and
odometry will potentially require some amount of work to integrate into
a ROS system. The velodyne is deemed more simple. Most likely, we will
provide two separate localization systems with mapping running on its own, without
integrating GPS coordinates. However, the system will be able to re-initialize
using GPS coordinates if we can not track our position in the map.

To localize using velodyne data, there are several different options.
Considering the environment at the test site, robust localization will
require that we use the full 3D data as opposed to a 2D slice.
Importantly, we fill focus on localizing with respect to a pre-built map.
The initial map will be built using an off-the-shelf SLAM approach.
Then, one option is to use [insert Xi's ICP registration stuff]. However, this
might need further adaptation to encompass the sparseness of the measurements.
One approach that has proven to work in similar scenarios is the NDT-MCL
framework from Örebro, *Normal Distributions Transform Monte-Carlo Localization*
by Saarinen et al., which uses the *Normal Distribution
Transform* for registration and *Monte Carlo localization* for state estimation.
Johan will work on registration of the velodyne data, something which might
potentially be used for localization with respect to a point-based map.

The scenarios also require that we have some tracking of dynamic objects.
At the end of the project the system will be able to distinguish between
static and dynamic elements in the map. Further, the dynamic elements should
be separated into pedestrians and car. This might feed back into the planning
system to adapt the behaviour to different situations. Again, there are
several possibilities for distinguishing between classes of objects in
laser point clouds, especially with applications in driving scenarios.
Ingmar Posner and his group have done a lot of work in this area and they
have presented some systems that should be relatively straightforward to
implement. In particular, they presented one paper last year that combined
a method based on a kind of convolution in combination with a linear
classifier. This can efficiently and precisely detect cars and
pedestrians oriented along a few main directions, an assumption that should
hold in our application scenario.
There are several other benchmarks which perform better on the widely used
*Kitti* data set. One such example with code available is
*3D Object Proposals for Accurate Object Class Detection* from Toronto.
Similar to other well performing methods, they employ neural nets for
classification. Another example of a high scoring implementation with
code available is *Faster R-CNN: Towards Real-Time Object
Detection with Region Proposal Networks* from University of Technology
and Science of China and Microsoft Research.
The plan is to integrate one of the neural net based systems is possible
and if not implement a simple classifier like the from Posner's group.


*TODO, this is just what I remember from the meetings/talking, I am sure I'm wrong, please update!*

* Nils and Xi: Integrate ROS NDT framework from Örebro in order to do mapping and localization of the vehicle.

* Nils: Dynamic object tracking from velodyne data.

* Xi: Road, Obstacle, Not Road classification of point clouds. That can be used to provide a local grid map for
planning.

* Xi: (tentative) Road map using road/not-road grid-map. Should
  contain center of lanes as connected line-segments and route graph.

* Erik: Offline road map, hard-coded as Lanelet file. Use of existing code from iQMatic project
  for this.

* Silvia, Johan: Detection and tracking of moving objects using stereo camera

* Johan: point cloud registration for object modeling and mapping

* Niclas, Erik: Trajectory planning, Werling, for risk averse behavior

* Niclas: Integration of motion planner

* Erik: Risk inference in intersection using probabilistic modeling of other vehicle's future motion.

* Rui: Lateral/longitudinal MPC, interface with lower level systems of the RCV to ROS.

### Learning outcomes

* Erik: Understanding of possibilities and limitations of perception
  algorithms based on velodyne data and state of the art computer
  vision algorithms for use in higher level behaviors. For this
  project my primary goal is to implement and test base-line methods
  based on a time-gap model for junction navigation and if possible
  test my own research on the RCV.

* Niclas: Previously my work has focused on path generation in
  unstructured areas such as parking lots or open pit mining areas. In
  this project I want to gain more understanding for methods for
  trajectory generation in structured environments such as when
  driving in a road network. I want to test the methods presented by
  Werling to generate a vast amount of candidate trajectories than can
  later be evaluated according to cost functions based on safety,
  comfort, progress etc. A big part will be to find suitable cost
  functions and integrate with the behavioural layer and risk
  evaluation methods developed by Erik.

* Silvia: Deep understanding of state of the art computer vision algorithms for
  object detection and tracking, and learning how to apply to real problems and 
  for real time applications. For this project my goal is to implement detection 
  and tracking system, mainly for obstacle avoidance purpose, using a stereo camera system.

### Deliverables

*TODO, I don't know if we can provide a list of deliverables that are prioritized, so that if we run out of time we don't do some of them? How strict is it to meet all of the deliverables we promise here?*
