
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include "KinematicChain.h"
namespace ob=ompl::base;
namespace og=ompl::geometric;
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "memory"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/samplers/BridgeTestValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include "KinematicChain.h"
#include <ompl/base/OptimizationObjective.h>
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "clear_checker.h"
#include <vector>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
 #include "ompl/geometric/planners/prm/PRMstar.h"
#include "ompl/base/objectives/StateCostIntegralObjective.h"
bool segmentsIntersect(const Segment &s0, const Segment &s1);
bool selfIntersectionTest(const std::vector<Segment> &segments);
bool environmentIntersectionTest(const std::vector<Segment>& segments, const Environment& env);
bool isStateValid(double x, double y, double theta0, double theta1, double theta2, double theta3, double theta4, const Environment& env);

//// Helper functions to check if two segments intersect
//// This is done by checking if the endpoints of the segments lie on opposite sides of the line formed by the other segment
bool segmentsIntersect(const Segment &s0, const Segment &s1) {
    double x1 = s0.x0, y1 = s0.y0, x2 = s0.x1, y2 =s0.y1; // get the endpoints of the first segment
    double x3 = s1.x0, y3 = s1.y0, x4 = s1.x1, y4 = s1.y1; // get the endpoints of the second segment, by accessing the struct fields, x0, y0, x1, y1

    double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4); 
    const double EPSILON = 0.05;
    if(x2==x3 && y2==y3){
        if (((x1 >= x4 - EPSILON && x1 <= x4 + EPSILON) && 
         (y1 >= y4 - EPSILON && y1 <= y4 + EPSILON))) {
            return true;
        }
        else{
            return false;
        }
    }                                                             // calculate the denominator of the formula, which is the determinant of the matrix formed by the endpoints of the segments
    if (denom == 0 ) {
        return false; // if the determinant is zero, the segments are parallel and do not intersect
    }
    double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom; // calculate the t parameter, which is the ratio of the determinant of the matrix formed by the endpoints of the first segment and the denominator
    double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom; // calculate the u parameter, which is the ratio of the determinant of the matrix formed by the endpoints of the second segment and the denominator

    return t >= 0 && t <= 1 && u >= 0 && u <= 1; // if both t and u are between 0 and 1, the segments intersect
}

bool linksquareintersection(const std::vector<Segment> &segments,
                           const std::vector<Segment> &square_seg) {
    for (size_t i = 1; i < segments.size(); i++) { // iterate over all pairs of segments
        for (size_t j = 0; j < square_seg.size(); j++) { // check if the segments intersect 
            if (segmentsIntersect(segments[i], square_seg[j])) {
                return false;
            }
        }
    }
    return true;
}
bool selfIntersectionTest(const std::vector<Segment>& segments) {
    for (size_t i = 0; i < segments.size(); i++) { // iterate over all pairs of segments
        for (size_t j = i + 1; j < segments.size(); j++) { // check if the segments intersect
            
            if (segmentsIntersect(segments[i], segments[j])) {
                return false;
            }
        }
    }
    return true;
}

bool environmentIntersectionTest(const std::vector<Segment>& segments, const Environment& env) {
    for (const auto& segment : segments) { // iterate over all segments, checking for intersection with each obstacle, represented as a segment, in the environment
        for (const auto& obstacle : env) {
            if (segmentsIntersect(segment, obstacle)) {
                return false;
            }
        }
    }
    return true;
}

bool isStateValid(double x, double y, double theta0, double theta1, double theta2, double theta3, double theta4, const Environment& env) {
    std::vector<Segment> segments; // create a vector of tuples to store the segments of the robot
    std::vector<Segment> square_seg;
    double linklength = 1.0; // set the length of the links 
    //double theta = theta0; // set the initial angle of the robot
    double xN = x, yN = y; // set the initial position of the robot
    Segment vl((-0.5)*cos(theta0)-(-0.5)*sin(theta0)+x,(-0.5)*sin(theta0)+(-0.5)*cos(theta0)+y,
                (-0.5)*cos(theta0)-(+0.5)*sin(theta0)+x,(-0.5)*sin(theta0)+(+0.5)*cos(theta0)+y);
    square_seg.emplace_back(vl);

 /////right vertical edge
 Segment vr((+0.5)*cos(theta0)-(-0.5)*sin(theta0)+x,(+0.5)*sin(theta0)+(-0.5)*cos(theta0)+y,
                (+0.5)*cos(theta0)-(+0.5)*sin(theta0)+x,(+0.5)*sin(theta0)+(+0.5)*cos(theta0)+y);
    square_seg.emplace_back(vr);
 /////upper horizontal edge
 Segment vu((+0.5)*cos(theta0)-(+0.5)*sin(theta0)+x,(+0.5)*sin(theta0)+(+0.5)*cos(theta0)+y,
                (-0.5)*cos(theta0)-(+0.5)*sin(theta0)+x,(-0.5)*sin(theta0)+(+0.5)*cos(theta0)+y);
    square_seg.emplace_back(vu);

/////lower horizontal edge
Segment vd((-0.5)*cos(theta0)-(-0.5)*sin(theta0)+x,(-0.5)*sin(theta0)+(-0.5)*cos(theta0)+y,
                (+0.5)*cos(theta0)-(-0.5)*sin(theta0)+x,(+0.5)*sin(theta0)+(-0.5)*cos(theta0)+y);
    square_seg.emplace_back(vd);

    segments.reserve(5); // 5 segments for the robot
    double thetas[] = {theta1, theta2, theta3, theta4}; // array of the angles of the robot's joints
    double theta=theta0;
    for (double t : thetas) { // iterate over the angles of the robot's joints
        double xPrev = xN, yPrev = yN; // store the previous position of the robot
        theta += t; // update the angle of the robot
        xN = xPrev + cos(theta) * linklength; // calculate the new x position of the robot
        yN = yPrev + sin(theta) * linklength; // calculate the new y position of the robot
        Segment seg(xPrev, yPrev, xN, yN);
        segments.emplace_back(seg); // add the segment to the vector
    }
    //std::cout << "And I checked all the states" << std::endl;
    return linksquareintersection(segments,square_seg) && selfIntersectionTest(segments) && environmentIntersectionTest(segments, env); // check if the robot intersects itself and the environment
}
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si,const Environment &env,bool optimal) :
        ob::StateValidityChecker(si),env_(env),optimal_(optimal) {}
 
    // Returns whether the given state's position overlaps the
    // circular obstacle
    
    bool isValid(const ob::State* state) const
    {
        const auto *r2state = state->as<ob::CompoundState>();

        //  the first component  is SE2StateSpace 
        const auto *se2state = r2state->as<ob::SE2StateSpace::StateType>(0);
        double x = se2state->getX();
        double y = se2state->getY();
        double theta0 = se2state->getYaw();

        //  the second component (index 1) is RealVectorStateSpace (for theta1, theta2, theta3, theta4)
        const auto *angles = r2state->as<ob::RealVectorStateSpace::StateType>(1);
        double theta1 = angles->values[0];
        double theta2 = angles->values[1];
        double theta3 = angles->values[2];
        double theta4 = angles->values[3];
        //std::cout << "they came for me" << std::endl;
        return isStateValid(x,y,theta0,theta1,theta2,theta3,theta4,env_);
    }

   
 
    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State* state) const
    {   
       const auto *r2state = state->as<ob::CompoundState>();
       //std::cout << "I operated this time big" << std::endl;
        //  the second component  is SE2StateSpace 
        const auto *se2state = r2state->as<ob::SE2StateSpace::StateType>(0);
        double x = se2state->getX();
        double y = se2state->getY();
        double theta0 = se2state->getYaw();

        //  the first component (index 1) is RealVectorStateSpace (for theta1, theta2, theta3, theta4)
        const auto *angles = r2state->as<ob::RealVectorStateSpace::StateType>(1);
        double theta1 = angles->values[0];
        double theta2 = angles->values[1];
        double theta3 = angles->values[2];
        double theta4 = angles->values[3];
        if (optimal_){
            return link_clearance(x, y, theta0,  theta1,  theta2,  theta3, theta4);
        }
        return centre_clear(x,y);
    }   

     private:
    Environment env_;
    bool optimal_;


    
};


class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true)
    {
    }
 
    ob::Cost stateCost(const ob::State* s) const
    {
        return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));
    }
};

void writePath(const ob::PathPtr &path, const std::string& filename, Environment &env )
{
    std::ofstream pathFile(filename);
    if (!pathFile.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << " for writing." << std::endl;
        return;
    }

    auto* pathGeometric = path->as<og::PathGeometric>();
    pathGeometric->interpolate(300);
    if (!pathGeometric) {
        std::cerr << "Error: Path is not a geometric path." << std::endl;
        return;
    }
    
    std::cout << "Writing path with " << pathGeometric->getStateCount() << " states." << std::endl;
    
    for (std::size_t i = 0; i < pathGeometric->getStateCount(); ++i)
    {
        const auto* state = pathGeometric->getState(i);
        if (!state) {
            std::cerr << "Error: Null state at index " << i << std::endl;
            continue;
        }
        /*******this needs to be done *********/
        const auto *r2state = state->as<ob::CompoundState>();
        const auto *se2state = r2state->as<ob::SE2StateSpace::StateType>(0);
        double x = se2state->getX();
        double y = se2state->getY();
        double theta0 = se2state->getYaw();

        //  the first component (index 1) is RealVectorStateSpace (for theta1, theta2, theta3, theta4)
        const auto *angles = r2state->as<ob::RealVectorStateSpace::StateType>(1);
        double theta1 = angles->values[0];
        double theta2 = angles->values[1];
        double theta3 = angles->values[2];
        double theta4 = angles->values[3];

 
        pathFile << x << " " << y <<" " <<theta0 <<" " << theta1 <<" " <<theta2<<" " << theta3<<" " <<theta4 <<std::endl;
      
        
       
    }
    pathFile.close();
    std::cout << "Path written to " << filename << std::endl;
}








void makeScenario1(Environment &env, std::vector<double> &start, std::vector<double> &goal)
{

    start.reserve(7);
    start.assign(7, 0.0);

    goal.reserve(7);
    goal.assign(7, 0.0);

    start[0] = -3;
    start[1] = -3;
    goal[0] = 2 ; 
    goal[1] = 2 ; 
    goal[2] = 0; 
    
    goal[4] = -0.5*M_PI;
   


    //Obstacle 1
    env.emplace_back(2, -1, 2.8, -1);
    env.emplace_back(2.8, -1, 2.8, 0.5);
    env.emplace_back(2.8, 0.5, 2, 0.5);
    env.emplace_back(2, 0.5, 2, -1);

    //Obstacle 2
    env.emplace_back(3.2, -1, 4, -1);
    env.emplace_back(4, -1, 4, 0.5);
    env.emplace_back(4, 0.5, 3.2, 0.5);
    env.emplace_back(3.2, 0.5, 3.2, -1);

}

void makeScenario2(Environment &env, std::vector<double> &start, std::vector<double> &goal)
{
    start.reserve(7);
    start.assign(7, 0.0);

    goal.reserve(7);
    goal.assign(7, 0.0);

    start[0] = -4;
    start[1] = -4;
    start[2] = 0;
    goal[0] = 3; 
    goal[1] = 3; 
    goal[2] = 0; 

    //Obstacle 1
    env.emplace_back(-1, -1, 1, -1);
    env.emplace_back(1, -1, 1, 1);
    env.emplace_back(1, 1, -1, 1);
    env.emplace_back(-1, 1, -1, -1);
}

void planScenario1(ompl::geometric::SimpleSetup &ss, Environment &env)
{   
    // TODO: Plan for chain_box in the plane, and store the path in path1.txt. 
    auto planner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));
   
    ss.setPlanner(planner);
    ob::PlannerStatus solved=ss.solve(1.0);
       if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = ss.getProblemDefinition()->getSolutionPath();
        std::cout << "Found solution for kine robot:" << std::endl;

        // print the path to screen
        path->print(std::cout);

        // Save the path to a file
        writePath(path, "narrow_path.txt",env);
    }
    else
        std::cout << "No solution found" << std::endl;
}


void benchScenario1(ompl::geometric::SimpleSetup &ss)
{
    //TODO: Benchmark PRM with uniform, bridge, gaussian, and obstacle-based Sampling. Do 20 trials with 20 seconds each 
    double runtime_limit = 20, memory_limit = 1024;
    int run_count = 20;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, "ChainBox_Narrow");

    auto uniformPRM = std::make_shared<og::PRM>(ss.getSpaceInformation());
    // Set name
    uniformPRM->setName("UniformPRM");
    auto gaussianPRM = std::make_shared<og::PRM>(ss.getSpaceInformation());
    gaussianPRM->setName("GaussianPRM");
    auto bridgePRM = std::make_shared<og::PRM>(ss.getSpaceInformation());
    bridgePRM->setName("BridgePRM");
    auto obstaclePRM = std::make_shared<og::PRM>(ss.getSpaceInformation());
    obstaclePRM->setName("ObstaclePRM");

    ss.getSpaceInformation()->setValidStateSamplerAllocator(
        [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
            return std::make_shared<ob::UniformValidStateSampler>(si);
        });
    b.addPlanner(uniformPRM);


    ss.getSpaceInformation()->setValidStateSamplerAllocator(
        [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
            return std::make_shared<ob::GaussianValidStateSampler>(si);
        });
    b.addPlanner(gaussianPRM);

    
    ss.getSpaceInformation()->setValidStateSamplerAllocator(
        [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
            return std::make_shared<ob::BridgeTestValidStateSampler>(si);
        });
    b.addPlanner(bridgePRM);


    ss.getSpaceInformation()->setValidStateSamplerAllocator(
        [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
            return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
        });
    b.addPlanner(obstaclePRM);


    b.benchmark(request);
    b.saveResultsToFile("benchmark_uniform_gaussian_bridge_obstacle.log");
    }

void planScenario2(ompl::geometric::SimpleSetup &ss,Environment &env,bool optimal)
{
    // TODO: Plan for chain_box in the plane, with a clearance optimization objective, with an Asymptoticallly optimal planner of your choice and store the path in path2.txt
   
    ss.setOptimizationObjective(std::make_shared<ClearanceObjective>(ss.getSpaceInformation()));
    // TODO: Plan for chain_box in the plane, and store the path in path1.txt. 
    auto planner=std::make_shared<og::PRMstar>(ss.getSpaceInformation());
  
    

    ss.setPlanner(planner);
    ob::PlannerStatus solved=ss.solve(20.0);
       if (solved)
    {
       
        ob::PathPtr path = ss.getProblemDefinition()->getSolutionPath();
        
        std::cout << "Found solution for kine robot:" << std::endl;

        // print the path to screen
        path->print(std::cout);
        if(optimal){
        // if we are looking for optimal solution , we have to select 3 option which will save file as path2
        writePath(path, "path2.txt",env);}
        else{
        writePath(path, "clear_path.txt",env);}

        }
    
    else{
        std::cout << "No solution found" << std::endl;
    }
}


void benchScenario2(ompl::geometric::SimpleSetup &ss)
{
    //TODO: Benchmark RRT*, PRM*, RRT# for 10 trials with 60 secounds timeout.
    double runtime_limit = 60, memory_limit = 1024;
    int run_count = 10;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, "ChainBox_Clearance");

    auto objective = std::make_shared<ClearanceObjective>(ss.getSpaceInformation());
    ss.setOptimizationObjective(objective);

    auto prm = std::make_shared<og::PRM>(ss.getSpaceInformation());
    prm->setName("PRM");
    b.addPlanner(prm);

    auto rrt = std::make_shared<og::RRT>(ss.getSpaceInformation());
    rrt->setName("RRT");
    b.addPlanner(rrt);

    auto rrtsharp = std::make_shared<og::RRTsharp>(ss.getSpaceInformation());
    rrtsharp ->setName("RRTSharp");
    b.addPlanner(rrtsharp);

    b.benchmark(request);
    b.saveResultsToFile("benchmark_prm_rrt_rrtsharp.log");
}

std::shared_ptr<ompl::base::CompoundStateSpace> createChainBoxSpace()
{   //TODO Create the Chainbox ConfigurationSpace
    auto space = std::make_shared<ompl::base::CompoundStateSpace>();  
   
    //space->addSubspace(ompl::base::StateSpacePtr(new ompl::base::SE2StateSpace()),1.0);
    auto SEspace(std::make_shared<ob::SE2StateSpace>());
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);

    SEspace->setBounds(bounds);
    space->addSubspace(SEspace,1.0);
    space->addSubspace(ompl::base::StateSpacePtr(new KinematicChainSpace(4,1,nullptr)), 1.0);
    
    return space;
}
void setupCollisionChecker(ompl::geometric::SimpleSetup &ss, Environment &env,bool optimal)

    {
     
        ss.setStateValidityChecker(std::make_shared<ValidityChecker>(ss.getSpaceInformation(),env,optimal));
     
    }



    
int main(int argc, char **argv)
{

    int scenario; 
    Environment env;
    std::vector<double> startVec;
    std::vector<double> goalVec;
    bool optimal=false;
    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) Robot Reaching Task" << std::endl;
        std::cout << " (2) Robot Avoiding Task" << std::endl;
        std::cout << " (3) Robot Avoiding Task with maximum clearance" << std::endl;

        std::cin >> scenario;
    } while (scenario < 1 || scenario > 4);

    switch (scenario)
    {
        case 1:
            makeScenario1(env, startVec, goalVec);
            break;
        case 2:
            makeScenario2(env, startVec, goalVec);
            break;
        case 3:
            makeScenario2(env, startVec, goalVec);
            break;
        default:
            std::cerr << "Invalid Scenario Number!" << std::endl;
    }

    auto space = createChainBoxSpace();
    ompl::geometric::SimpleSetup ss(space);
    if(scenario==3){
    setupCollisionChecker(ss, env,true);
    scenario=2;
    optimal=true;}
    else
    {
    setupCollisionChecker(ss, env,false);}
    
   
    //setup Start and Goal
    ompl::base::ScopedState<> start(space), goal(space);
    space->setup();
    space->copyFromReals(start.get(), startVec);
    space->copyFromReals(goal.get(), goalVec);
    ss.setStartAndGoalStates(start, goal);

    switch (scenario)
    {
        case 1:
            planScenario1(ss,env);
           benchScenario1(ss);
            break;
        case 2:
            planScenario2(ss,env,optimal);
            benchScenario2(ss);
            break;

        default:
            optimal=false;
            std::cerr << "Invalid Scenario Number!" << std::endl;
    }

}



