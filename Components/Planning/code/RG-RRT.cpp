///////////////////////////////////////
// RBE550
// Project 4
// Authors: Omri Green
//////////////////////////////////////

#include "RG-RRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>


namespace ob = ompl::base;
namespace oc = ompl::control;

ompl::control::RG_RRT::RG_RRT(const SpaceInformationPtr &si) : base::Planner(si, "RG_RRT"), kNearest_(10)
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &RG_RRT::setGoalBias, &RG_RRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RG_RRT::setIntermediateStates, &RG_RRT::getIntermediateStates,
                                "0,1");
}

void ompl::control::RG_RRT::setKNearest(int k) 
{ 
    kNearest_ = k; 
}

int ompl::control::RG_RRT::getKNearest() const 
{ 
    return kNearest_; 
}

ompl::control::RG_RRT::~RG_RRT()
{
    freeMemory();
}

void ompl::control::RG_RRT::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::control::RG_RRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::RG_RRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::control::RG_RRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    // Adds the start state with more efficient initialization
    std::vector<Motion*> startMotions;
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        nn_->add(motion);
        startMotions.push_back(motion);
    }

    // Early exit for invalid start states
    if (startMotions.empty())
    {
        OMPL_ERROR("%s: No valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Lazy initialization of samplers
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u initial states", getName().c_str(), startMotions.size());

    Motion *solution = nullptr;
    Motion *bestApproxSolution = nullptr;
    double bestApproxDistance = std::numeric_limits<double>::infinity();

    // motion and state objects
    Motion *randomMotion = new Motion(siC_);
    base::State *randomState = randomMotion->state;
    Control *randomControl = randomMotion->control;
    base::State *intermediateState = si_->allocState();

    // adaptive exploration variables
    size_t iterationCount = 0;
    const size_t MAX_ITERATIONS = 5000;  
    const double INITIAL_GOAL_BIAS = 0.1;
    const double MAX_GOAL_BIAS = 0.5;

    while (!ptc && iterationCount < MAX_ITERATIONS)
    {
        iterationCount++;
        
        // Adaptive goal biasing
        double dynamicGoalBias = std::min(INITIAL_GOAL_BIAS * (1.0 + iterationCount / 1000.0), MAX_GOAL_BIAS);
        
        // Sample goal or random state with adaptive bias
        if (goal_s && rng_.uniform01() < dynamicGoalBias && goal_s->canSample())
            goal_s->sampleGoal(randomState);
        else
            sampler_->sampleUniform(randomState);

        // Find nearest neighbors 
        std::vector<Motion*> nearestNeighbors;
        nn_->nearestK(randomMotion, 5, nearestNeighbors);  

        Motion *bestMotion = nullptr;
        Control *bestControl = siC_->allocControl();
        unsigned int bestControlDuration = 0;
        // arbitrarilly large distance
        double bestDistance = std::numeric_limits<double>::infinity();

        // nearest neighbor selection (only if better)
        for (Motion *neighborMotion : nearestNeighbors) 
        {
            Control *trialControl = siC_->allocControl();
            unsigned int controlDuration = controlSampler_->sampleTo(
                trialControl, neighborMotion->control, 
                neighborMotion->state, randomState
            );

            if (controlDuration >= siC_->getMinControlDuration()) 
            {
                std::vector<base::State*> intermediateStates;
                controlDuration = siC_->propagateWhileValid(
                    neighborMotion->state, trialControl, controlDuration, 
                    intermediateStates, true
                );

                if (!intermediateStates.empty()) 
                {
                    double distance = si_->distance(intermediateStates.back(), randomState);
                    if (distance < bestDistance) 
                    {
                        bestDistance = distance;
                        bestMotion = neighborMotion;
                        siC_->copyControl(bestControl, trialControl);
                        bestControlDuration = controlDuration;
                    }

                    // Clean up intermediate states
                    for (auto& state : intermediateStates)
                        si_->freeState(state);
                }
            }
            siC_->freeControl(trialControl);
        }

        // uses normal RRT if RG-RRT Doesn't work
        Motion *selectedMotion = bestMotion ? bestMotion : nn_->nearest(randomMotion);
        
        // Extends path and checks goal, terminate early if necessary
        Motion *newMotion = new Motion(siC_);
        si_->copyState(newMotion->state, randomState);
        siC_->copyControl(newMotion->control, bestControl ? bestControl : randomControl);
        newMotion->steps = bestControlDuration;  
        newMotion->parent = selectedMotion;
        
        nn_->add(newMotion);

        // Goal checking with approximation
        double goalDistance = 0.0;
        bool goalReached = goal->isSatisfied(newMotion->state, &goalDistance);
        
        if (goalReached) 
        {
            solution = newMotion;
            break;
        }

        if (goalDistance < bestApproxDistance) 
        {
            bestApproxDistance = goalDistance;
            bestApproxSolution = newMotion;
        }

        // Free best control if allocated
        if (bestControl) 
        {
            siC_->freeControl(bestControl);
            bestControl = nullptr;
        }
    }

    // Solution reconstruction
    bool solved = false;
    bool approximate = false;

    if (!solution && bestApproxSolution) 
    {
        solution = bestApproxSolution;
        approximate = true;
    }

    if (solution) 
    {
        std::vector<Motion*> solutionPath;
        while (solution) 
        {
            solutionPath.push_back(solution);
            solution = solution->parent;
        }

        auto path = std::make_shared<PathControl>(si_);
        for (auto it = solutionPath.rbegin(); it != solutionPath.rend(); ++it) 
        {
            if ((*it)->parent)
                path->append((*it)->state, (*it)->control, 
                             (*it)->steps * siC_->getPropagationStepSize());
            else
                path->append((*it)->state);
        }

        solved = true;
        pdef_->addSolutionPath(path, approximate, bestApproxDistance, getName());
    }

    // Memory cleanup
    si_->freeState(randomMotion->state);
    siC_->freeControl(randomMotion->control);
    delete randomMotion;
    si_->freeState(intermediateState);

    OMPL_INFORM("%s: Planning completed. Total states: %u, Iterations: %zu", 
                getName().c_str(), nn_->size(), iterationCount);

    return {solved, approximate};
}

void ompl::control::RG_RRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                            control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}
