///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Omri Green
//////////////////////////////////////

#ifndef RG_RRT_H
#define RG_RRT_H


  

  
 #include "ompl/control/planners/PlannerIncludes.h"
 #include "ompl/datastructures/NearestNeighbors.h"
  

 namespace ompl
 {
     namespace control
     {
         class RG_RRT : public base::Planner
         {
         public:
             RG_RRT(const SpaceInformationPtr &si);
  
             ~RG_RRT() override;
            
             void setKNearest(int k);
             int getKNearest() const;
             base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
  
             void clear() override;
  
             void setGoalBias(double goalBias)
             {
                 goalBias_ = goalBias;
             }
  
             double getGoalBias() const
             {
                 return goalBias_;
             }
  
             bool getIntermediateStates() const
             {
                 return addIntermediateStates_;
             }
  
             void setIntermediateStates(bool addIntermediateStates)
             {
                 addIntermediateStates_ = addIntermediateStates;
             }
  
             void getPlannerData(base::PlannerData &data) const override;
  
             template <template <typename T> class NN>
             void setNearestNeighbors()
             {
                 if (nn_ && nn_->size() != 0)
                     OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                 clear();
                 nn_ = std::make_shared<NN<Motion *>>();
                 setup();
             }
  
             void setup() override;
         private:
            int kNearest_;

         protected:
             class Motion
             {
             public:
                 Motion() = default;
  
                 Motion(const SpaceInformation *si)
                   : state(si->allocState()), control(si->allocControl())
                 {
                 }
  
                 ~Motion() = default;
  
                 base::State *state{nullptr};
  
                 Control *control{nullptr};
  
                 unsigned int steps{0};
  
                 Motion *parent{nullptr};
             };
  
             void freeMemory();
  
             double distanceFunction(const Motion *a, const Motion *b) const
             {
                 return si_->distance(a->state, b->state);
             }
  
             base::StateSamplerPtr sampler_;
  
             DirectedControlSamplerPtr controlSampler_;
  
             const SpaceInformation *siC_;
  
             std::shared_ptr<NearestNeighbors<Motion *>> nn_;
  
             double goalBias_{0.05};
  
             bool addIntermediateStates_{false};
  
             RNG rng_;
  
             Motion *lastGoalMotion_{nullptr};
         };
     }
 }
  
 #endif