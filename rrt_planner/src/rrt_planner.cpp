
#include <rrt_planner/rrt_planner.h>

namespace rrt_planner {

    RRTPlanner::RRTPlanner(costmap_2d::Costmap2DROS *costmap, 
            const rrt_params& params) : params_(params), collision_dect_(costmap) {

        costmap_ = costmap->getCostmap();
        map_width_  = costmap_->getSizeInMetersX();
        map_height_ = costmap_->getSizeInMetersY();

        random_double_x.setRange(-map_width_, map_width_);
        random_double_y.setRange(-map_height_, map_height_);

        nodes_.reserve(params_.max_num_nodes);
    }

    bool RRTPlanner::planPath() {

        // clear everything before planning
        nodes_.clear();

        // Start Node
        createNewNode(start_, -1);

        double *p_rand, *p_new;
        Node nearest_node;

        for (unsigned int k = 1; k <= params_.max_num_nodes; k++) {

            p_rand = sampleRandomPoint();
            nearest_node = nodes_[getNearestNodeId(p_rand)];
            p_new = extendTree(nearest_node.pos, p_rand); // new point and node candidate
            ROS_INFO("Random point %f %f ", p_rand[0], p_rand[1]);
            ROS_INFO("Nearest vector %f %f ", nearest_node.pos[0], nearest_node.pos[1]);
            ROS_INFO("Values of the vector");

            for( int i = 0; i < nodes_.size(); i++){
                ROS_INFO("%f %f ", nodes_[i].pos[0], nodes_[i].pos[1]);
            }
            if (!collision_dect_.obstacleBetween(nearest_node.pos, p_new)) {
                createNewNode(p_new, nearest_node.node_id);

            } else {
                continue;
            }

            if(k > params_.min_num_nodes) {
                
                if(computeDistance(p_new, goal_) <= params_.goal_tolerance){
                    return true;
                }
            }
        }

        return false;
    }

    int RRTPlanner::getNearestNodeId(const double *point) {

        double min = 100000,aux = 0;
        int id = -100;
        for( int i = 0; i < nodes_.size(); i++){
            aux = computeDistance(nodes_[i].pos,point);
            if (aux < min){
                min = aux;
                id = i;
            }
        }
        if(id == -100) id=0;
        return id;

    }

    void RRTPlanner::createNewNode(const double* pos, int parent_node_id) {

        Node new_node(pos, nodes_.size()+1,parent_node_id);
        ROS_INFO("New Node");
        nodes_.emplace_back(new_node);

        
    }

    double* RRTPlanner::sampleRandomPoint() {

        rand_point_[0] = random_double_x.generate();
        rand_point_[1] = random_double_y.generate();

        return rand_point_;
    }

    double* RRTPlanner::extendTree(const double* point_nearest, const double* point_rand) {

        double result_vector[] = {point_rand[0]-point_nearest[0],point_rand[1]-point_nearest[1]};
        double zero_vector[] = {0.0,0.0};
        double norm = computeDistance(result_vector,zero_vector);


        candidate_point_[0] = point_rand[0];
        candidate_point_[1] = point_rand[1];

        return candidate_point_;
    }

    const std::vector<Node>& RRTPlanner::getTree() {

        return nodes_;
    }

    void RRTPlanner::setStart(double *start) {

        start_[0] = start[0];
        start_[1] = start[1];
    }

    void RRTPlanner::setGoal(double *goal) {

        goal_[0] = goal[0];
        goal_[1] = goal[1];
    }

};