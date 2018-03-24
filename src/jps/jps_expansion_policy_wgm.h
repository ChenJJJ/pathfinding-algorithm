#ifndef WARTHOG_JPS_EXPANSION_POLICY_WGM_H
#define WARTHOG_JPS_EXPANSION_POLICY_WGM_H

// jps_expansion_policy_wgm.h
//
// A variant of Jump Point Search adapted to gridmaps with weighted costs.
// The algorithm works similar to the variant for uniform-cost maps;
// The main difference is that we stop whenever the current node has a 
// successor with a different terrain type.
//
// Further details in: 
// (to appear)
//
// @author: dharabor
// @created: 2014-09-22
//

#include "blocklist.h"
#include "weighted_gridmap.h"
#include "helpers.h"
#include "jps.h"
#include "jps_wgm.h"
#include "online_jump_point_locator_wgm.h"
#include "problem_instance.h"
#include "search_node.h"

#include "stdint.h"

namespace warthog
{
class jps_expansion_policy_wgm
{
	public:
		jps_expansion_policy_wgm(warthog::weighted_gridmap* map);
		~jps_expansion_policy_wgm();

		// create a warthog::search_node object from a state description
		// (in this case, an id)
		inline warthog::search_node*
		generate(uint32_t node_id)
		{
			return nodepool_->generate(node_id);
		}


		// reset the policy and discard all generated nodes
		inline void
		clear()
		{
			reset();
			nodepool_->clear();
		}


		void 
		expand(warthog::search_node*, warthog::problem_instance*);

		inline void
		first(warthog::search_node*& ret, warthog::cost_t& cost)
		{
			which_ = 0;
			ret = neighbours_[which_];
			cost = costs_[which_];
		}

		inline bool
		has_next()
		{
			if((which_+1) < num_neighbours_) { return true; }
			return false;
		}

		inline void
		n(warthog::search_node*& ret, warthog::cost_t& cost)
		{
			ret = neighbours_[which_];
			cost = costs_[which_];
		}

		inline void
		next(warthog::search_node*& ret, warthog::cost_t& cost)
		{
			if(which_ < num_neighbours_)
			{
				which_++;
			}
			ret = neighbours_[which_];
			cost = costs_[which_];
		}

		inline uint32_t
		mem()
		{
			return sizeof(*this) + map_->mem() + nodepool_->mem() + jpl_->mem();
		}

		uint32_t 
		mapwidth()
		{
			return map_->width();
		}

	private:
		warthog::weighted_gridmap* map_;
		warthog::blocklist* nodepool_;
		warthog::online_jump_point_locator_wgm* jpl_;
		uint32_t which_;
		uint32_t num_neighbours_;
		warthog::search_node* neighbours_[9];
		warthog::cost_t costs_[9];

		// computes the direction of travel; from a node n1
		// to a node n2.
		inline warthog::jps::direction
		compute_direction(warthog::search_node* n1, warthog::search_node* n2)
		{
			if(n1 == 0) { return warthog::jps::NONE; }

			uint32_t x, y, x2, y2;
			warthog::helpers::index_to_xy(n1->get_id(), map_->width(), x, y);
			warthog::helpers::index_to_xy(n2->get_id(), map_->width(), x2, y2);
			warthog::jps::direction dir = warthog::jps::NONE;
			if(y2 == y)
			{
				if(x2 > x)
					dir = warthog::jps::EAST;
				else
					dir = warthog::jps::WEST;
			}
			else if(y2 < y)
			{
				if(x2 == x)
					dir = warthog::jps::NORTH;
				else if(x2 < x)
					dir = warthog::jps::NORTHWEST;
				else // x2 > x
					dir = warthog::jps::NORTHEAST;
			}
			else // y2 > y 
			{
				if(x2 == x)
					dir = warthog::jps::SOUTH;
				else if(x2 < x)
					dir = warthog::jps::SOUTHWEST;
				else // x2 > x
					dir = warthog::jps::SOUTHEAST;
			}
			assert(dir != warthog::jps::NONE);
			return dir;
		}

		inline void
		reset()
		{
			which_ = 0;
			num_neighbours_ = 0;
			neighbours_[0] = 0;
		}
};
}

#endif

