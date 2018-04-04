#ifndef WARTHOG_CPD_HEURISTIC_H
#define WARTHOG_CPD_HEURISTIC_H
#define EXTRACT_ALL_AT_ONCE

#include "constants.h"
#include "helpers.h"
#include "Entry.h"
#include <cstdlib>
#include <vector>
#include <iostream>
#include "weighted_gridmap.h"
#include "gridmap.h"
// #include "scenario_manager.h"

namespace warthog
{

class cpd_heuristic
{
	public:
		cpd_heuristic(warthog::gridmap* map, unsigned int mapwidth, unsigned int mapheight, unsigned int width, unsigned int height) 
	    	: map_(map), mapwidth_(mapwidth), mapheight_(mapheight), width_(width), height_(height) { }
		~cpd_heuristic() { }
		//we deliver the map into cpd_heuristic class here, actually we can remove the mapwidth_ and so on

		inline warthog::cost_t
		h_cpd(unsigned int id, unsigned int id2, void *reference, std::vector<double> &h_array, std::vector<int> &exp_id, int current_id)
		{
			std::vector<xyLoc> thePath;
			xyLoc s, g;
			bool done;

			id -= 3 * mapwidth_;
			s.y = id / mapwidth_;
			s.x = id % mapwidth_;

			id2 -= 3 * mapwidth_;
			g.y = id2 / mapwidth_;
			g.x = id2 % mapwidth_;

			unsigned int unpadded_id = s.y * width_ + s.x;
			unsigned int unpadded_id2 = g.y * width_ + g.x;

			warthog::dbword costt = map_->get_label_weight(unpadded_id);
			// int costt = map_->get_label_weight(id);
			// int costt = weight[unpadded_id];
			
			// //version 2
			// if(h_array.empty()){
			// 	double len = 0;
			// 	for (int i=0; i<= height_ *width_ + width_; i++)
			// 	{
			// 		h_array.push_back(0);
			// 	}
			// 	do{
			// 		done = warthog::GetPath(reference, s, g, thePath);
			// 	}while(done==false);

			// 	for (int x = (int)thePath.size()-1; x >0; x--)
			// 	{
			// 		if (thePath[x].x == thePath[x-1].x || thePath[x].y == thePath[x-1].y)
			// 		{
			// 			len++;
			// 		}
			// 		else {
			// 			len += 1.4142;
			// 		}
					
			// 		h_array[thePath[x-1].y * width_ + thePath[x-1].x] = len;
			// 	}
				
			// 	//std::cerr<<h_array.size()<<' '<<h_array[0]<<' '<<h_array[1];
			// 	return len * warthog::ONE;
			// }
			
			//else{
			// 	double len = 0;
			// 	int x=0;
			// 	do{
			// 		done = warthog::GetPath(reference, s, g, thePath);
			// 		if (h_array[thePath[x].y * width_ + thePath[x].x] !=0){
			// 			done = true;
			// 			store = h_array[thePath[x].y * width_ + thePath[x].x];
			// 			break;
			// 		}
			// 		x++
			// 	}while(done==false);

			// 	for (int x = (int)thePath.size()-1; x >0; x--)
			// 	{
			// 		if (thePath[x].x == thePath[x-1].x || thePath[x].y == thePath[x-1].y)
			// 		{
			// 			len++;
			// 		}
			// 		else {
			// 			len += 1.4142;
			// 		}
					
			// 		h_array[thePath[x-1].y * width_ + thePath[x-1].x] = len;
			// 	}
			// 	return len * warthog::ONE;
			// }
			//}

			if(exp_id[unpadded_id] == current_id){			//current_id indicate which problem we are solving in one scenario
				if(h_array[unpadded_id] != 0)
				{
					return (h_array[unpadded_id] * warthog::ONE);
				}
			}
			double len = 0;
			double store = 0;
			int x=0;
			do{				
				done = warthog::GetPath(reference, s, g, thePath);		
				if (h_array[thePath[x].y * width_ + thePath[x].x] !=0 && exp_id[thePath[x].y * width_ + thePath[x].x]==current_id){
					done = true;
					store = h_array[thePath[x].y * width_ + thePath[x].x]; // in the process of finding cpd path, if we can find already stored and correct h cost, we use it and stop search.
					break;
				}
				x++;
			}while(done==false);

			for (int x = (int)thePath.size()-1; x > 0; x--)					//after finding the path basing on cpd, we store the cpd value for each node on the path
			{
				if (thePath[x].x == thePath[x-1].x || thePath[x].y == thePath[x-1].y)
				{
					// len += map_->get_label(thePath[x].x,thePath[x].y);
					len +=1;
				}
				else {
					// len += 1.4142*(map_->get_label(thePath[x].x,thePath[x].y));
					len += 1.4142;
				}
				h_array[thePath[x-1].y * width_ + thePath[x-1].x] = len + store;
				exp_id[thePath[x-1].y * width_ + thePath[x-1].x] = current_id;
				/**
				 * exp_id indicate which problem is the h_cost stored in h_array for
				 * if current_id is not equal to the exp_id, it means the value in h_array is for different problem(different s-t pair), 
				 * thus useless
				 **/
																					
			}
			return ((len + store) * warthog::ONE) ;


			// //Version 1
			// std::vector<xyLoc> thePath;
			// xyLoc s, g;
			// bool done;

			// id -= 3 * mapwidth_;
			// s.y = id / mapwidth_;
			// s.x = id % mapwidth_;

			// id2 -= 3 * mapwidth_;
			// g.y = id2 / mapwidth_;
			// g.x = id2 % mapwidth_;			
			// if(h_array.empty()){
			// 	double len = 0;
			// 	do{
			// 		done = warthog::GetPath(reference, s, g, thePath);
			// 	}while(done==false);

			// 	for (int x = (int)thePath.size()-1; x >0; x--)
			// 	{
			// 		if (thePath[x].x == thePath[x-1].x || thePath[x].y == thePath[x-1].y)
			// 		{
			// 			len++;
			// 		}
			// 		else {
			// 			len += 1.4142;
			// 		}
			// 		h_array.push_back(thePath[x-1].x);
			// 		h_array.push_back(thePath[x-1].y);
			// 		h_array.push_back(len * warthog::ONE);
			// 	}

			// 	return len * warthog::ONE;
			// }

			// else{
			// 	for(int i= 0 ; i <= h_array.size()-3 ; i+=3 )
			// 	{
			// 		if(s.x == h_array[i] && s.y == h_array[i+1])
			// 		{
			// 			return h_array[i+2];
			// 		}
			// 	}
				
			// 	double len = 0;
			// 	int x=0;
			// 	double store = 0;
			// 	do{
					
			// 		done = warthog::GetPath(reference, s, g, thePath);
					
			// 		for(int i= 0 ; i <= h_array.size()-3 ; i+=3 )
			// 		{
			// 			if(thePath[x].x == h_array[i] && thePath[x].y == h_array[i+1])
			// 			{
			// 				done = true;
			// 				store = h_array[i+2];
			// 				break;
			// 			}
			// 		}
			// 		x++;
			// 	}while(done==false);

			// 	for (int x = (int)thePath.size()-1; x > 0; x--)
			// 	{
			// 		if (thePath[x].x == thePath[x-1].x || thePath[x].y == thePath[x-1].y)
			// 		{
			// 			len++;
			// 		}
			// 		else {
			// 			len += 1.4142;
			// 		}
			// 		h_array.push_back(thePath[x-1].x);
			// 		h_array.push_back(thePath[x-1].y);
			// 		h_array.push_back(len * warthog::ONE + store);
			// 	}
			// 	return (len * warthog::ONE + store);

			// }


		}
		
		/**
		 * For calculating the UB, we just need to read the actual cost rather than 1 or 1.4.
		 * if vertical or horizontal, we get 2 weight addition and /2, if diagonal direction, we need get 4 addition and /4
		 * i.e. keep consistent with how we calculate in expand
		 **/

		inline warthog::cost_t
		h_cpd_UB(unsigned int id, unsigned int id2, void *reference, std::vector<double> &h_array, std::vector<int> &exp_id, int current_id)
		{
			std::vector<xyLoc> thePath;
			xyLoc s, g;
			bool done;

			id -= 3 * mapwidth_;
			s.y = id / mapwidth_;
			s.x = id % mapwidth_;

			id2 -= 3 * mapwidth_;
			g.y = id2 / mapwidth_;
			g.x = id2 % mapwidth_;

			unsigned int unpadded_id = s.y * width_ + s.x;
			unsigned int unpadded_id2 = g.y * width_ + g.x;

			// int i = weight[unpadded_id];
			if(exp_id[unpadded_id] == current_id){
				if(h_array[unpadded_id] != 0)
				{
					return (h_array[unpadded_id] * warthog::ONE);
				}
			}
			double len = 0;
			double store = 0;
			int x=0;
			do{				
				done = warthog::GetPath(reference, s, g, thePath);		
				if (h_array[thePath[x].y * width_ + thePath[x].x] !=0 && exp_id[thePath[x].y * width_ + thePath[x].x]==current_id){
					done = true;
					store = h_array[thePath[x].y * width_ + thePath[x].x]; 
					break;
				}
				x++;
			}while(done==false);
			for (int x = (int)thePath.size()-1; x > 0; x--)					
			{
				if (thePath[x].x == thePath[x-1].x || thePath[x].y == thePath[x-1].y)
				{
					// // len += map_->get_label(thePath[x].x,thePath[x].y);
					// len +=1;
				}
				else {
					// // len += 1.4142*(map_->get_label(thePath[x].x,thePath[x].y));
					// len += 1.4142;
				}
				h_array[thePath[x-1].y * width_ + thePath[x-1].x] = len + store;
				exp_id[thePath[x-1].y * width_ + thePath[x-1].x] = current_id;															
			}
			return ((len + store) * warthog::ONE) ;
		}


inline warthog::cost_t
		h(int32_t x, int32_t y, int32_t x2, int32_t y2)
		{
            // NB: precision loss when warthog::cost_t is an integer
			double dx = abs(x-x2);
			double dy = abs(y-y2);
			if(dx < dy)
			{
				return dx * warthog::ROOT_TWO + (dy - dx) * warthog::ONE;
			}
			return dy * warthog::ROOT_TWO + (dx - dy) * warthog::ONE;
		}

		inline warthog::cost_t
		h(unsigned int id, unsigned int id2)
		{
			unsigned int x, x2;
			unsigned int y, y2;
			warthog::helpers::index_to_xy(id, mapwidth_, x, y);
			warthog::helpers::index_to_xy(id2,mapwidth_, x2, y2);
			return this->h(x, y, x2, y2);
		}

	private:
		unsigned int mapwidth_;
		unsigned int mapheight_;
		unsigned int width_;
		unsigned int height_;
		warthog::gridmap* map_;
		// warthog::gridmap_expansion_policy* expander_;
		// warthog::gridmap* map_;
		//warthog::gridmap_expansion_policy* unpadded_;
};

}

#endif
