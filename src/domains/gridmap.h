#ifndef WARTHOG_GRIDMAP_H
#define WARTHOG_GRIDMAP_H

// gridmap.h
//
// A uniform cost gridmap implementation.  The map is stored in
// a compact matrix form. Nodes are represented as single bit quantities.
//
// NB: To allow efficient lookups this implementation uses a padding scheme
// that adds extra elements to the end of each row and also introduces an 
// empty row immediately before and immediately after the start and end of 
// the gridmap data.
// Padding allows us to refer to tiles and their neighbours by their indexes
// in a one dimensional array and also to avoid range checks when trying to 
// identify invalid neighbours of tiles on the edge of the map.
//
// @author: dharabor
// @created: 08/08/2012
// 

#include "constants.h"
#include "helpers.h"
#include "gm_parser.h"

#include <climits>
#include "stdint.h"

namespace warthog
{

class gridmap
{
	public:
		gridmap(uint32_t height, unsigned int width);
		gridmap(const char* filename);
		~gridmap();

		// here we convert from the coordinate space of 
		// the original grid to the coordinate space of db_. 
		inline uint32_t
		to_padded_id(uint32_t node_id)
		{
			return node_id + 
				// padded rows before the actual map data starts
				padded_rows_before_first_row_*padded_width_ +
			   	// padding from each row of data before this one
				(node_id / header_.width_) * padding_per_row_;
		}

		// here we convert from the coordinate space of 
		// the original grid to the coordinate space of db_. 
		inline uint32_t
		to_padded_id(uint32_t x, uint32_t y)
		{
			return to_padded_id(y * this->header_width() + x);
		}

		inline void
		to_unpadded_xy(uint32_t padded_id, uint32_t& x, uint32_t& y)
		{
			padded_id -= padded_rows_before_first_row_* padded_width_;
			y = padded_id / padded_width_;
			x = padded_id % padded_width_;
		}

		inline void
		to_padded_xy(uint32_t padded_id, uint32_t& x, uint32_t& y)
		{
			y = padded_id / padded_width_;
			x = padded_id % padded_width_;
		}

		inline int
		to_unpadded_id(uint32_t padded_id)
		{
			padded_id -= padded_rows_before_first_row_* padded_width_;
			int y = padded_id / padded_width_;
			int x = padded_id % padded_width_;
			int unpadded_id = y * (this->header_.width_) + x;
			return unpadded_id;
		}

		// get the immediately adjacent neighbours of @param node_id
		// neighbours from the row above node_id are stored in 
		// @param tiles[0], neighbours from the same row in tiles[1]
		// and neighbours from the row below in tiles[2].
		// In each case the bits for each neighbour are in the three 
		// lowest positions of the byte.
		// position :0 is the nei in direction NW, :1 is N and :2 is NE 
		inline void
		get_neighbours(uint32_t padded_id, uint8_t tiles[3])
		{
			// 1. calculate the dbword offset for the node at index padded_id
			// 2. convert padded_id into a dbword index.
			uint32_t bit_offset = (padded_id & warthog::DBWORD_BITS_MASK);
			uint32_t dbindex = padded_id >> warthog::LOG2_DBWORD_BITS;

			// compute dbword indexes for tiles immediately above 
			// and immediately below node_id
			uint32_t pos1 = dbindex - dbwidth_;
			uint32_t pos2 = dbindex;
			uint32_t pos3 = dbindex + dbwidth_;

			// read from the byte just before node_id and shift down until the
			// nei adjacent to node_id is in the lowest position
			tiles[0] = (uint8_t)(*((uint32_t*)(db_+(pos1-1))) >> (bit_offset+7));
			tiles[1] = (uint8_t)(*((uint32_t*)(db_+(pos2-1))) >> (bit_offset+7));
			tiles[2] = (uint8_t)(*((uint32_t*)(db_+(pos3-1))) >> (bit_offset+7));
		}
		
		inline void
		get_neighbours(uint32_t db_id, uint32_t ids[9], warthog::dbword costs[9])
		{
            // calculate ids of all adjacent neighbours
            ids[4] = db_id;
            ids[3] = db_id-1; // west
            ids[5] = db_id+1; // east
            ids[1] = db_id - this->padded_width_; // north
            ids[7] = db_id + this->padded_width_; // south
            ids[0] = ids[3] - this->padded_width_; // northwest
            ids[2] = ids[5] - this->padded_width_; // northeast
            ids[6] = ids[3] + this->padded_width_; // southwest
            ids[8] = ids[5] + this->padded_width_; // southeast
			
			unsigned int current = to_unpadded_id(db_id);
            unsigned int west = to_unpadded_id(db_id)-1; // west
            unsigned int east = to_unpadded_id(db_id)+1; // east
            unsigned int north = to_unpadded_id(db_id - this->padded_width_); // north
           	unsigned int south = to_unpadded_id(db_id + this->padded_width_); // south
            unsigned int northwest = to_unpadded_id(db_id-1 - this->padded_width_); // northwest
           	unsigned int northeast = to_unpadded_id(db_id+1 - this->padded_width_); // northeast
            unsigned int southwest = to_unpadded_id(db_id-1 + this->padded_width_); // southwest
            unsigned int southeast = to_unpadded_id(db_id+1 + this->padded_width_); // southeast

            // read terrain costs
            costs[0] = weight[northwest];			//we store the NE in 0, but in get_neighbour method, we store the north as the first element
            costs[1] = weight[north];
            costs[2] = weight[northeast];
            costs[3] = weight[west];
            costs[4] = weight[current];
            costs[5] = weight[east];
            costs[6] = weight[southwest];
            costs[7] = weight[south];
            costs[8] = weight[southeast];
		}

		
		// fetches a contiguous set of tiles from three adjacent rows. each row is
		// 32 tiles long. the middle row begins with tile padded_id. the other tiles
		// are from the row immediately above and immediately below padded_id.
		void
		get_neighbours_32bit(uint32_t padded_id, uint32_t tiles[3])
		{
			// 1. calculate the dbword offset for the node at index padded_id
			// 2. convert padded_id into a dbword index.
			uint32_t bit_offset = (padded_id & warthog::DBWORD_BITS_MASK);
			uint32_t dbindex = padded_id >> warthog::LOG2_DBWORD_BITS;

			// compute dbword indexes for tiles immediately above 
			// and immediately below node_id
			uint32_t pos1 = dbindex - dbwidth_;
			uint32_t pos2 = dbindex;
			uint32_t pos3 = dbindex + dbwidth_;

			// read 32bits of memory; padded_id is in the 
			// lowest bit position of tiles[1]
			tiles[0] = (uint32_t)(*((uint64_t*)(db_+pos1)) >> (bit_offset));
			tiles[1] = (uint32_t)(*((uint64_t*)(db_+pos2)) >> (bit_offset));
			tiles[2] = (uint32_t)(*((uint64_t*)(db_+pos3)) >> (bit_offset));
		}

		// similar to get_neighbours_32bit but padded_id is placed into the
		// upper bit of the return value. this variant is useful when jumping
		// toward smaller memory addresses (i.e. west instead of east).
		inline void
		get_neighbours_upper_32bit(uint32_t padded_id, uint32_t tiles[3])
		{
			// 1. calculate the dbword offset for the node at index padded_id
			// 2. convert padded_id into a dbword index.
			uint32_t bit_offset = (padded_id & warthog::DBWORD_BITS_MASK);
			uint32_t dbindex = padded_id >> warthog::LOG2_DBWORD_BITS;
			
			// start reading from a prior index. this way everything
			// up to padded_id is cached.
			dbindex -= 4;

			// compute dbword indexes for tiles immediately above 
			// and immediately below node_id
			uint32_t pos1 = dbindex - dbwidth_;
			uint32_t pos2 = dbindex;
			uint32_t pos3 = dbindex + dbwidth_;

			// read 32bits of memory; padded_id is in the 
			// highest bit position of tiles[1]
			tiles[0] = (uint32_t)(*((uint64_t*)(db_+pos1)) >> (bit_offset+1));
			tiles[1] = (uint32_t)(*((uint64_t*)(db_+pos2)) >> (bit_offset+1));
			tiles[2] = (uint32_t)(*((uint64_t*)(db_+pos3)) >> (bit_offset+1));
		}

		// get the label associated with the padded coordinate pair (x, y)


		inline void
		bomb(unsigned int seed,unsigned int radius, warthog::dbword bomb_cost)
		{
			this->set_label_weight(seed, bomb_cost);
			for(int i=1; i<=radius;i++){
				this->set_label_weight(seed+i, bomb_cost);
				this->set_label_weight(seed-i, bomb_cost);
			}
			// this->set_label_weight(seed, bomb_cost);
			// this->set_label_weight(seed+1, bomb_cost); 	//east
			// this->set_label_weight(seed-1, bomb_cost);	//west
			for(int i=1; i<=radius;i++){
				this->set_label_weight(seed+ i*header_width(), bomb_cost);
				for(int j=1; j<=radius;j++){
					this->set_label_weight(seed+ i*header_width()+j, bomb_cost);
					this->set_label_weight(seed+ i*header_width()-j, bomb_cost);
				}
				this->set_label_weight(seed- i*header_width(), bomb_cost);
				for(int j=1; j<=radius;j++){
					this->set_label_weight(seed- i*header_width()+j, bomb_cost);
					this->set_label_weight(seed- i*header_width()-j, bomb_cost);
				}
			}
			// this->set_label_weight(seed+ header_width(), bomb_cost);	//south
			// this->set_label_weight(seed- header_width(), bomb_cost);	//north
			// this->set_label_weight(seed+ header_width() +1, bomb_cost);//SE
			// this->set_label_weight(seed+ header_width()-1, bomb_cost);//SW
			// this->set_label_weight(seed- header_width()+1, bomb_cost);//NE
			// this->set_label_weight(seed- header_width()-1, bomb_cost);//NW
		}
		
		inline bool
		get_label(uint32_t x, unsigned int y)
		{
			return this->get_label(y*padded_width_+x);
		}

		inline warthog::dbword 
		get_label(uint32_t padded_id)
		{
			// now we can fetch the label
			warthog::dbword bitmask = 1;
			bitmask <<= (padded_id & warthog::DBWORD_BITS_MASK);
			uint32_t dbindex = padded_id >> warthog::LOG2_DBWORD_BITS;
			if(dbindex > max_id_) { return 0; }
			return (db_[dbindex] & bitmask) != 0;
			// if ((db_[dbindex] & bitmask) != 0){
			// 	return db_[padded_id];
			// }
			// else{
			// 	return 0;
			// }
		}

		// set the label associated with the padded coordinate pair (x, y)
		inline void
		set_label(uint32_t x, unsigned int y, bool label)
		{
			this->set_label(y*padded_width_+x, label);
		}

		inline void 
		set_label(uint32_t padded_id, bool label)
		{
			warthog::dbword bitmask =
				(1 << (padded_id & warthog::DBWORD_BITS_MASK));
			uint32_t dbindex = padded_id >> warthog::LOG2_DBWORD_BITS;
			if(dbindex > max_id_) { return; }
			if(label)
			{
				db_[dbindex] |= bitmask;
			}
			else
			{
				db_[dbindex] &= ~bitmask;
			}
		}

		inline void
		set_label_weight(unsigned int i, warthog::dbword weight_value)
		{
			weight[i] = weight_value;

		}

		inline warthog::dbword 
		get_label_weight(unsigned int i)
		{
			return weight[i];

		}


		inline uint32_t
		padded_mapsize()
		{
			return padded_width_ * padded_height_;
		}

		inline uint32_t 
		height() const
		{ 
			return this->padded_height_;
		} 

		inline uint32_t 
		width() const 
		{ 
			return this->padded_width_;
		}

		inline uint32_t 
		header_height()
		{
			return this->header_.height_;
		}

		inline uint32_t 
		header_width()
		{
			return this->header_.width_;
		}

		inline const char*
		filename()
		{
			return this->filename_;
		}

		void 
		print(std::ostream&);
		
		void
		printdb(std::ostream& out);

		uint32_t 
		mem()
		{
			return sizeof(*this) +
			sizeof(warthog::dbword) * db_size_;
		}


	private:
		warthog::gm_header header_;
		warthog::dbword* db_;

		warthog::dbword* weight;

		char filename_[256];

		uint32_t dbwidth_;
		uint32_t dbheight_;
		uint32_t db_size_;
		uint32_t padded_width_;
		uint32_t padded_height_;
		uint32_t padding_per_row_;
		uint32_t padding_column_above_;
		uint32_t padded_rows_before_first_row_;
		uint32_t padded_rows_after_last_row_;
		uint32_t max_id_;

		gridmap(const warthog::gridmap& other) {}
		gridmap& operator=(const warthog::gridmap& other) { return *this; }
		void init_db();
};

}

#endif

