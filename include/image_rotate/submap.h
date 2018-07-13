#ifndef SUBMAP_H
#define SUBMAP_H

#include <vector>
#include "nav_msgs/OccupancyGrid.h"

namespace submap
{
	using std::vector;
	class Submap
	{
	public:
		using Map = nav_msgs::OccupancyGrid;
		void operator()(const Map& m, int l, int r, int t, int d, Map& result)
		{
			result.header = m.header;
			result.info.map_load_time = m.info.map_load_time;
			result.info.width = r - l + 1;
			result.info.height = d - t + 1;
			result.info.resolution = m.info.resolution;
			result.info.origin.orientation = m.info.origin.orientation;
			result.info.origin.position.x = m.info.origin.position.x + l * m.info.resolution;
			result.info.origin.position.y = m.info.origin.position.y + t * m.info.resolution;
			result.data = vector<int8_t>(result.info.width * result.info.height, 0);
			size_t index = 0, origin_index = t * m.info.width;
			int i, j;
			for (j = t; j < d; j++)
			{
				for (i = l; i < r; i++)
				{
					result.data[index] = m.data[origin_index + i];
					index++;
				}
				index++;
				origin_index += m.info.width;
			}
		}
	};
}


#endif // !SUBMAP_H
